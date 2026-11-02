"""guardrail/paraphraser_service.py - the Paraphraser as a separate service
(WP2-13, WP4-10, ARCH-23): templated NL in, validated paraphrases out, over
gRPC with mutual TLS.

    python tests/test_paraphraser_service.py      (no pytest needed)

The grant calls the paraphraser a "separate service" (Prefix Compiler page,
"Templated NL adapter") in a control plane that is "gRPC + mTLS" (data-flow
page). Until 2026-11-02 it was a module every runner imported. These tests
show the service answers over a process boundary exactly what the module
answers in process (same ids, same validator), that every paraphrase it
sends passed the validator, that the paraphraser's refusals arrive as gRPC
status codes, and that the transport refuses plaintext off loopback and a
client certificate off the allow-list. The core needs no grpc; the transport
tests print SKIP where grpcio or the stubs cannot load, and the mutual-TLS
test where `cryptography` is missing (it writes a lab PKI).
"""
from __future__ import annotations

import atexit
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import paraphraser as P                              # noqa: E402
from guardrail import paraphraser_service as S                      # noqa: E402

SKIP = "SKIP"
_TMP: list[Path] = []

# the compiler's current renderings: a re-authored C8 set, and a task text
CSP_SITL = ("Stay between 10 m and 20 m altitude. Keep at least 5 m away from anything you "
            "are following. Keep at least 10 m away from the pedestrian you are following. "
            "Keep speed at or below 4 m/s, climb rate below 2 m/s and turn rate below 45 deg/s.")
TASK_CAR = "follow a red car"


def _tmpdir(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


@atexit.register
def _cleanup():
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)


def _grpc():
    try:
        S._grpc_modules()
    except S.ServiceError:
        return None
    import grpc
    return grpc


def _code(fn, *a, **kw) -> str:
    try:
        fn(*a, **kw)
    except Exception as e:                                       # noqa: BLE001
        return S._code_for(e)[0]
    raise AssertionError(f"{getattr(fn, '__name__', fn)} accepted {a}")


# --------------------------------------------------------------------------- #
# the core, no transport
# --------------------------------------------------------------------------- #

def test_core_returns_validated_paraphrases_the_module_would():
    core = S.ParaphraserCore()
    r = core.paraphrase(CSP_SITL, 4, 1001, "stored", include_canonical=True)
    assert len(r["paraphrases"]) == 4
    direct = P.paraphrase(CSP_SITL, 4, 1001, "stored")
    assert [p["paraphrase_id"] for p in r["paraphrases"]] == [p.paraphrase_id for p in direct]
    for p in r["paraphrases"]:
        assert P.validate(CSP_SITL, p["text"]).ok, p
        assert p["backend"] == "stored" and p["fallback_from"] == ""
    assert r["validator_digest"] == P.validator_digest()
    assert r["store_digest"] == P.store_digest()
    assert r["canonical"]["backend"] == "identity" and r["canonical"]["text"] == CSP_SITL
    # deterministic in (text, n, seed, backend)
    again = core.paraphrase(CSP_SITL, 4, 1001, "stored")
    assert [p["paraphrase_id"] for p in again["paraphrases"]] == \
        [p["paraphrase_id"] for p in r["paraphrases"]]
    assert again["canonical"] is None


def test_core_says_when_auto_fell_back_to_the_template_bank():
    text = ("Stay between 12 m and 28 m altitude. Keep at least 5 m from any building. Keep "
            "speed at or below 5 m/s, climb rate below 2 m/s and turn rate below 45 deg/s.")
    r = S.ParaphraserCore().paraphrase(text, 2, 7, "auto")
    assert all(p["backend"].startswith("template") and p["fallback_from"] == "stored"
               for p in r["paraphrases"]), r["paraphrases"]
    assert all(P.validate(text, p["text"]).ok for p in r["paraphrases"])


def test_core_refusals_carry_the_paraphrasers_reason_as_a_grpc_code():
    core = S.ParaphraserCore()
    unknown = "Stay between 1 m and 2 m altitude. Never enter zone 'nfz-nowhere'."
    assert _code(core.paraphrase, unknown, 1, 0, "stored") == "NOT_FOUND"
    aerial = "<image>\nFly forward and find the target. a red car\nAction: "
    assert _code(core.paraphrase, aerial, 1, 0, "auto") == "FAILED_PRECONDITION"
    assert _code(core.paraphrase, CSP_SITL, 9, 0, "stored") == "OUT_OF_RANGE"
    assert _code(core.paraphrase, CSP_SITL, 1, 0, "llm:none") == "FAILED_PRECONDITION"
    assert _code(core.paraphrase, CSP_SITL, 1, 0, "magic") == "INVALID_ARGUMENT"
    assert _code(core.paraphrase, "   ", 1, 0, "auto") == "INVALID_ARGUMENT"
    assert _code(core.paraphrase, CSP_SITL, 0, 0, "stored") == "INVALID_ARGUMENT"


def test_core_validate_and_status():
    core = S.ParaphraserCore()
    ok = core.validate(TASK_CAR, "track a car that is red and follow it")
    assert ok["ok"] and ok["validator_digest"] == P.validator_digest(), ok
    bad = core.validate(TASK_CAR, "follow a red truck")
    assert not bad["ok"] and any(r.startswith("object") for r in bad["reasons"]), bad
    st = core.status()
    assert st["stored_sources"] == 14 and st["stored_paraphrases"] == 112, st
    assert st["places_source"] == "guardrail.compiler.PLACES", st


# --------------------------------------------------------------------------- #
# over gRPC
# --------------------------------------------------------------------------- #

def test_committed_stubs_are_what_the_proto_generates():
    """The generated files are committed; a .proto edited without
    regenerating them fails here (as tests/test_harness_service.py does for
    harness.proto)."""
    try:
        import grpc_tools                                     # noqa: F401
    except ImportError:
        return SKIP
    out = _tmpdir("protoc_para_")
    (out / "guardrail" / "protos").mkdir(parents=True)
    shutil.copy(ROOT / "guardrail" / "protos" / "paraphraser.proto",
                out / "guardrail" / "protos" / "paraphraser.proto")
    r = subprocess.run([sys.executable, "-m", "grpc_tools.protoc", "-I", ".",
                        "--python_out=.", "--grpc_python_out=.",
                        "guardrail/protos/paraphraser.proto"],
                       cwd=str(out), capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    for name in ("paraphraser_pb2.py", "paraphraser_pb2_grpc.py"):
        got = (out / "guardrail" / "protos" / name).read_bytes()
        have = (ROOT / "guardrail" / "protos" / name).read_bytes()
        assert got == have, f"guardrail/protos/{name} is stale: regenerate it"


def test_over_grpc_on_loopback_the_service_answers_what_the_module_does():
    grpc = _grpc()
    if grpc is None:
        return SKIP
    srv = S.serve(port=0)
    try:
        assert not srv.secure
        cl = S.ParaphraserClient(srv.address)
        try:
            r = cl.paraphrase(CSP_SITL, 3, 42, "stored", include_canonical=True)
            local = S.ParaphraserCore().paraphrase(CSP_SITL, 3, 42, "stored",
                                                   include_canonical=True)
            assert r["paraphrases"] == [{**p, "seed": p["seed"]} for p in local["paraphrases"]]
            assert r["canonical"]["paraphrase_id"] == local["canonical"]["paraphrase_id"]
            assert r["canonical"]["seed"] == 0          # proto3: no null
            assert r["validator_digest"] == P.validator_digest()
            for p in r["paraphrases"]:
                assert P.validate(CSP_SITL, p["text"]).ok
            v = cl.validate(TASK_CAR, "follow two red cars")
            assert not v["ok"] and v["reasons"], v
            assert cl.status()["stored_paraphrases"] == 112
            # the paraphraser's refusals arrive as status codes, with its words
            try:
                cl.paraphrase("Never enter zone 'nfz-nowhere'.", 1, 0, "stored")
                raise AssertionError("an unknown source was served")
            except grpc.RpcError as e:
                assert e.code() == grpc.StatusCode.NOT_FOUND, e
                assert "nfz-nowhere" in e.details()
            try:
                cl.paraphrase(CSP_SITL, 9, 0, "stored")
                raise AssertionError("a padded draw was served")
            except grpc.RpcError as e:
                assert e.code() == grpc.StatusCode.OUT_OF_RANGE, e
        finally:
            cl.close()
    finally:
        srv.stop(0)


def test_plaintext_is_refused_off_loopback():
    if _grpc() is None:
        return SKIP
    try:
        S.serve(host="0.0.0.0", port=0)
    except ValueError as e:
        assert "loopback only" in str(e)
    else:
        raise AssertionError("an insecure port off loopback was opened")


def test_mutual_tls_admits_the_allow_listed_client_only():
    grpc = _grpc()
    if grpc is None:
        return SKIP
    from guardrail import harness_service as H
    try:
        import cryptography                                   # noqa: F401
    except ImportError:
        return SKIP
    pki = _tmpdir("para_pki_")
    H.generate_pki(pki)
    H.issue_cert(pki, "other-client")                  # signed by the CA, not allowed
    srv = S.serve(port=0, pki_dir=pki)
    try:
        assert srv.secure
        addr = f"localhost:{srv.port}"
        cl = S.ParaphraserClient(addr, pki_dir=pki)
        try:
            assert cl.status()["stored_sources"] == 14
            r = cl.paraphrase(TASK_CAR, 2, 3, "stored")
            assert len(r["paraphrases"]) == 2
        finally:
            cl.close()
        other = S.ParaphraserClient(addr, pki_dir=pki, client_name="other-client")
        try:
            other.status()
            raise AssertionError("a client off the allow-list was answered")
        except grpc.RpcError as e:
            assert e.code() == grpc.StatusCode.PERMISSION_DENIED, e
        finally:
            other.close()
        # no client certificate at all: the handshake itself refuses
        anon = grpc.secure_channel(addr, H.channel_credentials(pki / "ca.cert.pem"))
        try:
            S._grpc_modules().pb_grpc.ParaphraserStub(anon).GetStatus(
                S._grpc_modules().pb.StatusRequest(), timeout=10)
            raise AssertionError("a client without a certificate was answered")
        except grpc.RpcError as e:
            assert e.code() == grpc.StatusCode.UNAVAILABLE, e
        finally:
            anon.close()
    finally:
        srv.stop(0)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs grpcio / grpcio-tools / cryptography)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
