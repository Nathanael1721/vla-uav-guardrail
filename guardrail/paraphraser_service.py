"""The Paraphraser as a separate service: templated NL in, validated
paraphrases out, over gRPC with mutual TLS (WP2-13, WP4-10, ARCH-23).

    python -m guardrail.paraphraser_service serve --insecure        # loopback only
    python -m guardrail.paraphraser_service serve --pki <dir>       # mTLS, the harness lab CA
    python -m guardrail.paraphraser_service paraphrase --addr 127.0.0.1:50061 \\
        --insecure --n 3 --seed 1001 "Stay between 10 m and 20 m altitude. ..."
    python -m guardrail.paraphraser_service validate --addr ... SOURCE CANDIDATE

WHAT THE GRANT ASKS FOR

Prefix Compiler page, "Templated NL adapter": "Free-form paraphrasing is the
paraphraser's job (Q3 deliverable, separate service); the Prefix Compiler
emits the canonical templated form so the paraphraser has a clean source."
The reference data-flow page draws the "Paraphraser service (Q3 deliverable)"
in the control plane, feeding the orchestrator, which "issue[s] tasks (and
paraphrased variants) to the VLA"; that plane is "gRPC + mTLS". Until
2026-11-02 the paraphraser was a module every runner imported
(docs/DESIGN-paraphraser.md: "It is not a separate service: there is no
process boundary, no RPC and no mTLS").

WHAT THIS IS

A thin transport over guardrail.paraphraser, in the harness's own pattern
(guardrail/harness_service.py): `ParaphraserCore` is plain Python (no gRPC in
it) and answers with dicts; `ParaphraserServicer` maps it onto the generated
stubs (guardrail/protos/paraphraser.proto); `serve()` puts it on a port, with
mutual TLS from the harness's lab PKI (gen-pki) and a client common-name
allow-list, or - on a loopback address only - without; `ParaphraserClient` is
the other end.

  Paraphrase(text, n, seed, backend)  -> paraphraser.paraphrase(): every
      paraphrase returned has passed validate(text, paraphrase), and the
      candidates refused on the way come back too, with their reasons. The
      reply carries validator_digest and store_digest, so a trial record can
      name the exact validator and stored set its wording came from.
  Validate(source, candidate)         -> paraphraser.validate().
  GetStatus()                         -> the digests, the store's size, the
      backends this server can serve.

Errors are the paraphraser's own, mapped onto gRPC codes (CODES below): a
source with no stored set is NOT_FOUND, AerialVLA's protected prompt and a
backend this build cannot run are FAILED_PRECONDITION, n beyond the stored set
is OUT_OF_RANGE, a tampered store is DATA_LOSS.

WHO CALLS IT

Since 2026-11-07 the stress harness - the reference design's orchestrator,
which "issue[s] tasks (and paraphrased variants) to the VLA" - draws every
paraphrase arm through this service (guardrail/harness_service.py,
`Harness(paraphraser=ParaphraserClient(...))`, `serve --paraphraser
HOST:PORT`), with no in-process fallback: the reference design describes
none. The headless batch sweep (experiments/sweep_scenarios.py) still draws
in process.

WHAT IT IS NOT

No pilot has read the paraphrases in flight yet (WP4-09 needs a text-reading
VLA, and the harness's headless pilot reads no text), so the per-paraphrase
robustness KPI is still unmeasured; this serves the wordings it will be
measured on.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from . import paraphraser as P

DEFAULT_PORT = 50061
BACKENDS = ("auto", "stored", "template")
_MODS: SimpleNamespace | None = None

# The paraphraser's exceptions as gRPC status codes, most specific first.
CODES = (
    (P.UnknownSource, "NOT_FOUND"),
    (P.ProtectedTextError, "FAILED_PRECONDITION"),
    (P.BackendUnavailable, "FAILED_PRECONDITION"),
    (P.InsufficientParaphrases, "FAILED_PRECONDITION"),
    (P.StoreIntegrityError, "DATA_LOSS"),
    (P.ParaphraserError, "FAILED_PRECONDITION"),
)


class ServiceError(Exception):
    """A refused request: a gRPC status code name and the reason, in words."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def _grpc_modules() -> SimpleNamespace:
    """grpc and the generated stubs, on first use (the core runs without)."""
    global _MODS
    if _MODS is None:
        try:
            import grpc

            from .protos import paraphraser_pb2, paraphraser_pb2_grpc
        except ImportError as e:
            raise ServiceError("FAILED_PRECONDITION",
                               f"the Paraphraser service needs grpcio >= 1.84.0 and "
                               f"protobuf >= 7.35.1 (the versions guardrail/protos was "
                               f"generated with): {e}") from e
        _MODS = SimpleNamespace(grpc=grpc, pb=paraphraser_pb2, pb_grpc=paraphraser_pb2_grpc)
    return _MODS


def _code_for(e: BaseException) -> tuple[str, str]:
    if isinstance(e, ServiceError):
        return e.code, e.message
    for cls, code in CODES:
        if isinstance(e, cls):
            return code, str(e)
    if isinstance(e, ValueError) and "never padded" in str(e):
        return "OUT_OF_RANGE", str(e)
    if isinstance(e, (ValueError, TypeError)):
        return "INVALID_ARGUMENT", str(e)
    return "INTERNAL", f"{type(e).__name__}: {e}"


def _record(p: P.Paraphrase) -> dict:
    out = p.to_record()
    out["novelty"] = p.novelty if p.novelty is not None else 0.0
    out["fallback_from"] = p.provenance.get("fallback_from", "")
    return out


class ParaphraserCore:
    """The service's behaviour without a transport. `store_dir` defaults to
    experiments/paraphrases."""

    def __init__(self, store_dir: str | Path | None = None):
        self.store_dir = Path(store_dir) if store_dir is not None else P.STORE_DIR

    def paraphrase(self, text: str, n: int, seed: int, backend: str = "auto",
                   include_canonical: bool = False) -> dict:
        if not isinstance(text, str) or not text.strip():
            raise ServiceError("INVALID_ARGUMENT", "text is empty: send the templated NL")
        backend = backend or "auto"
        if backend not in BACKENDS and not backend.startswith("llm:"):
            raise ServiceError("INVALID_ARGUMENT",
                               f"backend {backend!r}: one of {', '.join(BACKENDS)} or llm:<id>")
        refusals: list = []
        drawn = P.paraphrase(text, n, seed, backend, store_dir=self.store_dir,
                             refusals=refusals)
        # Defence in depth: paraphrase() validates every candidate; a reply
        # that says "validated" re-checks what it is about to send.
        for p in drawn:
            v = P.validate(text, p.text)
            if not v.ok:
                raise ServiceError("INTERNAL", f"{p.paraphrase_id} failed the validator "
                                               f"after the draw: {list(v.reasons)}")
        src = P.normalise_source(text)
        return {"source_id": P.make_source_id(src), "source_text": src,
                "paraphrases": [_record(p) for p in drawn],
                "canonical": _record(P.canonical(text)) if include_canonical else None,
                "refusals": [{"backend": r["backend"], "index": r["index"],
                              "text": r["text"], "reasons": list(r["reasons"])}
                             for r in refusals],
                "validator_digest": P.validator_digest(),
                "store_digest": P.store_digest(self.store_dir)}

    def validate(self, source: str, candidate: str) -> dict:
        if not isinstance(source, str) or not source.strip():
            raise ServiceError("INVALID_ARGUMENT", "source is empty")
        v = P.validate(source, candidate or "")
        return {**v.to_dict(), "validator_digest": P.validator_digest()}

    def status(self) -> dict:
        store = P.load_store(self.store_dir)
        P._places()                           # sets PLACES_SOURCE
        return {"validator_digest": P.validator_digest(),
                "store_digest": P.store_digest(self.store_dir),
                "stored_sources": len(store),
                "stored_paraphrases": sum(len(e.paraphrases) for e in store.values()),
                "backends": list(BACKENDS), "places_source": P.PLACES_SOURCE}


class ParaphraserServicer:
    """The generated ParaphraserServicer's three methods, onto a core. Not a
    subclass of the generated class, so this module imports without grpc.
    `allowed_clients` None means no identity is required (the insecure,
    loopback-only mode); otherwise every call must arrive over TLS with a
    client certificate whose common name is in the set."""

    def __init__(self, core: ParaphraserCore, allowed_clients: set[str] | None):
        self.core = core
        self.allowed = None if allowed_clients is None else set(allowed_clients)
        self._M = _grpc_modules()

    def _call(self, context, fn):
        if self.allowed is not None:
            ac = context.auth_context() or {}
            cns = [c.decode("utf-8", "replace") for c in ac.get("x509_common_name", [])]
            if not cns or cns[0] not in self.allowed:
                context.abort(self._M.grpc.StatusCode.PERMISSION_DENIED,
                              f"client certificate {cns[0] if cns else '(none)'!r} is not "
                              f"on the paraphraser allow-list {sorted(self.allowed)}")
        try:
            return fn()
        except Exception as e:                                   # noqa: BLE001
            code, msg = _code_for(e)
        context.abort(getattr(self._M.grpc.StatusCode, code), msg)

    def _rec(self, d: dict | None):
        if d is None:
            return None
        # the canonical arm has no seed: proto3 has no null, so it is 0 there
        return self._M.pb.ParaphraseRecord(**{k: (0 if v is None else v) for k, v in d.items()})

    def Paraphrase(self, request, context):                      # noqa: N802
        def run():
            r = self.core.paraphrase(request.text, int(request.n), int(request.seed),
                                     request.backend or "auto", request.include_canonical)
            return self._M.pb.ParaphraseReply(
                source_id=r["source_id"], source_text=r["source_text"],
                paraphrases=[self._rec(p) for p in r["paraphrases"]],
                canonical=self._rec(r["canonical"]),
                refusals=[self._M.pb.Refusal(**x) for x in r["refusals"]],
                validator_digest=r["validator_digest"], store_digest=r["store_digest"])
        return self._call(context, run)

    def Validate(self, request, context):                        # noqa: N802
        return self._call(context, lambda: self._M.pb.Verdict(
            **self.core.validate(request.source, request.candidate)))

    def GetStatus(self, request, context):                       # noqa: N802
        return self._call(context, lambda: self._M.pb.Status(**self.core.status()))


@dataclass
class ParaphraserServer:
    server: Any
    port: int
    host: str
    secure: bool
    servicer: ParaphraserServicer

    @property
    def address(self) -> str:
        h = f"[{self.host}]" if ":" in self.host else self.host
        return f"{h}:{self.port}"

    def stop(self, grace: float | None = None) -> None:
        self.server.stop(grace).wait()


def serve(core: ParaphraserCore | None = None, *, host: str = "127.0.0.1", port: int = 0,
          pki_dir: str | Path | None = None, server_name: str | None = None,
          allowed_clients=None, max_workers: int = 8) -> ParaphraserServer:
    """Start the service and return it (port 0 picks a free port).

    With `pki_dir` (a folder `python -m guardrail.harness_service gen-pki`
    wrote: the control plane's one lab CA), mutual TLS: the server presents
    <server_name>.cert.pem (default harness-server), requires a client
    certificate signed by ca.cert.pem, and requires its common name to be in
    `allowed_clients` (default harness-client: the orchestrator). Without it,
    plaintext, on a loopback address only."""
    from concurrent import futures

    from .harness_service import (DEFAULT_CLIENT_CN, SERVER_CN, is_loopback, pki_paths,
                                  server_credentials)
    M = _grpc_modules()
    if pki_dir is None and not is_loopback(host):
        raise ValueError(f"refusing an insecure gRPC port on {host!r}: plaintext is for "
                         f"loopback only; give pki_dir for mutual TLS")
    allowed = None if pki_dir is None else set(allowed_clients or (DEFAULT_CLIENT_CN,))
    servicer = ParaphraserServicer(core or ParaphraserCore(), allowed)
    server = M.grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
    M.pb_grpc.add_ParaphraserServicer_to_server(servicer, server)
    addr = f"[{host}]:{port}" if ":" in host and not host.startswith("[") else f"{host}:{port}"
    if pki_dir is None:
        bound = server.add_insecure_port(addr)
    else:
        ca, cert, key = pki_paths(pki_dir, server_name or SERVER_CN)
        for f in (ca, cert, key):
            if not f.is_file():
                raise FileNotFoundError(f"{f} is missing: write the lab PKI with "
                                        f"python -m guardrail.harness_service gen-pki")
        bound = server.add_secure_port(addr, server_credentials(ca, cert, key))
    if not bound:
        raise RuntimeError(f"could not bind {addr}")
    server.start()
    return ParaphraserServer(server=server, port=bound, host=host.strip("[]"),
                             secure=pki_dir is not None, servicer=servicer)


class ParaphraserClient:
    """The other end, answering with the core's dicts. `pki_dir` and
    `client_name` (default harness-client) for mutual TLS; without them the
    channel is plaintext (a loopback server)."""

    def __init__(self, address: str, *, pki_dir: str | Path | None = None,
                 client_name: str | None = None, server_name: str | None = None,
                 timeout_s: float = 30.0):
        from .harness_service import DEFAULT_CLIENT_CN, channel_credentials, pki_paths
        M = _grpc_modules()
        self._M = M
        self.timeout_s = timeout_s
        # What a caller records about where its paraphrases came from (the
        # stress harness puts both in every arm record it draws through here).
        self.address, self.secure = address, pki_dir is not None
        if pki_dir is None:
            self.channel = M.grpc.insecure_channel(address)
        else:
            ca, cert, key = pki_paths(pki_dir, client_name or DEFAULT_CLIENT_CN)
            # the server certificate names the address (gen-pki --host); an
            # override is only for a name it does not carry
            opts = ([("grpc.ssl_target_name_override", server_name)] if server_name else [])
            self.channel = M.grpc.secure_channel(address, channel_credentials(ca, cert, key),
                                                 options=opts)
        self.stub = M.pb_grpc.ParaphraserStub(self.channel)

    def close(self) -> None:
        self.channel.close()

    @staticmethod
    def _fields(msg, names) -> dict:
        return {k: (list(getattr(msg, k)) if k == "reasons" or k == "backends"
                    else getattr(msg, k)) for k in names}

    def paraphrase(self, text: str, n: int, seed: int, backend: str = "auto",
                   include_canonical: bool = False) -> dict:
        r = self.stub.Paraphrase(self._M.pb.ParaphraseRequest(
            text=text, n=n, seed=seed, backend=backend,
            include_canonical=include_canonical), timeout=self.timeout_s)
        rec = [f.name for f in self._M.pb.ParaphraseRecord.DESCRIPTOR.fields]
        return {"source_id": r.source_id, "source_text": r.source_text,
                "paraphrases": [self._fields(p, rec) for p in r.paraphrases],
                "canonical": (self._fields(r.canonical, rec) if r.HasField("canonical")
                              else None),
                "refusals": [self._fields(x, ("backend", "index", "text", "reasons"))
                             for x in r.refusals],
                "validator_digest": r.validator_digest, "store_digest": r.store_digest}

    def validate(self, source: str, candidate: str) -> dict:
        v = self.stub.Validate(self._M.pb.ValidateRequest(source=source, candidate=candidate),
                               timeout=self.timeout_s)
        return self._fields(v, ("ok", "reasons", "novelty", "length_ratio",
                                "validator_digest"))

    def status(self) -> dict:
        s = self.stub.GetStatus(self._M.pb.StatusRequest(), timeout=self.timeout_s)
        return self._fields(s, [f.name for f in self._M.pb.Status.DESCRIPTOR.fields])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m guardrail.paraphraser_service")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve", help="serve the Paraphraser over gRPC")
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=DEFAULT_PORT)
    g = s.add_mutually_exclusive_group(required=True)
    g.add_argument("--pki", help="the harness lab PKI folder (mutual TLS)")
    g.add_argument("--insecure", action="store_true", help="plaintext, loopback only")
    s.add_argument("--server-name", default=None)
    s.add_argument("--allow-client", action="append", default=None)
    s.add_argument("--store", default=None)
    for name in ("paraphrase", "validate", "status"):
        c = sub.add_parser(name)
        c.add_argument("--addr", default=f"127.0.0.1:{DEFAULT_PORT}")
        c.add_argument("--pki", default=None)
        c.add_argument("--insecure", action="store_true")
        c.add_argument("--client-name", default=None)
        if name == "paraphrase":
            c.add_argument("text")
            c.add_argument("--n", type=int, default=3)
            c.add_argument("--seed", type=int, default=0)
            c.add_argument("--backend", default="auto")
            c.add_argument("--canonical", action="store_true")
        if name == "validate":
            c.add_argument("source")
            c.add_argument("candidate")
    args = ap.parse_args(argv)

    if args.cmd == "serve":
        srv = serve(ParaphraserCore(args.store), host=args.host, port=args.port,
                    pki_dir=None if args.insecure else args.pki,
                    server_name=args.server_name, allowed_clients=args.allow_client)
        print(f"paraphraser service on {srv.address} "
              f"({'mutual TLS' if srv.secure else 'plaintext, loopback'}); Ctrl+C stops it")
        try:
            srv.server.wait_for_termination()
        except KeyboardInterrupt:
            srv.stop(1.0)
        return 0
    if not args.insecure and not args.pki:
        ap.error("give --pki (mutual TLS) or --insecure (a loopback server)")
    cl = ParaphraserClient(args.addr, pki_dir=None if args.insecure else args.pki,
                           client_name=args.client_name)
    try:
        if args.cmd == "paraphrase":
            out = cl.paraphrase(args.text, args.n, args.seed, args.backend, args.canonical)
        elif args.cmd == "validate":
            out = cl.validate(args.source, args.candidate)
        else:
            out = cl.status()
    finally:
        cl.close()
    print(json.dumps(out, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
