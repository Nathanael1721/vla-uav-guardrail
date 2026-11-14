"""The REST hot-apply listener on the perception rail's own Shield (WP1-15).

demo/pas_ardupilot/perception_node.py serves guardrail.api.FlightHotApply on
the Shield it flies, as sitl/ros2_shield_node.py and sitl/run_sitl_demo.py
serve it on theirs (since 2026-11-15; until then the perception rail took no
mid-flight event at all). What is the same as on the ROS 2 rail:

  * the five flags (`add_arguments`): --hotapply-port, --hotapply-host
    (default 127.0.0.1), --hotapply-pki, --hotapply-server-cert,
    --hotapply-client (repeat), with the ROS 2 node's names and defaults
    (tests/test_pas_hotapply.py compares the two parsers);
  * the transport rules, which are guardrail.api.serve_in_background's own:
    plaintext on a loopback address only; any other address needs
    --hotapply-pki, mutual TLS with the harness CA
    (`python -m guardrail.harness_service gen-pki`), a client certificate
    signed by it and its common name on the allow-list (default
    harness-client);
  * the routes and bodies (guardrail/api.py: POST /dynamic_nfz, PATCH and
    DELETE /dynamic_nfz/{id}, POST /time_window_switch, /corridor_swap,
    /hot_apply, the legacy /nfz; GET /health, /policy, /events);
  * the record: each accepted event is a generation, written as
    policy_g<N>.json and csp_g<N>.json (EpisodeRecord.hot_applied), and a
    `hot_apply_rest` event with its receipt-to-swap time; the listener's own
    `hot_apply_listener` event; metrics.json `hot_apply` is
    FlightHotApply.summary(); a dynamic_nfz spawned in flight counts in the
    zone dwell from its arrival (FlightHotApply.live_from);
  * the listener starts before anything flies, so one that cannot start
    (FastAPI or uvicorn missing, the port taken, a non-loopback address
    without the PKI, a PKI file missing) stops the run before arming; it is
    closed before the episode's end is written, and later requests get 503.

THE LOCK. The perception node is one thread: the hover's checks and the
mission loop call the Shield from it. The listener is a second thread, which
swaps the Shield's rules (Shield.hot_apply) under `LoopLock.lock`. From the
hover before the mission to the episode's end the node's thread HOLDS that
lock and lets go of it only while it sleeps (`LoopLock.sleeper` wraps the
`sleep` every wait and the loop's pacing use), so an event lands between two
ticks - never between a decision and its row, nor between a hover check and
its record - as on the pymavlink rail. A sleep shorter than LOCK_YIELD_S (an
overrunning loop sleeps 0) is followed by a wall-clock yield of
LOCK_YIELD_S, because the lock is not fair: without it the node's thread
would take the lock straight back and a waiting request could wait for as
long as the overrun lasts. Before the hover (the scene, the detector, SITL,
the take-off) nothing on the node's thread uses the Shield and the lock is
not held: an event is applied as it arrives, as on the ROS 2 rail, and one
that arrives before the mission clock changes the mission-start policy.

THE APPLY PATH. Shield.hot_apply swaps the policy in place (Shield._commit
writes the new rules into the policy object every holder shares: the episode
record, the audit log, the rows' policy_hash / generation, FenceBackstop), so
the rows after an event carry the new generation with nothing else done. The
one thing the ROS 2 node does after an event that this rail must do too is
re-publish the keep-out zones on the autopilot's GeoFence. The listener's
thread never touches the MAVLink link (it is the node's thread's), so
FenceBackstop.recheck, which the hover and the loop call every tick, uploads
the zones again on the first tick whose policy generation differs from the
last upload's, holding back a zone the aircraft is near as for every upload.
"""
from __future__ import annotations

import threading
import time
from typing import Callable

# As sitl/ros2_shield_node.py's DEFAULT_HOTAPPLY_HOST.
DEFAULT_HOTAPPLY_HOST = "127.0.0.1"
# The shortest the node's thread lets go of the lock in one sleep, in wall
# seconds (sitl/run_sitl_demo.py's LOCK_YIELD_S).
LOCK_YIELD_S = 0.001


def add_arguments(ap) -> None:
    """The listener's five flags, as the ROS 2 Shield node defines them
    (sitl/ros2_shield_node.py make_parser)."""
    ap.add_argument("--hotapply-port", type=int, default=None,
                    help="serve the REST hot-apply API (guardrail/api.py) on "
                         "this port, started before arming")
    ap.add_argument("--hotapply-host", default=DEFAULT_HOTAPPLY_HOST,
                    help="address for --hotapply-port (default 127.0.0.1; any "
                         "other needs --hotapply-pki)")
    ap.add_argument("--hotapply-pki", default=None, metavar="DIR",
                    help="mutual TLS with the harness CA: a folder written by "
                         "python -m guardrail.harness_service gen-pki")
    ap.add_argument("--hotapply-server-cert", default=None, metavar="NAME",
                    help="the server certificate's name in --hotapply-pki "
                         "(default harness-server)")
    ap.add_argument("--hotapply-client", action="append", default=None,
                    metavar="CN",
                    help="client certificate common name allowed in (repeat; "
                         "default harness-client)")


class LoopLock:
    """The lock the listener swaps the Shield's rules under, held by the
    node's thread from `hold()` to `let_go()` except inside the sleeps that
    `sleeper(sleep)` wraps. Only the node's thread calls hold, let_go and the
    wrapped sleep; the listener takes `lock` (an RLock: FlightHotApply.close,
    called on the node's thread while it holds it, takes it again)."""

    def __init__(self, yield_s: float = LOCK_YIELD_S,
                 wall_sleep: Callable[[float], None] = time.sleep) -> None:
        self.lock = threading.RLock()
        self.yield_s = float(yield_s)
        self._wall_sleep = wall_sleep
        self.held = False
        # Sleeps the node's thread let go of the lock in, and how many of
        # them were the wall-clock yield after a sleep shorter than yield_s.
        self.n_sleeps = 0
        self.n_yields = 0

    def hold(self) -> None:
        if not self.held:
            self.lock.acquire()
            self.held = True

    def let_go(self) -> None:
        if self.held:
            self.held = False
            self.lock.release()

    def sleeper(self, sleep: Callable[[float], None]) -> Callable[[float], None]:
        """`sleep`, letting go of the lock for its duration while it is held.
        The pacing is untouched: `sleep` gets exactly the seconds asked for,
        and the yield after a short one is on the wall clock."""
        def sleep_unlocked(dt):
            if not self.held:
                return sleep(dt)
            self.lock.release()
            try:
                self.n_sleeps += 1
                sleep(dt)
                if not dt or dt < self.yield_s:
                    self.n_yields += 1
                    self._wall_sleep(self.yield_s)
            finally:
                self.lock.acquire()
        sleep_unlocked.loop_lock = self
        return sleep_unlocked


def start(args, shield, record, mission_t: Callable[[], float | None], *,
          log: Callable[[str], None] = print):
    """(FlightHotApply, LoopLock) serving the REST hot-apply API on `shield`,
    or (None, None) without --hotapply-port. Raises when the listener cannot
    start, before anything flies (guardrail.api.serve_in_background: a
    plaintext listener off loopback, a missing PKI file, FastAPI / uvicorn not
    installed, the port taken). `mission_t()` is the run's mission time, None
    before the mission clock."""
    port = getattr(args, "hotapply_port", None)
    if not port:
        return None, None
    from guardrail.api import FlightHotApply
    gate = LoopLock()
    hot = FlightHotApply(shield, record, gate.lock, mission_t=mission_t,
                         log_fn=lambda m: log(f"[hotapply] {m}"))
    th = hot.start(getattr(args, "hotapply_host", None) or DEFAULT_HOTAPPLY_HOST,
                   int(port), pki_dir=getattr(args, "hotapply_pki", None),
                   server_name=getattr(args, "hotapply_server_cert", None),
                   allowed_clients=getattr(args, "hotapply_client", None))
    ep = hot.endpoint or {}
    log(f"[hotapply] REST hot-apply API at {th.url}"
        + (f" (mutual TLS, clients {ep.get('allowed_clients')})" if ep.get("mutual_tls")
           else " (plaintext, loopback only)"))
    return hot, gate


def stop(hot, gate, why: str, *, log: Callable[[str], None] = print) -> None:
    """No more events (later ones would land after the episode's records:
    503 with `why`), the server told to stop, and the node's thread lets go of
    the lock so a request still waiting on it gets its 503. Idempotent; never
    raises (it runs on the way out of a flight)."""
    if hot is not None:
        try:
            hot.close(why)
        except Exception as exc:                                  # noqa: BLE001
            log(f"[hotapply] *** close failed: {type(exc).__name__}: {exc}")
    if gate is not None:
        try:
            gate.let_go()
        except Exception as exc:                                  # noqa: BLE001
            log(f"[hotapply] *** lock release failed: {type(exc).__name__}: {exc}")


def live_from(hot) -> dict | None:
    """The zones spawned over REST and their mission time, for
    guardrail.replay.zone_dwell(live_from=...); None without a listener or
    with no such zone."""
    if hot is None:
        return None
    return dict(hot.live_from) or None


def report_line(summary: dict | None) -> str:
    """One console line for the run's report."""
    if summary is None:
        return "hot-apply API: not served (--hotapply-port not given)"
    ep = summary.get("endpoint") or {}
    return (f"hot-apply API: {summary.get('n_applied')} event(s) at {ep.get('url')}"
            + (" (mutual TLS)" if ep.get("mutual_tls") else "")
            + (f", receipt to swap max {summary.get('receipt_to_swap_ms_max')} ms "
               f"(spawn target {summary.get('spawn_target_ms'):g} ms, "
               f"{summary.get('spawns_over_target')} over)"
               if summary.get("n_applied") else "")
            + (f"; *** {summary.get('n_record_failed')} generation record(s) failed"
               if summary.get("n_record_failed") else ""))
