"""Minimal escalation FSM for the Phase-1 slice (safety-shield.md).

The full FSM (Brake -> Loiter -> RTL -> Land with N-in-T thresholds and recovery)
lands in Phase 2. The slice supports exactly the transitions the mid-term demo
needs: pass-through when clean, ``Brake`` when a repair succeeds, and a direct
escalation to ``RTL`` / ``Land`` when a violation is unrepairable or the rule's
``violation_action`` demands it.
"""

from __future__ import annotations

from enum import StrEnum


class ShieldState(StrEnum):
    NORMAL = "Normal"
    BRAKE = "Brake"
    RTL = "RTL"
    LAND = "Land"


def next_state(
    current: ShieldState,
    *,
    has_violation: bool,
    repaired: bool,
    direct_action: str | None,
) -> ShieldState:
    """Compute the next FSM state for this monitor tick.

    ``direct_action`` is the violation_action of the unrepairable rule, if any
    (e.g. ``"RTL"`` / ``"land"``); it forces a direct escalation.
    """
    if not has_violation:
        return ShieldState.NORMAL
    if direct_action == "RTL":
        return ShieldState.RTL
    if direct_action == "land":
        return ShieldState.LAND
    if repaired:
        return ShieldState.BRAKE
    # unrepairable, no explicit direct action -> conservative RTL
    return ShieldState.RTL
