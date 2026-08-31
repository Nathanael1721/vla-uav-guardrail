"""One definition of the demo's two-view layout.

Left: what the drone sees, with the detected box and the live telemetry drawn
on it. Right: the third-person chase view. Side by side, because the claim only
reads if both are visible at once - a word chose the box, the box drove the
aircraft, and the guardrail was underneath the whole time.

This lives here rather than in `tools/make_demo_video.py` because there are now
two consumers: that tool, which stitches the recorded frames afterwards, and
`demo/recorder.py --live-view`, which shows the same pair while the aircraft is
still flying. Two open-coded copies of a layout drift, and the drift would be
invisible until someone compared a live screenshot against the delivered video.

Deliberately takes arrays, not paths. The recorder already holds decoded frames
and must not re-read them from disk to draw a window.
"""
from __future__ import annotations

from typing import Optional


def side_by_side(left, right, height: Optional[int] = None):
    """Stack two BGR arrays horizontally, matching their heights.

    Either may be None - a camera that returned nothing yields the other panel
    alone rather than no frame at all. Returns None only when both are missing.

    `height` scales both panels before stacking; None keeps them as they are.
    """
    import cv2
    import numpy as np

    def fit(im):
        if im is None or height is None:
            return im
        s = height / im.shape[0]
        return cv2.resize(im, (max(1, int(im.shape[1] * s)), height))

    a, b = fit(left), fit(right)
    if a is None and b is None:
        return None
    if a is None:
        return b
    if b is None:
        return a
    if a.shape[0] != b.shape[0]:
        # Match the LEFT panel's height: it carries the detector overlay, and
        # rescaling it would resample the box the whole demo is about.
        s = a.shape[0] / b.shape[0]
        b = cv2.resize(b, (max(1, int(b.shape[1] * s)), a.shape[0]))
    return np.hstack([a, b])


def pil_to_bgr(im):
    """PIL RGB -> OpenCV BGR, or None."""
    import cv2
    import numpy as np
    if im is None:
        return None
    return cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
