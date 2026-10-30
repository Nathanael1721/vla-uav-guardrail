"""AIOT-VLA: the lab's own small vision-language-action model for the follow.

A frozen open-vocabulary vision-language backbone (OWL-ViT B/32, the same
weights the flight's detector already loads) turns the front frame and the
instruction's object words into a grounding map ("where in this frame is
'a red car'") and a compact appearance map; a small trained head reads a
short history of those maps plus the aircraft's own motion and answers the
next step (forward, down, heading change over the label horizon). No
detector box, no hand-written servo and no compass hint reach the head: the
language reaches the action only through the backbone's grounding.

    core.py         stdlib + numpy: history offsets, proprio, label grids,
                    soft targets, decoding, mirroring (train and serve share it)
    backbone.py     OWL-ViT wrapper (torch, transformers): grounding + PCA maps
    model.py        the trained head (torch)
    build_cache.py  backbone features for every capture of a recording
    train.py        behaviour cloning on the follow expert's recordings
    serve           demo/vla_server_foreign.py --model aiotvla

Design notes, numbers and limits: docs/DESIGN-aiotvla.md.
"""
VERSION = "0.1.0"
