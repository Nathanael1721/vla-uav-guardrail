# The companion image plus the Policy DSL's REST hot-apply server (guardrail/api.py, the
# pyproject `api` extra: fastapi + uvicorn), which the stress harness drives on a hil
# episode (WP4-03 / WP4-05 / WP4-11, tools/harness_hil_episode.py). It is a layer on the
# companion image already built on the Orin, because the Orin has no internet route: the
# wheels come from a named build context (a wheelhouse made on the desktop with
# `pip download`, docs/RUNBOOK-orin-hil.md "REST hot-apply in the companion"), installed
# with --no-index and pinned by constraints-companion-api.txt, the versions WSL
# ~/venv-ros ran its REST hot-apply flights with (docs/DESIGN-python-versions.md).
#
# No `# syntax=` line: that would pull a frontend image, and the Orin cannot; the built-in
# BuildKit frontend reads RUN --mount and named contexts.
#
#   docker build --build-context wheels="$HOME/wheelhouse-api" \
#       -f deploy/docker/companion-api.Dockerfile -t vlaguard/companion:jazzy-api .
ARG COMPANION_IMAGE=vlaguard/companion:jazzy
FROM ${COMPANION_IMAGE}

COPY deploy/docker/constraints-companion-api.txt /tmp/req/
RUN --mount=type=bind,from=wheels,target=/tmp/wheels \
    /opt/venv/bin/pip install --no-cache-dir --no-index --find-links /tmp/wheels \
      -c /tmp/req/constraints-companion-api.txt fastapi uvicorn \
 && /opt/venv/bin/python -c 'import sys, fastapi, uvicorn, pydantic; print("fastapi", fastapi.__version__, "uvicorn", uvicorn.__version__, "pydantic", pydantic.VERSION)'
