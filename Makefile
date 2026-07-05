.PHONY: sync lint type test check demo schema

sync:
	uv sync --python 3.11

lint:
	uv run ruff check packages demo

# ROS nodes (ros2_ws/, demo/ros_vla_stub.py) need a ROS 2 env and are excluded;
# they are syntax-checked via py_compile in CI instead.
type:
	uv run mypy packages/*/src demo/mid_term_demo.py demo/vla_stub.py

test:
	uv run pytest

check: lint type test

# Mid-term acceptance demo (functional-rail vertical slice, offline)
demo:
	uv run python -m demo.mid_term_demo

# Export the published DSL JSON Schema contract
schema:
	uv run policy-dsl schema -o packages/policy-dsl/policy_dsl.schema.json
