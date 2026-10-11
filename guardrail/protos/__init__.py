"""Generated gRPC stubs for the stress harness control plane.

harness_pb2.py and harness_pb2_grpc.py are GENERATED from harness.proto and
committed as generated files; do not edit them. Regenerate from the repository
root after changing the .proto:

    python -m grpc_tools.protoc -I . --python_out=. --grpc_python_out=. \\
        guardrail/protos/harness.proto

tests/test_harness_service.py regenerates them into a temporary folder and
fails if the committed files differ. Nothing imports this package unless the
gRPC harness is used (guardrail/harness_service.py imports it lazily), so the
Shield still imports where grpcio and protobuf are absent.
"""
