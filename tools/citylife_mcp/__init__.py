"""Scripts that build the CityLife level through the editor's MCP endpoint.

`PASBlocks/` is gitignored, so the level itself is not in this repository.
These scripts are the recipe: each one talks to a running Unreal editor over
the MCP JSON-RPC endpoint (see `ue_rpc.py`) and makes one kind of change.
"""
