"""Minimal JSON-RPC client for the Unreal editor's MCP endpoint.

The session-level MCP client in a coding assistant goes stale whenever the
editor restarts; the endpoint itself does not. This talks to it directly.

    python -m tools.citylife_mcp.ue_rpc script <file.py>   # ProgrammaticToolset
    python -m tools.citylife_mcp.ue_rpc call <toolset> <tool> < args.json

A ProgrammaticToolset script runs inside the editor's sandbox: it may import only
json, math, time, re, datetime and copy, must define run() returning a dict,
and every execute_tool() call inside it lets the game advance a frame - so a
multi-actor "snapshot" read that way is not simultaneous.

TRAP: a call can time out on the client and still have RUN in the editor. A
script that creates actors must therefore be idempotent (look before creating),
or a retry duplicates them - which happened once with the crossing volumes.
"""
import json
import sys
import urllib.request

URL = "http://127.0.0.1:8000/mcp"
HEADERS = {"Content-Type": "application/json",
           "Accept": "application/json, text/event-stream"}
PROGRAMMATIC = "editor_toolset.toolsets.programmatic.ProgrammaticToolset"


def _post(payload, session=None, timeout=900):
    hdrs = dict(HEADERS)
    if session:
        hdrs["Mcp-Session-Id"] = session
    req = urllib.request.Request(URL, json.dumps(payload).encode("utf-8"),
                                 hdrs, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        sid = r.headers.get("Mcp-Session-Id") or session
        raw = r.read().decode("utf-8", "replace")
    body = raw
    if raw.startswith("event:") or "\ndata:" in raw:
        for line in raw.splitlines():
            if line.startswith("data:"):
                body = line[5:].strip()
    try:
        return json.loads(body), sid
    except json.JSONDecodeError:
        return {"raw": raw}, sid


def connect() -> str:
    init = {"jsonrpc": "2.0", "id": 1, "method": "initialize",
            "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                       "clientInfo": {"name": "citylife_mcp", "version": "1"}}}
    _, sid = _post(init)
    _post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    return sid


def call(toolset: str, tool: str, args: dict, sid: str = None) -> dict:
    sid = sid or connect()
    payload = {"jsonrpc": "2.0", "id": 2, "method": "tools/call",
               "params": {"name": "call_tool",
                          "arguments": {"toolset_name": toolset,
                                        "tool_name": tool, "arguments": args}}}
    res, _ = _post(payload, sid)
    return res


def text_of(res: dict) -> str:
    out = res.get("result", res)
    for block in (out.get("content") or []):
        if block.get("type") == "text":
            return block["text"]
    return json.dumps(out)


def run_script(code: str, sid: str = None) -> dict:
    """Run a ProgrammaticToolset script; returns run()'s dict, or raises."""
    res = call(PROGRAMMATIC, "execute_tool_script", {"script": code}, sid)
    txt = text_of(res)
    try:
        outer = json.loads(txt)
    except json.JSONDecodeError:
        raise RuntimeError(txt[:2000])
    if "returnValue" not in outer:
        raise RuntimeError(txt[:2000])
    return json.loads(outer["returnValue"])


def main():
    mode = sys.argv[1]
    if mode == "script":
        code = open(sys.argv[2], encoding="utf-8").read()
        print(json.dumps(run_script(code), indent=1))
    else:
        args = json.load(sys.stdin) if not sys.stdin.isatty() else {}
        print(text_of(call(sys.argv[2], sys.argv[3], args)))


if __name__ == "__main__":
    main()
