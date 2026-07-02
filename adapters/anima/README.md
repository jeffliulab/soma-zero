# anima adapter — mounting SOMA as an ANIMA "world"

[ANIMA](https://github.com/jeffliulab/anima-zero) is an MCP-host-style brain: to it, every
body/environment is a **"world"** — a standard MCP server mounted at `/mcp`, discovered by
URL. Plugging SOMA in requires **zero code changes on ANIMA's side**: start the SOMA world
server, add one entry to ANIMA's `ANIMA_WORLDS` list, done.

> **Status: docs first.** perception/ and control/ have no runnable behavior yet, so there is
> deliberately no server skeleton here — a server whose tools do nothing would fake the
> pipeline. This document pins down the protocol so the server can land here the moment the
> body has something real to execute.

## What ANIMA expects from a world (AWI over MCP)

| MCP primitive | Endpoint | Meaning |
|---|---|---|
| Tools | `tools/list` / `tools/call` | High-level actions in human-readable terms (e.g. `move`), never joint angles. Each tool declares `kind ∈ {tool, read, judge}`; `read`/`judge` get `readOnlyHint` so ANIMA's safety gate waves them through |
| Resource | `resources/read "anima://observation"` | One observation per read: structured state JSON + a PNG frame |
| Prompt | `prompts/get "guidance"` | The world's self-description, injected into the brain's system prompt |
| Resource (optional) | `resources/read "anima://services"` | Advisory services this world brings along, `[{"name","url"}]` (e.g. a chess engine); ANIMA auto-connects them after the handshake |

**Out-of-band HTTP (never inside MCP — ANIMA red line):**

| Endpoint | Purpose |
|---|---|
| `GET /health` | Liveness probe (not counted as AWI traffic) |
| `GET /stream` | MJPEG live view for the web UI (MCP is JSON-RPC text; video stays out) |
| `GET /status` | God's-eye ground truth for humans/debugging — must never feed perception |

## How the server will be built

1. **Copy the protocol adapter** `awi_mcp.py` (~166 lines, self-contained, depends only on
   `mcp` + `anyio`) from the anima-zero repo — any of its world folders carries an identical
   copy (e.g. `world/sim-desk/awi_mcp.py`).

   > ⚠️ **Sync caveat:** anima-zero's `tests/test_awi_mcp_copies.py` byte-checks only the
   > copies *inside that repo* — the copy living here is **not** guarded. When ANIMA
   > upgrades the protocol, this copy must be re-synced by hand (or given its own check).

2. **Implement the world object** with three methods — `capabilities()`, an observe method
   returning `(state_dict, png_bytes)`, and `invoke(name, *, _progress=None, **args)` — backed
   by SOMA's real [`perception/`](../../perception/) and [`control/`](../../control/).

3. **Wire it up**: `build_awi_mcp(world, guidance=..., server_name="soma")` and mount the
   returned ASGI app at `/mcp` in a small FastAPI server.

4. **Long actions**: a physical pick-and-place takes tens of seconds. Declaring the
   keyword-only `_progress` parameter lets the adapter report
   `_progress(0.5, "piece grasped, moving to e4")` — ANIMA keeps the action alive as long as
   progress arrives and only declares it dead on silence.

## Connecting from ANIMA

Ports already taken by ANIMA's stock worlds/services: `8100` (sim-desk), `8102` (sim-chess),
`8104` (camera), `8106` (gazebo-chess), `8108` (chess-engine service). Suggested port for the
SOMA world: **`8112`** (configurable, of course — never hardcode it).

```bash
# on the ANIMA side — APPEND to the existing list, never replace it
ANIMA_WORLDS="soma=http://localhost:8112,sim-desk=http://localhost:8100,..." <start ANIMA>
```

## Reference implementations (in the anima-zero repo)

- `world/sim-desk/` — the minimal template: `server.py` (~120 lines) + `world.py` (~110 lines)
  + the `awi_mcp.py` copy.
- `world/gazebo-chess/` — the closest analogue to a real arm: ROS2 with a single dedicated
  spin thread (never spin from request threads), action clients for the arm, `_progress`
  reporting during physical moves.
