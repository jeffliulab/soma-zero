# interface — the neutral brain↔body contract

The body and whatever brain drives it never import each other's code. They communicate only
through the small, versioned, **framework-neutral** contract defined here. This seam is what
keeps the brain swappable and the body reusable across tasks.

Four data flows:

```
body  ──▶  observation       ──▶  brain   (structured scene state + camera frame)
brain ──▶  action intent     ──▶  body    (one atomic action, e.g. "move piece a2 → a4")
body  ──▶  execution result  ──▶  brain   (done / failed / why / self-checks)
body  ──▶  progress events   ──▶  brain   (heartbeat during long physical actions)
```

**Planned schemas:**
- Observation schema — structured scene state + image frame; first task profile (chess):
  squares, pieces, coordinate frame
- Action-intent schema — one atomic action in human-readable terms (source → target,
  captures, promotions); never joint angles
- Execution-result schema — success, failure reason, execution self-checks (early-stop
  hints, not verdicts — judging success is the brain's job)
- Progress-event convention — fraction done + human-readable note, so the brain can tell
  "slow" from "dead" during actions that take tens of seconds

## Contract → protocol mapping

The contract itself names *what* crosses the seam. *How* it crosses is each adapter's job
(see [`adapters/`](../adapters/)). For the first supported brain (ANIMA, AWI over MCP):

| Contract flow | ANIMA wire protocol |
|---|---|
| observation | MCP `resources/read "anima://observation"` (state JSON + PNG) |
| action intent | MCP `tools/call` |
| execution result | MCP tool result (`ok` / `message` / `data`) |
| progress events | MCP progress notifications |

Keep this contract small and stable — changing it ripples across the body and every adapter.

🚧 Early — see the [roadmap](../README.md#roadmap).
