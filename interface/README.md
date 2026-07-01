# interface — the brain↔body contract

ANIMA (brain) and SOMA (body) never import each other's code. They communicate only through
the small, versioned contract defined here. This seam is what keeps the brain swappable and
the body reusable across tasks.

```
SOMA  ──▶  board state      ──▶  ANIMA   (what does the world look like?)
ANIMA ──▶  action intent    ──▶  SOMA    (e.g. "move piece A2 → A4")
SOMA  ──▶  execution result ──▶  ANIMA   (done / failed / current pose)
```

**Planned:**
- Board-state schema (squares, pieces, coordinate frame)
- Action-intent schema (source square → target square, captures, promotions)
- Execution-result schema (success, failure reason, verification)

Keep this contract small and stable — changing it ripples across both repos.

🚧 Early — see the [roadmap](../README.md#roadmap).
