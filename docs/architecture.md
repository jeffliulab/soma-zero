# Architecture

## The one boundary that matters: thinking vs. acting

The system is split into two repos along the seam between cognition and embodiment:

- **ANIMA** (`anima-zero`) — the brain. Decides *what* to do.
- **SOMA** (`soma-zero`, this repo) — the body. Senses the world and *does* it.

They never share code. They share only a versioned contract (`interface/`). This means:

- The brain can be replaced (a different engine, a stronger model) without touching the body.
- The body can be reused for a different task by swapping the brain and the contract's verbs.
- Either side can be tested in isolation against a mock of the other.

## Data flow (VLA chess)

```
   ┌─────────────────────────── SOMA (this repo) ───────────────────────────┐
   │                                                                          │
   │   camera ─▶ perception ──(board state)──┐                                │
   │                                          │                               │
   └──────────────────────────────────────────┼───────────────────────────────┘
                                              ▼
                                  ┌────────────────────────┐
                                  │   ANIMA (anima-zero)   │
                                  │   chooses a move       │
                                  └────────────────────────┘
                                              │ action intent
   ┌──────────────────────────────────────────┼───────────────────────────────┐
   │                                          ▼                                │
   │   arm ◀── control ◀──(verify + retry)──(action intent)                    │
   │                                                                          │
   └─────────────────────────── SOMA (this repo) ───────────────────────────┘
```

## Why split into separate repos (and not a monorepo)

ANIMA and SOMA have **different runtimes** (cognitive stack vs. real-time robot control) and
this Zero line is meant to be **independently open-sourced and showcased**. As long as the
`interface/` contract stays stable, the two evolve on their own cadence. The cost of the split
— keeping the contract in sync — is paid in one small, deliberately stable place.

## Status

Early. See the [roadmap](../README.md#roadmap) for what's built vs. planned.
