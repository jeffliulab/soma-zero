# Architecture

## A body, with the brain outside the system boundary

SOMA Zero is the **body**: everything between the sensors and the actuators. The brain —
whatever framework decides what to do — sits *outside* this repo's boundary and talks to the
body through a thin adapter.

```
   ┌────────────────────────── SOMA Zero (this repo) ──────────────────────────┐
   │                                                                            │
   │   sensors ──▶ perception ──▶ structured scene state ─┐                     │
   │   (camera)      (eyes)                               │                     │
   │                                              interface/ contract           │
   │                                                       │                    │
   │   actuator ◀── control ◀───── action intent ◀─────────┤                    │
   │   (arm)        (hands, VLA)                           │                    │
   │                                                       │                    │
   └───────────────────────────────────────────────────────┼────────────────────┘
                                                            │
                                             adapters/<brain>  (protocol translation)
                                                            │
                                                   ┌────────▼────────┐
                                                   │   any brain     │
                                                   │ (decides, judges│
                                                   │  retries)       │
                                                   └─────────────────┘
```

## Layering principles

- **The body executes one atomic action per command and reports honestly.** It never plans,
  never judges task success, never retries on its own. Retry, recovery, and verification
  belong to the brain — *any* brain. Execution self-checks (gripper closed? force plausible?)
  are included in the result as early-stop hints, not as verdicts.
- **The body holds physical reality, not task truth.** Perception reports what the camera
  sees; whether that constitutes "the move worked" is the brain's judgement.
- **The body core never imports a brain framework.** All framework-specific code lives in
  [`adapters/`](../adapters/) — one thin translation layer per brain.

## Why an independent repo

- **Different runtimes.** Real-time robot control (high-frequency, hardware-coupled) and a
  cognitive stack (slow, LLM-driven) want different processes, dependencies, and cadences.
- **One body, many brains.** The point of the neutral contract is that the same body can be
  driven by different frameworks; welding the body into any single brain's repo would defeat
  that.
- **Independently showcased.** The Zero line open-sources the body as a project in its own
  right, with embodied strategy (perception + VLA control) as the core contribution.

The cost of the split — keeping the contract in sync — is paid in one small, deliberately
stable place ([`interface/`](../interface/)).

## Status

Early. See the [roadmap](../README.md#roadmap) for what's built vs. planned.
