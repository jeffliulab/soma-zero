# SOMA Zero

**An open embodied-intelligence project: the body side of a robot.** SOMA Zero is about
**embodied strategy** — how a physical robot *perceives* its workspace and *acts* in it with a
learned Vision-Language-Action (VLA) policy. It is **brain-agnostic**: any decision-making
framework can drive this body through a small, neutral contract.

> **Flagship task — physical chess.** A real robot arm plays chess on a real board: the camera
> reads the scene, a brain (whichever one is plugged in) chooses a move, and SOMA carries it
> out — picking up and placing the piece with a VLA policy. Chess is the *first task profile*,
> chosen because it is long-horizon and unforgiving of sloppy manipulation — it is not the
> project's identity.

This is the **Zero** line — fully open source, meant to show the project end to end.
Deeper production work continues in private repos.

---

## The body's promise (to any brain)

SOMA does the *acting* half of the classic **System 1 / System 2** split — fast, reflexive,
high-frequency. Its promise to whatever brain is attached:

- **One atomic action per command** ("move piece e2 → e4"), human-readable, never joint angles.
- **An honest report back** — success / failure / why, including execution self-checks.
- **No thinking on its own**: it does not plan, does not judge task success, does not retry.
  Retry, recovery, and verification always belong to the brain — *whichever* brain that is.

```
                 ┌──────────────── SOMA Zero (this repo) ────────────────┐
                 │                                                        │
   sensors ────▶ │  perception (eyes) ──▶ scene state ─┐                  │
                 │                                     │   neutral        │
                 │                              interface/ contract       │
                 │                                     │                  │
   actuator ◀─── │  control (hands, VLA) ◀── action intent ◀──┐           │
                 │                                            │           │
                 └────────────────────────────────────────────┼───────────┘
                                                              │
                                              adapters/ ──▶ any brain
                                              (protocol translation)
```

The body core never imports any brain framework. Each brain gets a thin **adapter** that
translates the neutral contract into that framework's wire protocol.

## What's inside

| Folder | Role | Status |
|---|---|---|
| [`perception/`](perception/) | Eyes — turn camera images of the workspace into structured scene state | 🚧 early |
| [`control/`](control/) | Hands — VLA policy + arm execution for one atomic action | 🚧 early |
| [`interface/`](interface/) | The neutral brain↔body contract (observation / action intent / result / progress) | 🚧 early |
| [`adapters/`](adapters/) | Per-brain protocol translation (one subfolder per supported brain) | 🚧 docs first |
| [`docs/`](docs/) | Architecture & design notes | 🚧 early |

## Supported brains

| Brain | Protocol | Status |
|---|---|---|
| [`anima-zero`](https://github.com/jeffliulab/anima-zero) | AWI over MCP — SOMA mounts as one of ANIMA's "worlds" | 📝 [integration guide](adapters/anima/) (docs first) |

Any framework that can act as an MCP host — or speak a similarly small tool/observation
protocol — can drive this body the same way. See [`adapters/`](adapters/) for the pattern.

## Hardware

- Episode servo/serial robot arm with a servo gripper
- RGB webcam (no depth) for workspace perception

> ⚠️ **Safety.** This arm has no effective hardware e-stop — cutting power is the only real
> stop, and the joints go limp when power is removed. Every command that moves the physical
> arm is run by a human operator, never autonomously from CI or scripts.

## Before Soma Zero

SOMA Zero grew out of earlier hand-built robot-arm attempts (the *soma-arm* era). Two of the
earliest snapshots are kept under
[`docs/legacy/soma-arm-early/`](docs/legacy/soma-arm-early/) as a record of where this
started; the full history of those attempts also lives in this repo's git log.

## Roadmap

- [ ] Workspace perception: camera → reliable structured scene state (first task profile: chessboard)
- [ ] One-shot extrinsic calibration (board square → world coordinates)
- [ ] VLA pick-and-place: execute a single move as **one** atomic action, returning an honest
      self-report (`ActionResult`)
- [ ] Expose the body as a server so any supported brain can drive it (first adapter:
      [`adapters/anima/`](adapters/anima/)); full task loop — brain perceives → decides →
      SOMA executes one action → repeat

> The closed loop (retry, recovery, verifying an action actually worked) lives in the **brain**,
> not here. SOMA's job is to do one action well and report back honestly.

## License

[MIT](LICENSE) © 2026 Jeff Liu
