# SOMA Zero

**The open body of a chess-playing robot.** SOMA Zero is the *body* half of an embodied
chess system: it **sees the board and moves the pieces**. The *brain* that decides which
move to play lives in a separate repo, [`anima-zero`](https://github.com/jeffliulab/anima-zero).

> **Flagship demo — VLA Chess.** A real robot arm plays physical chess: a camera reads the
> board, ANIMA (the brain) chooses a move, and SOMA (this body) carries it out — picking up
> and placing the piece with a learned Vision-Language-Action (VLA) policy.

This is the **Zero** line — fully open source, meant to show the project end to end.
Deeper production work continues in private repos.

---

## Thinking vs. acting — why two repos

The system is split along the one boundary that actually matters: **thinking vs. acting.**

```
            ┌──────────────────────────┐
   camera ─▶│  SOMA · perception (eyes) │── board state ─┐
            └──────────────────────────┘                │
                                                         ▼
                                            ┌────────────────────────┐
                                            │  ANIMA (brain, anima-  │
                                            │  zero) — decides move  │
                                            └────────────────────────┘
                                                         │ action intent
            ┌──────────────────────────┐                ▼
   arm   ◀──│  SOMA · control (hands)  │◀── "move piece e2→e4" ──────┘
            └──────────────────────────┘
```

- **ANIMA** (brain, *System 2*) — slow, deliberate. Given the board, it decides the best
  move, judges whether the move actually landed, and decides whether to retry. Separate repo.
- **SOMA** (body, *System 1*) — fast, reflexive. It reads the world and, when commanded,
  carries out **one** atomic action in physical space. It does **not** plan, **not** judge
  success, and **not** retry on its own — that is the brain's job.

The two never import each other's code. They talk only through a small, versioned
**contract** (see [`interface/`](interface/)): board state in, action intent out, execution
result back. That seam keeps the brain swappable and the body reusable — and it is exactly
how SOMA plugs into ANIMA as its real-world "world".

## What's inside

| Folder | Role | Status |
|---|---|---|
| [`perception/`](perception/) | Eyes — read the chessboard from an RGB camera into a board state | 🚧 early |
| [`control/`](control/) | Hands — VLA policy + arm execution to move a single piece | 🚧 early |
| [`interface/`](interface/) | The brain↔body contract (board-state in, action intent out, result back) | 🚧 early |
| [`docs/`](docs/) | Architecture & design notes | 🚧 early |

## Hardware

- Episode servo/serial robot arm with a servo gripper
- RGB webcam (no depth) for board perception

> ⚠️ **Safety.** This arm has no effective hardware e-stop — cutting power is the only real
> stop, and the joints go limp when power is removed. Every command that moves the physical
> arm is run by a human operator, never autonomously from CI or scripts.

## Before Soma Zero

SOMA Zero grew out of earlier hand-built robot-arm attempts (the *soma-arm* era). Two of the
earliest snapshots are kept under
[`docs/legacy/soma-arm-early/`](docs/legacy/soma-arm-early/) as a record of where this
started; the full history of those attempts also lives in this repo's git log.

## Roadmap

- [ ] Board perception: camera → reliable board state
- [ ] One-shot board extrinsic calibration (board square → world coordinates)
- [ ] VLA pick-and-place: execute a single move as **one** atomic action, returning an honest
      self-report (`ActionResult`)
- [ ] Plug into ANIMA as a world; full game loop — ANIMA perceives → decides → SOMA executes
      one move → repeat

> The closed loop (retry, recovery, verifying a move actually worked) lives in **ANIMA**, not
> here. SOMA's job is to do one action well and report back honestly.

## License

[MIT](LICENSE) © 2026 Jeff Liu
