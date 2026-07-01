# SOMA Zero

**The open body of an embodied chess-playing robot.** SOMA Zero is the *body* half of a
two-part system: it sees the board and moves the pieces. The *brain* that decides which
move to play lives in a separate repo, [`anima-zero`](https://github.com/jeffliulab).

> **Flagship demo — VLA Chess.** A real robot arm plays physical chess: a camera reads the
> board, ANIMA (the brain) chooses a move, and SOMA (this repo) executes it by picking up
> and placing the piece with a learned Vision-Language-Action (VLA) policy.

This is the **Zero** release line — fully open source, meant to show the project end to end.
Deeper production work continues in private repos.

---

## Why two repos (ANIMA + SOMA)

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
   arm   ◀──│  SOMA · control (hands)  │◀── "move piece A2→A4" ──────┘
            └──────────────────────────┘
```

- **ANIMA** = cognition. Given the board, what is the best move? (separate repo)
- **SOMA** = embodiment. Read the world; carry out the chosen action in physical space.

The two never import each other's code — they talk only through a small, versioned
**contract** (see [`interface/`](interface/)). That contract is the seam that keeps the brain
swappable and the body reusable.

## What's inside

| Folder | Role | Status |
|---|---|---|
| [`perception/`](perception/) | Eyes — read the chessboard from an RGB camera into a board state | 🚧 early |
| [`control/`](control/) | Hands — VLA policy + arm execution to move a piece | 🚧 early |
| [`interface/`](interface/) | The brain↔body contract (board-state in, action commands out) | 🚧 early |
| [`docs/`](docs/) | Architecture & design notes | 🚧 early |

## Hardware

- Servo/serial robot arm (Episode) with a servo gripper
- RGB webcam (no depth) for board perception

> ⚠️ **Safety.** This arm has no effective hardware e-stop — cutting power is the only real
> stop, and joints go limp when power is removed. All commands that move the physical arm
> are run by a human operator, never autonomously from CI or scripts.

## Roadmap

- [ ] Board perception: camera → reliable board state
- [ ] One-shot board extrinsic calibration (square → world coordinates)
- [ ] VLA pick-and-place policy for a single move
- [ ] Closed-loop retry until the move is verified (reach high single-move success)
- [ ] Full game loop: perceive → ANIMA decides → SOMA executes → repeat

## License

[MIT](LICENSE) © 2026 Jeff Liu
