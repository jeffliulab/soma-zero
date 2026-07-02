# perception — the eyes

Turns RGB camera images of the physical workspace into a **structured scene state** a brain
can reason about. First task profile: the chessboard — which piece sits on which square.

Sensors observe; they do not think. This package only produces state — all decisions happen
in whichever brain is attached (see [`adapters/`](../adapters/)).

**Planned:**
- Workspace detection + one-shot extrinsic calibration (board square → world coordinates)
- Per-square occupancy / piece classification (first task profile: chess)
- Scene state published over the [`interface`](../interface/) contract

🚧 Early — see the [roadmap](../README.md#roadmap).
