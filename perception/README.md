# perception — the eyes

Turns an RGB camera image of the physical board into a **board state** the brain can reason
about (which piece sits on which square).

Sensors observe; they do not think. This package only produces state — all move decisions
happen in ANIMA (`anima-zero`).

**Planned:**
- Board detection + one-shot extrinsic calibration (board square → world coordinates)
- Per-square occupancy / piece classification
- Board state published over the [`interface`](../interface/) contract

🚧 Early — see the [roadmap](../README.md#roadmap).
