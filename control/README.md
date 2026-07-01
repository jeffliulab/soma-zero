# control — the hands

Executes a chosen move in physical space: pick up the piece, place it on the target square,
using a learned Vision-Language-Action (VLA) policy on the robot arm.

**Planned:**
- VLA pick-and-place policy for a single chess move
- Closed-loop retry: verify the piece landed correctly, re-attempt if not
- Servo-gripper handling within safe limits

> ⚠️ **Safety.** Commands here move the real arm, which has no effective e-stop. They are run
> by a human operator, never autonomously from CI. See the [top-level note](../README.md#hardware).

🚧 Early — see the [roadmap](../README.md#roadmap).
