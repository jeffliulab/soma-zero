# control — the hands

Executes **one atomic manipulation action** in physical space, using a learned
Vision-Language-Action (VLA) policy on the robot arm. First task profile: a single chess
move — pick up the piece, place it on the target square.

Control acts; it does not judge. It performs the commanded action once, then returns an
honest execution result (including self-checks like gripper closure as early-stop hints).
Whether the action *actually worked* — and whether to retry — is always the brain's call.

**Planned:**
- VLA pick-and-place policy for one atomic action (first task profile: a chess move)
- Honest execution self-report (`ActionResult`: success / failure reason / self-checks)
- Servo-gripper handling within safe limits

> ⚠️ **Safety.** Commands here move the real arm, which has no effective e-stop. They are run
> by a human operator, never autonomously from CI. See the [top-level note](../README.md#hardware).

🚧 Early — see the [roadmap](../README.md#roadmap).
