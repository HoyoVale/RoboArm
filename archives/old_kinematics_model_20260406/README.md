This archive stores the last simplified kinematics implementation that has now been retired.

Archived files:
- `src/kinematics.py`
- `tools/control/kinematics_probe.py`

Why it was archived:
- It treated the arm as a simplified open-chain model.
- It assumed servo2 directly mapped to the absolute angle of `L3`.
- It assumed servo3 directly mapped to the relative angle between `L4` and `L3`.
- The latest measured mechanism and drawing show a closed-chain transmission:
  - servo2 directly drives `L3`
  - servo3 drives linkage `B1` and changes `L4` through the external linkage
  - servo3 does not change the angle of `L3`
  - `L5` stays horizontal
  - `L6` stays vertical

Authoritative source for the new rebuild:
- `docs/机械臂参数/README.md`
- `docs/机械臂参数/机械臂建模.png`
