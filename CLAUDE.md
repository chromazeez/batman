# 393B — VEX V5 robot code

PROS 4 + LemLib + liblvgl. Team 393B, Batman theme.

## Build

The toolchain is not on PATH by default in a fresh shell. Use:

```powershell
$env:PROS_TOOLCHAIN = "C:\Users\azizr3780\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr"
$env:PATH = "$env:PROS_TOOLCHAIN\bin;$env:PATH"
& "C:\Users\azizr3780\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-cli-windows\pros.exe" build
```

`compile_commands.json` has previously gone stale pointing at a different copy of this project
(`C:\Users\azizr3780\Documents\override\batman`). If clangd reports phantom errors the real build
does not, check the `directory` field. `pros build-compile-commands` appends rather than replaces.

## Layout

- `src/main.cpp` — devices, chassis, `initialize/autonomous/opcontrol`, toggle sensing, failsafe
- `src/autons.cpp` — auton building blocks, routes, the selector slot list
- `src/screen.cpp` — the whole brain screen (LVGL, not LLEMU)
- `include/robot.h` — everything shared between those three

## Hardware notes

- Drivetrain: `leftMotors {19,17,6}`, `rightMotors {-20,-18,-3}`. The middle motor of each side is a
  green cartridge, fixed up in `initialize()` because a MotorGroup takes one gearset.
- The robot's front is the old back. **Whether LemLib's left/right match the physical sides is
  unverified** — turns had never been properly tested as of the last session.
- Cascade: two green motors, position in motor degrees from power-on, so it must start fully down.
  `CASCADE_SCORE` (900) may be above the real hard stop; the stall check is what ends the move.
- Optical sensor reads toggle color by hue. Red 353, blue 195, yellow 51, tolerance 25.
- The horizontal odom pod sits **exactly at the tracking center**, confirmed in CAD, so its offset
  of `0` is correct. Do not "fix" it. X/Y moving during a pure turn is the robot really sliding on
  its omnis, and the pod reporting it correctly.

## Tuning log

Angular, tuned 2026-11-10 on a 90 degree turn. Final: `angularController(7, 0, 45, ...)`.

| kP | kD | lands at | settle |
|----|----|----------|--------|
| 1.7 | 12 | 84.82 | stalled short, ran to timeout |
| 5 | 12 | 91.22 | heavy rocking |
| 5 | 25 | 90.01 | heavy rocking |
| 5 | 45 | 87.49 | calm but over-damped, stalls short |
| 7 | 45 | 90.83 | slight oscillation, accepted |
| 7 | 50 | 90.5 | oscillated *more*, so 45 is the ceiling |

Past kD 45 the oscillation grows instead of shrinking: kD acts on rate of change and the IMU is
noisy, so more kD amplifies that noise. The small residual wobble is sensor noise, not tuning.

The last of the wobble went away by loosening the exit conditions from `1, 100, 3, 500` to
`3, 90, 7, 250`, ported from EZ-Template's turn defaults. 1 degree is below what the IMU resolves,
so the turn was hunting for a target it could not see. Final: lands 90.91 on a 90, clean stop.

Lateral needed no tuning. `linearController(7.5, 0, 10, 3, 1, 100, 3, 500, 20)` drove 24.25" on a
24" target with no overshoot, which also confirms the 2.75" drive wheel setting.

`moveToPoint` does not control final heading, only position, so ending a straight drive a degree or
two off is expected rather than a bug. Use `moveToPose` if a route ever needs a specific heading on
arrival. The current route turns after every straight, so it does not.

**Unresolved:** a single 90 degree turn reports roughly 2.7" of X and 4.9" of Y displacement.
There is no vertical tracking wheel (`OdomSensors` has nullptr for both), so forward position comes
from the drive encoders, which scrub during a pivot. The X is probably real drift on the omnis; the
Y is suspect. Not yet checked against a tape measure.

The original kP of 1.7 gave only 8.8 motor power at 5 degrees of error, which cannot break static
friction, so the turn stalled short and sat there until the timeout. Raising kP fixed the stall;
raising kD fixed the rocking that caused; raising kP again fixed the stall that kD caused.

## Gotchas that have already bitten

- **LemLib `async` defaults to `true`.** Every motion returns immediately unless you pass
  `{}, false`. This has caused real bugs three times.
- **`timeout` is a give-up cap, not a wait.** A motion that always takes exactly its timeout never
  met an exit condition — that's a bug, not tuning.
- **`motor.move(0)` sets volts to zero and coasts.** Use `motor.brake()` to apply the brake mode.
- **Never `tare_position()` the cascade in `opcontrol()`** — auton can hand over with it raised,
  which would drag `CASCADE_TOP` / `CASCADE_BOTTOM` with it.
- LLEMU is deliberately unused. The screen is raw LVGL so the logo can be in color. All screen
  updates run inside an `lv_timer`, i.e. LVGL's own task — nothing outside `screen.cpp` touches an
  LVGL object.

## Controls

Left stick drive, right stick turn. R1/R2 claw. L1/L2 cascade up/down (hold). L1 tap sends the
cascade to `CASCADE_SCORE`. Up/down arrows spin the toggle by hand. A latches the optical sensor
out (failsafe). B+DOWN runs the selected auton, practice only, dead when a comp switch is connected.

## Style the user wants

- Comments: lowercase fragments, no trailing periods, no full sentences. End-of-line where it fits;
  block comments only when an idea genuinely needs several lines.
- No clever code. Basic and explainable beats elegant — the user is beginner-to-intermediate C++ and
  has to explain all of this to competition judges.
- Route bodies stay bare, no per-line comments.
- Route step values (distances, timeouts) are inlined, not hoisted into named constants. Constants
  are only for values shared across files.
