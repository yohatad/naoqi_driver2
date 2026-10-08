# On-robot test plan: feature/modern-cpp

Delete this file once the branch has passed on Pepper.

Every commit on this branch builds and passes `colcon test`. This plan covers
what only a real robot can show. Run it on the branch tip first. If something
fails, use the bisect steps at the end to find the commit that broke it.

## 1. Safety setup (every session)

- Someone stands within reach of the emergency stop button (back of the neck).
- For base motion: 1 m clear in front of the robot, smooth floor, charging flap closed.
- For everything else: keep the charging flap **open** so the wheels cannot move.

## 2. Prepare the robot

On the robot (`ssh nao@<ip>`):

```bash
qicli call ALMotion.wakeUp                       # head and arms must be stiff
qicli call ALBasicAwareness.pauseAwareness       # stop it turning towards people
qicli call ALMotion.setBreathEnabled Body false  # idle breathing would move the head
```

Undo afterwards with `resumeAwareness` and `setBreathEnabled Body true`.

## 3. Build the branch and enable every sensor

```bash
cd ~/ros2_ws
colcon build --packages-select naoqi_driver
colcon test --packages-select naoqi_driver && colcon test-result --verbose   # optional, off-robot

# Use the full-sensor config for the test, keeping a copy of yours.
CFG=install/naoqi_driver/share/naoqi_driver/share/boot_config.json
cp $CFG /tmp/boot_config.backup.json
cp src/naoqi_driver2/test/robot/boot_config_full.json $CFG
```

Restore your config when done: `cp /tmp/boot_config.backup.json $CFG`. A rebuild
also reinstalls your own config, so copy the full one again after any rebuild.

Launch the driver on its own (not `pepper_bringup`, so gscam2 does not compete
for the front camera):

```bash
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<ip>
```

## 4. Automated checks

In a second terminal, from `~/ros2_ws` with the workspace sourced. Run the
stages in this order; each prints PASS / FAIL / WARN / SKIP and a summary.

| Stage | Command | Robot does |
|---|---|---|
| a. Read-only | `python3 src/naoqi_driver2/test/robot/robot_checks.py` | nothing |
| b. LEDs + speech | `... robot_checks.py --leds --speech` | eyes turn blue, says two sentences (the second is cut off) |
| c. Motion | `... robot_checks.py --motion` (flap closed, area clear) | head turns, base drives about 30 cm forward |
| d. Touch | `... robot_checks.py --interactive` | asks you to press a bumper, a hand and the head |

What stage (a) covers: every sensor stream and its rate, camera calibration
values, TF, the `get_language` / `get_robot_config` services, and the
`cmd_vel_timeout` parameter. Stage (c) checks the `/cmd_vel` watchdog, that
zero and NaN velocities behave, that invalid joint commands are rejected, and
that a `moveto` in the odom frame no longer freezes the driver (skipped if
something publishes odom TF, since that goal would really move the robot).

A WARN on laser or sonar is expected while the robot is at rest.

## 5. Manual checks

| # | Check | How | Pass when |
|---|---|---|---|
| M1 | **Ctrl+C stops the base** | `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`, then Ctrl+C the **driver** (not the publisher) | the base stops; the driver exits cleanly |
| M2 | Ctrl+C stops speech | `ros2 action send_goal /naoqi_driver/speech_with_feedback naoqi_bridge_msgs/action/SpeechWithFeedback "{say: 'one two three four five six seven eight nine ten'}"`, Ctrl+C the driver mid-sentence | speech stops |
| M3 | Watchdog can be disabled | relaunch with `cmd_vel_timeout:=0.0`, publish as in M1, Ctrl+C the **publisher** | base keeps moving (old behaviour); stop it with a zero Twist |
| M4 | Shutdown with NAOqi gone | unplug the robot's network, then Ctrl+C the driver | exits without `terminate called` |
| M5 | NAOqi logs | the full config enables logs; `ros2 topic echo /rosout` | NAOqi log lines appear, no crash |
| M6 | Password login (only if you use one) | launch with `password:=<pw>` | connects |
| M7 | Your normal bringup | restore your config, `ros2 launch naoqi_driver pepper_bringup.launch.py nao_ip:=<ip>` | works as before |
| M8 | Driver logging | watch the driver's console and `ros2 topic echo /rosout` during startup | messages carry levels (INFO/WARN/ERROR), no raw colour codes, the robot banner is one line |

## 6. If something fails: find the commit

The checks map to these commits (oldest first, `git log --oneline master..`):

| # | Commit | Covered by |
|---|---|---|
| 1 | Migrate from Boost to the C++17 standard library | a, b, d, M5, M6 |
| 2 | Make the driver stop the robot when commands stop or it exits | c, M1, M2, M3 |
| 3 | Fix crashes, data races and unchecked NAOqi data | a, b, d, M1, M4, M5, M6 |
| 4 | Add the on-robot test plan, live checks and CI workflow location | none (tools and docs) |
| 5 | Resolve gscam2 lazily in pepper_bringup and disable audio by default | M7 |
| 6 | Reformat C++ sources with the ROS 2 uncrustify style | build, `colcon test` |
| 7 | Clean up the code: logging, idioms, naming, includes, dead code | a, b, c, d, M8 |
| 8 | Restructure registration and replace the type-erasure wrappers | a, b, c, d, M7 |

Commits 6 to 8 are meant to change no behaviour. If a check passes on commit 5
but fails on the tip, the cause is in 6 to 8.

Commits 1 to 3 do not contain the check script or the test config, so copy
them out first. Every build reinstalls your own `boot_config.json`, so copy the
full config again after each build:

```bash
cd ~/ros2_ws
cp src/naoqi_driver2/test/robot/robot_checks.py src/naoqi_driver2/test/robot/boot_config_full.json /tmp/
CFG=install/naoqi_driver/share/naoqi_driver/share/boot_config.json

git -C src/naoqi_driver2 checkout <commit>
colcon build --packages-select naoqi_driver
cp /tmp/boot_config_full.json $CFG
# relaunch the driver, then:
python3 /tmp/robot_checks.py <flags of the stage that failed>
```

On commits before the watchdog, the timeout check reports SKIP, not FAIL.
Return with `git -C src/naoqi_driver2 checkout feature/modern-cpp`.
