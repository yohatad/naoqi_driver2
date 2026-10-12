# On-robot test plan: feature/modern-cpp

Delete this file once the branch has passed on Pepper.

Every commit on this branch builds and passes `colcon test`. This plan covers
what only a real robot can show. Run it on the branch tip first. If something
fails, use the bisect steps at the end to find the commit that broke it.

Run it on native Ubuntu, not WSL. With WSL's default (NAT) networking the robot
cannot connect back to the computer, so audio fails, and other machines on the
network do not see the topics.

## 1. Safety setup (every session)

- Someone stands within reach of the emergency stop button (back of the neck).
- For base motion: 1 m clear in front of the robot, smooth floor, charging flap closed.
- For everything else: keep the charging flap **open** so the wheels cannot move.

## 2. Prepare the robot

On the robot (`ssh nao@<ip>`):

```bash
qicli call ALAutonomousLife.setState disabled                   # no autonomous movement, speech or apps; the robot goes to rest
qicli call ALMotion.wakeUp                                      # stiffen it again (after disabling, not before)
qicli call ALMotion.getExternalCollisionProtectionEnabled Move  # must print true
```

Autonomous Life would otherwise turn the head towards people, gesture, or start
an app during the checks. The base's collision protection belongs to ALMotion,
so it stays on; the last command confirms it. Undo afterwards with
`qicli call ALAutonomousLife.setState solitary` (a reboot also restores it), and
do so before M7, which checks your usual setup.

## 3. Build the branch and enable every sensor

Get the branch (it needs the `main` branch of your `naoqi_bridge_msgs2` fork too):

```bash
cd ~/ros2_ws/src
git clone https://github.com/yohatad/naoqi_driver2.git   # or, in an existing checkout:
cd naoqi_driver2 && git fetch origin && git switch feature/modern-cpp
```

Build it, and optionally run the unit tests:

```bash
cd ~/ros2_ws
colcon build --packages-select naoqi_driver
# optional, off-robot; --test-result-base keeps other packages' old results out
colcon test --packages-select naoqi_driver
colcon test-result --test-result-base build/naoqi_driver --verbose
```

Launch the driver on its own (not `pepper_bringup`, so gscam2 does not compete
for the front camera), with every sensor on. `full_sensors.yaml` overrides your
`boot_config.json` for this launch only, without changing it:

```bash
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<ip> \
  params_file:=$HOME/ros2_ws/src/naoqi_driver2/test/robot/full_sensors.yaml
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
| e. Recording | `... robot_checks.py --record` | nothing; writes a 3 s bag where the driver was started |

What stage (a) covers: every sensor stream and its rate, camera calibration
values, TF, the `get_language` / `get_robot_config` services, the recording
services, and the parameters this branch added (`cmd_vel_timeout`,
`moveto_max_distance`, the speed caps, `listen_timeout`, `minidump_duration`,
`converters.*`). On older commits the newer ones report WARN, not FAIL. Stage (c) checks the `/cmd_vel` watchdog, that
zero and NaN velocities behave, that invalid joint commands are rejected, and
that a `moveto` in the odom frame no longer freezes the driver (skipped if
something publishes odom TF, since that goal would really move the robot).

A WARN on laser or sonar is expected while the robot is at rest.

## 5. Manual checks

| # | Check | How | Pass when |
|---|---|---|---|
| M1 | **Ctrl+C stops the base** | `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`, then Ctrl+C the **driver** (not the publisher) | the base stops; the driver exits cleanly |
| M2 | Ctrl+C and cancel stop speech | `ros2 action send_goal /naoqi_driver/speech_with_feedback naoqi_bridge_msgs/action/SpeechWithFeedback "{say: 'one two three four five six seven eight nine ten'}"`, Ctrl+C the driver mid-sentence; then again, but Ctrl+C the `send_goal` command instead (that cancels the goal) | speech stops |
| M3 | Watchdog can be disabled | relaunch with `cmd_vel_timeout:=0.0`, publish as in M1, Ctrl+C the **publisher** | base keeps moving (old behaviour); stop it with a zero Twist |
| M4 | Shutdown with NAOqi gone | unplug the robot's network, then Ctrl+C the driver | exits without `terminate called` |
| M5 | NAOqi logs | the full config enables logs; `ros2 topic echo /rosout` | NAOqi log lines appear, no crash |
| M6 | Password login (only if you use one) | launch with `password:=<pw>` | connects |
| M7 | Your normal bringup | restore Autonomous Life (`setState solitary`), `ros2 launch naoqi_driver pepper_bringup.launch.py nao_ip:=<ip>` (no `params_file`) | works as before |
| M8 | Driver logging | watch the driver's console and `ros2 topic echo /rosout` during startup | messages carry levels (INFO/WARN/ERROR), no raw colour codes, the robot banner is one line |
| M9 | `moveto` distance cap | flap closed, area clear: `ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: base_footprint}, pose: {position: {x: 5.0}, orientation: {w: 1.0}}}"`, then the same with `x: 0.3` | the 5 m goal is refused with a warning and nothing moves; the 0.3 m goal moves the robot about 30 cm |
| M10 | Speed caps | relaunch with `max_linear_speed:=0.1`, publish `{linear: {x: 0.3}}` at 10 Hz for 2 s, watch `ros2 topic echo /pepper_odom --field twist.twist.linear.x` | about 0.1, not 0.3 |
| M11 | `listen` | `ros2 action send_goal /listen naoqi_bridge_msgs/action/Listen "{expected: ['hello'], language: 'en'}"`: once saying "hello", once staying silent; then with `expected: ['say \"hi\"']` | "hello" succeeds; silence aborts after 30 s (`listen_timeout`); the quoted phrase is rejected at once |
| M12 | NAOqi errors are logged | `ros2 topic pub --once /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed "{joint_names: ['BadJoint'], joint_angles: [0.1], speed: 0.1}"` | the driver logs `ALMotion.setAngles failed: ...` |
| M13 | Minidump | relaunch with `minidump_duration:=10`, wait 15 s, `ros2 service call /naoqi_driver/minidump std_srvs/srv/Trigger`, then `ros2 bag info <folder it answers>` | the bag holds about the last 10 s; delete it afterwards |
| M14 | Camera timestamps | `ros2 topic delay /camera/front/image_raw` for 10 s | the delay is positive and steady, under 0.2 s (before commit 14 it is about 0: stamped on arrival) |
| M15 | Sensor parameters | write `/tmp/no_front.yaml` with `/**: {ros__parameters: {converters: {front_camera: {enabled: false}}}}`, relaunch with `params_file:=/tmp/no_front.yaml`; then `ros2 param set /naoqi_driver converters.front_camera.enabled true` | no `camera/front` topics; the `param set` is refused |
| M16 | Clean exit | Ctrl+C the driver: connected with the robot resting, connected with the robot awake (`qicli call ALMotion.wakeUp`), and with a wrong `nao_ip` (while it retries) | the launch prints `process has finished cleanly`, never `exit code -6` or `-11`, and no `failed on shutdown` warning |
| M17 | Reconnect | with the robot resting, `ssh nao@<ip> nao restart` (the robot goes limp until NAOqi is back) | the driver logs the lost connection, exits, is respawned, retries (1, 2, 4 ... s) and reconnects |
| M18 | Joystick safety (needs a pad, laser and sonar enabled) | `ros2 launch naoqi_driver safe_teleop.launch.py`; hold LB and push the stick; release; drive slowly towards a box; back away | moves only while LB is held, at most 0.2 m/s; stops at once on release; slows and stops about 10 cm before the box; backing away works |

### Robot watchdog

Install `robot_watchdog.py` on the robot as the README describes, and launch the
driver with `respawn:=false` so that it does not come back during these checks.
Watch the watchdog's log with `ssh nao@<ip> tail -f robot_watchdog.log`.

| # | Check | How | Pass when |
|---|---|---|---|
| W1 | It finds the driver | start the driver | the log says `naoqi_driver heartbeat found, watching it` |
| W2 | A crashed driver | flap closed, area clear: publish `{linear: {x: 0.05}}` at 10 Hz, then `pkill -9 -f naoqi_driver_node` and stop the publisher | the base stops about 1 s after the kill; the log says `No naoqi_driver heartbeat ... stopped the base` |
| W3 | A lost network | as W2, but unplug the network of the computer running the driver instead of killing it | the base stops about 1 s after unplugging |
| W4 | A clean exit | Ctrl+C the driver | the log says `naoqi_driver exited cleanly` and nothing else happens |

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
| 9 | Add motion safety limits, surface NAOqi errors, test against a fake NAOqi | b, c, M9, M10, M12 |
| 10 | Run ROS callbacks on their own threads and shut down cleanly | c, M1, M2, M4, M17 |
| 11 | Make the listen action time out, validate its goals and set up off-thread | M11 |
| 12 | Stop the base from the robot when the driver dies, hangs or loses the network | W1 to W4 |
| 13 | Record sensor data into rosbag2 bags through ROS 2 services | a, e, M13 |
| 14 | Stamp camera images at capture, make the boot config parameters, fix the crash on exit | a, M14, M15, M16 |
| 15 | Add a joystick launch with a dead-man switch and a collision monitor, fix CI | M18 |
| 16 | Remove stale docs and leftovers, extend the robot checks | a, e |
| 17 | Fix the crash on exit with joint states and stopping speech on NAOqi 2.5 | M2, M16 |

Commits 6 to 8 and 16 are meant to change no behaviour. If a check passes on
commit 5 but fails on commit 8, the cause is in 6 to 8.

Commits 1 to 3 do not contain the check script or the test configs, so copy
them out first. Commits before 14 do not take `params_file`: there, put the
full config in place of the installed `boot_config.json` after each build.
`--remove-destination` replaces the installed file; with `--symlink-install` a
plain `cp` would write through the link into your own `share/boot_config.json`.

```bash
cd ~/ros2_ws
cp src/naoqi_driver2/test/robot/robot_checks.py src/naoqi_driver2/test/robot/boot_config_full.json \
  src/naoqi_driver2/test/robot/full_sensors.yaml /tmp/
CFG=install/naoqi_driver/share/naoqi_driver/share/boot_config.json

git -C src/naoqi_driver2 checkout <commit>
colcon build --packages-select naoqi_driver
cp --remove-destination /tmp/boot_config_full.json $CFG   # commits before 14
# relaunch the driver (commits 14 and later: params_file:=/tmp/full_sensors.yaml), then:
python3 /tmp/robot_checks.py <flags of the stage that failed>
```

The next build puts your own config back.

On commits before the watchdog, the timeout check reports SKIP, not FAIL.
Return with `git -C src/naoqi_driver2 checkout feature/modern-cpp`.
