# naoqi_driver2

This repo defines the __naoqi_driver__ package for ROS2. The driver is in charge of providing bridging capabilities between ROS2 and NAOqiOS.


## How it works

The __naoqi_driver__ is a ROS node.
It connects to a robot running NAOqi using libQi.
To support audio extraction, __naoqi_driver__ opens a public endpoint, on a random port by default.

> You can set a local endpoint instead by setting the `qi_listen_url` argument,
> *e.g.* `qi_listen_url:=tcp://127.0.0.1:12345`.
> In turn, audio extraction will be disabled.


## Installation

### Dependencies

To run, the driver requires the [`naoqi_libqi`](https://github.com/ros-naoqi/libqi),
[`naoqi_libqicore`](https://github.com/ros-naoqi/libqicore)
and [`naoqi_bridge_msgs`](https://github.com/ros-naoqi/naoqi_bridge_msgs2) packages.
Those can be installed using apt-get (if they have been released for your ROS distro) or from source.
Additionally, [`pepper_meshes`](https://github.com/ros-naoqi/pepper_meshes2)
and/or [`nao_meshes`](https://github.com/ros-naoqi/nao_meshes2) can be useful to display the robot in RViz.

On Ubuntu, install them using:

```sh
sudo apt-get install ros-<distro>-naoqi-libqi ros-<distro>-naoqi-libqicore ros-<distro>-naoqi-bridge-msgs ros-<distro>-pepper-meshes ros-<distro>-nao-meshes
```

### Installing from source

In your ROS2 workspace, clone the repo and its dependencies:

```sh
cd <ws>/src
git clone https://github.com/yohatad/naoqi_driver2.git
vcs import < naoqi_driver2/dependencies.repos
cd <ws>
rosdep install --from-paths src --ignore-src --rosdistro <distro> -y
```

> To install vcs: `sudo apt-get install python3-vcstool`

Then build the workspace:

```sh
cd <ws>
colcon build --symlink-install
```

> Meshes can only be built on x86(_64) platforms.
> You can skip them by building with these arguments:
>
> ```sh
> colcon build --packages-skip nao_meshes pepper_meshes
> ```

#### License of the meshes

The source repositories include
[`pepper_meshes2`](https://github.com/ros-naoqi/pepper_meshes2)
and [`nao_meshes2`](https://github.com/ros-naoqi/nao_meshes2),
which require an interactive agreement to be provided.
If you agree to their license terms (
[CC BY-NC-ND 4.0](https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode.en):
[`pepper_meshes2` LICENSE](https://github.com/ros-naoqi/pepper_meshes/blob/master/LICENSE),
[`nao_meshes2` LICENSE](https://github.com/ros-naoqi/nao_meshes/blob/master/LICENSE)).
you may skip the interactive prompt by setting
the `AGREE_TO_NAO_MESHES_LICENSE` and `I_AGREE_TO_PEPPER_MESHES_LICENSE` environment variables:

```sh
I_AGREE_TO_NAO_MESHES_LICENSE=1 I_AGREE_TO_PEPPER_MESHES_LICENSE=1 colcon build --symlink-install
```

Automated jobs on non-interactive environments
(`DEBIAN_FRONTEND=noninteractive`)
defaults to agreeing to the licenses,
assuming the author of the job has agreed to the license terms.


## Launch

### Avoid interference with autonomous life

To have full control of the robot with ROS,
you may want to disable autonomous behaviors first:

```sh
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```


### NAOqi 2.8 and lower

The driver can be launched from a remote machine this way:

```sh
source /opt/ros/<distro>/setup.bash # or source <ws>/install/setup.bash if built from source
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host>
```


### NAOqi 2.9 and higher

Username and password arguments are required
for robots running NAOqi 2.9 or greater:

```sh
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> password:=<robot_password>
```

> When password is set, the driver automatically switches TLS on, and switches to port 9503.


### From a Docker container

It should work as usual from a Docker container,
but to enable audio features, you will need to forward the port the driver listens to.
In turn, you should specify the port libQi should listen, *e.g.* for port 56000:

```sh
source /opt/ros/<distro>/setup.bash # or source <ws>/install/setup.bash if built from source
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> qi_listen_url:=tcp://0.0.0.0:56000
```

Then you must [expose](https://docs.docker.com/engine/reference/commandline/run/#publish) it from the container.


### On the robot or on the same machine

If you run the driver directly from the robot (or your machine running a virtual robot),
you can omit the options:

```sh
ros2 launch naoqi_driver naoqi_driver.launch.py
```

### With the front camera (gscam2)

The `pepper_bringup.launch.py` launch file starts the driver together with the
[`gscam2`](../gscam2) node that consumes Pepper's front-camera GStreamer stream
and publishes it into the driver's TF tree (frame `CameraTop_optical_frame`,
topics `/pepper/front/image_raw` and `/pepper/front/camera_info`).

Start the camera stream on the robot first, then launch:

```sh
ssh nao@<robot_host> '~/start_camera.sh'
ros2 launch naoqi_driver pepper_bringup.launch.py nao_ip:=<robot_host>
```

The camera is controlled by the `use_camera` argument (default `true`). To run
the driver on its own, disable it:

```sh
ros2 launch naoqi_driver pepper_bringup.launch.py nao_ip:=<robot_host> use_camera:=false
```

> `pepper_bringup.launch.py` accepts all the `naoqi_driver.launch.py` arguments
> (`nao_ip`, `nao_port`, `user`, `password`, `network_interface`,
> `qi_listen_url`, `namespace`, `publish_wheel_odom_tf`) and forwards them.
> Remember to stop the stream afterwards with `ssh nao@<robot_host> '~/stop_camera.sh'`.

### Choose the sensors

`share/boot_config.json` sets which sensors the driver reads, and how often.
Each of its settings is also a parameter named after its path, such as
`converters.front_camera.enabled`, and a parameter overrides the file. Put the
overrides in a YAML file:

```yaml
/**:
  ros__parameters:
    converters:
      front_camera:
        enabled: true
        resolution: 1  # QQVGA = 0, QVGA = 1, VGA = 2
      laser:
        enabled: false
```

and pass it at launch (`pepper_bringup.launch.py` accepts it too):

```sh
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> params_file:=<file.yaml>
```

The driver reads these settings once, at start-up, so `ros2 param set` refuses
to change them.

## Check that the node is running correctly

Check that the driver is connected:

```sh
ros2 node info /naoqi_driver
```

### Hello, world

Make the robot say hello:

```sh
ros2 topic pub --once /speech std_msgs/String "data: hello"
```

### Listen to words

You can setup speech recognition and get one result.

```sh
ros2 action send_goal listen naoqi_bridge_msgs/action/Listen "expected: [\"hello\"]
language: \"en\""
```

### Move the head

Check that you can move the head by publishing on `/joint_angles`:

```sh
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "{header: {stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5,0.1], speed: 0.1, relative: 0}"
```

You can see the published message with `ros2 topic echo /joint_angles`

### Move around

Check that you can move the robot by publishing on `cmd_vel` to make the robot move:

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.785"
```

> Make some room around the robot!

To stop the robot, you must send a new message with linear and angular velocities set to 0:

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Drive with a joystick, safely

`safe_teleop.launch.py` puts a dead-man switch and a collision monitor between a
joystick and `/cmd_vel`. It needs packages the driver itself does not:

```sh
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy \
  ros-humble-nav2-collision-monitor ros-humble-nav2-lifecycle-manager
ros2 launch naoqi_driver safe_teleop.launch.py
```

- Hold LB to move and release it to stop. Speeds are capped at 0.2 m/s and
  0.5 rad/s. The buttons and axes in `config/safe_teleop/teleop_twist_joy.yaml`
  are for an Xbox-style pad; check yours with `ros2 topic echo /joy` and pass
  your own file with `joy_config:=<file>`.
- The robot moves at half speed within 0.6 m of an obstacle, and slows down
  further towards one so that it stops about 0.1 m short of it. Moving away
  stays possible.
- Obstacles come from the laser and the sonars, so enable
  `converters.laser.enabled` and `converters.sonar.enabled`. Without their
  data the collision monitor lets every command through.

The driver's own limits still apply: `max_linear_speed` and `max_angular_speed`,
and `cmd_vel_timeout`.

### Stop the base if the driver dies

The driver stops the base when `/cmd_vel` goes quiet and when it exits, but it
cannot when it crashes, hangs or loses the network. For those cases, run
`robot_watchdog.py` on the robot (NAOqi 2.5 with SSH access; Python 2.7 or 3):

```sh
scp $(ros2 pkg prefix naoqi_driver)/share/naoqi_driver/scripts/robot_watchdog.py nao@<robot_host>:
ssh nao@<robot_host>
nohup python robot_watchdog.py > robot_watchdog.log 2>&1 &
exit
```

The driver writes a counter to ALMemory (`NaoqiDriver/Heartbeat`) every 0.2 s.
The watchdog calls `ALMotion.stopMove()` when the counter has not changed for
1 s (`--timeout`). It does nothing until a driver runs, and nothing after the
driver exits cleanly. Start it again after the robot reboots, or have NAOqi
start it by adding its full path under `[python]` in
`/home/nao/naoqi/preferences/autoload.ini`.

### Audio Playback

The driver provides several services and an action for audio playback:

#### Load and Play Audio File

Load an audio file and play it:

```sh
# Load audio file from robot's filesystem
ros2 service call /load_audio_file naoqi_bridge_msgs/srv/LoadAudioFile "{remote_path: '/home/nao/audio.wav'}"

# Or load from raw audio data (WAV/OGG bytes)
ros2 service call /load_audio_file naoqi_bridge_msgs/srv/LoadAudioFile "{audio_data: [0,0,0,0,...]}"

# Play the loaded audio (file_id from load response)
ros2 action send_goal /play_audio naoqi_bridge_msgs/action/PlayAudio "{file_id: 0, volume: 0.8, pan: 0.0, loop: false}"
```

#### Audio Control Services

```sh
# Set volume (0.0-1.0)
ros2 service call /set_audio_volume naoqi_bridge_msgs/srv/SetAudioVolume "{file_id: 0, volume: 0.8}"

# Set master volume
ros2 service call /set_audio_volume naoqi_bridge_msgs/srv/SetAudioVolume "{file_id: -1, volume: 0.8}"

# Stop all playback
ros2 service call /stop_audio naoqi_bridge_msgs/srv/StopAudio "{}"

# Pause current playback
ros2 service call /pause_audio naoqi_bridge_msgs/srv/SetAudioVolume "{}"

# Resume paused playback
ros2 service call /resume_audio naoqi_bridge_msgs/srv/SetAudioVolume "{}"
```

#### Stream Raw Audio Buffer

Send raw audio directly to speakers (max 16384 frames):

```sh
ros2 service call /send_audio_buffer naoqi_bridge_msgs/srv/SendAudioBuffer "{audio_data: [0,0,0,0,...]}"
```

> **Note**: Audio data must be 16-bit stereo interleaved PCM.

### Record sensor data

The driver can record every enabled sensor into a rosbag2 bag, under the same
topic names it publishes, whether or not anything subscribes to them. The bag
is a folder in the directory the driver was started from:

```sh
ros2 service call /naoqi_driver/start_recording std_srvs/srv/Trigger
ros2 service call /naoqi_driver/stop_recording std_srvs/srv/Trigger  # answers with the bag folder
ros2 bag info <bag_folder>
```

To keep the last seconds of data in memory and save them after something
happened, start the driver with `minidump_duration:=10` (seconds; the default 0
turns it off), then call:

```sh
ros2 service call /naoqi_driver/minidump std_srvs/srv/Trigger
```

The minidump is refused once the bags in that directory take more than 2 GB.


## Development

Check how to [install the driver from source](#installing-from-source),
or use `docker compose` to get a more reproducible dev environment:

```sh
ROS_DISTRO=iron docker compose up dev --build
```

> To get all logs in live and build debuggable binaries,
> you may build with this command:
>
> ```sh
> colcon build --event-handlers console_direct+ --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
> ```
>
> It works also when running tests:
>
> ```sh
> colcon test --event-handlers console_direct+ --ctest-args tests
> ```


## Build Status

ROS Distro | Binary Status | Source Status | GitHub Status |
|-------------------|-------------------|-------------------|--------------------|
Humble | [![ros2-humble-jammy-bin-status-badge](https://build.ros2.org/job/Hbin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary) | [![ros2-humble-jammy-src-status-badge](https://build.ros2.org/job/Hsrc_uJ__naoqi_driver__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uJ__naoqi_driver__ubuntu_jammy__source) | [![ros2-humble-jammy](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/humble_jammy.yml/badge.svg?branch=main)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/humble_jammy.yml)
Iron | [![ros2-iron-jammy-bin-status-badge](https://build.ros2.org/job/Ibin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary) | [![ros2-iron-jammy-src-status-badge](https://build.ros2.org/job/Isrc_uJ__naoqi_driver__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Isrc_uJ__naoqi_driver__ubuntu_jammy__source) | [![ros2-iron-jammy](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/iron_jammy.yml/badge.svg?branch=main)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/iron_jammy.yml)
