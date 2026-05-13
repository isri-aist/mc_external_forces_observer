# mc_external_forces_observer

## Overview
The `mc_external_forces_observer` observer estimates external forces acting on a robot by fusing multiple data sources:
- A momentum-based residual method
- Joint torque sensor measurements
- Force/Torque (F/T) sensor readings (if available)

This approach improves the accuracy of external force estimation by combining multiple sources of information, making it suitable for applications such as contact detection, force control, and robotic interaction with the environment.

## Features
- Fusion of multiple force estimation methods
- Adaptable to different robots with joint torque sensors
- Compatible with various F/T sensors
- Real-time external force estimation
- Tested on the Kinova Gen3 cobot with a Bota Sensone F/T sensor

## Requirements
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc): A modular framework for real-time robot control
- [mc_ros_force_sensor](https://github.com/mathieu-celerier/mc_ros_force_sensor) (only required if the F/T sensor communicates via ROS1/2)

## Installation
It is recommended to use [mc_rtc superbuild](https://github.com/mc-rtc/mc-rtc-superbuild) to install the plugin. However, you can install it as a standalone, like the following.
Ensure you have `mc_rtc` installed on your system. If your F/T sensor requires ROS, install `mc_ros_force_sensor` as well. Then, clone and build the plugin:

```sh
cd ~/workspace/src  # Or any preferred workspace
git clone https://github.com/bastien-muraccioli/mc_external_forces_observer
cd mc_external_forces_observer
mkdir build && cd build
cmake ..
make
sudo make install
```

## Usage
To enable the plugin in `mc_rtc`, add the following entry in your configuration file (`$HOME/.config/mc_rtc/mc_rtc.yaml`):

```yaml
Plugins: ExternalForcesEstimator
```

### Configuration
You can create a configuration file at (`$HOME/.config/mc_rtc/plugins/ExternalForcesEstimator.yaml`), to define:
- Filtering gains
- Reference frame of the F/T sensor
- Plugins parameters

Example:
```yaml
residual_gain: 10
reference_frame: FT_sensor_wrench
use_force_sensor: false
forward_dynamics_mode: Default
bias_term_mode: Default
ros_force_sensor: true
```

## Adapting to Other Robots and Sensors
The plugin can be used with different robots and F/T sensors as long as:
- The robot has joint torque sensors
- The F/T sensor can be integrated into `mc_rtc`

For new robot models, modify the configuration files to reflect the correct sensor names and kinematic parameters.
