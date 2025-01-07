# ROS 2 integration for Franka Robotics research robots

Mingrui's comments:
* [Refernce](https://github.com/frankaemika/franka_ros2/issues/34#issuecomment-2031700642)
* This is a ROS2 driver for Franka Emika Panda.
* Tested on Ubuntu 22.04 + ROS Humble + libfranka 0.9.3.
* libfranka 0.9.3 can be installed via `sudo dpkg -i libfranka-0.9.3-x86_64.deb`.
* The joint position controller is unsupported. The joint velocity or torque controller has been tested.

Test: 
```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=<robot_ip> use_rviz:=true load_gripper:=false
```


---

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros2` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs
