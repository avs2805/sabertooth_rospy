# sabertooth_rospy

ROS node to control a diff-drive robot with a sabertooth motor controller with Teleop_twist_joy node

### TODO

- [x] ROS node to subscribe to cmd_vel from teleop_twist_joy
- [x] Convert cmd_vel to diff-drive velocities for each wheel
- [x] Sabertooth_rospy node to detect a connected sabertooth device
- [x] Send drive commands to sabertooth motor controller
- [x] teleop_twist_joy tuning to get adequate speed on robot
- [x] change linear and angular axis for driving robot
- [x] launch file to launch teleop, cmd_vel conversion and sabertooth to motor comm
- [ ] Code cleanup and commenting
- [ ] README.md instructions
