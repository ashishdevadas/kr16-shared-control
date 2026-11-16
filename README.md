# cern_shared_control

Shared-control framework for safe robotic inspection and intervention in CERN-like environments
(KUKA KR16, ROS 2, `ecn_manip` simulator).

- **Mode 1, autonomous inspection:** ARIS-like scan, camera at fixed distance and normal to the surface *(in progress)*
- **Mode 2, assisted teleoperation:** guidance and forbidden-region virtual fixtures *(in progress)*
- **Common safety layer:** keep-out zones and anti-collision stop *(in progress)*

Built on [ecn_manip](https://github.com/oKermorgant/ecn_manip) (MIT License, © Olivier Kermorgant).
