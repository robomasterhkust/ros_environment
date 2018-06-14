Brief:
This ros package receives topic "/cmd_vel", either published by teleop keyboard or by the pid/gimbal controller for self aiming.
This ros package publishes topic "/sent_messages" in can frame message type, which will later on be transmitted to TX2 by the socketcan bridge node.
Maintainer: Pang sui, Yang Shaohui
