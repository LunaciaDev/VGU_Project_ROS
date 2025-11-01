# Dev Journal

Recording of how we worked on this project.

## Todo List

- [x] Write the description for the ur10e mounted with rg2
- [x] Create the moveit package
- [x] Migrate the message and service definition
- [x] Migrate the planner
    - [x] The service does not receive return value from Moveit?
- [ ] Picking up the object

## Journal

### 20 Oct 2025

The implementation needed to be moved back to ROS 1. Back to XML launch files.
The configuration has to be redone as the ur10e description changed between ROS 1 and ROS 2 package.
This time, not going to include the world link in the base urdf description.

The moveit package is generated via moveit_config_assistant.
We still need the calibration data for the actual robot.

### 27 Oct 2025

Our instructor suggested the paper [Control Barrier Functions via Minkowski Operations for Safe Navigation among Polytopic Sets](https://arxiv.org/abs/2504.00364) as a new way to navigate the robot. It seems to perform as a local navigator, but seems to be in 2D only. Could be 2 way this can be applied:

- Since all joint can be decomposed into a set of joints each having only one freedom of movement, which mean it is navigating in a 2D space. We converted a 3D problem into a set of 2D problem to be solved by the paper.
- Attempt to apply the tactic stated in the paper in a 3D space.

Also learned about how sampling planner works. To put the end effector of the robot into a position in its working space (w-space), there are many way we can move the robot there, proportional to the amount of joints the robot have.
Evaluating directly in w-space is difficult, so we define a new space, where each vector defines a joint configuration of the robot (c-space).
One vector in c-space map to one vector in w-space, while one vector in w-space can map to many vector in c-space.

The starting position of the robot in c-space is its current joint configuration. To figure out the goal, we need to solve an inverse kinematic problem to convert the w-space vector into a c-space vector.
The c-space vector need to also be validated for self-collision and obstacle collision, which is trivial with forward kinematic.

Then, we have 2 vectors: v_start and v_goal. Of course, the simplest path between them is a linear interpolation over time t: v_t = (1-t)v_start + t * v_goal. But we may encounter obstacle in the way, again, since all immediate vector can be generated via linear interpolation, checking for collision is relatively simple.
What we can do instead is sample N random vector within c-space, then removing all vector that is not a valid configuration due to constraint or collision.
Then, from the remaining vector, we can connect them all together via linear interpolation, removing any edges that intersect an invalid configuration.
What we are left with, is a graph, and the problem become shortest path traversal on graph, which can be solved with known algorithm like A*, Dijikstra, etc.
Said path then can be post-processed to smooth out corners, allowing the robot to move faster along them.

### 29 Oct 2025

We could create a virtual link that is at the location where we want the gripper to be, which should make it easier to position since we do not need to worry too much about offsets. Probably.

It seems to work at least on RViz. Has to fix a funny bug where the targeting point is 21 meters(!) away from the robot, but all is good now. Just need to figure out why we are not getting the return from MoveIt! - the planning works, but the code seems to not respond.

### 30 Oct 2025

Apparently, you need another spinner thread to receive Moveit plan callback? I have absolutely no idea, documentation as far as I can find did not mention this at all, and in ROS2 you only need 1 spinner thread(?) at least as far as I can tell?

Well, the service, sort of worked. Now is making it pick up the object.

It seems like the issue is on Unity side first thing - it does not actuate the hand properly, this will take a while to fix.

### 31 Oct 2025

The gripper is fixed. Turns out Unity cannot handle the way the RG2 is constructed, so removing the inner link's collision box fix the issue. It does not move exactly the same as in reality, however.

Switching into pub/sub model would allow us to attach Unity into a working robot pipeline without too much disruption. Another day, another rewrite.

### 01 Nov 2025

As one may notice, the pub-sub migration stopped dead at simple hardware implementation.

Using Moveit fake controller, everything happen inside Moveit as far as I can tell. Potentials topics yielded not much usable for controlling the robot simulation. So, a fake hardware interface is needed, as we switch to ROS' simple controllers. This way, we can hook up into the communication between Moveit and the hardware interface to control the Unity simulator. If we was allowed to use the actual robot, not much change is needed. Also, that allow us to switch to commands, so the system is free to react and course-correct, which proved difficult if implemented using service.

Now is just writing the hardware node, ugh.