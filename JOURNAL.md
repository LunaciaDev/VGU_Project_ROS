# Dev Journal

Recording of how we worked on this project.

## Todo List

- [x] Write the description for the ur10e mounted with rg2
- [x] Create the moveit package
- [x] Migrate the message and service definition
- [ ] Migrate the planner
    - [ ] The service does not receive return value from Moveit?

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