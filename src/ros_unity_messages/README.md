# ros_unity_messages

This package provides the custom messages used for communication between application code and Unity.

# Overview

[`msg`](msg/) holds the message definition.

We defines two custom messages:

- [`UnityRequest.msg`](msg/UnityRequest.msg) define the message sent when Unity request planning. It contains the current joint configuration in Unity (currently unused); Position and orientation of both the object to pick and where it needed to be placed; A vector of all static scene object in Unity that the robot need to avoid colliding.
- [`UnityObject.msg`](msg/UnityObject.msg) define the data for each object in the Unity scene. It contains the object's ID as a string (equivalent to the name of that object in the scene); Position, orientation and scale of the object.