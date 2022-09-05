# ReconCycle Context Action Framework

This package provides a collection of common class definitions that feature extractors (e.g., in vision pipeline) and action predictors should use.

## Introduction

The system is split into the following components:

- Controller: requests and carries out actions
- Context
- [Action Predictor](https://github.com/ReconCycle/action_predictor): provides the **get next action** service
- Action Blocks:
  - Cut Block
  - Lever Block
  - Move Block
  - Push Block
  - Turn Over Block
  - Vice Block
  - Vision Block

The system has action blocks, which are high level components, for example: a move block that specifies the start and end positions of a pick-and-place operation and a vision block that gets object positions based on camera images. 

The system has a controller that requests the next action to take from the [get next action service](https://github.com/ReconCycle/action_predictor) a.k.a. action predictor. The action predictor doesn't know anything about the controller. The controller is responsible for carrying out the actions.

![Diagram](./readme_diagram.svg)

The action predictor stores the previous predicted actions and whether they were successful. It uses this in a prediction model to determine which action should be performed next given the current context. The action predictor returns the next action to the controller. The system is illustrated in the above diagram.

##  Programming guideline

This package also serves as a library that provides a collection of common class definitions that should be used over the entire project.

## Controller

<!-- https://user-images.githubusercontent.com/2089122/182888132-7814702b-4494-4a88-8e98-1fd78438b069.png -->
<!-- https://www.figma.com/file/yUK2I6GPWmI2sBdQeOaIkF/Reconcycle-Action-Framework?node-id=0%3A1 -->

The controller requests the next action to take from the action predictor. The action predictor returns an [action block](#action-blocks). The controller carries out the action as described by the action block and returns the context to the action predictor in the form of action details.

The controller is implemented as a FlexBe behaviour.

![image](https://user-images.githubusercontent.com/2089122/182890952-a0f812a2-0ac7-4095-9e6d-cc6bc34675e0.png)

### FlexBE

Several FlexBE states should be created:
- `Get context` should return the current context and pass it to the next state
- `Get next action` (read recommended action) is a wrapper to the action prediction model, and should return the next action given the context


For each of the actions, a separate FlexBe state that calls an appropriate ActionBlock is needed. 

## Action Predictor

The [action predictor](https://github.com/ReconCycle/action_predictor) is described in detail in it's own readme.
The action predictor provides the **get next action** service and it returns an [action block](#action-blocks).

## Context

The context is defined as the state of the system including the work-cell module that is being operated in, the positions of objects in the system, and the state of the robots and the modules.

These are all specified as enums in [types.py](src/context_action_framework/types.py).


The modules are: 
- vision
- panda1
- panda2
- vice
- cutter

The robots are:
- panda1
- panda2

The End effectors are:
- soft hand
- soft gripper
- screwdriver

The Cameras are:
- basler
- realsense

The Labels of objects are:
- hca front
- hca back
- hca side 1
- hca side 2
- battery
- pcb
- internals
- pcb covered
- palastic clip

The actions are:
- none
- start
- end
- cut
- lever
- move
- push
- turn over
- vision
- vice

These are all specified as enums in [types.py](src/context_action_framework/types.py).

## Action Blocks

An action block is a high level specification of an operation that can be carried out on the Reconcycle cells. The action block can be a physical movement, an information extractor from the physical environment, or a combination of the two.

Action blocks are high level blocks and an action block can consist of multiple actions, for example, the cut block moves an object into the cutter, and then the cutter is activated to cut the object.


### [Cut Block](msg/CutBlock.msg)

The cut block should specify the initial position of the object and the cutter module, where the object is to be cut.

[CutBlock.msg](msg/CutBlock.msg): 
- enum from_module
- Transform from_tf
- enum to_module
- Transform to_tf
- array obb_3d
- enum robot
- int end_effector

[CutDetails.msg](msg/CutDetails.msg): 
- bool success

### [Lever Block](msg/LeverBlock.msg)

The lever block should specify from where to where to carry out the levering action and with which end effector and robot.

[LeverBlock.msg](msg/LeverBlock.msg):
- enum module
- Transform from_tf
- Transform to_tf
- array obb_3d
- enum robot
- enum end_effector

[LeverDetails.msg](msg/LeverDetails.msg):
- bool success

### [Move Block](msg/MoveBlock.msg)

The move block specifies the start and end positions of an object and which end effector and robot should do the moving.

[MoveBlock.msg](msg/MoveBlock.msg): 
- enum from_module
- Transform from_tf
- enum to_module
- Transform to_tf
- array obb_3d
- enum robot
- enum end_effector

[MoveDetails.msg](msg/MoveDetails.msg):
- bool success

### [Push Block](msg/PushBlock.msg)

The push block specifies the start and end positions of the pushing action and with which robot and end effector the push action should be carried out with.

[PushBlock.msg](msg/PushBlock.msg):
- enum module
- Transform from_tf
- Trnasform to_tf
- array obb_3d
- enum robot
- enum end_effector

[PushDetails.msg](msg/PushDetails.msg):
- bool success

### [Turn Over Block](msg/TurnOverBlock.msg)

The turn over block specifies the position and 3d oriented bounding box of the object that should be picked up, rotated 180 degrees, and placed down again, with the specified robot and end effector.

[TurnOverBlock.msg](msg/TurnOverBlock.msg):
- enum module
- Transform tf
- array obb_3d
- enum robot
- enum end_effector

[TurnOverDetails.msg](msg/TurnOverDetails.msg):
- bool success

### [Vice Block](msg/ViceBlock.msg)

The vice block specifies whether the vice should clamp and turn over or only clamp, or only turn over.

[ViceBlock.msg](msg/ViceBlock.msg):
- enum module
- bool clamp
- bool turn_over

[ViceDetails.msg](msg/ViceDetails.msg):
- bool success

### [Vision Block](msg/VisionBlock.msg)

The vision block specifies whether gap detections should be carried out, which camera to use and above which module. Gap detection is only possible with the realsense camera and also only the realsense camera can be moved to a specified position.

The gap detection is useful for levering actions. The parts detection is useful for moving actions. All coordinates of parts are given in world coordinates with respect to the module. 

The parts detection uses a neural network called Yolact for parts segmentation. It uses a kalman filter for tracking and reidentification. 

The gap detection uses the depth image and a classical clusturing approach to determine gaps in the device.

The vision details are a list of detections and gaps (if gap detections were requested and available).

[VisionBlock](msg/VisionBlock.msg):
- enum camera
- enum module
- transform tf
- bool gap_detection

[VisionDetails](msg/VisionDetails.msg): 
- bool gap_detection
- Detection[] detections
- Gap[] gaps

A detection is defined as the whole or part of a device.

[Detection](msg/Detection.msg):
- int id
- int tracking_id
- enum label
- float score
- Transform to_px
- array box_px
- array obb_px
- array obb_3d_px
- Transform tf
- array box
- array obb
- array obb_3d
- array polygon_px

[Gap](msg/Gap.msg):
- int id
- Transform from_tf
- Transform to_tf
- float from_depth
- float to_depth
- array obb
- array obb_3d

#### Camera Position

The camera position needs to be known such that we can transform from image coordinates to world-coordinates relative to the module we are looking at.

The camera position can be fixed or mounted to the robot hand just above the end-effector.

When the camera is fixed, the world coordinates are determined by the position of the work surface in the image.

When the camera is mounted to the robot, the extrinsic position of the camera is determined by the robot transform and the hand-eye transform. The position of the object is then calculated based on camera intrinsics and distance of object from camera. Without the depth it is not possible to determine the position of an object when the object dimensions are unknown.

### More Tactile Skills to come...

Currently, we only have vision. In the future, this will be extended to tactile skills as well.

