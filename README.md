# ReconCycle Context Action Framework

This package provides a collection of common class definitions that feature extractors (e.g., in vision pipeline) and action predictors should use.

In this framework, a general (dis)assembly process is inspired by a Markov Decision Process (MDP). At time $T_j$ the next (dis)assembly action $A_j$ is predicted based on $A_{j-1}, ..., A_0$ and current context (e.g., sensory data, workcell state, above which module the robot is currently manipulating).

The system is currently made up of a vision pipeline, an action framework and a controller. In principle these frameworks don't know anything about each other. The controller is responsible for deciding on the actions to be performed in the system.

The controller does this by getting information from the vision framework, and possibly other sources. It uses actions history and a prediction model, to determine which action should be performed next given current context. To carry out this action it calls the appropriate action block and passes it the appropriate low-level information needed to carry out the action.


## Controller


![image](https://user-images.githubusercontent.com/2089122/182888132-7814702b-4494-4a88-8e98-1fd78438b069.png)
<!-- https://www.figma.com/file/yUK2I6GPWmI2sBdQeOaIkF/Reconcycle-Action-Framework?node-id=0%3A1 -->

The controller stores a history of previous task steps (requests to the vision system, the vision response, the action prediction, and the robot action success).

Based on this historic data, current cell state (`Get context` block), and a prediction model, the next action is predicted (`Get next action` block).

## Context

The context is defined as the work-cell module that is being operated in and the sensory information.

The modules are: vice module, cutter module, empty table. 






## Modality-specific feature extractors 

Currently, we only have vision (see [Vision Library below](#Vision-Library)). In the future, this will be extended to tactile skills as well.

## Vision Library

The vision system is given a specific task such as "gap detection" or "parts detection". The gap detection is useful for levering actions. The parts detection is useful for moving actions. All coordinates of parts are given in world coordinates with respect to the module. 

The gap detection runs parts detection as well.

### Parts Detection

The parts detection uses a neural network called Yolact for parts segmentation. It uses a kalman filter for tracking and reidentification. 



Input: rgb image, module, image to world coord transform
Output: list(part)

### Gap Detection

The gap detection is written to provide the gaps in the device that are most useful for levering.

Input: aligned rgb and depth image, module, camera position
Output: parts detection, gap centroid, levering direction, gap 3D box


### Camera Position

The camera position needs to be known such that we can transform from image coordinates to world-coordinates relative to the module we are looking at.

The camera position can be fixed or mounted to the robot hand just above the end-effector.

When the camera is fixed, the world coordinates are determined by the position of the work surface in the image.

When the camera is mounted to the robot, the extrinsic position of the camera is determined by the robot transform and the hand-eye transform. The position of the object is then calculated based on camera intrinsics and distance of object from camera. Without the depth it is not possible to determine the position of an object when the object dimensions are unknown.


## Action Blocks Library

The action blocks are functions that carry out actions on the robot(s). These action blocks may use other action blocks within them. For example, the cut block moves an object into the cutter, and this movement is carried out using the move block.

<!-- | Action | Features | 
| -------- | -------- |
| Lever    | 3D box + centroid, levering direction    | 
| Cut      | length of the object to be cut, where to make the cut, which side to insert     | 
| Move     | Initial object pose, final object pose, (preferable grasping tool & tool specific parameters)     | 
| Push     | Pushing pose, pushing force, (preferable pushing tool with parameters)     |  -->

### Move Block

The move block picks up an object and moves it to a new position on the same or different module.

Input: $(x_1, y_1, \text{module}_1), (\text{width}, \text{height}), (x_2, y_2, \text{module}_2), \text{robot}, \text{end effector}$

Output: Success/Failure

### Lever Block

Input: (x, y, z), 3D bounding box, direction, robot, module, end effector

Output:Success/Failure

### Cut Block

The cut block should move the object from a position to the vice, where it is to be cut.

Input: part, module, where to make cut, which side to insert

Output: Success/Failure

### Push Block

Input: Pushing pose, pushing force, (preferable pushing tool with parameters)

Output: Success/Failure

### Turn Over Block

Turn over the part 180 degrees.

Input: part, module, robot, end effector

Output: Success/Failure



##  Programming guideline

This package also serves as a library that provides a collection of common class definitions that feature extractors and action predictors should use.

### Part Definition

A part is defined as the whole or part of a device (the device itself is considered a part.

A part detection has the following properties: `id, label, score, box, mask, obb corners, obb center, obb rotation quarternion`

Each part is defined with the following properties:

| Name | Type | Description |
| -------- | -------- | -------- |
| id     | int     | Text     |
| label     | int (enum)     | Text     |
| score  | float     | Text     |
| box     | array     | Text     |
| mask    | Tensor     | Text     |
| \*mask contour    | Tensor     | Text     |
| \*mask polygon    | Tensor     | Text     |
| obb corners (camera-coords)| Text     | Text     |
| obb center (camera-coords)    | Text     | Text     |
| obb rotation quarternion     | Text     | Text     |
| obb corners world-coords     | Text     | Text     |
| obb center (world-coords)     | Text     | Text     |
| id     | Text     | Text     |

Rows with a \* are not published because they are big or not necessary outside of the vision library.

### Label Definition

The part label is an enum with the following definition:

| Name | Int Value | 
| -------- | -------- | 
| hca_front     | 0     |
| hca_back     | 1     |
| hca_side1     | 2     |
| hca_side2     | 3     |
| battery     | 4     |
| pcb     | 5     |
| internals     | 6     |
| pcb_covered     | 7     |
| plastic_clip     | 8     |

### Action Definition

The action is an enum with the following definition:



| Name | Int Value | 
| -------- | -------- |
| move     | 0     |
| move     | 1     |
| move     | 2     |
| move     | 3     |
| move     | 4     |


### Python Classes Structure

Each Action block should inherit from ActionBlock superclass, which defines the following methods:
...

Each Feature extraction block should inherit from FeatureExtractor superclass, which defines the following methods:
...

### ROS integration

Each feature extractor should provide a ROS service wrapper in the `/feature_extractors` namespace.

In the request, the object id should be given

For example, calling `/feature_extractor/levering`, with `'id: hca_1'` returns: (parameters for levering that are used in the LeveringActionBlock)


### FlexBe integration

The controller is implemented as a FlexBe behaviour. 
![image](https://user-images.githubusercontent.com/2089122/182890952-a0f812a2-0ac7-4095-9e6d-cc6bc34675e0.png)


Several FlexBE states should be created:
- `Get context` should return the current context and pass it to the next state
- `Get next action` (read recommended action) is a wrapper to the action prediction model, and should return the next action given the context


For each of the actions, a separate FlexBe state that calls an appropriate ActionBlock is needed. 
