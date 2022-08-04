# ReconCycle Context Action Framework

This package provides a collection of common class definitions that feature extractors (e.g., in vision pipeline) and action predictors should use.

In this framework, a general (dis)assembly process is inspired by a Markov Decision Process (MDP). At time $T_j$ the next (dis)assembly action $A_j$ is predicted based on $A_{j-1}, ..., A_0$ and current context (e.g., sensory data, workcell state, above which module the robot is currently manipulating).

The system is currently made up of a vision pipeline, an action framework and a controller. In principle these frameworks don't know anything about each other. The controller is responsible for deciding on the actions to be performed in the system.

The controller does this by getting information from the vision framework, and possibly other sources. It uses actions history and a prediction model, to determine which action should be performed next given current context. To carry out this action it calls the appropriate action block and passes it the appropriate low-level information needed to carry out the action.


![image](https://user-images.githubusercontent.com/2089122/182888132-7814702b-4494-4a88-8e98-1fd78438b069.png)
<!-- https://www.figma.com/file/yUK2I6GPWmI2sBdQeOaIkF/Reconcycle-Action-Framework?node-id=0%3A1 -->


# FlexBE example

![image](https://user-images.githubusercontent.com/2089122/182890952-a0f812a2-0ac7-4095-9e6d-cc6bc34675e0.png)
