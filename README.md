sawConstraintController
=======================

Constraint based controller.

Links
=====
 * Documentation: http://github.com/jhu-saw/sawConstraintController/wiki
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/
 
Dependencies
============
 * Linux, Mac OS, Windows.
 * cisst libraries: https://github.com/jhu-cisst/cisst


This repo now includes a 'lighter' wrapping for the constraint controller created by Henry Phalen with the goal to make implementation of new constraints and bridging of functionality to ROS more understandable to users. A description of those features are below:

A re-implementation of the sawConstraintController that aims to expose the most useful functionality, while simplifying the creation and use of
constraints. This is built on top of the existing framework, so it does inherit a lot of the prior terminology and classes under the hood.
Several prior features have been removed for simplicity (e.g. ControllerMode and the separation of VFData and VF types).

While the addition of too many features was one of the difficulties I found associated with the previous implementation, I have taken the risk to
add a few features that I find convenient. Most of them should be optional. These include:
- Allow for the configuration of constraints via a JSON file
- Allow for the specification of members that are exposed to ROS via provided interfaces (e.g. to set if a constraint is active, or some associated value)

You add terms to the following constrained optimization problem by creating a derived version of mtsConstraintBase. Examples are provided for the cpp and h files

Essentially, you define a constained optimization problem on joint velocity x
argmin_x: ||Ax+b||, s.t. Cx>d, Ex=f

Potential features that could be added
- Some way to not have to add constraint types in this cpp when you make new ones. It is a small addition, but still a bit annoying
  essentially, there is likely a 'factory' of sorts that could be used to simplify the process of adding new constraint implementations
- Templates for cpp and h files
- End-to-end examples