### grasp_generater
* [x] visualisations
* [x] ideal_grasp_pose_ is being set using the direction of the robot to the object to be picked, directness preferred.
* [x] enable (iterations > max_iterations)
* [x] robot agnostic pre grasp and post retreat
* [x] grasp generation for cylinders

### grasp_data
* [x] angle_resolution_
* [x] taking joint_positions

### grasp_generator_test
* [x] add test for cylinder and cube

---

## MoveIt! Grasps TODO
* [x] Sorting of the grasp scores.
* [ ] Joint position management needs to be handled properly. [Issue Tracker](https://github.com/davetcoleman/moveit_grasps/issues/7)
* [ ] pre grasp approach and post grasp retreat can be more intelligent. We can probably use the direction of the grasp to define the pregrasp approach. Is post grasp retreat always going to be up?
* [ ] The grasp generator works with cylinder and cuboid only. Also, cylinder's function needs improvement.
* [ ] Only the grasp_generator and the grasp_plan work.
* [ ] grasp_pose_to_eef_transform settting needs to be verified. I recently noticed that it was being applied twice. [here](https://github.com/Ridhwanluthra/moveit_grasps/blob/kinetic-devel/src/grasp_generator.cpp#L394) and [here](https://github.com/Ridhwanluthra/moveit_grasps/blob/kinetic-devel/src/grasp_generator.cpp#L720)
* [ ] Ideal Pose setting and scoring based on that needs a look. Even after manually setting the exact ideal pose required and sorting the grasp scores. The grasps are still not the best ones. This may also be a problem because of something else, this is just my best guess.

## Grasping Tutorial
### TODO
* [ ] Fixing links
* [ ] Adding demo
* [ ] once the scoring and setting of ideal_grasp_pose is handled need to add that here also.
* [ ] Link with the perception pipeline demo

### Blockers
* [ ] The package needs to work reliably.
