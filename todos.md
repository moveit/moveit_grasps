### grasp_generater
* [x] visualisations
* [x] ideal_grasp_pose_
* [x] enable (iterations > max_iterations)
* [x] robot agnostic pre grasp and post retreat
* [ ] grasp generation for cylinders
* [ ] cleanup

### grasp_data
* [x] angle_resolution_
* [ ] taking joint_positions
* [ ] verify the need for so many functions

### grasp_generator_test
* [x] add test for cylinder and cube

### for_tutorial
* [ ] explain ideal pose setting


### General Package TODO
* [ ] Sorting of the grasp scores.
* [ ] Joint position management needs to be handled properly. [Issue Tracker](https://github.com/davetcoleman/moveit_grasps/issues/7)
* [ ] pre grasp approach and post grasp retreat can be more intelligent. We can probably use the direction of the grasp to define the pregrasp approach. Is post grasp retreat always going to be up?
* [ ] The grasp generator works with cylinder and cuboid only. Also, cylinder's function needs improvement.
* [ ] Only the grasp_generator and the grasp_plan work.
* [ ] 
