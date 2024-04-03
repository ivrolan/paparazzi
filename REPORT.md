# Report Team 6

### Intuitive approach

- Configuration: `conf/userconf/tudelft/course_conf_intuitive.xml` 
  - Airframe: `conf/airframes/tudelft/bebop_course_ground_following_guided.xml`
    - Perception module: `conf/modules/cv_filter_ground3.xml`
    - Decision module: `conf/modules/ground_follower_guided.xml`, implemented in `sw/airborne/modules/ground_follower_guided/ground_follower_guided.c`
    

#### Communication between modules

The `cv_filter_ground3` modules sends the message of type `GROUND_FILTER_DETECTION` with the following values as stated in `conf/abi.xml`:
- `count_left`: number of green (ground) pixels to the left
- `count_center`: maximum risk (green pixels in a column) of the center
- `count_right`: number of green pixels to the right

### SVM approach


- Configuration: `conf/userconf/tudelft/course_conf_svm.xml` 
  - Airframe: `conf/airframes/tudelft/bebop_course_svm_binary_ground_guided.xml`
    - Perception module: `conf/modules/cv_svm_decision.xml`
    - Decision module: `conf/modules/svm_follower_guided.xml`, implemented in `sw/airborne/modules/svm_follower_guided/svm_follower_guided.c`

#### Communication between modules

The `cv_svm_decision` modules sends the message of type `SVM_DECISION_MSG` with the following values as stated in `conf/abi.xml`:
- `value`: output of the function of the SVM
- `decision`: boolean value after applying the threshold, `true` means no obstacle, `false` means we have to turn 