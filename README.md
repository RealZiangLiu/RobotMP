# RobotMP
## Introduction
Simple wrapper for OMPL planners to use in robotics applications. Abstracts away redundant processes and simplifies OMPL API for planning. Easy and fast planning with as few as three function calls.
## Installation
### Standard Process (Tested on Ubuntu 18.04)
* Download installation script
  * https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
* Install with Python bindings
  * ```shell
    chmod u+x install-ompl-ubuntu.sh
    ./install-ompl-ubuntu.sh --python
    ```
  * Python bindings are installed under ```ompl-1.5.2/py-bindings/ompl```
* Add ompl path to system PATH
  * ```shell
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join('YOUR_OMPL_INSTALL_PATH', 'ompl-1.5.2', 'py-bindings'))
    ```
### Without Python Bindings
* ```sudo apt-get install libompl-dev ompl-demos```
### Notes
* CastXML had a bug with c++14 strings, which was fixed in 2016. Installing through apt on Ubuntu 16.04 still gives
the pre-bugfix version.
    * Remove old castxml: ```sudo apt remove castxml```
    * Install newest version through pip: ```pip3 install castxml```

## Usage
```python
  import numpy as np
  from numpy.linalg import norm

  import robotmp as rmp

  # create planner
  planner = rmp.OMPLPlanner(state_space_bounds = [[-1., -1., -1.],[1., 1., 1.]],
                            state_validity_checker = is_safe,
                            planner = "rrt_connect")
  
  # set start and goal states
  planner.set_start_and_goal(start = [-0.5, 1.0, 1.0], goal = [0.5, 0.5, 0.5])
  
  # solve!
  path, cost, t = planner.plan(time_limit = 3.0)
```

### Planner Settings
* Set/Update start state and goal state
  * ```python
    set_start_and_goal(start = [0,0,0], goal = [1,1,1])
    ```
* Set max distance of a single expand step for random tree related planners (RRT, etc.)
  * ```python
    set_step_distance(step_distance = 0.5)
    ```
* TODO
