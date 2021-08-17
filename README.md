# RobotMP
## Installation
### Standard Process (Tested on Ubuntu 18.04)
* Download installation script
  * https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
* Install with Python bindings
  * ```
    chmod u+x install-ompl-ubuntu.sh
    ./install-ompl-ubuntu.sh --python
    ```
  * Python bindings are installed under ```ompl-1.5.2/py-bindings/ompl```
* Add ompl path to system PATH
  * ```
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
```
  # create planner
  planner = rmp.OMPLPlanner(state_space_bounds = [[-1., -1., -1.],[1., 1., 1.]],
                            state_validity_checker = is_safe,
                            planner = "rrt_connect")
  
  # set start and goal states
  planner.set_start_and_goal(start = [-0.5, 1.0, 1.0], goal = [0.5, 0.5, 0.5])
  
  # solve!
  path, cost, t = planner.plan(time_limit = 3.0)
```
