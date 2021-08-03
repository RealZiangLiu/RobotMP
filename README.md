# RobotMP
## Installation
### Standard Process
* https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
* ```chmod u+x install-ompl-ubuntu.sh```
* ```./install-ompl-ubuntu.sh --python```
  * Python bindings are installed under ompl-1.5.2/py-bindings/ompl
* Add ompl path to system PATH
  * ```
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join('/home', 'ziang', 'Workspace', 'lib', 'ompl-1.5.2', 'py-bindings'))
    ```
### Without Python Bindings
* ```sudo apt-get install libompl-dev ompl-demos``` No python bindings!
## Steps
* Removed realsense repo from sources.list, because the server was down, blocking apt update
* https://docs.google.com/document/d/1_ncHlFyim1gMGIV_77-Pti_m1WyNrC6Vt1UF7WtFxHM/edit#
* ```export PYTHONPATH="~/Workspace/lib/ompl-1.5.2/py-bindings/:$PYTHONPATH"```

