# RobotMP
## Installation
### Standard Process (Works for Ubuntu 18.04)
* https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
* ```chmod u+x install-ompl-ubuntu.sh```
* ```./install-ompl-ubuntu.sh --python```
  * Python bindings are installed under ```ompl-1.5.2/py-bindings/ompl```
* Add ompl path to system PATH
  * ```
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join('/home', 'ziang', 'Workspace', 'lib', 'ompl-1.5.2', 'py-bindings'))
    ```
### Without Python Bindings
* ```sudo apt-get install libompl-dev ompl-demos```
### Notes
* CastXML had a bug with c++14 strings, which was fixed in 2016. Installing through apt on Ubuntu 16.04 still gives
the pre-bugfix version.
    * Remove old castxml: ```sudo apt remove castxml```
    * Install newest version through pip: ```pip3 install castxml```
* 
