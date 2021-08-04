# RobotMP
## Installation
### Standard Process
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
* CastXML had a bug with c++14 strings, which was fixed in 2016. Installing through apt still gives
the pre-bugfix version.
    * Remove old castxml: ```sudo apt remove castxml```
    * Install newest version through pip: ```pip3 install castxml```
* 
## Backtrack Notes
* Removed realsense repo from sources.list, because the server was down, blocking apt update
* https://docs.google.com/document/d/1_ncHlFyim1gMGIV_77-Pti_m1WyNrC6Vt1UF7WtFxHM/edit#
* Added files
    * /usr/lib/x86_64-linux-gnu/libode.so.6
    * libboost_serialization.so.1.65.1
    * libboost_filesystem.so.1.65.1
 * Removed files
    * rm /usr/lib/x86_64-linux-gnu/libboost_python.so
ln -s /usr/lib/x86_64-linux-gnu/libboost_python-py35.so /usr/lib/x86_64-linux-gnu/libboost_python.so
