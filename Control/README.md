# Control code for Franka Emika panda arm
This code is used to control the robot arm based on [moveit-tutorial](https://github.com/ros-planning/moveit_tutorials)

## Implementation
After connecting the gel-based sensors and caliberated camera to your computer device, you can run:
```shell
python api_server.py
```
`api_server.py` is to collect the captured images from the sensors and camera.

Then, you can run the program simply with:
```shell
python franka_demo.py
```
which will begin the whole process. You can control the whole process with string inputs, which correspond to every function in the script `franka_step.py`. The `dict_func` in `franka_demo.py` demonstrates all the functions you can use with string inputs.
