# ROS communicating with LabVIEW.
This is a part of MAST for MDH.
In this part it is supposed to implement a node that connect the nvidia tx2 to the roborio.
# How to start each induvidual node for ros labview

```
cd ~/mast/
source devel/setup.bash
rosrun ros_labview main.py
rosrun ros_labview emstop.py
rosrun ros_labview to_controll.py
rosrun distance_sense main.py
```
