# Line Tracking with Event Cameras

[![Power Line Tracking with Event Cameras](img/thumbnail_github.png)](https://www.youtube.com/watch?v=KnBJqed5qDI)

## Publication
If you use this code in an academic context, please cite the following [IROS 2021 paper](http://rpg.ifi.uzh.ch/docs/IROS21_Dietsche.pdf).

A. Dietsche, G. Cioffi, J. Hidalgo-Carrio and D. Scaramuzza,
"**Power Line Tracking with Event Cameras**,"
IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS). 2021.

```
@InProceedings{Dietsche2021
  author = {Dietsche, Alexander and Cioffi, Giovanni and Hidalgo-Carrio, Javier and Scaramuzza, Davide},
  title = {PowerLine Tracking with Event Cameras},
  booktitle = {IEEE/RSJ Int. Conf. Intell. Robot. Syst. (IROS)},
  year = {2021}
}
```

## Dataset

The dataset can be downloaded [here](https://download.ifi.uzh.ch/rpg/powerline_tracking_dataset/)

## Installation

The code has been tested on Ubuntu 18.04 and ROS Melodic.

1. Create a catkin workspace:
```
mkdir -p line_tracking_with_event_cameras_ws/src
cd line_tracking_with_event_cameras_ws
catkin init
```
2. Add the *dvs_msgs* package, from [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros), to your workspace.
3. Clone this repository and add it to your workspace.
4. Build the tracker:
```
catkin build line_event_tracker 
```
5. If you want to visualize the lines, build the visualization package as well:
```
catkin build line_event_tracker_visualizer
```
6. If you want to visualize event rosbag data, build the `dvs_renderer` as well:
```
catkin build dvs_renderer
```

## Parameter Tuning
The parameters and their description can be found [here](https://github.com/uzh-rpg/line_tracking_using_event_cameras/blob/main/line_event_tracker/param/param.yaml).

## Undistort
The tracker performs much better on undistorted events. To that end, a undistort step was added. The camera parameters need to be defined in the *run.launch* file.
For visualization package needs the camera parameters defined separately in their *run.launch* file.

## Run
To run the tracker:
```
roslaunch line_event_tracker run.launch
```

To run the tracker with the visualization:
```
roslaunch line_event_tracker run_with_visualizer.launch
```

## Power line demo
To run power line demo download the rosbag of flight2 [here](https://download.ifi.uzh.ch/rpg/powerline_tracking_dataset/) and run:
```
roslaunch line_event_tracker run_with_visualizer.launch

rosbag play -s 15 flight2_2020-12-17-17-52-17.bag 
```

In the rqt window select `/line_visualization` in the drop down menu.