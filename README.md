# Semantic mapping
Semantic mapping pipeline using Realsense D435i on a Stretch Robot. The scene graph is generated using Hydra and are kept up-to-date using the object_processor. 

## Installation Instructions

The system has been tested on Ubuntu 20.04 and ROS Noetic. Please make sure ROS is installed before continuing. Then, make sure you have some general requirements
```bash
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool
```

If you haven't setup rosdep yet:
```bash
sudo rosdep init
rosdep update
```

## Building the workspace
SparseInst requires detectron2 to be setup. We use custom detectron2 to generate a scene graph. Use https://github.com/aswingururaj/detectron2.git to install the custom version.

```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF \
              -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --skiplist hdf5_map_io mesh_msgs_hdf5 label_manager mesh_tools \
                         rviz_map_plugin minkindr_python

cd src
git clone https://github.com/aswingururaj/semantic_mapping.git
git submodule update --recursive

rosdep install --from-paths . --ignore-src -r -y
sudo apt install libprotobuf-dev protobuf-compiler

catkin build
```

If minkindr_ros installation fails, install directly from the source https://github.com/ethz-asl/minkindr_ros

## Running the pipeline

Record rosbags from the robot using the following command:
```bash
rosbag record -O static_world.bag /camera/aligned_depth_to_color/camera_info /camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /odom /tf /tf_static 
```
Make sure to take multiple bags with one bag that represents the static scene.

Copy the files over to the host system using scp/rsync.

Set use_sim_time to true:
```bash
rosparam set use_sim_time true
```

Run hydra using:
```bash
roslaunch hydra_dsg_builder hydra_realsense_builder.launch
```

Run ORBSLAM using:
```bash
roslaunch object_processor orb_slam2_d435_rgbd.launch
```

Run SparseInst using:
```bash
roslaunch object_processor sparseinst_ros.launch
```

Run object_processor and tf_correction node for stretch using:
```bash
roslaunch object_processor object_processor.launch
```

Run the bags using:
```bash
rosbag play --clock static_scene.bag
```

Running the above commands with static scene will generate a scene graph for static environment. For subsequent runs, this scene graph will be used as a basis to keep track of dynamic objects. Set ```static_mapping``` param to ```true``` in ```object_processor.launch``` for the other runs.