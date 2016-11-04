OpenChisel
==========

An open-source version of the Chisel chunked TSDF library. It contains two packages:

##open_chisel
`open_chisel` is an implementation of a generic truncated signed distance field ([TSDF](https://graphics.stanford.edu/papers/volrange/volrange.pdf)) 3D mapping library; based on the Chisel mapping framework developed originally for Google's [Project Tango](https://www.google.com/atap/project-tango/). It is a complete re-write of the original mapping system (which is proprietary). `open_chisel` is chunked and spatially hashed [inspired by this work from Neissner et. al](http://www.graphics.stanford.edu/~niessner/niessner2013hashing.html), making it more memory-efficient than fixed-grid mapping approaches, and more performant than octree-based approaches. A technical description of how it works can be found in our [RSS 2015 paper](http://www.roboticsproceedings.org/rss11/p40.pdf).

This reference implementation does not include any pose estimation. Therefore **the pose of the sensor must be provided from an external source**. This implementation also *avoids the use of any GPU computing*, which makes it suitable for limited hardware platforms. It does not contain any system for rendering/displaying the resulting 3D reconstruction. It has been tested on Ubuntu 14.04 in Linux with ROS hydro/indigo.

### API Usage
Check the `chisel_ros` package source for an example of how to use the API. The `ChiselServer` class makes use of the `chisel_ros` API.

###Dependencies
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* C++11
* [catkin](http://wiki.ros.org/catkin) build system

Compilation note:
For speed, it is essential to compile `open_chisel` with optimization. You will need to add the flag `-DCMAKE_BUILD_TYPE=Release` to your `catkin_make` command when building.

##chisel_ros
`chisel_ros` is a wrapper around `open_chisel` that interfaces with ROS-based depth and color sensors. The main class `chisel_ros` provides is `ChiselServer`, which subscribes to depth images, color images, TF frames, and camera intrinsics.

Note: you will also need to get the messages package, [chisel_msgs](https://github.com/personalrobotics/chisel_msgs) to build this.

###Supported ROS image types:
**Depth Images**
* 32 bit floating point mono in meters (`32FC1`)
* 16 bit unsigned characters in millimeters (`16UC1`)

**Color Images**
* `BRG8`
* `BGRA8`
* `Mono8`

###Dependencies
* Eigen
* C++11
* catkin (`ros-hydro` or `ros-indigo` or higher)
* [PCL 1.8](http://pointclouds.org/) compiled with stdC++11 enabled.
* ROS OpenCV [cv_bridge](http://wiki.ros.org/cv_bridge)

### A note on PCL
Unfortunately, PCL 1.7x (the standard PCL included in current versions of ROS) doesn't work with C++11. This project makes  use of C++11, so in order to use Chisel, you will have to download and install PCL 1.8 from source, and compile it with C++11 enabled.

1. Download PCL 1.8 from here: https://github.com/PointCloudLibrary/pcl
2. Modify line 91 of `CMakeLists.txt` in PCL to say `SET(CMAKE_CXX_FLAGS "-Wall -std=c++11 ...`
3. Build and install PCL 1.8
4. Download `pcl_ros` from here: https://github.com/ros-perception/perception_pcl
5. Change the dependency from `PCL` to `PCL 1.8` in `find_package` of the `CMakeLists.txt` 
6. Compile `pcl_ros`.
4. Rebuild Chisel

If PCL does not gain `c++11` support by default soon, we may just get rid of `c++11` in `OpenChisel` and use `boost` instead.

###Launching chisel_ros Server

Once built, the `chisel_ros` server can be launched by using a launch file. There's an example launch file located at `chisel_ros/launch/launch_kinect_local.launch`. Modify the parameters as necessary to connect to your camera and TF frame.
```XML
<launch>
    <!-- Use a different machine name to connect to a different ROS server-->
    <machine name="local" address="localhost" default="true"/>
    <!-- The chisel server node-->
    <node name="Chisel" pkg="chisel_ros" type="ChiselNode" output="screen"> 
        <!-- Size of the TSDF chunks in number of voxels -->
        <param name="chunk_size_x" value="16"/>
        <param name="chunk_size_y" value="16"/>
        <param name="chunk_size_z" value="16"/>
        <!--- The distance away from the surface (in cm) we are willing to reconstuct -->
        <param name="truncation_scale" value="10.0"/>
        <!-- Whether to use voxel carving. If set to true, space near the sensor will be
             carved away, allowing for moving objects and other inconsistencies to disappear -->
        <param name="use_voxel_carving" value="true"/>
        <!-- When true, the mesh will get colorized by the color image.-->
        <param name="use_color" value="false"/>
        <!-- The distance from the surface (in meters) which will get carved away when
             inconsistencies are detected (see use_voxel_carving)-->
        <param name="carving_dist_m" value="0.05"/>
        <!-- The size of each TSDF voxel in meters-->
        <param name="voxel_resolution_m" value="0.025"/>
        <!-- The maximum distance (in meters) that will be constructed. Use lower values
             for close-up reconstructions and to save on memory. -->
        <param name="far_plane_dist" value="1.5"/>
        <!-- Name of the TF frame corresponding to the fixed (world) frame -->
        <param name="base_transform" value="/base_link"/>
        <!-- Name of the TF frame associated with the depth image. Z points forward, Y down, and X right -->
        <param name="depth_image_transform" value="/camera_depth_optical_frame"/>
        <!-- Name of the TF frame associated with the color image -->
        <param name="color_image_transform" value="/camera_rgb_optical_frame"/>
        <!-- Mode to use for reconstruction. There are two modes: DepthImage and PointCloud.
             Only use PointCloud if no depth image is available. It is *much* slower-->
        <param name="fusion_mode" value="DepthImage"/>
    
        <!-- Name of the depth image to use for reconstruction -->
        <remap from="/depth_image" to="/camera/depth/image"/>
        <!-- Name of the CameraInfo (intrinsic calibration) topic for the depth image. -->
        <remap from="/depth_camera_info" to="/camera/depth/camera_info"/>
        <!-- Name of the color image topic -->
        <remap from="/color_image" to="/camera/color/image"/>
        <!-- Name of the color camera's CameraInfo topic -->
        <remap from="/color_camera_info" to="/camera/color/camera_info"/>
        
        <!-- Name of a point cloud to use for reconstruction. Only use this if no depth image is available -->
        <remap from="/camera/depth_registered/points" to="/camera/depth/points"/>
    </node>
</launch>
```
Then, launch the server using `roslaunch chisel_ros <your_launchfile>.launch`. You should see an output saying that `open_chisel` received depth images. Now, you can visualize the results in `rviz`. 

Type `rosrun rviz rviz` to open up the RVIZ visualizer. Then, add a `Marker` topic with the name `/Chisel/full_mesh`. This topic displays the mesh reconstructed by Chisel.

### Services
`chisel_ros` provides several ROS services you can use to interface with the reconstruction in real-time. These are:

* `Reset` -- Deletes all the TSDF data and starts the reconstruction from scratch.
* `TogglePaused` -- Pauses/Unpauses reconstruction
* `SaveMesh` -- Saves a `PLY` mesh file to the desired location of the entire scene
* `GetAllChunks` -- Returns a list of all of the voxel data in the scene. Each chunk is stored as a seperate entity with its data stored in a byte array.
