# Hawkeye

Hawkeye - Localization with LiDAR, GNSS/INS and ortho image map

[Demo Video](https://youtu.be/cXr0e-J7uIU)

[![](http://img.youtube.com/vi/cXr0e-J7uIU/0.jpg)](https://youtu.be/cXr0e-J7uIU "Hawkeye")

[Demo Video (with Ground Segmentation)](https://youtu.be/HDGBGOr2-rA)

[![](http://img.youtube.com/vi/HDGBGOr2-rA/0.jpg)](https://youtu.be/HDGBGOr2-rA "Hawkeye with Ground Segmentation")

## What is Hawkeye
This is a limited re-implementation of [this method](https://doi.org/10.11351/jsaeronbun.51.824). This estimates the pose with LiDAR, [Eagleye](https://github.com/MapIV/eagleye.git) and ortho image map ***without radar sensor***. This implementation is extended to accommodate cases where GNSS/INS(Eagleye) errors accumulate for some reason.

### Method
This method corrects GNSS/INS position by maching the reflection intensity between the ground and a pre-made map.
Only the x and y coordinates are corrected. The roll, the pitch and the yaw of GNSS/INS is not corrected and the z coordinate is ignored.
The paper also uses Millimeter Wave Radar and the map of it as same as reflection intensity by LiDAR, but it is not implemented.

* A histogram filter centered on the GNSS/INSS position is used. The GNSS/INS pose is corrected by the center of gravity in the histogram.
* A map of the ground reflection intensity around the vehicle is generated from the accumulated LiDAR observations.
* This filter and map are of the same resolution and orientation as the pre-made map, independent of the vehicle orientation.
* The histogram is updated by the results of template matching between the runtime map and the pre-made map.
* Please see [this paper](https://doi.org/10.11351/jsaeronbun.51.824) for details.

As an extension, the center of the histogram optionally shifts when the estimated position is far from the center of the histogram. The histogram points newly made at the center shift are initialized by the nearby point. This can be invalid by setting the parameter `CENTER_SHIFT_THRESHOLD` or `center_shift_threshold` zero.

## Recommended Environment
Ubuntu 18.04 - with ROS melodic

## Sample
You can download the following sample files from [here](https://www.dropbox.com/sh/rfx1ef0ag2v3uf8/AADYxuWFQE80wF7IkfpltA6ea?dl=0). Download all files and unzip ortho_image.zip.

*This sample does not perform ground segmentation of LiDAR point cloud. To increase accuracy, place a ground segmentation node (not implemented in this repository) between the LiDAR driver and Hawkeye.*

* Clone this repository.
```
mkdir -p hawkeye_ws/src
cd hawkeye_ws/src
git clone https://github.com/MapIV/hawkeye.git
```

* Clone [Eagleye](https://github.com/MapIV/eagleye.git) and checkout to [the specified commit](https://github.com/MapIV/eagleye/tree/e40555433b219d3f7df5e502c69ec04b54bcfc62) with the following command.
When `sample.launch` is not used, `main_ros1` branch can be used instead.
```
git clone --recursive https://github.com/MapIV/eagleye.git
cd eagleye
git checkout e40555433b219d3f7df5e502c69ec04b54bcfc62
git submodule update
cd ..
git clone https://github.com/MapIV/rtklib_ros_bridge.git
git clone https://github.com/MapIV/nmea_comms.git
git clone https://github.com/MapIV/nmea_ros_bridge.git
```

* Install MapIV's fork of RTKLIB by following [the README of Eagleye](https://github.com/MapIV/eagleye/blob/e40555433b219d3f7df5e502c69ec04b54bcfc62/README.md#rtklib).

* Clone [Hesai LiDAR driver](https://gitlab.com/perceptionengine/pe-drivers/hesai_lidar.git) and checkout to [the specified commit](https://gitlab.com/perceptionengine/pe-drivers/hesai_lidar/tree/c111cc2276e98c86f5e20c59ae75ae73bc64f004) with the following command.
When `sample.launch` is not used, `master` branch can be used instead.
```
git clone https://gitlab.com/perceptionengine/pe-drivers/hesai_lidar.git
cd hesai_lidar
git checkout c111cc2276e98c86f5e20c59ae75ae73bc64f004
cd ..
```

* Download ROS dependencies and build.
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```

* Launch Roscore.
```
roscore
```

* Play Rosbag file.
```
cd <SAMPLE_DIRECTORY>
rosparam set use_sim_time true
rosbag play --clock rosbag/2021-05-18-13-37-24_0.bag
```

* Launch sample.launch.
```
roslaunch hawkeye sample.launch sample_dir:="<SAMPLE_DIRECTORY>"
```

## Realtime Hawkeye Node

### Usage
```
$ rosrun hawkeye hawkeye_rt <ORTHOMAP_INFO> <YAML_FILE> <LIDAR_TOPIC_NAME> \
        [-e <ERROR_RATE>] \
        [-d <COEFF_DIMIMISH>] \
        [-w <COEFF_WEIGHT>] \
        [-n <COEFF_NEGATIVE>] \
        [-g <COEFF_GAIN>] \
        [-a <ACCUMULATION_COUNT>] \
        [-A <ACCUMULATION_LENGTH>] \
        [-s <CENTER_SHIFT_THRESHOLD>] \
        [-c|-C] \
        [-o]
```

This works with eagleye_rt.launch and eagleye_fix2pose.launch. `/eagleye/pose` and `/eagleye/fix` are subscribed.

### Arguments and Parameters
* `ORTHOMAP_INFO` : An orthomap file. For more information, See [README of orthomap_viewer](./orthomap_viewer/README.md).
* `YAML_FILE` : A configuration YAML file. See [this example](./hawkeye/config/hawkeye.yaml).
* `LIDAR_TOPIC_NAME` : A topic name for LiDAR data. The topic type must be sensor_msgs/PointCloud2. The points should be classified by whether they are ground or not and be only the points of ground. If they are not classified, it may be less accuracy.
* Some optional parameters overwrite the corresponding configurations in `YAML_FILE`. The explanations of the parameters are on [the expample yaml file](./hawkeye/config/hawkeye.yaml).
  * `ERROR_RATE` and `error_rate`: $\alpha$ in the paper
  * `COEFF_DIMIMISH` and `coeff_diminish`: $\beta$ in the paper
  * `COEFF_WEIGHT` and `coeff_weight`: $k$ in the paper
  * `COEFF_NEGATIVE` and `coeff_negative`: $b$ in the paper
  * `COEFF_GAIN` and `coeff_gain`: $g$ in the paper
  * `ACCUMULATION_COUNT` and `lidar_accumulate/max_count`: The threshold count of LiDAR accumulation
  * `ACCUMULATION_LENGTH` and `lidar_accumulate/max_length`: The threshold length of LiDAR accumulation
  * `CENTER_SHIFT_THRESHOLD` and `center_shift_threshold`: The threshold distance for the shift of the histogram center
  * `hawkeye/edge_copy_shift` can be overwritten by `-c` (true) or `-C` (false): If true, The histogram points newly made are initialized by the nearby point. Otherwise, those are initialized by zero.
* Small overhead `-o` : This option stops publishing some topics for visualization.

### Output
* Console
  * Estimated pose relative to the center of the map
    * x, y and yaw
  * Elapsed time for each step
    * The whole step time and the mean are also calculated. `Essential` time means the time without I/O and process for visualization.
* Topic for RVIZ
  * Static TF by tf2
    * `local_map` from `map`: The center of the map
  * TF by tf2 and trajectory(nav_msgs/Path)
    * `raw_tf` and `raw_path` from `local_map`: The Eagleye pose synchronized to the estimated pose
    * `estimated` and `estimated_path` from `local_map`: The estimated pose
    * `histogram_center` and `histogram_center_path` from `local_map`: The center of the histogram
    * `histogram_peak` and `histogram_peak_path` from `local_map`: The average pose of the histogram
    * `match_peak` and `match_peak_path` from `local_map`: The average pose of the match result
  * `ortho_map`(nav_msgs/OccupancyGrid): Grayscale map around the estimated pose
    * This is not published very often but takes much time to publish. The time increases and the frequency decrease with the size of each ortho image.
  * `point_cloud`(sensor_msgs/PointCloud2): LiDAR point cloud
    * Only the points of the last LiDAR data.
  * `histogram`(visualization_msgs/MarkerArray): The histogram filter
    * It is gradated from red to green.
    * Points not used for average estimation are darker than used ones.
  * Grayscale images(sensor_msgs/Image)
    * `template_image`
    * `submap_image`
    * `match_image`: The result of the maching between the above two images.

## Realtime Hawkeye Node with Weighted Histogram
Another extention for the method. Only the initialization of the shifted histogram is different.
In this implementation,
The histogram points newly made at the center shift are initialized by zero and
the histogram is weighted by the accumulated time.

### Usage
```
$ rosrun hawkeye hawkeye_rt_ws <ORTHOMAP_INFO> <YAML_FILE> <LIDAR_TOPIC_NAME> \
        [-e <ERROR_RATE>] \
        [-d <COEFF_DIMIMISH>] \
        [-w <COEFF_WEIGHT>] \
        [-n <COEFF_NEGATIVE>] \
        [-g <COEFF_GAIN>] \
        [-a <ACCUMULATION_COUNT>] \
        [-A <ACCUMULATION_LENGTH>] \
        [-s <CENTER_SHIFT_THRESHOLD>] \
        [-H <HISTOGRAM_WEIGHT>] \
        [-o]
```

### Arguments and Parameters
`hawkeye/edge_copy_shift` is invalid and `HISTOGRAM_WEIGHT` and `histogram_weight` are valid. This parameter means how the newly created histograms are weighted by the accumulated time. The detail is on [the expample yaml file](./hawkeye/config/hawkeye.yaml).

### Output
A grayscale image `weight_image`(sensor_msgs/Image) is puplished additionaly. This image shows the accumulated time.

## Research Papers
1. D Hirano, K Yoneda, R Yanase, A Mohammad, N Suganuma, "LiDAR and Radar Sensor Fusion for Localizing Autonomous Vehicles", Transactions of Society of Automotive Engineers of Japan 51(5) 824-829, 2020 [Link](https://www.jstage.jst.go.jp/article/jsaeronbun/51/5/51_20204428/_article/-char/en)

## License
Hawkeye is provided under the [BSD 3-Clause](./LICENSE) License.

## Contacts
If you have further question, email to map4@tier4.jp.