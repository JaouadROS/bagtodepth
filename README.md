# bagtodepth
Get true depth information from bagfile

This package has been tested using:

*  ROS Indigo
*  Ubuntu 14.04

The purpose of this package is to get true depth from bagfile without losing precision

Quick Start:
===============

To use this package, clone the repository to your disc and build it:

    $ cd catkin_ws/src 
    $ git clone https://github.com/JaouadROS/bagtodepth
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash

run bagtodepth node (newdepth is bagfile's name)

    $ rosrun bag_to_depth bagtodepth newdepth

```sh
      [ INFO] [1467017857.694315287]: Opened /home/jros/catkin_ws/src/bag_to_depth/bagfile/newdepth.bag
      bagsize: 154
      Ros message / Frame number: 
```

Knowing the bagsize you can choose one message to extract to depth ```cv::Mat``` image. The depth information will be stored in ```depthclean``` for future processing, and ```depthvisualization``` is provided for visualization purpose
