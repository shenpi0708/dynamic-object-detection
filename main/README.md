# 操作步驟

## Network config

- IP and Make
  - IP: 192.168.1.77
  - MakeL: 255.255.255.0

- Restart Network
- check VLP-16 3D LiDAR connect
  - Ping

    ```shell
    $ ping 192.168.1.201
    ```

  - web browser
    <http://192.168.1.201/>

## Reality

- Test VLP-16 3D LiDAR
  - Open VLP-16 3D LiDAR

    ```shell
    $ roslaunch velodyne_pointcloud VLP16_points.launch
    ```

  - Run rviz look VLP-16 3D LiDAR and gui

    ```shell
    $ roslaunch dynamic_object_detection view.launch
    ```

  - Run VLP-16 3D LiDAR

      ```shell
      $ rosrun dynamic_object_detection method
      ```

### Reference

- [ubuntu-16.04 ROS-kinetic 下 VLP-16 激光雷達測試詳細教程](https://blog.csdn.net/weixin_44387339/article/details/110350326)
# 編譯 catkin_make -j1 -l1

################################################################

參考資料

SLAM之鐳射雷達Velodyne vlp-16使用
https://www.itread01.com/content/1547352842.htmla

velodyne Tutorials
http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16

AI葵 yuotube
https://www.youtube.com/channel/UC7UlsMUu_gIgpqNGB4SqSwQ

工廠 Gazebo github
https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

################################################################

下載 import

run my python file
    pip install pandas

    pip install scikit-learn

    pip install fuzzy-c-means

    pip install cython

run my cpp file
    sudo apt-get install libpcap0.8-dev

python讀中文字
# -*- coding: UTF-8 -*-






export XDG_RUNTIME_DIR=/path/to/runtime/directory
sudo rosdep init
rosdep update