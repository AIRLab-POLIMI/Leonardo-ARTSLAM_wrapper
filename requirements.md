sudo apt install libsuitesparse-dev

git clone git@github.com:RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_UNITTESTS=OFF -DG2O_USE_CHOLMOD=ON -DG2O_USE_CSPARSE=ON -DBUILD_WITH_MARCH_NATIVE=ON ..
make
sudo make install

git clone git@github.com:PointCloudLibrary/pcl.git --branch pcl-1.13.1
cd pcl
mkdir build && cd build
cmake ..
make
sudo make install

git clone git@github.com:AIRLab-POLIMI/LOTS-CORE.git
cd LOTS-CORE
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make

git clone git@github.com:SMRT-AIST/fast_gicp.git
cd fast_gicp
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install

git clone git@github.com:borglab/gtsam.git --branch 4.1.1
cd gtsam
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make
sudo make install

git clone git@github.com:AIRLab-POLIMI/LOTS-SLAM.git
cd LOTS-SLAM
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DKITTI_OFFLINE=OFF ..
make

source /opt/ros/humble/setup.bash
sudo apt install ros-humble-geodesy ros-humble-nmea-msgs
mkdir -p artslam_ws/src
cd artslam_ws/src
git clone git@github.com:ros-perception/perception_pcl.git
git clone -b ros2 git@github.com:AIRLab-POLIMI/ARTSLAM_wrapper.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
