### 库安装
```
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make -j
sudo make install

```


### 编译
```
mkdir build && cd build
cmake ..
make -j

cd ../catkin_ws
catkin_make
```