### 介绍
用与进行点云icp和误差评估

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

### 用法
#### 点云配准
1. 无初值
```
l2c_icp source.csv target.csv leaf_size height_limit
```
如果没有匹配成功可以通过按键移动点云，点击退出键后，则会保存初值。从而获取初值
保存的初值文件为：manual_transformation_matrix.txt（通过手动对齐获取的初值）
ndt获取的结果文件为：initial_transformation_matrix.txt
按键包括：
* Left，Right（x轴平移）
* Up，Down（y轴平移）
* n,m（z轴平移）
* k，l（绕z轴旋转）
* 7, 8（绕x轴旋转）
* 9, 0（绕y轴旋转）
* a（增加每次平移的距离）d（减少每次平移的距离）

2. 有初值
```
l2c_icp source.csv target.csv leaf_size height_limit initial_transformation_matrix_path
```

#### 计算误差
```
rosrun occupancy_grid_to_pcd mtvoc_reader_node <bin_file_path> <txt_file_path> <gt_file_path>
```


