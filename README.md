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
如果没有匹配成功可以通过按键移动点云，从而获取初值
按键包括：
* Left，Right（x轴平移）
* Up，Down（y轴平移）
* n,m（z轴平移）
* k，l（绕z轴旋转）
* a（增加每次平移的距离）d（减少每次平移的距离）

2. 有初值
```
l2c_icp source.csv target.csv leaf_size height_limit initial_transformation_matrix_path
```
#### 计算误差
```
computer_error <transformation_matrix_file1> <transformation_matrix_file2>
```