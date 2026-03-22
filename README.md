# BRT7K
Mobileroboticprojet 2

## Installation

Pull repository with VSCode or terminal. 

```bash
git clone https://github.com/Oderlesspanic/BRT7K.git
```
### Bulid

```bash
cd BRT7K
docker compose build
```

### Usage

```bash
docker compose run --rm ros2-dev
```
and 

```bash
colcon build
source install/setup.bash
```

### Navigation

linux like with 
```bash
cd /ros2_ws/src
```
### Create ROS2 C++ package
```bash
ros2 pkg create --build-type ament_cmake <PACKAGE_NAME>
```
### Create ROS2 python package
```bash
ros2 pkg create --build-type ament_python <PACKAGE_NAME>
```