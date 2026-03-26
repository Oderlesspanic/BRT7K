# BRT7K
## Description
Mobileroboticprojet 2
## Features

## Installation

Pull repository with VSCode or terminal. 

```bash
git clone https://github.com/Oderlesspanic/BRT7K.git
git submodule update --init --recursive
```
### Bulid

```bash
cd BRT7K
docker compose build
```

## Usage

```bash
docker compose run --rm ros2-dev
```
and 

```bash
colcon build
source install/setup.bash
```
or build only one package
```bash
colcon build --packages-select <package_name>
source install/setup.bash
```

or clean build one package
```bash
colcon build --packages-select <package_name> --cmake-clean-cache
source install/setup.bash
```

for second terminal in container use
```bash
docker exec -it <full_container_name> bash
```
### Navigation

linux like with 
```bash
cd /ros2_ws/src
```

### Update submodules
```bash
cd <path_to_submodule>
git pull

git add <path_to_submodule>
git commit -m "update <submodule>"
```

### Create ROS2 C++ package
```bash
ros2 pkg create --build-type ament_cmake <PACKAGE_NAME>
```
### Create ROS2 python package
```bash
ros2 pkg create --build-type ament_python <PACKAGE_NAME>
```

## Work together
Please don't work at main branch.
Create a new branch for every new feature or if you change an existing, working feature.
Don't work at the same branch with an other simultaneous.

### Create feature branch
#### 1. Pull current status
```bash
git checkout main
git pull origin main
```
fetch all existing branches
```bash
git fetch --all
```

#### 2. Check existing branches
```bash
git branch
```
Switch to existing branch
```bash
git switch <branch_name>
```

#### 3. Create a new branch 
for every new feature or if you change an existing, working feature.
VSCode: Source Control -> Changes -> ... -> Branch -> Create Branch -> <meaningful_branch_name>

```bash
git switch -c <meaningful_branch_name>
```
Check changes
```bash
git status
```

### commit changes
#### 1. prepare commit
Add all changed files to commit
```bash 
git add .
```
or 
```bash
git add <filename1.end> <filename2.end>
```
Check for added files
```bash
git status
```

#### 2. commit
```bash
git commit -m "Message"
```
#### 3. push commit
```bash
git push -u origin <branch_name>
```

### Pull request

## Unit test
gtest is included in ROS2 Jazzy

### Add
#### package file
```xml
<test_depend>ament_cmake_gtest</test_depend>
```
#### Cmake file
```Cmake
find_package(ament_cmake_gtest REQUIRED)

if(BUILD_TESTING)

  ament_add_gtest(<test_name>
    <path_to_test_file.cpp>
  )
  if(TARGET <test_name>)
    target_link_libraries(<test_name>
      ${PROJECT_NAME}_lib
    )
    ament_target_dependencies(<test_name>

    )
  endif()
endif()
```

### Usage
```bash
colcon test --packages-select <package_name> --event-handlers console_direct+
colcon test-result --verbose
```

### Test C++ file
```cpp
```