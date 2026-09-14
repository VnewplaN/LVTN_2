# Bản nháp controller cho USV

Phần này giả sử USV đã được spawn vào Gazebo Sim và `robot_state_publisher`
đã cung cấp parameter `robot_description`.

## 1. Khai báo ros2_control trong Xacro

Tạo `usv_description/urdf/ros2_control.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="usv">
  <ros2_control name="UsvSystem" type="system">
    <hardware>
      <!-- ROS 2 Humble + Gazebo Fortress -->
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <joint name="usv_body_to_pump">
      <!-- Effort phù hợp với bơm thẳng đứng có khối lượng 145 kg. -->
      <command_interface name="effort">
        <param name="min">-5000.0</param>
        <param name="max">5000.0</param>
      </command_interface>

      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```

Include file này trong Xacro chính:

```xml
<xacro:include filename="$(find usv_description)/urdf/ros2_control.xacro"/>
```

Tên `usv_body_to_pump` phải giống tuyệt đối với tên joint trong URDF.

## 2. Nạp plugin ros2_control vào Gazebo

Tạo `usv_description/urdf/gazebo.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="usv">
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <parameters>$(find usv_controller)/config/config.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

Include trong Xacro chính:

```xml
<xacro:include filename="$(find usv_description)/urdf/gazebo.xacro"/>
```

Plugin này tạo `/controller_manager` bên trong Gazebo. Vì vậy không chạy thêm
`ros2_control_node` độc lập.

## 3. Cấu hình controller

Tạo `usv_controller/config/config.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    pump_controller:
      type: joint_trajectory_controller/JointTrajectoryController

pump_controller:
  ros__parameters:
    joints:
      - usv_body_to_pump

    command_interfaces:
      - effort

    state_interfaces:
      - position
      - velocity

    gains:
      usv_body_to_pump:
        p: 3000.0
        i: 500.0
        d: 1000.0
        i_clamp: 2000.0
```

PID nhận sai số vị trí và tạo lực cho khớp. Giá trị âm kéo bơm lên; giá trị
dương hạ bơm theo axis `0 0 -1`.

## 4. Bật controller sau khi spawn

Sau action spawn robot trong launch file, tạo hai spawner:

```python
joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_state_broadcaster",
        "--controller-manager",
        "/controller_manager",
    ],
    output="screen",
)

pump_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "pump_controller",
        "--controller-manager",
        "/controller_manager",
    ],
    output="screen",
)
```

Để chỉ bật chúng sau khi node spawn kết thúc:

```python
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

start_controllers_after_spawn = RegisterEventHandler(
    OnProcessExit(
        target_action=gz_spawn_robot,
        on_exit=[
            joint_state_broadcaster_spawner,
            pump_controller_spawner,
        ],
    )
)
```

Thêm `start_controllers_after_spawn` vào `LaunchDescription`.

## 5. Dependency cần thiết

Trong `usv_controller/package.xml`:

```xml
<exec_depend>controller_manager</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>joint_trajectory_controller</exec_depend>
<exec_depend>gz_ros2_control</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

Đảm bảo CMake cài thư mục cấu hình và launch:

```cmake
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
```

## 6. Build và kiểm tra

```bash
cd ~/LVTN_2
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_description usv_controller --symlink-install
source install/setup.bash
```

Sau khi launch Gazebo:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Kết quả cần có:

```text
joint_state_broadcaster  active
pump_controller          active
usv_body_to_pump/effort  claimed
```

## 7. Gửi mục tiêu cho bơm

Hạ xuống 2 m trong 5 giây:

```bash
ros2 topic pub --once \
/pump_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['usv_body_to_pump'],
  points: [{positions: [2.0], time_from_start: {sec: 5}}]}"
```

Thu về vị trí 0:

```bash
ros2 topic pub --once \
/pump_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['usv_body_to_pump'],
  points: [{positions: [0.0], time_from_start: {sec: 5}}]}"
```

Theo dõi vị trí:

```bash
ros2 topic echo /joint_states
```

Không cần publish bằng `-r`. `JointTrajectoryController` tiếp tục thực hiện và
giữ mục tiêu sau khi nhận một message hợp lệ.

