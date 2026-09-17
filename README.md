# LVTN_2 — USV simulation, control, SLAM and coverage

Đây là file ghi chú duy nhất của workspace. Tài liệu giải thích kiến trúc, logic code, plugin, cách setup, build, chạy và debug để có thể tự ôn lại dự án.

## 1. Mục tiêu

Workspace mô phỏng một USV trong hồ bùn công nghiệp bằng ROS 2 Humble và Gazebo Fortress. Hệ thống gồm:

- mô hình nổi, hydrodynamics, gió và hai LiDAR 2D;
- điều khiển bàn phím hoặc lệnh `[u_d, v_d, psi_d]`;
- đọc odometry và đổi sang quy ước trục mũi tàu;
- ghép scan 360°, xây map bằng `slam_toolbox`;
- tự chạy một vòng bốn cạnh để thu thập map;
- tạo lawnmower/zigzag cách nhau 0,25 m;
- guidance ALOS và PI/PID để bám từng lane;
- playback toàn coverage trong một khoảng thời gian ngắn bằng cách đặt pose trực tiếp.

## 2. Cấu trúc

```text
LVTN_2/
├── README.md
└── src/
    ├── usv_description/   model, world, Gazebo, bridge, RViz
    ├── usv_controller/    state reader và wrench controller
    └── usv_navigation/    SLAM, mapping route, coverage, guidance
```

`build/`, `install/`, `log/` do colcon sinh ra. Chỉ sửa code trong `src/`, sau đó build và restart node.

## 3. Package `usv_description`

Các file chính:

```text
urdf/usv.urdf.xacro    link, joint, mass, inertia, sensor frame
urdf/gazebo.xacro      hydrodynamics, LiDAR, odometry plugin
urdf/environment.sdf  hồ, nước, bờ, địa hình, world plugin
launch/gazebo.launch.py
launch/display.launch.py
rviz/display.rviz
meshes/*.stl
```

`gazebo.launch.py` chuyển Xacro thành URDF, chạy `robot_state_publisher`, mở world `industrial_sludge_pond`, spawn model `usv` tại `(-25,-40,0.2817)` và tạo bridge Gazebo–ROS.

### Bridge

| Gazebo | ROS | Chiều |
|---|---|---|
| `/clock` | `/clock` | GZ → ROS |
| `/model/usv/odometry` | `/odom` | GZ → ROS |
| `/model/usv/pose` | `/tf` | GZ → ROS |
| `/model/usv/joint_state` | `/joint_states` | GZ → ROS |
| `/lidar/front/scan` | cùng tên | GZ → ROS |
| `/lidar/back/scan` | cùng tên | GZ → ROS |
| `/world/industrial_sludge_pond/wrench` | cùng tên | ROS → GZ |
| `/world/industrial_sludge_pond/set_pose` | ROS service | ROS → GZ |

### Plugin Gazebo

World plugin trong `environment.sdf`:

| Plugin | Vai trò |
|---|---|
| Physics | rigid-body physics |
| UserCommands | spawn/xóa/thay đổi entity |
| SceneBroadcaster | gửi scene cho GUI |
| Sensors | GPU LiDAR qua Ogre2 |
| WindEffects | lực gió |
| ApplyLinkWrench | nhận wrench từ ROS |
| Buoyancy | lực nổi theo mật độ chất lỏng |

Plugin/sensor trong `gazebo.xacro`:

| Plugin | Vai trò |
|---|---|
| Hydrodynamics | added mass và damping |
| GPU LiDAR front/back | scan 360°, một lớp ngang |
| OdometryPublisher | odometry của USV |
| JointStatePublisher | trạng thái joint |

World dùng `max_step_size=0.001 s`, `real_time_factor=1.0`. Không tăng real-time factor khi đánh giá PID nếu controller vẫn dùng wall timer.

Hồ vật lý là hình vuông 100 × 100 m, tâm `(0,0)`, bờ tại `x,y=±50 m`. Các vật cản bên trong đã được xóa; còn đáy, bùn nền, nước, bốn bờ và terrain ngoài hồ.

## 4. Package `usv_controller`

### `state_reader.cpp`

Subscribe `/odom`, publish:

```text
/state_value = [x, y, psi, u, v, r]
```

| Biến | Ý nghĩa | Đơn vị |
|---|---|---|
| `x,y` | vị trí trong odom | m |
| `psi` | hướng mũi tàu | rad |
| `u` | surge theo mũi | m/s |
| `v` | sway ngang | m/s |
| `r` | yaw rate | rad/s |

Link gốc lệch 90° so với mũi tàu. Tham số `body_yaw_offset=pi/2` được dùng:

```text
psi = wrap(link_yaw + pi/2)
[u]   [ cos(a)  sin(a)] [vx_link]
[v] = [-sin(a)  cos(a)] [vy_link]
```

### `thruster_controller.cpp`

Nhận:

```text
/state_value
/trajectory_cmd = [u_d, v_d, psi_d]
```

Xuất `EntityWrench` lên `/world/industrial_sludge_pond/wrench`.

Gain hiện tại:

```text
Surge PI:    Kp=441.348, Ki=117.693
Sway PI:     Kp=500.000, Ki=612.500
Heading PID: Kp=523.152, Ki=74.736, Kd=280.260
```

Sai số heading:

```text
e_psi = atan2(sin(psi_d-psi), cos(psi_d-psi))
```

Giới hạn:

```text
max_force=10000 N
max_yaw_torque=5000 N.m
```

`ApplyLinkWrench` chỉ giữ wrench trong một physics step. Controller 20 Hz có chu kỳ 50 ms, physics step 1 ms, nên `wrench_step_scale=50` để giữ impulse trung bình.

Không reset integral mỗi lần nhận `/trajectory_cmd`: topic được gửi liên tục 20 Hz; reset liên tục làm PI chỉ còn gần như P và tạo sai số tốc độ tĩnh.

### Bàn phím

| Phím | Lệnh |
|---|---|
| W/S | tiến/lùi |
| A/D | sway trái/phải |
| Q/E | quay trái/phải |

Launch bàn phím mở `gnome-terminal` vì stdin của node chạy trực tiếp bằng `ros2 launch` là pipe.

## 5. Package `usv_navigation`

| Executable | Vai trò |
|---|---|
| `laser_scan_merger` | ghép hai scan thành `/lidar/merged_scan` |
| `mapping_route` | chạy vòng bốn cạnh để xây map |
| `coverage_planner` | tạo `/coverage_path` |
| `trajectory_node` | bám lane, phát `/trajectory_cmd` |
| `coverage_playback` | tua nhanh path bằng SetEntityPose |

### `laser_scan_merger.cpp`

Node tra TF từ `lidar_front` và `lidar_back` sang `usv_body`, đổi range thành điểm Cartesian, đưa về bin góc và giữ range nhỏ nhất.

```text
input:   /lidar/front/scan, /lidar/back/scan
output:  /lidar/merged_scan
frame:   usv_body
angle:   -pi → pi
samples: 2880
timer:   20 Hz
```

Hai LiDAR chỉ dùng một lớp đứng góc 0°. Nhiều lớp đứng có thể làm bridge LaserScan lấy nhầm lớp nhìn xuống nước hoặc thân tàu.

### `slam.yaml`

```text
map_frame: map
odom_frame: odom
base_frame: usv_body
scan_topic: /lidar/merged_scan
resolution: 0.05 m/pixel
mode: mapping
```

`slam_toolbox` phát `/map` và TF `map→odom`. Nếu transform này drift, Gazebo vẫn đúng theo world/odom nhưng RViz dùng `map` sẽ nhìn lệch.

### `mapping_route.cpp`

Node đọc `/state_value`, phát `/trajectory_cmd`, chạy vòng:

```text
(45,-45) → (-45,-45) → (-45,45) → (45,45) → quay lại điểm đầu
```

Phải quay lại điểm đầu để đủ bốn cạnh. Tham số:

```text
speed=0.8 m/s
arrival_radius=2.0 m
route_limit=45.0 m
```

### `coverage_planner.cpp`

Planner đợi `/map` báo hệ thống sẵn sàng rồi tạo lawnmower hình vuông trong `odom`:

```text
x,y:               -47 → 47 m
track_spacing:      0.25 m
pond margin:        3 m
lanes:              377
pose mỗi lane:      2 endpoint
tổng pose:          754
```

Chỉ lưu hai endpoint mỗi lane. Nếu lấy mẫu 0,25 m dọc mọi lane, path có hơn 140.000 pose và DDS/RViz có thể nghẽn. Khoảng cách 0,25 m cần giữ giữa lane, không phải mật độ điểm trên đoạn thẳng.

Path tĩnh dùng timestamp 0 để RViz lấy TF mới nhất, tránh bị bỏ khi `map→odom` xuất hiện sau path.

Khung PGM hình chữ nhật không phải biên hồ. `map_saver` cắt ảnh theo phạm vi cell đã quan sát. Không dùng width/height PGM làm biên điều khiển vật lý.

### `trajectory_node.cpp`

Node đọc `/coverage_path` và `/state_value`. Các điểm liên tiếp có cùng x được gom thành lane. Nếu path không ở `odom`, TF được dùng để đổi waypoint sang `odom` trước khi trừ với state.

State machine:

```text
WAIT_PATH → GO_TO_START → ALIGN → FOLLOW_LANE
          → SHIFT_LANE → FOLLOW_LANE ... → FINISHED
```

ALOS:

```text
chi_d = chi_path - atan2(e_y, Delta) - beta_hat
beta_hat_dot = gamma * e_y
```

Đổi hướng vận tốc thế giới sang body:

```text
relative = chi_d - psi
u_d = U*cos(relative)
v_d = U*sin(relative)
```

`psi_ref` giữ theo lane đầu. Ở lane ngược chiều, `u_d` có thể âm để chạy lùi thay vì quay thân 180°.

Tham số chính:

```text
speed source default=0.30 m/s
coverage.launch default=1.0 m/s
go_start_speed mặc định bằng speed
shift_speed mặc định bằng speed
alos_lookahead=3.0 m
alos_gamma=0.015
beta_max_deg=35
start_acceptance=0.30 m
lane_acceptance=0.30 m
shift_acceptance=0.08 m
heading_acceptance_deg=5
```

Với lane 0,25 m nên bắt đầu bằng `speed:=0.5`, không dùng 5–10 m/s để đánh giá bám đường.

### `coverage_playback.cpp`

Playback không dùng PID/wrench. Nó tính tổng chiều dài path, nội suy vị trí theo thời gian và gọi `SetEntityPose` mỗi 50 ms. Default `duration=60 s`.

Đây là chế độ trình diễn nhanh, không dùng để đánh giá động lực học hoặc controller.

## 6. Luồng dữ liệu

### Điều khiển

```text
Gazebo /odom
  → state_reader
  → /state_value [x,y,psi,u,v,r]
  → trajectory_node hoặc mapping_route
  → /trajectory_cmd [u_d,v_d,psi_d]
  → thruster_controller
  → /world/industrial_sludge_pond/wrench
  → Gazebo
```

### SLAM

```text
/lidar/front/scan ─┐
                   ├→ laser_scan_merger → /lidar/merged_scan
/lidar/back/scan  ─┘                         ↓
                                         slam_toolbox
                                             ↓
                                         /map + map→odom
```

### Frame

```text
map       frame toàn cục do SLAM hiệu chỉnh
odom      frame liên tục của Gazebo odometry
usv_body  link gốc của USV
lidar_front, lidar_back
```

`coverage.rviz` dùng Fixed Frame `odom` để robot và quỹ đạo khớp Gazebo. Map có thể xoay nếu SLAM drift.

## 7. Setup

Yêu cầu:

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Fortress / ros_gz
slam_toolbox
nav2_map_server
RViz2
colcon
```

Cài dependency chính:

```bash
sudo apt update
sudo apt install \
  ros-humble-ros-gz \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions
```

Mỗi terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/LVTN_2
source install/setup.bash
```

Có thể thêm vào `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/LVTN_2/install/setup.bash
```

## 8. Build

```bash
cd ~/LVTN_2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build riêng:

```bash
colcon build --symlink-install --packages-select usv_navigation
```

Sau khi sửa C++, phải restart node; source lại không thay executable đã nạp trong process cũ.

## 9. Cách chạy

### Xem URDF

```bash
ros2 launch usv_description display.launch.py
```

### Điều khiển bàn phím

Ba terminal:

```bash
ros2 launch usv_description gazebo.launch.py
```

```bash
ros2 launch usv_controller state.launch.py
```

```bash
ros2 launch usv_controller controller.launch.py keyboard_control:=true
```

### Xây map tự động

Năm terminal:

```bash
ros2 launch usv_description gazebo.launch.py
```

```bash
ros2 launch usv_controller state.launch.py
```

```bash
ros2 launch usv_controller controller.launch.py keyboard_control:=false
```

```bash
ros2 launch usv_navigation slam.launch.py
```

```bash
ros2 run usv_navigation mapping_route --ros-args \
  -p use_sim_time:=true -p speed:=0.8 -p arrival_radius:=0.5
```

Khi báo `Mapping lap complete`, lưu map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/LVTN_2/src/usv_navigation/maps/usv_map
```

### Xem map lưu và zigzag

Không chạy cùng `slam.launch.py`:

```bash
ros2 launch usv_navigation saved_coverage.launch.py track_spacing:=0.25
```

Map lưu được remap sang `/saved_map`, tránh tranh `/map` với SLAM.

### Bám quỹ đạo vật lý

Sau khi Gazebo, state reader, controller và SLAM đã chạy:

```bash
ros2 launch usv_navigation coverage.launch.py \
  execute:=true track_spacing:=0.25 speed:=0.5
```

Không chạy `mapping_route` cùng lúc vì cả hai publish `/trajectory_cmd`.

### Playback 60 giây

Ba terminal:

```bash
ros2 launch usv_description gazebo.launch.py
```

```bash
ros2 launch usv_navigation saved_coverage.launch.py
```

```bash
ros2 run usv_navigation coverage_playback --ros-args -p duration:=60.0
```

Không chạy controller vật lý trong playback.

## 10. Debug

```bash
ros2 node list
ros2 topic list
ros2 topic info /map -v
ros2 topic info /trajectory_cmd -v
ros2 topic echo /trajectory_cmd --once
ros2 topic echo /state_value --once
ros2 topic echo /coverage_path --once --field header
ros2 topic hz /lidar/merged_scan
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom usv_body
```

Mỗi chế độ chỉ nên có một publisher `/map` và một publisher `/trajectory_cmd`.

| Hiện tượng | Nguyên nhân thường gặp |
|---|---|
| Không đọc phím | node không có terminal thật |
| `u_d=0.5`, `u≈0.2` mãi | integral bị reset hoặc lực giới hạn |
| Gazebo đúng, RViz lệch | `map→odom` của SLAM drift |
| Path không hiện | TF/timestamp/QoS sai hoặc message quá lớn |
| USV chạy vào bờ | dùng kích thước PGM làm biên vật lý |
| Map méo, nhiều hình quạt | scan/TF sai hoặc scan matching drift |
| SHM `open_and_lock_file failed` | Fast DDS SHM cũ hoặc process trùng |
| Nhiều `/map` | map_server và slam_toolbox chạy cùng lúc |

## 11. Thứ tự đọc source để ôn

1. `environment.sdf`: world và plugin.
2. `usv.urdf.xacro`: link, joint, trục, inertia.
3. `gazebo.xacro`: hydrodynamics, LiDAR, odometry.
4. `gazebo.launch.py`: bridge Gazebo–ROS.
5. `state_reader.cpp`: vector state và xoay trục.
6. `thruster_controller.cpp`: PI/PID và wrench.
7. `laser_scan_merger.cpp`, `slam.yaml`: mapping.
8. `mapping_route.cpp`: tuyến xây map.
9. `coverage_planner.cpp`: tạo lane.
10. `trajectory_node.cpp`: state machine và ALOS.
11. `coverage_playback.cpp`: tua nhanh.

Luôn tách ba lớp khi debug:

```text
reference: quỹ đạo và lệnh mong muốn
control:   PI/PID biến sai số thành wrench
plant:     Gazebo, hydrodynamics, buoyancy, sensor
```

Lỗi hiển thị RViz không nhất thiết là lỗi controller; map đẹp cũng không đảm bảo TF đúng. Luôn kiểm tra topic, frame và timestamp trước khi chỉnh gain.
