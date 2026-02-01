# 🐕Go2 Robot SLAM & Navigation Guide

```
git clone -b humble https://git.irasc.kro.kr/irasc-lab/go2_humble.git
```
## 🐳Docker
```
cd docker
docker compose up -d --build
```
## 🗺️ 1. SLAM (Mapping)

```bash
ros2 launch go2_robot_sdk robot_mapping.launch.py
```

### rviz display
```
- /scan
- /tf
- /map
```

### save map
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

## 🧭 2. Nav2 (Navigation)
```
ros2 launch go2_robot_sdk navigation.launch.py start_mode:=0
```

### rviz display
```
- /scan
- /tf
- /path
- /map
    - Durability Policy: Transient Local
```
