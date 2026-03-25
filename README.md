# SEANO Logger Launch Guide
Sebelum pindah device, **jangan lupa ubah path SSD** di `logger_node.py`.

Sesuaikan self.external_mount_point dengan lokasi SSD di device yang dipakai.

## Run SEANO Logger
```bash
cd ~/seano_ws
colcon build --symlink-install --packages-select seano_sensors seano_logger
source install/setup.bash
ros2 launch seano_logger seano_all.launch.py
