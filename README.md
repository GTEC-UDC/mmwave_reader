# README

This repository includes several tools and ROS nodes to read data from TI IWR6843 radar sensor.

- **IWR6843ISK_out_of_box_reader**: reads data through USB port from a IWR6843 (ISK or AOP). The device must be flashed with the lab: [Out of box Demo](https://dev.ti.com/tirex/explore/node?node=APR4NbV00IeYkywyV9UT7g__VLyFKFf__LATEST)
- **IWR6843ISK_people_counting_reader**: reads data through USB port from a IWR6843 (ISK or AOP). The device must be flashed with the lab: [People Counting Demo](https://dev.ti.com/tirex/explore/node?node=A__AD7Cm.UWpaYqqCsnR6Gl-A__com.ti.mmwave_industrial_toolbox__VLyFKFf__LATEST)
- **threshold_filter**: filters the measurements by SNR or Doppler.
- **buffer_filter**: filters the measurements that appear only a brief moment and then dissapear.


The ```launch``` folder contains some launch files to launch the algorithms with some parameters.

## ROS2 Launch Commands

For ROS2, the following launch files are available:

### People Counting Launch Files

- **ISK_people_counting.launch.py**: Launches the people counting reader for IWR6843ISK radars
  ```bash
  ros2 launch gtec_mmwave_reader ISK_people_counting.launch.py [arguments]
  ```

- **AOP_people_counting.launch.py**: Launches the people counting reader for IWR6843AOP radars
  ```bash
  ros2 launch gtec_mmwave_reader AOP_people_counting.launch.py [arguments]
  ```

### Available Launch Arguments

Both people counting launch files support the following arguments:

- `uart_port`: UART port of the radar (default: `/dev/ttyUSB0` for ISK, `/dev/ttyACM0` for AOP)
- `data_port`: DATA port of the radar (default: `/dev/ttyUSB1` for ISK, `/dev/ttyACM1` for AOP)
- `config_file_path`: Path to the radar configuration file
- `radar_id`: Identifier for the radar (default: `isk` or `aop`)
- `radar_pos_x`, `radar_pos_y`, `radar_pos_z`: Radar position coordinates (default: 0.0, 0.0, 1.0)
- `radar_yaw`, `radar_pitch`, `radar_roll`: Radar orientation angles (default: 0.0, 0.0, 0.0)
- `elev_tilt`: Elevation tilt angle (default: 5.0)

### Example Usage

```bash
# Launch ISK radar with custom port and position
ros2 launch gtec_mmwave_reader ISK_people_counting.launch.py uart_port:=/dev/ttyUSB2 data_port:=/dev/ttyUSB3 radar_pos_z:=2.0

# Launch AOP radar with custom configuration
ros2 launch gtec_mmwave_reader AOP_people_counting.launch.py config_file_path:=/path/to/custom.cfg radar_yaw:=1.57
```

### Radar Placement

- **radar_placer.launch.py**: Sets up TF transforms for radar positioning and pose publishing
  ```bash
  ros2 launch gtec_mmwave_reader radar_placer.launch.py radar_id:=isk_0 radar_pos_x:=1.0 radar_pos_y:=2.0
  ```

This repository is related with the next paper. Please cite us if this code is useful to you.

Barral, V., Dominguez-Bolano, T., Escudero, C. J., & Garcia-Naya, J. A. *An IoT System for Smart Building Combining Multiple mmWave FMCW Radars Applied to People Counting.*