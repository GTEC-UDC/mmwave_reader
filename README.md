# README

This repository includes several tools and ROS nodes to read data from TI IWR6843 radar sensor.

- **IWR6843ISK_out_of_box_reader**: reads data through USB port from a IWR6843 (ISK or AOP). The device must be flashed with the lab: [Out of box Demo](https://dev.ti.com/tirex/explore/node?node=APR4NbV00IeYkywyV9UT7g__VLyFKFf__LATEST)
- **IWR6843ISK_people_counting_reader**: reads data through USB port from a IWR6843 (ISK or AOP). The device must be flashed with the lab: [People Counting Demo](https://dev.ti.com/tirex/explore/node?node=A__AD7Cm.UWpaYqqCsnR6Gl-A__com.ti.mmwave_industrial_toolbox__VLyFKFf__LATEST)
- **threshold_filter**: filters the measurements by SNR or Doppler.
- **buffer_filter**: filters the measurements that appear only a brief moment and then dissapear.


The ```launch``` folder contains some launch files to launch the algorithms with some parameters.

This repository is related with the next paper. Please cite us if this code is useful to you.

Barral, V., Dominguez-Bolano, T., Escudero, C. J., & Garcia-Naya, J. A. *An IoT System for Smart Building Combining Multiple mmWave FMCW Radars Applied to People Counting.*