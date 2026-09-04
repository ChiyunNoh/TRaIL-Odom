<h1 align="center"><b><em>TRaIL-Odom</em></b>: Tightly Coupled Continuous Time<br/>Radar-IMU-LiDAR Odometry with Adaptive Doppler Weighting</h1>

<p align="center">
  <a href="https://chiyunnoh.github.io/TRaIL-Odom/"><img src="https://shieldcn.dev/badge/Project-Page-gray?size=xs" alt="Project Page" /></a>
  <a href="https://arxiv.org/abs/2609.03561"><img src="https://shieldcn.dev/badge/arXiv-2609.03561-b31b1b?logo=arxiv&amp;size=xs" alt="arXiv" /></a>
  <a href="LICENSE"><img src="https://shieldcn.dev/badge/License-MIT-green?size=xs" alt="License: MIT" /></a>
  <a href="https://www.youtube.com/watch?v=YpFgrUllgts"><img src="https://shieldcn.dev/badge/YouTube-red?logo=youtube&amp;size=xs" alt="YouTube" /></a>
  <a href="https://drive.google.com/drive/folders/1h7RYsnLOgKopE9LnrQ_HXnvU73e8aGJr?usp=drive_link"><img src="https://shieldcn.dev/badge/Dataset-Download-blue?size=xs" alt="Dataset" /></a>
</p>

<p align="center">
  <a href="https://chiyunnoh.github.io/"><strong>Chiyun Noh<sup>1</sup></strong></a>
  ·
  <a href="https://www.turcantuna.com/"><strong>Turcan Tuna<sup>2</sup></strong></a>
  ·
  <a href="https://scholar.google.com/citations?user=ODqL7LoAAAAJ&hl=en"><strong>William Talbot<sup>2</sup></strong></a>
  ·
  <a href="https://rsl.ethz.ch/"><strong>Marco Hutter<sup>2</sup></strong></a>
  ·
  <a href="https://scholar.google.com/citations?user=lTmh1e0AAAAJ&hl=en"><strong>Laurent Kneip<sup>3*</sup></strong></a>
  ·
  <a href="https://ayoungk.github.io/"><strong>Ayoung Kim<sup>1*</sup></strong></a>
  <br/>
  <small><sup>1</sup>Seoul National University</small> &emsp;
  <small><sup>2</sup>ETH Zürich</small> &emsp;
  <small><sup>3</sup>Robotics and AI Institute</small>
  <br/>
  <small><sup>*</sup>Corresponding authors</small>
</p>

This repository contains the official ROS 2 implementation of **TRaIL-Odom**, a tightly coupled continuous-time Radar-IMU-LiDAR odometry framework with degeneracy-aware Doppler weighting.

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#news">News</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#dataset">Dataset</a></li>
    <li><a href="#quick-start">Quick Start</a></li>
    <li><a href="#docker">Docker</a></li>
    <li><a href="#ros-2-interface">ROS 2 Interface</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#citation">Citation</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

## News

- **September 3, 2026:** The paper is available on [arXiv](https://arxiv.org/abs/2609.03561).
- **August 23, 2026:** TRaIL-Odom was accepted for publication in IEEE Robotics and Automation Letters.

## Abstract

<details>
  <summary>Click to expand</summary>

TRaIL-Odom adapts the contribution of radar Doppler measurements to the directional observability of LiDAR geometry. The framework diagnoses weak translational directions from each LiDAR scan, reweights individual radar constraints toward that weak subspace, and schedules the overall radar gain according to scan anisotropy. These modules are integrated with asynchronous IMU and LiDAR measurements in a tightly coupled continuous-time B-spline estimator. Across 13 evaluated sequences, TRaIL-Odom remains accurate in both well-constrained and geometrically degenerate scenes. On three degenerate ablation sequences, the combined adaptive modules reduce RMSE ATE by 86.0% and RTE by 78.5% compared with fixed radar weighting.

</details>

## Dataset

The public TRaIL-Odom dataset contains six real-world sequences collected with an ANYmal-based Boxi platform in three geometrically challenging environments: **BikeTunnel**, **Park**, and **Airfield**. Position ground truth is provided by a Leica MS60 total station and a GRZ101 360° Mini Prism.

| Item | Description |
| --- | --- |
| Platform | ANYmal + Boxi |
| Sequences | 6 sequences across 3 environments |
| Sensor suite | Livox Mid-360, Honeywell HG4930, D3 Embedded RS-1843AOPU mmWave radar |
| Ground truth | Leica MS60 + GRZ101 360° Mini Prism |

- [Download the TRaIL-Odom dataset](https://drive.google.com/drive/folders/1h7RYsnLOgKopE9LnrQ_HXnvU73e8aGJr?usp=drive_link)
- [TRaIL-Odom dataset details](https://chiyunnoh.github.io/TRaIL-Odom/#dataset)
- [GaRLILEO dataset](https://garlileo.github.io/GaRLILEO/)

The repository provides configuration and launch presets for both the Boxi and GaRLILEO datasets.

## Quick Start

### Dependencies

The code is tested with:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- C++17
- Ceres Solver 2.2.0
- PCL 1.13.0
- Eigen 3.4.0
- Zenoh RMW (`rmw_zenoh_cpp`)

Install the Zenoh RMW implementation:

```bash
sudo apt update
sudo apt install ros-humble-rmw-zenoh-cpp
```

### Build

Clone the repository recursively into the `src` directory of a ROS 2 workspace, then build the bundled third-party dependencies.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/ChiyunNoh/TRaIL-Odom.git
cd TRaIL-Odom
chmod +x build_thirdparty.sh
./build_thirdparty.sh

source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select trail
source install/setup.bash
```

> [!IMPORTANT]
> 1. Set `Configor.DataStream.OutputPath` in the selected dataset configuration to a writable directory. Results are saved under `<OutputPath>/trail_output`.
> 2. Set `default_bag_path` in the matching launch file (`launch/trail_boxi.launch.py` or `launch/trail_garlileo.launch.py`) to the path of your rosbag.

### Launch

TRaIL-Odom uses Zenoh as its ROS 2 middleware. Start the Zenoh daemon in a
separate terminal before launching TRaIL-Odom:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Keep the daemon running. In another terminal, select the same RMW
implementation and source the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Two launch presets are included:

| Dataset | Launch file | Configuration |
| --- | --- | --- |
| Boxi / TRaIL-Odom | `trail_boxi.launch.py` | `dataset/Boxi/config.yaml` |
| GaRLILEO | `trail_garlileo.launch.py` | `dataset/GaRLILEO/config.yaml` |

Run the preset matching your dataset. The configured `default_bag_path` is
used unless you override it with the `rosbag_path` launch argument:

```bash
ros2 launch trail trail_boxi.launch.py
```

or

```bash
ros2 launch trail trail_garlileo.launch.py
```

## Docker

Clone the repository and edit `docker/run.sh` before starting the container.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/ChiyunNoh/TRaIL-Odom.git
cd TRaIL-Odom/docker
```

> [!IMPORTANT]
> 1. Set `PROJECT_DIR` in `docker/run.sh` to the absolute path of the cloned repository.
> 2. Set `DATASET_DIR` to the directory containing your downloaded rosbag data.
> 3. Set `OutputPath` to a persistent location inside the mounted project, such as `/root/ros2_ws/src/TRaIL-Odom/results/Boxi`.
> 4. Set `default_bag_path` in the matching launch file to the rosbag path inside the container, under `/root/data`.

Choose one of the following methods to obtain the Docker image.

**Option 1: Pull the pre-built image from Docker Hub**

```bash
docker pull chiyunnn/trail-odom:latest
chmod +x run.sh
./run.sh
```

**Option 2: Build the image locally**

```bash
chmod +x build.sh run.sh
./build.sh
./run.sh
```

Both methods use the image name `chiyunnn/trail-odom:latest`. The Docker image
installs `rmw_zenoh_cpp`, and `run.sh` selects it through the
`RMW_IMPLEMENTATION` environment variable.

Inside the container:

```bash
source /opt/ros/humble/setup.bash
git config --global --add safe.directory /root/ros2_ws/src/TRaIL-Odom
git config --global --add safe.directory /root/ros2_ws/src/TRaIL-Odom/thirdparty/ctraj
git config --global --add safe.directory /root/ros2_ws/src/TRaIL-Odom/thirdparty/ctraj/thirdparty/tiny-viewer

cd /root/ros2_ws/src/TRaIL-Odom
chmod +x build_thirdparty.sh
./build_thirdparty.sh

cd /root/ros2_ws
colcon build --packages-select trail
source install/setup.bash
```

After the build finishes, open another host terminal, attach it to the running
container, and start the Zenoh daemon:

```bash
docker exec -it trail_odom bash
source ~/.bashrc
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Keep `rmw_zenohd` running in this terminal. Then return to the first container
terminal and launch TRaIL-Odom:

```bash
ros2 launch trail trail_boxi.launch.py
```

## Acknowledgments

We thank the members of the [Robotic Systems Lab at ETH Zürich](https://rsl.ethz.ch/) and the [Robotics and AI Institute](https://rai-inst.com/) for their support and insightful discussions.

## Citation

If you use TRaIL-Odom or its dataset in your research, please cite:

```bibtex
@article{noh2026trailodom,
  title   = {{TRaIL-Odom}: Tightly Coupled Continuous Time Radar-IMU-LiDAR Odometry with Adaptive Doppler Weighting},
  author  = {Noh, Chiyun and Tuna, Turcan and Talbot, William and Hutter, Marco and Kneip, Laurent and Kim, Ayoung},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  url     = {https://chiyunnoh.github.io/TRaIL-Odom/}
}
```

## Contact

For questions, please contact:

- Chiyun Noh ([gch06208@snu.ac.kr](mailto:gch06208@snu.ac.kr))

## License

This project is released under the [MIT License](LICENSE).
