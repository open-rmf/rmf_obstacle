# rmf_human_detector_oakd

## Requirements
* [OAK-D camera](https://docs.luxonis.com/projects/hardware/en/latest/)- Tested on OAK-D Lite and OAK-D
* [depthai-core](https://github.com/luxonis/depthai-core)
* [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)- Tested on `humble` and  `galactic`
* [Open-RMF](https://github.com/open-rmf/rmf)


## Setup
Follow the [instructions] to build `depthai` as a **shared library** and install it to a location that is a standard library path on your system. See [Troubleshooting](#troubleshooting) in case of issues.

Then clone this package into your `Open-RMF` workspace

```bash
git clone https://github.com/open-rmf/rmf_human_detector_oakd.git
```

Then run `rosdep` from the root of the workspace folder to install any missing dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src --ros-distro galactic --skip-keys "depthai" -y
```

Then build the new package after sourcing the ROS 2 distribution
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to rmf_human_detector_oakd
```

## Troubleshooting
If `depthai` is not built and installed properly, a number of build and runtime errors may arise. If you're facing issues, try building it following these instructions
```bash
cd ~/
git clone https://github.com/luxonis/depthai-core.git
cd depthai-core
git submodule update --init --recursive
cmake -S. -Bbuild -D'BUILD_SHARED_LIBS=ON'
cmake --build build --target install
```

Next symlink the built shared libraries to `/usr/lib/`
```bash
cd /usr/lib/
sudo ln -s ~/Downloads/depthai-core/build/install/lib/libdepthai-core.so .
sudo ln -s ~/Downloads/depthai-core/build/install/lib/libdepthai-opencv.so .
```

Then build this package by setting the `depthai_DIR` flag.
```bash
colcon build --packages-up-to rmf_human_detector_oakd --cmake-args -DCMAKE_BUILD_TYPE=Release -Ddepthai_DIR=/home/USER/depthai-core/build/install/lib/cmake/depthai

```
