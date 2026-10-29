FROM ros:humble-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Keep MuJoCo aligned with the version used by the local thesis environment.
RUN python3 -m pip install --no-cache-dir "numpy<2" "mujoco==3.14.0" scipy
