FROM ros:humble-ros-base AS base

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Keep MuJoCo aligned with the version used by the local thesis environment.
RUN python3 -m pip install --no-cache-dir "numpy<2" "mujoco==3.14.0" scipy

# Optional desktop for the macOS simulation. Xvfb and Mesa render MuJoCo
# inside Linux; x11vnc exposes that display to macOS Screen Sharing.
FROM base AS gui

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        xvfb x11vnc mesa-utils libgl1-mesa-dri fluxbox \
    && rm -rf /var/lib/apt/lists/*

# Preserve the original lightweight image as the default build target.
FROM base AS runtime
