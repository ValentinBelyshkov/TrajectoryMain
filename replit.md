# TerraSLAM

GPS-denied localization system that transforms SLAM (Simultaneous Localization and Mapping) results from local relative coordinates into global geospatial (GPS) coordinates.

## Project Overview

TerraSLAM uses ORB-SLAM3 wrapped in ROS 2 Humble, running inside a Docker container, to perform visual-inertial tracking and align local 3D reconstructions with GIS models.

**This project is designed to run inside a Docker container** with ROS 2 Humble, ORB-SLAM3, and optionally CUDA/NVIDIA Jetson support. The Replit environment runs the Flask management gateway only — the full SLAM pipeline requires the Docker environment described in the README.

## Architecture

- `container_root/server.py` — Flask HTTP gateway (port 5000) for managing ROS 2 tasks
- `TerraSLAM_relay/system_manager.py` — FastAPI system manager (runs inside Docker container)
- `TerraSLAM_relay/relay.py` — Coordinate transformation relay (SLAM → GPS)
- `terraslam_cli.py` — CLI client for interacting with the gateway API
- `ORB_SLAM3/` — Core SLAM library (C++)
- `orb_slam3_ros2_wrapper/` — ROS 2 Humble wrapper for ORB-SLAM3 (C++)
- `Database/` — Video frames, calibration files, metadata

## Running in Replit

The Flask gateway server runs on port 5000 and exposes HTTP endpoints:

- `GET /health` — Health check
- `POST /start` — Start a task (`slam`, `image_pub`, `relay`, `callback`)
- `GET /status/<task_id_or_type>` — Get task status
- `GET /list` — List all tasks
- `POST /stop/<task_id_or_type>` — Stop a task

## Full Setup (Docker)

See `README.md` for full Docker-based setup instructions including ROS 2, ORB-SLAM3 build, and data downloads.

## User Preferences
