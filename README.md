# From-Data-Extraction-to-Data-Driven-Dynamic-Modeling-for-Collaborative-Robots
Code from "From Data Extraction to Data-Driven Dynamic Modeling for Collaborative Robots: A method using Multi-Objective Optimization" 

# Uso

## Instalación

```
docker-compose build
```

A .env file is needed in the main directory, with the following structure:

```
ROS_DISTRO=humble
GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/ros2_dev/iiwa_ros2/
DCMAKE_BUILD_TYPE=Debug
MLFLOW_TRACKING_URI=<path to mlflow tracking directory (anything works)>
```

