# From-Data-Extraction-to-Data-Driven-Dynamic-Modeling-for-Collaborative-Robots
Code from "From Data Extraction to Data-Driven Dynamic Modeling for Collaborative Robots: A method using Multi-Objective Optimization" 

# Uso

## Instalación

Begin by building the docker container where the simulation takes place.
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

To fine-tune a set of PD controllers to a specific trajectory use the following command:
```
docker compose run --rm kuka_container sh -c "python3 /optimization/optimize.py <configuration file (e.g. /optimization/experiments/spiral/spiral.json)"
```

To use the fine-tuned controllers to extract data from the trajectories use extract_data.py:
```
docker compose run --rm kuka_container sh -c "python3 /optimization/extract_data.py"
```

Finally, to use the BRNN in conjuntion with the PD controller, use simulate_BRNN.py. Requirements listed in requirements.txt might need to be installed in the local machine, as the main loop is done outside of the container. Custom_msg and robot_models contents must also be installed.  
```
python3 optimization/simulate_BRNN.py
```

To train a BRNN model, use the contents of the ML_model directory. There you can find a training script (train.py) and the data used for both models shown in the paper (generic and fine-tuned).
