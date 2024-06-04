sudo rocker --x11 --env-file .env --network none --volume ./data:/ros_app/data ./mlflow:/mlflow ./optimization:/optimization ./paths:/ros_app/paths -- ros $@
  
