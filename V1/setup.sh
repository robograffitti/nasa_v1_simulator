V1_dir=`rospack find V1`
config_directories=`find $V1_dir -mindepth 1 -maxdepth 1 -type d \( ! -iname ".git" \)`

for d in $config_directories; do
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$d/V1/
done
