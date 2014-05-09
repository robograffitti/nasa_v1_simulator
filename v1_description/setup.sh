v1_description_dir=`rospack find v1_description`
config_directories=`find $v1_description_dir -mindepth 1 -maxdepth 1 -type d \( ! -iname ".git" \)`

for d in $config_directories; do
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$d/v1_description/
done
