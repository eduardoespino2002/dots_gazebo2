
#!/bin/bash

echo $GAZEBO_MODEL_PATH
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_PLUGIN_PATH

echo "Starting gzweb in dir:" `pwd`

GZWEB=/home/dots/gzweb

# For gzweb to work, the models need to be copied from GAZEBO_MODEL_PATH
# to ~/gzweb/http/assets and fixed up so that all bitmap graphics are .png.
# The scripts get_local_models.py and webify_models_v2.py do this.
#
# The original deploy.sh rolls this process into the build process, which
# is not convenient when the models may change, and when we want a docker-based 
# system. The docker build does all the standard gazebo models and materials, 
# leaving just the local models to be processed at simulator start time.
#
# We don't want to process all the resources on GAZEBO_RESOURCE_PATH, because 
# that includes the standard gazebo materials, and they have already been processed
# so:
#
#   Trim the standard material path
# https://askubuntu.com/questions/433329/how-to-remove-a-path-from-system-pathpath-using-terminal-commands
echo $GAZEBO_RESOURCE_PATH
export GAZEBO_RESOURCE_PATH="$( echo $GAZEBO_RESOURCE_PATH| tr : '\n' |grep -v gazebo-11 | paste -s -d: )"
echo $GAZEBO_RESOURCE_PATH

# Collate the local models and materials
mkdir -p $GZWEB/http/client/local_models
$GZWEB/get_local_models.py $GZWEB/http/client/local_models

# Webify them

# Move into the main asset dir
cp -r $GZWEB/http/client/local_models/* $GZWEB/http/client/assets

cd $GZWEB
while true; do
    npm start -p 8085
done

