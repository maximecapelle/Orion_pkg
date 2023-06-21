#!/bin/bash

# This file is the file that will build the image and run the containers automatically. 

######################################################## HELP MESSAGES #########################################################

# If the command is run incorrecly ($0 is a special variable that stores the name as a variable)
help_text="How to run this file from commandline:
        Type: sudo sh "$0" [TYPE]
    
        sudo - is needed to give all permissions from the command line.
        sh   - to run the file as a bash file
        "$0" - the name of the start file in the docker folder,
        [TYPE] - will be -(run type), currently only have -c, -l
        -c :config
        -l :launch
    
"

######################################################## MAIN #########################################################
### Main
# Initialize parameters 
container_name_ancestor="orion"
image_name="orion"
username="maxcap" 

# Process parameters
if [ "$1" != "" ]; then
    case $1 in
        -c | --config)          
                                container_name="${container_name_ancestor}_config"     # Edits the container name to differentiate between the different run types
                                start_command="config"                                 # Define the start command that is needed in when docker run is called.
                                options="--rm"                                         # Removes the docker container when closing the docker.
                                ;;
        -l | --launch)          
                                container_name="${container_name_ancestor}_launch"     # Edits the container name to differentiate between the different run types
                                start_command="launch"                                 # Define the start command that is needed in when docker run is called.
                                options="--rm"                                         # Removes the docker container when closing the docker.
                                ;;

        * )                     echo "ERROR: Not a valid mode!"             # If anything else but what is listed here, throw this error message.
                                echo "$help_text"                           # *) means anything else I think
                                exit 1
    esac
else
    echo "ERROR: You need to specify a mode!"
    echo "$help_text"
    exit
fi

######################################################## BUILD IMAGE #########################################################

# Build docker image from Dockerfile in directory 
directory=$( dirname "$0" )                         # dirname "..." gives the file path to this file. Is needed so that you can give context to where you are building this Dockerfile from.                            
docker build -t $image_name $directory              # docker build - makes all the image from which all the containers from
                                                    # -t is the tag command, where you overwrite the standard ID name and give it your own name. 
                                                    # $image_name - gives it the name you specified
                                                    # $directory - gives context to docker to where we are running the dockerfile from.

# Stop any of the 3 containers if running
RUNNING_CONTAINERS=$( docker container ls -a -q --filter ancestor=$container_name_ancestor) # Filters and saves all the containers that are running with the name of container.
if [ -n "$RUNNING_CONTAINERS" ]; then                                                       # For each container:
    docker stop "$RUNNING_CONTAINERS"                                                       # stops the container from running
    docker rm -f "$RUNNING_CONTAINERS"                                                      # removes the container, -f does this forcefully
fi 

######################################################## RUN CONTAINER FROM IMAGE #########################################################
#Define where the package can be found and where it needs to go
source_hostmachine="/home/$username/WS/orion_ws/src"    # Define the location of the package on hostmachine
target_docker_container="/WS/orion_ws/src"              # Define where to place the package in docker container 

# Run docker container
docker run \
    -it \
    --mount type=bind,source=$source_hostmachine,target=$target_docker_container\
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /mnt/usb:/mnt/usb \
    -v /dev/video0:/dev/video0 \
    -v /opt/vc:/opt/vc \
    --env LD_LIBRARY_PATH=/opt/vc/lib \
    -p 8000:8000 \
    -p 8080:8080 \
    -p 9090:9090 \
    --name "${container_name}" \
    "${options}" \
    "${image_name}" \
    "${start_command}" \

# -it: Open with interative command line in container
# --mount: the folders from host to docker container
# --privileged: Give full access to host machine (Can be unsafe when public)
# -v: Give access to all USB devices
# -v: Also putting the usb devices on the docker container "-volumes"
# -p: Open netork ports?
# "${options}": Runs the extra options specified with run type.
# "${image_name}": Define the name of image from which the container based on.
# "${start_command}": Run defined start command  