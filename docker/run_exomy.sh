#!/bin/bash
# run_exomy- A script to run containers for dedicated functions of exomy

help_text="Usage: "$0" [MODE] [OPTIONS]
    A script to run ExoMy in different configurations
    Options:
        -c, --config    Only have Config at the moment while testing. 
"

### Main
# Initialize parameters 
container_name="orion"
image_name="orion"

# Process parameters
if [ "$1" != "" ]; then
    case $1 in
        # -a | --autostart)       
        #                         container_name="${container_name}_autostart"
        #                         start_command="autostart"
        #                         options="--restart always"
                                 
        #                         ;;
        # -s | --stop_autostart)  
        #                         docker container stop "${container_name}_autostart"
        #                         exit     
        #                         ;;
        -c | --config)          
                                container_name="${container_name}_config"
                                start_command="config"
                                options="--rm"
                                ;;
        # -d | --devel)           
        #                         container_name="${container_name}_devel"
        #                         start_command="devel"
        #                         options="--restart always"
        #                         ;;  
        # -h | --help )           echo "$help_text"
        #                         exit
        #                         ;;
        * )                     echo "ERROR: Not a valid mode!"
                                echo "$help_text"
                                exit 1
    esac
else
    echo "ERROR: You need to specify a mode!"
    echo "$help_text"
    exit
fi

# Build docker image from Dockerfile in directory 
directory=$( dirname "$0" )
docker build -t $image_name $directory

# Stop any of the 3 containers if running
RUNNING_CONTAINERS=$( docker container ls -a -q --filter ancestor=exomy )
if [ -n "$RUNNING_CONTAINERS" ]; then
    docker stop "$RUNNING_CONTAINERS"
    docker rm -f "$RUNNING_CONTAINERS"
fi

# Run docker container
docker run \
    -it \
    --mount type=bind,source=/home/maxcap/WS/orion_ws/src,target=/WS/orion_ws/src \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /mnt/usb:/mnt/usb \
    -v /opt/vc:/opt/vc \
    --env LD_LIBRARY_PATH=/opt/vc/lib \
    -p 8000:8000 \
    -p 8080:8080 \
    -p 9090:9090 \
    --privileged \
    ${options} \
    --name "${container_name}" \
    "${image_name}" \
    "${start_command}"


# Bus 002 Device 019: ID 054c:05c4 Sony Corp. DualShock 4 [CUH-ZCT1x]
# docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb -v /mnt/usb:/mnt/usb my_ros_container
