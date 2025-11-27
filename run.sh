xhost +local:root

docker run -it --rm --net=host --privileged --device /dev/video0 -e DISPLAY=$DISPLAY -v /home/oskar/Documents/STUDIA/projektnosir:/vol ros2_ur_project 
