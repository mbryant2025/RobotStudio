cd docker

sudo docker run -it --rm \
  -v $(dirname "$(pwd)"):/root/dev/robot \
  --privileged -v /dev:/dev \
  robot-studio