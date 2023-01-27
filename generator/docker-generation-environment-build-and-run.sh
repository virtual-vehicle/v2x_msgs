# Author: Christoph Pilz

docker build -t vehicle_captain/ros2_msg_generation_docker:0.1 -f Dockerfiles/Dockerfile.environment .
docker run -t -d --name ros2_msg_gen vehicle_captain/ros2_msg_generation_docker:0.1
