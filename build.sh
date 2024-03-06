set -e

cd catkin_ws

catkin build

echo "source /home/robond/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
