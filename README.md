# Weedbot

Additional collateral from the Weed Killer video

[<img src="https://img.youtube.com/vi/t7TLKLm5_10/0.jpg">](https://youtu.be/t7TLKLm5_10)

## Install

    git clone --recursive https://github.com/davesarmoury/WeedBot.git
    cd WeedBot/weedbot_detect/yolov5
    pip3 install -r requirements.txt
    cd ../..
    sudo apt-get install python3-catkin-pkg-modules
    cd ..

    git clone https://github.com/agilexrobotics/bunker_ros.git
    git clone https://github.com/DavesArmoury/ros_kortex.git
    git clone https://github.com/Kinovarobotics/ros_kortex_vision.git
    git clone https://github.com/agilexrobotics/ugv_sdk.git
    git clone https://github.com/LORD-MicroStrain/ROS-MSCL.git

    rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
    catkin_make -DUSE_CONAN=OFF
