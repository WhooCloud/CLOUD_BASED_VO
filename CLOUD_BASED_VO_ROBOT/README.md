# Backendserver
## Prerequisites 
###1. Chrony
sudo apt-get install chrony

###2. Casablanca
cd  
sudo apt-get install g++ git make zlib1g-dev libboost-all-dev libssl-dev cmake  

git clone https://github.com/Microsoft/cpprestsdk.git casablanca  
cd casablanca/Release  
mkdir build.debug  
cd build.debug  
cmake .. -DCMAKE_BUILD_TYPE=Debug  
make  
sudo make install 
 
###3. OpenCV
sudo apt-get install libopencv-dev

###4. Eigen
sudo apt-get install libeigen3-dev

###5. RapidJson
RapidJson is included in CBVOS

###6. PCL
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

## Enveironment Configuration
###1. Chrony
* Modify /etc/chrony/chrony.key
Write "1 admin" into the first line.
* Modify /etc/chrony/chrony.conf
Replace "server *" with "server ${SERVER_IP}"
* sudo service chrony restart

###2. CBVOR/parameters.txt
* modify "url=ws://13.124.50.218/websocket" to "url=ws://${SERVER_IP}/websocket"
* modify "rgb_dir=/home/yun/dataset/RGBD_SLAM/rgb_png/" to "rgb_dir=${RGB_DIR}"
* modify "depth_dir=/home/yun/dataset/RGBD_SLAM/depth_png/"" to "depth_dir=${DEPTH_DIR}"

***NOTE: NO WHITH_SPACE in parameters.txt***

###3. CBVOR/src/CMakeLists.txt
* add include directories of Eigen & RapidJson  
add these lines to your CMakeLists.txt  
INCLUDE_DIRECTORIES(/usr/include/eigen3/)  
INCLUDE_DIRECTORIES( /home/ubuntu/CLOUD_BASED_VO/rapidjson-master/include/ )  

###3. CBVOR/src/basic_client.cpp
StrImgName should be changed into your image path
StrUploadURL should be changed into server uploading url
StrWSURL should be changed into server websocket url


## Build
cd ${CBVOR}  
mkdir build  
cd build  
cmake ..  
make  

## Run CBVOR
./bin/robot
