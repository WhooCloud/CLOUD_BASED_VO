# Backendserver
## Prerequisites 
###1. Chrony
sudo apt-get install chrony
###2. Openresty
wget https://openresty.org/download/openresty-1.11.2.2.tar.gz  
cd  
sudo apt-get update  
sudo apt-get install libreadline-dev libncurses5-dev libpcre3-dev libssl-dev perl make build-essential 

tar -xzvf openresty-1.11.2.2.tar.gz  
cd openresty-1.11.2.2  
sudo ./configure -j8  
make -j8  
sudo make install -j8  
###3. OpenCV
sudo apt-get install libopencv-dev
###4. Eigen
sudo apt-get install libeigen3-dev
###5. RapidJson
RapidJson is included in CBVOS
## Enveironment Configuration
###1. Chrony
* Modify /etc/chrony/chrony.key
Write "1 admin" into the first line.
* Modify /etc/chrony/chrony.conf
Replace "allow *" with "allow 0/0"
* sudo service chrony restart
###2. core/slam/src/SlamInterface.cpp
* Modify the variable ***string robotPath*** with the absolute path of data directory ***${robotPath}*** on your server.
* chmod 777 -R ***${robotPath}***
###3. core/slam/src/CMakeLists.txt
* add include directories of Eigen & RapidJson
add this two lines to your CMakeLists.txt
INCLUDE_DIRECTORIES(/usr/include/eigen3/)
INCLUDE_DIRECTORIES(/home/ubuntu/CLOUD_BASED_VO/rapidjson-master/include/)
## Build
cd ${CBVOS/core/slam}
mkdir build
cd build
cmake ..
make
## Run CBVOS
cd /usr/local/openresty/nginx/sbin
sudo ./ -p ${CBVOS}
