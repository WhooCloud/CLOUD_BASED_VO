#include "slamBase.h"
#include "robotBase.h"
void uploadFileHTTP(const char* file_path, const char* url);
void uploadMatHTTP(const char* key, cv::Mat src, const char* file_path, const char* url);
string CompressString(string in_str);
string DecompressString(string in_str);
string matToString(cv::Mat m);

int main(int argc, char** argv)
{
	// Save an image to "./uploading.jpg", and then send it to "http://localhost/uploading"
	string StrImgName = "/home/yun/dataset/RGBD_SLAM/good_matches.png";
	string StrUploadFileName = "./uploading.jpg";
	string StrUploadURL = "http://localhost/uploading";
	cv::Mat src = cv::imread(StrImgName.c_str());
	cv::imshow("src", src);
	cv::imwrite(StrUploadFileName.c_str(), src);
	uploadFileHTTP(StrUploadFileName.c_str(), StrUploadURL.c_str());

	//Save a mat to "./uploading.yml", and then sent it to "http://localhost/uploading"
	StrUploadFileName = "./uploading.yml";
	cv::Mat m1= cv::Mat::eye(3,3, CV_8U);  
    uploadMatHTTP("Descriptor", m1, StrUploadFileName.c_str(), StrUploadURL.c_str());

    //Transform a mat into JSON format, and then send it to "ws://localhost/websocket"
    //!Note: the whole String should not be too large(>ROBOTJSON_MAXSTRINGLENGTH)
    //ROBOTJSON_MAXSTRINGLENGTH is defined in robotBase.h
    //If you change the ROBOTJSON_MAXSTRINGLENGTH, you should also change "max_payload_len" in load_balancing_webserver.lua
    string StrWSURL = "ws://localhost/websocket";
    RobotClient client;
    ParameterReader PD;
    client.setURL(StrWSURL.c_str());
    client.connect();

    cv::Mat m2 = cv::Mat::ones(3, 3, CV_8U);
    string StrDataSend = PD.getData("data_message");
    RobotJson JsonDataSend;
    JsonDataSend.setString(StrDataSend);
    JsonDataSend.setDocString("data", matToString(m2));
    StrDataSend = JsonDataSend.getString();
    string CStrDataSend = CompressString(StrDataSend);
    client.setDataSend(CStrDataSend);
    cout<<"main: "<<"StrDataSend: "<<StrDataSend<<endl;
    cout<<"main: "<<"StrDataSend.length() is "<<StrDataSend.length()<<endl; 
    cout<<"main: "<<"CStrDataSend.length() is "<<CStrDataSend.length()<<endl;
    client.sendBinaryData();

    client.receiveBinaryData();
    string StrDataReceive;
    string DStrDataReceive;
    StrDataReceive = client.getDataReceive();
    DStrDataReceive = DecompressString(StrDataReceive);
    cout<<"main: "<<"DStrDataReceive :"<<DStrDataReceive<<endl;

	return 0;
}