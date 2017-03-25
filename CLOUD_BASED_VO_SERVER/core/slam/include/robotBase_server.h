#ifndef ROBOT_BASE_SERVER_H
#define ROBOT_BASE_SERVER_H

#include <string>
#include <iostream>
#include <fstream>
#include <map>
// #include <cpprest/ws_client.h>
// #include <cpprest/filestream.h>
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include <vector>
//#include <sys/time.h> //getCurrentTime
//#include <iomanip> // setprecision

using namespace std;
// using namespace utility;
// using namespace web;
// using namespace web::websockets::client;
using namespace rapidjson;

#define ROBOTJSON_MAXSTRINGLENGTH 200*1024 //100KB

class RobotJson
{
	Document 	json_doc;
	string 		json_str;
	void stringToDoc();
	void docToString();
	void stringToCharArray(char* c, string s);
	void checkString();
public:
	RobotJson();
	void setString(const string _json_str);
	void setDocDouble(const char* key, const double value_d);
	void setDocInt(const char* key, const int value_i);
	void setDocString(const char* key, const string value_str);
	void addDocString(const char* key, const string value_str);
	string getString();
	double getDocDouble(const char* key);
	int getDocInt(const char* key);
	string getDocString(const char* key);
	vector<double> getDocDoubleArray(const char* key);
};

// class RobotClient
// {
// private:
// 	string 			 m_url;
// 	string 			 m_data_send;
// 	string 			 m_data_receive;
// 	websocket_client m_client;

// public:
// 	RobotClient();
// 	~RobotClient();

// 	void connect();
// 	void sendData();
// 	void setURL(const string &url);
// 	void receiveData();
// 	void setDataSend(const string &data_send);
// 	void close();

// 	string getDataReceive();
// };


#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

#endif