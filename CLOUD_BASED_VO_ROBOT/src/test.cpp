#include "slamBase.h"
#include "robotBase.h"
void uploadFileHTTP(const char* file_path, const char* url);
int CompressString(const char* in_str,size_t in_len,
    std::string& out_str, int level);
int DecompressString(const char* in_str,size_t in_len,
	std::string& out_str);

#define TEST_COMPRESSLEVEL 9
int main(int argc, char* argv[])
{   
	RobotClient client;
	ParameterReader PD;
	client.setURL("ws://localhost/test");
	client.connect();

	string StrSend = PD.getData("initial_message");
	string CStrSend;
	int flag_compressstring = CompressString(StrSend.c_str(), StrSend.length(), CStrSend, TEST_COMPRESSLEVEL);
	cout<<"main: "<<"CStrSend: "<<CStrSend<<endl;
	client.setDataSend(CStrSend);
	client.sendBinaryData();

	client.receiveBinaryData();
	string StrReceive = client.getDataReceive();
	cout<<"main: "<<"StrReceive: "<<StrReceive<<endl;
	string DStrReceive;
	int flag_decompressstring = DecompressString(StrReceive.c_str(), StrReceive.length(), DStrReceive);
	cout<<"main: "<<"DStrReceive: "<<DStrReceive<<endl;
	cout<<"main: "<<"flag_decompressstring: "<<flag_decompressstring<<endl;
	client.close();
	return 0;
}