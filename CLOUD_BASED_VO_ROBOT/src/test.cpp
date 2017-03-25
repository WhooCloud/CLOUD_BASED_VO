#include "slamBase.h"
#include "robotBase.h"
void uploadFileHTTP(const char* file_path, const char* url);
string CompressString(string in_str);
string DecompressString(string in_str);
double getCurrentTime();

#define TEST_COMPRESSLEVEL 9
int main(int argc, char* argv[])
{   
	
	RobotClient client;
	ParameterReader PD;
	client.setURL("ws://localhost/test");
	client.connect();

	string StrSend = PD.getData("initial_message");
	double time1 = getCurrentTime();
	client.setDataSend(CompressString(StrSend));
	cout<< YELLOW "main: "<<"client.setDataSend(CompressString(StrSend)) costs: "<<(getCurrentTime() - time1) * 1000<< " ms" RESET<<endl;
	client.sendBinaryData();

	client.receiveBinaryData();
	string StrReceive = client.getDataReceive();
	cout<<"main: "<<"StrReceive: "<<StrReceive<<endl;
	time1 = getCurrentTime();
	cout<<"main: "<<"Decompressed StrReceive is: "<<DecompressString(StrReceive)<<endl;
	cout<< YELLOW "main: "<<"DecompressString(StrReceive) costs: "<<(getCurrentTime() - time1) * 1000<< " ms" RESET<<endl;
	client.close();
	return 0;
}