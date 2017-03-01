#include "robotBase.h"
#include <iomanip> // setprecision

double getCurrentTime();
string createPayloadString(int length);

void* threadWebSocket(void* ptr)
{
	ParameterReader pd;
	static int pay_load_length = atoi(pd.getData("pay_load_length").c_str());
	static bool debug = pd.getData("debug") == string("yes");
	static string url = pd.getData("url");
	RobotClient rClient;
	rClient.setURL(url);
	rClient.connect();
	try
	{
		while(1)
		{
			RobotJson jsonSend;	//?? assert change into if >> json_doc is not an object.
			jsonSend.setDocDouble("NO", getCurrentTime());
			jsonSend.setDocDouble("timestamp", jsonSend.getDocDouble("NO"));
			string s = createPayloadString(pay_load_length);
			jsonSend.addDocString("payload", s);
			rClient.setDataSend(jsonSend.getString());
			s = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
			rClient.sendData();

			rClient.receiveData();
			s = pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
			RobotJson jsonReceive;
			jsonReceive.setString(rClient.getDataReceive());

			double time1, time2, delta_time;
			time2 = getCurrentTime();
			time1 = jsonReceive.getDocDouble("timestamp");
			delta_time = time2 - time1;
			//cout<<delta_time<<endl;
			if(!debug)
			{
				static string result_file_path = pd.getData("result_file_path");
				ofstream out(result_file_path.c_str(), ios::app);
				out<<"bytes: "<< pay_load_length\
		           <<setprecision(14)<<" NO: "<<jsonReceive.getDocDouble("NO")\
		           <<setprecision(6)<<" delay2 is: "<<delta_time*1000<<" ms"<<endl;
		        out.close();

			}

			cout<<"bytes: "<< pay_load_length\
		       <<setprecision(14)<<" NO: "<<jsonReceive.getDocDouble("NO")\
		       <<setprecision(6)<<" delay2 is: "<<delta_time*1000<<" ms"<<endl;
		}
	}
	catch(...)
	{
		if(!debug)
		{
			static string result_file_path = pd.getData("result_file_path");
			ofstream out(result_file_path.c_str(), ios::app);
			out<<setprecision(14)<<"current time: " << getCurrentTime()\
				<<setprecision(6)<<" bytes: "<< pay_load_length\
				<<" ERROR: DATALOSS"<<endl;
	        out.close();

		}
		cout<<setprecision(14)<<"current time: " << getCurrentTime()\
			<<setprecision(6)<<" bytes: "<< pay_load_length\
			<<" ERROR: DATALOSS"<<endl;
		//continue;
	}
	rClient.close();

	return 0;
}