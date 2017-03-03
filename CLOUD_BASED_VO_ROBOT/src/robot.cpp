#include "slamBase.h"
#include "robotBase.h"
// This order of above two head files should never be changed
string points2fToString(vector<cv::Point2f> points2f);
string points3fToString(vector<cv::Point3f> points3f);
Eigen::Isometry3d createTMatrix(vector<double> v_d);
Eigen::Isometry3d createTMatrix(string str_tMatrix);
double getCurrentTime();
int main()
{
	double time1 = getCurrentTime();
	cout<<"Loading Parameters..."<<endl;
	ParameterReader pd;
	string url = pd.getData("url");
	int startIndex = atoi(pd.getData("start_index").c_str() );
	int endIndex   = atoi(pd.getData("end_index").c_str() );
	string detector = pd.getData("detector");
	string descriptor = pd.getData("descriptor");
	CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
	bool visualize = pd.getData("visualize_pointcloud")==string("yes");
	int min_inliers = atoi(pd.getData("min_inliers").c_str());
	double max_norm = atof(pd.getData("max_norm").c_str());
	int min_good_match= atoi(pd.getData("min_good_match").c_str());
	string initial_message = pd.getData("initial_message");
	string mainloop_message = pd.getData("mainloop_message");
	string close_message = pd.getData("close_message");
	double good_match_threshold = atof(pd.getData("good_match_threshold").c_str());
	cout<<"Parameters Loaded Successfully!"<<endl;
	cout<< YELLOW "Loading Parameters Costs: "<<(getCurrentTime() - time1)*1000<<" ms" RESET <<endl;

	time1 = getCurrentTime();
	cout<<"Connecting to Server..."<<endl;
	RobotClient client;
	client.setURL(url);
	client.connect();
	cout<<"Server Coonnected Successfully!"<<endl;
	cout<< YELLOW "Connecting to Server Costs: "<<(getCurrentTime() - time1)*1000<<" ms" RESET <<endl;

	time1 = getCurrentTime();	
	cout<<"Initializing..."<<endl;
	int currIndex = startIndex;
	FRAME lastFrame = readFrame(currIndex, pd);
	computeKeyPointsAndDesp(lastFrame, detector, descriptor);
	PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);
	pcl::visualization::CloudViewer viewer("viewer");
	cout<<"Initializing Done!"<<endl;
	cout<< YELLOW "Initializing Costs: "<<(getCurrentTime() - time1)*1000<<" ms" RESET <<endl;

	time1 = getCurrentTime();
	cout<<"Sending Initializing Message to Server..."<<endl;
	RobotJson initialMessage;
	initialMessage.setString(initial_message);
	initialMessage.setDocInt("min_inliers", min_inliers);
	initialMessage.setDocDouble("max_norm", max_norm);
	client.setDataSend(initialMessage.getString());
	client.sendData();
	cout<<"Initializing Message Sent to Server Successfully!"<<endl;
	cout<<"Initializing Message is: "<<initialMessage.getString()<<endl;
	cout<< YELLOW "Sending Initializing Message to Server Costs: "<<(getCurrentTime() - time1)*1000<<" ms" RESET <<endl;

	time1 = getCurrentTime();	
	client.receiveData();
	string stringDataReceiveInitial = client.getDataReceive();
	cout<<"Initializing Message from Server: "<<stringDataReceiveInitial<<endl;
	RobotJson receivedMessageInitial;
	receivedMessageInitial.setString(stringDataReceiveInitial);
	if(receivedMessageInitial.getDocString("type")==string("ERROR"))
		cout<< RED "Error From Server "<<receivedMessageInitial.getDocString("result")<<" " RESET<<endl;
	cout<< YELLOW "Receiving Initializing Message from Server Costs: "<<(getCurrentTime() - time1)*1000<<" ms" RESET <<endl;

	cout<<"Running in Main Loop..."<<endl;
	RobotJson mainLoopMessage;
	mainLoopMessage.setString(mainloop_message);
	
	
	for(currIndex = startIndex + 1; currIndex < endIndex; currIndex++)
	{
		time1 = getCurrentTime();
		double time1_mainloop = getCurrentTime();
		cout<<"Loading Image "<<currIndex<<endl;
		FRAME currFrame = readFrame(currIndex, pd);
		cout<< YELLOW "--Loading Image Costs: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;

		time1_mainloop = getCurrentTime();
		computeKeyPointsAndDesp(currFrame, detector, descriptor);
		cout<< YELLOW "--Computing KeyPoints & Descriptor Costs: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;

		vector<cv::Point3f> ptsObj;
		vector<cv::Point2f> ptsImg;

		time1_mainloop = getCurrentTime();
		int err = computeObjAndImage(lastFrame, currFrame, ptsObj, ptsImg, camera, min_good_match, good_match_threshold);
		cout<< YELLOW "--Matching Costs: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;

		time1_mainloop = getCurrentTime();
		string strPtsObj = points3fToString(ptsObj);
		string strPtsImg = points2fToString(ptsImg);
		cout<< YELLOW "--Converting Points(2f,3f) to String: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;

		time1_mainloop = getCurrentTime();
		mainLoopMessage.setDocString("pts_obj", strPtsObj);
		mainLoopMessage.setDocString("pts_img", strPtsImg);
		client.setDataSend(mainLoopMessage.getString());
		client.sendData();

		client.receiveData();
		string stringDataReceive = client.getDataReceive();
		cout<<stringDataReceive<<endl;
		cout<< YELLOW "--Send & Wait & Receive Costs: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;

		time1_mainloop = getCurrentTime();
		RobotJson receivedMessage;
		receivedMessage.setString(stringDataReceive);
		string type = receivedMessage.getDocString("type");
		cout<<"type is: "<<type<<endl;
		cout<< YELLOW "--Parsing Message Type Costs: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;

		if(type == string("RESULT"))
		{
			time1_mainloop = getCurrentTime();
			string str_tMatrix = receivedMessage.getDocString("matrix");
			//cout<<str_tMatrix<<endl;
			int inliers = receivedMessage.getDocInt("inliers");
		    Eigen::Isometry3d TMatrix = createTMatrix(str_tMatrix);
		    cout<<TMatrix.matrix()<<endl;
		    cout<< YELLOW "--Parsing tMatrix&inliers&TMatrix Costs: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;

		    time1_mainloop = getCurrentTime();
			cloud = joinPointCloud(cloud, currFrame, TMatrix, camera);
		   	
			if(visualize == true)
				viewer.showCloud(cloud);
			lastFrame = currFrame;
			cout<< YELLOW "--JoinPointCloud Costs: "<<(getCurrentTime() - time1_mainloop)*1000<<" ms" RESET <<endl;
		}
		else
		{
			lastFrame = currFrame;
			cout<< BLUE "NO RESULT FROM SERVER!:(" RESET <<endl;
		}
		cout<< YELLOW "MainLoop for Frame"<<currIndex<<"Costs: "<<(getCurrentTime() - time1)*1000<<" ms" RESET <<endl;
	}

	client.setDataSend(close_message);
	client.sendData();

	client.receiveData();
	string stringDataReceiveClose = client.getDataReceive();
	cout<<stringDataReceiveClose<<endl;
	
	client.close();
	cout<<"Connection Closed!"<<endl;
	//pcl::io::savePCDFile("./data/result.pcd", *cloud);
    while(!viewer.wasStopped())
    {
    }
	return 0;

}