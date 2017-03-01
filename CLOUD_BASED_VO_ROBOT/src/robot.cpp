#include "slamBase.h"
#include "robotBase.h"
// This order of above two head files should never be changed
string points2fToString(vector<cv::Point2f> points2f);
string points3fToString(vector<cv::Point3f> points3f);
Eigen::Isometry3d createTMatrix(vector<double> v_d);
Eigen::Isometry3d createTMatrix(string str_tMatrix);
int main()
{
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
	double good_match_threshold = atof(pd.getData("good_match_threshold").c_str());
	cout<<"Parameters Loaded Successfully!"<<endl;

	cout<<"Connecting to Server..."<<endl;
	RobotClient client;

	client.setURL(url);
	//cout<<url<<endl;
	client.connect();
	cout<<"Server Coonnected Successfully!"<<endl;

	cout<<"Initiallizing..."<<endl;
	int currIndex = startIndex;
	FRAME lastFrame = readFrame(currIndex, pd);
	computeKeyPointsAndDesp(lastFrame, detector, descriptor);
	PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);
	pcl::visualization::CloudViewer viewer("viewer");
	cout<<"Initiallizing Done!"<<endl;

	cout<<"Sending Initiallizing Message to Server..."<<endl;
	RobotJson initialMessage;
	initialMessage.setString(initial_message);
	initialMessage.setDocInt("min_inliers", min_inliers);
	initialMessage.setDocDouble("max_norm", max_norm);
	client.setDataSend(initialMessage.getString());
	client.sendData();
	cout<<"Initiallizing Message Sent to Server Successfully!"<<endl;
	cout<<"initiallizing Message is: "<<initialMessage.getString()<<endl;

	cout<<"Running in Main Loop..."<<endl;
	RobotJson mainLoopMessage;
	mainLoopMessage.setString(mainloop_message);
	
	
	for(currIndex = startIndex + 1; currIndex < endIndex; currIndex++)
	{
		cout<<"Loading Image "<<currIndex<<endl;
		FRAME currFrame = readFrame(currIndex, pd);
		computeKeyPointsAndDesp(currFrame, detector, descriptor);

		vector<cv::Point3f> ptsObj;
		vector<cv::Point2f> ptsImg;
		int err = computeObjAndImage(lastFrame, currFrame, ptsObj, ptsImg, camera, min_good_match, good_match_threshold);
		string strPtsObj = points3fToString(ptsObj);
		string strPtsImg = points2fToString(ptsImg);
		mainLoopMessage.setDocString("pts_obj", strPtsObj);
		mainLoopMessage.setDocString("pts_img", strPtsImg);
		client.setDataSend(mainLoopMessage.getString());
		client.sendData();

		client.receiveData();
		string stringDataReceive = client.getDataReceive();
		cout<<stringDataReceive<<endl;
		RobotJson receivedMessage;
		receivedMessage.setString(stringDataReceive);
		string type = receivedMessage.getDocString("type");
		cout<<"type is: "<<type<<endl;
		// if(type == "RESULT")
		// {
		// 	vector<double> tMatrix;
		// 	tMatrix = receivedMessage.getDocDoubleArray("matrix");
		// 	int inliers = receivedMessage.getDocInt("inliers");
		//     Eigen::Isometry3d TMatrix = createTMatrix(tMatrix);
		//     //cout<<TMatrix.matrix()<<endl;
		// 	cloud = joinPointCloud(cloud, currFrame, TMatrix, camera);
		   	
		// 	if(visualize == true)
		// 		viewer.showCloud(cloud);

		// 	lastFrame = currFrame;
		// }
		if(type == string("RESULT"))
		{
			string str_tMatrix = receivedMessage.getDocString("matrix");
			//cout<<str_tMatrix<<endl;
			int inliers = receivedMessage.getDocInt("inliers");
		    Eigen::Isometry3d TMatrix = createTMatrix(str_tMatrix);
		    cout<<TMatrix.matrix()<<endl;
			cloud = joinPointCloud(cloud, currFrame, TMatrix, camera);
		   	
			if(visualize == true)
				viewer.showCloud(cloud);
			lastFrame = currFrame;
		}
		else
		{
			lastFrame = currFrame;
			cout<< BLUE "NO RESULT FROM SERVER!:(" RESET <<endl;
		}

	}

	client.close();
	cout<<"Connection Closed!"<<endl;
	//pcl::io::savePCDFile("./data/result.pcd", *cloud);
    while(!viewer.wasStopped())
    {
    }
	return 0;

}