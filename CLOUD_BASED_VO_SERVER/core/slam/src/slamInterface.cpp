#include "slamInterface.h"
#include "slamBase.h"
#include "robotBase.h"
using namespace cv;

double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
	return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec))) + fabs(cv::norm(tvec));
}

extern "C" char* FFIInterface(const char* data)
{
	cout<<"FFIInterface: "<<"Running..."<<endl;
	cout<<"FFIInterface: "<<"data is: "<<data<<endl;

	RobotJson dataJson;
	dataJson.setString(string(data));
	string robotPath("/home/yun/Code/CLOUD_BASED_VO_SERVER/core/slam/data/initialize_1.txt");
	string str_type = dataJson.getDocString("type");
	string result;
	if(str_type == string("initialize"))
	{
		cout<<"FFIInterface: "<<"This is an initialize package!"<<endl;
		ofstream out(robotPath.c_str());
		if(!out.is_open())
			cerr<<RED "FFIInterface: "<<"Cannot open initialize file" RESET<<endl;
		else
		{
			out<<"type"<<"="<<str_type<<endl;
			out<<"min_inliers"<<"="<<dataJson.getDocInt("min_inliers")<<endl;
			out<<"max_norm"<<"="<<dataJson.getDocDouble("max_norm")<<endl;
			out<<"camera_cx"<<"="<<dataJson.getDocDouble("camera_cx")<<endl;
			out<<"camera_cy"<<"="<<dataJson.getDocDouble("camera_cy")<<endl;
			out<<"camera_fx"<<"="<<dataJson.getDocDouble("camera_fx")<<endl;
			out<<"camera_fy"<<"="<<dataJson.getDocDouble("camera_fy")<<endl;
			out<<"camera_scale"<<"="<<dataJson.getDocDouble("camera_scale")<<endl;
			out.close();
		}
		result = string("{\"type\" : \"initialize\", \"result\" : \"Success\"}");
		return strdup(result.c_str());

	}
	else if(str_type == string("mainloop"))
	{
		cout<<"FFIInterface: "<<"This is a mainloop package!"<<endl;
		ParameterReader pd(robotPath.c_str());
		CAMERA_INTRINSIC_PARAMETERS camera;
		cout<<"FFIInterface: "<<"mainloop: "<<"Creating Camera Matirx..."<<endl;
		camera.fx = atof(pd.getData("camera_fx").c_str());
		camera.cx = atof(pd.getData("camera_cx").c_str());
		camera.fy = atof(pd.getData("camera_fy").c_str());
		camera.cy = atof(pd.getData("camera_cy").c_str());
		camera.scale = atof(pd.getData("camera_scale").c_str());
		double camera_matrix_data[3][3] = {
		{camera.fx, 0, camera.cx},
		{0, camera.fy, camera.cy},
		{0, 0, 1} };
		Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);
		cout<<"FFIInterface: "<<"mainloop: "<<"Camera Matirx Created Successfully!"<<endl;
		cout<<"FFIInterface: "<<"mainloop: "<<"Creating pts_img ..."<<endl;
		vector<cv::Point2f> pts_img;
		vector<cv::Point3f> pts_obj;
		string str_pts_img = dataJson.getDocString("pts_img");
		cout<<"FFIInterface: "<<"mainloop: "<<"str_pts_img is: "<< str_pts_img<<endl;
		int lastpos = 0;
		int currpos = 0;
		while( (currpos = str_pts_img.find(";", lastpos)) != -1 ) 
		{
			string str_temp = str_pts_img.substr(lastpos, currpos-lastpos+1);
			cout<<str_temp<<endl;
			float x,y;
			sscanf(str_temp.c_str(), "%f,%f;", &x, &y);
			cv::Point2f pt_img(x, y);
			cout<<"pt_img = "<<pt_img<<endl;
			pts_img.push_back(pt_img);

			lastpos = currpos +1;
		}
		cout<<"pts_img: "<<pts_img<<endl;
		string str_pts_obj = dataJson.getDocString("pts_obj");
		cout<<"FFIInterface: "<<"mainloop: "<<"str_pts_obj is: "<< str_pts_obj<<endl;
		lastpos = 0;
		currpos = 0;
		while( (currpos = str_pts_obj.find(";", lastpos)) != -1 ) 
		{
			string str_temp = str_pts_obj.substr(lastpos, currpos-lastpos+1);
			cout<<"FFIInterface: "<<"mainloop: "<<str_temp<<endl;
			float x,y,z;
			sscanf(str_temp.c_str(), "%f,%f,%f;", &x, &y, &z);
			cv::Point3f pt_obj(x, y, z);
			cout<<"FFIInterface: "<<"mainloop: "<<"pt_obj = "<<pt_obj<<endl;
			pts_obj.push_back(pt_obj);

			lastpos = currpos +1;
		}
		cout<<"FFIInterface: "<<"mainloop: "<<"pts_obj: "<<pts_obj<<endl;
		//pts_img = stringToPoints2f(dataJson.getDocString("pts_img"));
		cv::Mat rvec, tvec, inliers;
		cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, \
			false, 100, 1.0, 100, inliers);

		RESULT_OF_PNP resultofPNP;
		resultofPNP.rvec = rvec;
		resultofPNP.tvec = tvec;
		resultofPNP.inliers = inliers.rows;

		if(resultofPNP.inliers < atoi(pd.getData("min_inliers").c_str()))
		{
			cerr<<RED "FFIInterface: "<<"insufficient min_inliers!" RESET<<endl;
			result = string("{\"type\" : \"NOTICE\", \"result\" : \"insufficient min_inliers\"}");
			return strdup(result.c_str());
		}
		double norm = normofTransform(resultofPNP.rvec, resultofPNP.tvec);
		cout<<"norm = "<<norm<<endl;
		if(norm >= atof(pd.getData("max_norm").c_str()))
		{
			cerr<<RED "FFIInterface: "<<"Too large norm!" RESET<<endl;
			result = string("{\"type\" : \"NOTICE\", \"result\" : \"Too large norm\"}");
			return strdup(result.c_str());			
		}
		Eigen::Isometry3d T = cvMat2Eigen( resultofPNP.rvec, resultofPNP.tvec);
		cout<<"FFIInterface: "<<"mainloop: "<<endl<<T.matrix()<<endl;
		string str_T;
		stringstream ss;
		for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j < 4; j++)
			{
				ss<<T(i,j);
				if(j != 3)
					ss<<',';
			}
			ss<<';';
		}
		str_T = ss.str();
		cout<<"FFIInterface: "<<"mainloop: "<<"str_T is: "<<str_T<<endl;
		result = string("{\"type\" : \"RESULT\", \"matrix\" : \"1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1;\", \"inliers\" : 10}");
		RobotJson dataJson;
		dataJson.setString(result);
		dataJson.setDocString("matrix", str_T);
		dataJson.setDocInt("inliers", resultofPNP.inliers);
		cout<<"FFIInterface: "<<"mainloop: "<<"dataJson is: "<<dataJson.getString()<<endl;
		return strdup(dataJson.getString().c_str());
	}
	else
	{
		cerr<<RED "FFIInterface: "<<"Cannot identify the type of this package!" RESET<<endl;
		result = string("{\"type\" : \"ERROR\", \"result\" : \"Cannot identify the type of this package!\"}");
		return strdup(result.c_str());
	}
}

string processData(const char* data)
{
	cout<<"processData: "<<"Running..."<<endl;
	cout<<"processData: "<<"data is: "<<data<<endl;
	string s(data);
	s = s + s;
	cout<<"processData: "<<"s is: "<<s<<endl;
	return s;
}

