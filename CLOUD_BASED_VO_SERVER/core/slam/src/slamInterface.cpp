#include "slamInterface.h"
#include "slamBase_server.h"
#include "robotBase_server.h"
#include <unistd.h>

using namespace cv;

Mat_<float> createCamera(ParameterReader& pd);
vector<cv::Point2f> createPointsImg(const string& str_pts_img);
vector<cv::Point3f> createPointsObj(const string& str_pts_obj);
string eigenTToString(const Eigen::Isometry3d T);

string slamCore(vector<cv::Point3f> &pts_obj, vector<cv::Point2f> &pts_img, Mat_<float>& cameraMatrix, ParameterReader& pd);
double normofTransform(const cv::Mat rvec, const cv::Mat tvec);

extern "C" char* FFIInterface(const char* data)
{
	cout<<"FFIInterface: "<<"Running..."<<endl;
	cout<<"FFIInterface: "<<"data is: "<<data<<endl;

	RobotJson dataJson;
	dataJson.setString(string(data));
	string robotPath("/home/yun/git_repo/CLOUD_BASED_VO/CLOUD_BASED_VO_SERVER/core/slam/data/initialize_1.txt");
	string str_type = dataJson.getDocString("type");
	string result;
	if(str_type == string("initialize"))
	{
		cout<<"FFIInterface: "<<"This is an initialize package!"<<endl;
		ofstream out(robotPath.c_str());
		if(!out.is_open())
			result = string("{\"type\" : \"ERROR\", \"result\" : \"Cannot open initialize file\"}");
		else
		{
		   char cwd[1024];
		   if (getcwd(cwd, sizeof(cwd)) != NULL)
		       out<<"curr_dir"<<"="<<cwd<<endl;
		   else
		       result = string("{\"type\" : \"ERROR\", \"result\" : \"Cannot get curr_dir\"}");
			out<<"type"<<"="<<str_type<<endl;
			out<<"min_inliers"<<"="<<dataJson.getDocInt("min_inliers")<<endl;
			out<<"max_norm"<<"="<<dataJson.getDocDouble("max_norm")<<endl;
			out<<"camera_cx"<<"="<<dataJson.getDocDouble("camera_cx")<<endl;
			out<<"camera_cy"<<"="<<dataJson.getDocDouble("camera_cy")<<endl;
			out<<"camera_fx"<<"="<<dataJson.getDocDouble("camera_fx")<<endl;
			out<<"camera_fy"<<"="<<dataJson.getDocDouble("camera_fy")<<endl;
			out<<"camera_scale"<<"="<<dataJson.getDocDouble("camera_scale")<<endl;

			out.close();
			result = string("{\"type\" : \"initialize\", \"result\" : \"Success\"}");
		}
		return strdup(result.c_str());

	}
	else if(str_type == string("mainloop"))
	{
		cout<<"FFIInterface: "<<"This is a mainloop package!"<<endl;
		static ParameterReader pd(robotPath.c_str());
		
		cout<<"FFIInterface: "<<"mainloop: "<<"Creating Camera Matirx..."<<endl;
		Mat_<float> cameraMatrix =  createCamera(pd);
		cout<<"FFIInterface: "<<"mainloop: "<<"Camera Matirx Created Successfully!"<<endl;
		cout<<"FFIInterface: "<<"mainloop: "<<"Camera Matrix is: "<<cameraMatrix<<endl;

		cout<<"FFIInterface: "<<"mainloop: "<<"Creating pts_img ..."<<endl;
		string str_pts_img = dataJson.getDocString("pts_img");
		cout<<"FFIInterface: "<<"mainloop: "<<"str_pts_img is: "<< str_pts_img<<endl;
		vector<cv::Point2f> pts_img = createPointsImg(str_pts_img);
		cout<<"FFIInterface: "<<"mainloop: "<<"pts_img: "<<pts_img<<endl;
		cout<<"FFIInterface: "<<"mainloop: "<<"pts_img Created Successfully!"<<endl;

		cout<<"FFIInterface: "<<"mainloop: "<<"Creating pts_obj ..."<<endl;
		string str_pts_obj = dataJson.getDocString("pts_obj");
		cout<<"FFIInterface: "<<"mainloop: "<<"str_pts_obj is: "<< str_pts_obj<<endl;
		vector<cv::Point3f> pts_obj = createPointsObj(str_pts_obj);
		cout<<"FFIInterface: "<<"mainloop: "<<"pts_obj: "<<pts_obj<<endl;
		cout<<"FFIInterface: "<<"mainloop: "<<"pts_obj Created Successfully!"<<endl;

		cout<<"FFIInterface: "<<"mainloop: "<<"Running slamCore..."<<endl;
		result = slamCore(pts_obj, pts_img, cameraMatrix, pd);
		cout<<"FFIInterface: "<<"mainloop: "<<"slamCore Return: "<<result<<endl;
		cout<<"FFIInterface: "<<"mainloop: "<<"slamCore Implemented Successfully!"<<endl;

		return strdup(result.c_str());
	}
	else if(str_type == string("close"))
	{
		result = string("{\"type\" : \"close\", \"result\" : \"Success\"}");
		return strdup(result.c_str());
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

double normofTransform(const cv::Mat rvec, const cv::Mat tvec)
{
	return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec))) + fabs(cv::norm(tvec));
}

Mat_<float> createCamera(ParameterReader& pd)
{
	CAMERA_INTRINSIC_PARAMETERS camera;
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
	cout<<"createCamera: "<<cameraMatrix<<endl;
	return cameraMatrix;
}

vector<cv::Point2f> createPointsImg(const string& str_pts_img)
{
	int lastpos = 0;
	int currpos = 0;
	vector<cv::Point2f> pts_img;
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
	return pts_img;
}

vector<cv::Point3f> createPointsObj(const string& str_pts_obj)
{
	int lastpos = 0;
	int currpos = 0;
	vector<cv::Point3f> pts_obj;
	while( (currpos = str_pts_obj.find(";", lastpos)) != -1 ) 
	{
		string str_temp = str_pts_obj.substr(lastpos, currpos-lastpos+1);
		cout<<"createPointsObj: "<<str_temp<<endl;
		float x,y,z;
		sscanf(str_temp.c_str(), "%f,%f,%f;", &x, &y, &z);
		cv::Point3f pt_obj(x, y, z);
		cout<<"createPointsObj: "<<"pt_obj = "<<pt_obj<<endl;
		pts_obj.push_back(pt_obj);

		lastpos = currpos +1;
	}	
	return pts_obj;
}

string slamCore(vector<cv::Point3f> &pts_obj, vector<cv::Point2f> &pts_img, Mat_<float> &cameraMatrix, ParameterReader &pd)
{
	cout<<"slamCore: "<<"pts_obj is:"<<pts_obj<<endl;
	cout<<"slamCore: "<<"pts_img is:"<<pts_img<<endl;
	cout<<"slamCore: "<<"cameraMatrix: "<<cameraMatrix<<endl;
	cv::Mat rvec, tvec, inliers;
	cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, \
		false, 100, 1.0, 100, inliers);

	RESULT_OF_PNP resultofPNP;
	resultofPNP.rvec = rvec;
	resultofPNP.tvec = tvec;
	resultofPNP.inliers = inliers.rows;

	if(resultofPNP.inliers < atoi(pd.getData("min_inliers").c_str()))
	{
		cerr<<RED "slamCore: "<<"insufficient min_inliers!" RESET<<endl;
		return string("{\"type\" : \"NOTICE\", \"result\" : \"insufficient min_inliers\"}");
	}
	double norm = normofTransform(resultofPNP.rvec, resultofPNP.tvec);
	cout<<"norm = "<<norm<<endl;
	if(norm >= atof(pd.getData("max_norm").c_str()))
	{
		cerr<<RED "slamCore: "<<"Too large norm!" RESET<<endl;
		return string("{\"type\" : \"NOTICE\", \"result\" : \"Too large norm\"}");			
	}
	Eigen::Isometry3d T = cvMat2Eigen( resultofPNP.rvec, resultofPNP.tvec);
	string str_T = eigenTToString(T);
	cout<<"slamCore: "<<endl<<T.matrix()<<endl;
	cout<<"slamCore: "<<"str_T is: "<<str_T<<endl;
	string result;
	result = string("{\"type\" : \"RESULT\", \"matrix\" : \"1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1;\", \"inliers\" : 10}");
	RobotJson dataJson;
	dataJson.setString(result);
	dataJson.setDocString("matrix", str_T);
	dataJson.setDocInt("inliers", resultofPNP.inliers);
	cout<<"slamCore: "<<"dataJson is: "<<dataJson.getString()<<endl;
	return dataJson.getString();	
}

string eigenTToString(const Eigen::Isometry3d T)
{
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
	return str_T;
}