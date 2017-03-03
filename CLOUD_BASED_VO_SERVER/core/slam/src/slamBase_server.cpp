#include "slamBase_server.h"
#include <iostream>

FRAME readFrame(int index, ParameterReader& pd)
{
	FRAME f;
	string rgbDir 	= 	pd.getData("rgb_dir");
	string depthDir = 	pd.getData("depth_dir");
	string rgbExt	= 	pd.getData("rgb_extension");
	string depthExt = 	pd.getData("depth_extension");

	stringstream ss;
	ss<<rgbDir<<index<<rgbExt;
	string filename;
	ss>>filename;
	f.rgb = cv::imread(filename);
	ss.clear();
	filename.clear();
	ss<<depthDir<<index<<depthExt;
	ss>>filename;
	f.depth = cv::imread(filename, -1);
	return f;
}

// PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, \
// 								CAMERA_INTRINSIC_PARAMETERS& camera)
// {
// 	PointCloud::Ptr cloud(new PointCloud);

// 	for(int m = 0; m < depth.rows; m++)
// 		for(int n = 0; n < depth.cols; n++)
// 		{
// 			ushort d = depth.ptr<ushort>(m)[n];

// 			if(d == 0)
// 				continue;

// 			PointT p;
// 			p.z = double(d) / camera.scale;
// 			p.x = (n - camera.cx) * p.z / camera.fx;
// 			p.y = (m - camera.cy) * p.z / camera.fy;

// 			p.b = rgb.ptr<uchar>(m)[n*3];
// 			p.g = rgb.ptr<uchar>(m)[n*3+1];
// 			p.r = rgb.ptr<uchar>(m)[n*3+2];

// 			cloud->points.push_back(p);
// 		}

// 	cloud->height = 1;
// 	cloud->width = cloud->points.size();
// 	cloud->is_dense = false;

// 	return cloud;
// }

void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor)
{
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;

	//cv::initModule_nonfree();
	_detector = cv::FeatureDetector::create( detector.c_str());
	_descriptor = cv::DescriptorExtractor::create( descriptor.c_str());

	if(!_detector || !_descriptor)
	{
		cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<endl;
		return;
	}

	_detector->detect(frame.rgb, frame.kp);
	_descriptor->compute(frame.rgb, frame.kp, frame.desp);

	return;
}

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	cv::Point3f p;
	p.z = double(point.z) / camera.scale;
	p.x = (point.x - camera.cx) * p.z / camera.fx;
	p.y = (point.y - camera.cy) * p.z / camera.fy;
	return p;
}

int computeObjAndImage(FRAME& frame1, FRAME& frame2,\
		vector<cv::Point3f>& pts_obj, vector<cv::Point2f>& pts_img,\
		CAMERA_INTRINSIC_PARAMETERS& camera, \
		int min_good_match, double good_match_threshold)
{

	vector< cv::DMatch> matches;
	cv::BFMatcher matcher;
	matcher.match(frame1.desp, frame2.desp, matches);

	//cout<<"find total "<<matches.size()<<" matches."<<endl;
	vector<cv::DMatch> goodMatches;
	double minDis = 9999;
	for(size_t i = 0; i < matches.size(); i++)
	{
		if( matches[i].distance < minDis)
			minDis = matches[i].distance;
	}

	if(minDis < 10)
		minDis = 10;

	for(size_t i = 0; i < matches.size(); i++)
	{
		if( matches[i].distance < good_match_threshold*minDis)
			goodMatches.push_back(matches[i]);
	}

	//cout<<"good matches: "<<goodMatches.size()<<endl;
	if(goodMatches.size() <= min_good_match)
	{
		return -1;
	}

	for(size_t i = 0; i < goodMatches.size(); i++)
	{
		cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
		ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];
		if(d == 0)
			continue;
		pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));

		cv::Point3f pt(p.x, p.y, d);
		cv::Point3f pd = point2dTo3d(pt, camera);
		pts_obj.push_back(pd);
	}
	return 0;

}

// PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera)
// {
// 	PointCloud::Ptr newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);

// 	PointCloud::Ptr output(new PointCloud());
// 	pcl::transformPointCloud(*original, *output, T.matrix());
// 	*newCloud += *output;

// 	//Voxel gird filter subsamppling
// 	static pcl::VoxelGrid<PointT> voxel;
// 	static ParameterReader pd;
// 	double gridsize = atof(pd.getData("voxel_grid").c_str());
// 	voxel.setLeafSize(gridsize, gridsize,gridsize);
// 	voxel.setInputCloud(newCloud);
// 	PointCloud::Ptr tmp(new PointCloud());
// 	voxel.filter(*tmp);
// 	return tmp;
// }

Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
{
	cv::Mat R;
	cv::Rodrigues( rvec, R);
	Eigen::Matrix3d r;
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
		{
			r(i,j) = R.at<double>(i,j);
		}

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

	Eigen::AngleAxisd angle(r);
	T = angle;
	T(0, 3) = tvec.at<double>(0, 0);
	T(1, 3) = tvec.at<double>(1, 0);
	T(2, 3) = tvec.at<double>(2, 0);
	return T;
}