/*************************************************************************
	> File Name: SLAM/slam/include/slamBase.cpp
	> Author: Peng YUN
	> Mail: u201313922@hust.edu.cn
	> Created Time: 2017年02月22日 
    > Implementation of slamBase.h
 ************************************************************************/
#include "slamBase.h"
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	PointCloud::Ptr cloud(new PointCloud);
	for(int m = 0; m < depth.rows; m+=2)
		for(int n = 0; n < depth.cols; n+=2)
		{
			ushort d = depth.ptr<ushort>(m)[n];
			if(d == 0)
				continue;

			PointT p;
			p.z = double(d) / camera.scale;
			p.x = (n - camera.cx) * p.z / camera.fx;
			p.y = (m - camera.cy) * p.z / camera.fy;
			p.b = rgb.ptr<uchar>(m)[n*3];
			p.g = rgb.ptr<uchar>(m)[n*3+1];
			p.r = rgb.ptr<uchar>(m)[n*3+2];

			cloud->points.push_back(p);
		}

		cloud->height = 1;
		cloud->width = cloud->points.size();
		cloud->is_dense = false;

		return cloud;
}

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	cv::Point3f p;
	p.z = double(point.z) / camera.scale;
	p.x = (point.x - camera.cx) * p.z / camera.fx;
	p.y = (point.y - camera.cy) * p.z / camera.fy;
	return p;
}

void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor)
{
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _descriptor;

	_detector = cv::FeatureDetector::create(detector.c_str());
	_descriptor = cv::DescriptorExtractor::create(descriptor.c_str());
	if(!_detector | !_descriptor)
	{
		cerr<<"Unknown detector or descriptor type ! "<<detector<<", "<<descriptor<<endl;
		return;
	}

	_detector->detect(frame.rgb, frame.kp);
	_descriptor->compute(frame.rgb, frame.kp, frame.desp);
	return;
}

RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
	static ParameterReader pd;
	vector<cv::DMatch> matches;
	cv::BFMatcher matcher;

	matcher.match(frame1.desp, frame2.desp, matches);
}
