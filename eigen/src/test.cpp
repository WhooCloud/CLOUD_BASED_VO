#include "slamBase.h"

FRAME readFrame( int index, ParameterReader& pd );

int main(int argc, char** argv)
{
	ParameterReader pd;
	CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
	FRAME frame1, frame2;
	frame1 = readFrame(1, pd);
	frame2 = readFrame(2, pd);

	PointCloud::Ptr cloud = image2PointCloud(frame1.rgb, frame1.depth, camera);

	bool visual = pd.getData("visualize_pointcloud") == string("yes");
	if(visual)
	{
		pcl::visualization::CloudViewer viewer("viewer");
		viewer.showCloud(cloud);
		while(!viewer.wasStopped())
		{

		}
	}
	string detector = pd.getData("detector");
	string descriptor = pd.getData("descriptor");
	computeKeyPointsAndDesp(frame1, detector, descriptor);
	computeKeyPointsAndDesp(frame2, detector, descriptor);
	cv::Mat imgShow;
	cv::drawKeypoints(frame1.rgb, frame1.kp, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::imshow("keypoints", imgShow);
	cv::waitKey(0);

	vector<cv::DMatch> matches;
	cv::BFMatcher matcher;
	matcher.match(frame1.desp, frame2.desp, matches);
	cout<<"Find total "<<matches.size()<<" matches."<<endl;
	cv::Mat imgMatches;
	cv::drawMatches(frame1.rgb, frame1.kp, frame2.rgb, frame2.kp, matches, imgMatches);
	cv::imshow("matches", imgMatches);
	cv::waitKey(0);

	return 0;
}

FRAME readFrame(int index, ParameterReader& pd)
{
	FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");
    
    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread(filename);
    if(f.rgb.data == NULL)
    	cerr<<filename<<" does not exist "<<endl;

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;
    f.depth = cv::imread(filename, -1);
    if(f.depth.data == NULL)
    	cerr<<filename<<" does not exist "<<endl;
    return f;

}