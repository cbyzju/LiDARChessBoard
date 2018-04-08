#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <fstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

double f = 525;
double cx = 640 / 2;
double cy = 480 / 2;

int loadPoints(const string FILE_NAME, PointCloud::Ptr& cloud)
{
	string FILE_EXT = FILE_NAME.substr(FILE_NAME.size() - 3, FILE_NAME.npos);

	if (FILE_EXT == "txt")
	{
		ifstream infile;
		infile.open(FILE_NAME);
		if (!infile) { cout << "failed to open "+ FILE_NAME << endl; return -1; }
		string x, y, z;
		int npts = 0;
		while (infile >> x >> y >> z)
		{
			if (x == "nan" || y == "nan" || z == "nan")
				continue;
			PointT point;
			point.x = stof(x);
			point.y = stof(y);
			point.z = stof(z);
			//point.b = 255;
			//point.g = 255;
			//point.r = 0;
			//cout << stof(x) << ", " << stof(y) << ", " << stof(z) << endl;
			cloud->points.push_back(point);
			npts++;
		}
		infile.close();
		cloud->height = 1;
		cloud->width = npts;
		cloud->points.resize(npts);
	}
	else if (FILE_EXT == "png" || FILE_EXT == "bmp" || FILE_EXT == "jpg")
	{
		cv::Mat src = cv::imread(FILE_NAME, CV_LOAD_IMAGE_UNCHANGED);
		cloud->height = 1;
		cloud->width = src.rows*src.cols;
		cloud->points.resize(src.rows*src.cols);
		for (int i = 0; i < src.rows; ++i)
		{
			for (int j = 0; j < src.cols; ++j)
			{
				PointT point;
				ushort val = src.at<ushort>(i, j);		
				point.x = (j - cx)*val / f;
				point.y = (i - cy)*val / f;
				point.z = val;
				//point.b = 255;
				//point.g = 255;
				//point.r = 0;
				cloud->points[i*src.cols + j] = point;
			}
		}
	}

	return 0;
}

//transfer 3D points to PointCloud, whose format is pcd / ply
int main()
{
	string CONFIG = "E:/windows/work/vs2015/LiDARChessBoard/config.yaml";
	cv::FileStorage fs(CONFIG, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "Failed to open settings file at: " << CONFIG << endl;
		return -1;
	}
	std::string input_dir = fs["input_dir"];
	std::string output_dir = fs["output_dir"];
	std::cout <<"input dir: "<< input_dir << endl;
	std::cout <<"output dir: "<<output_dir << endl;

	boost::filesystem::path laser_path(input_dir);

	if (!boost::filesystem::exists(laser_path))
	{
		cout << input_dir << " does not exist" << endl;
		return -1;
	}
	boost::filesystem::recursive_directory_iterator begin_iter(laser_path);
	boost::filesystem::recursive_directory_iterator end_iter;
	std::vector<std::string> file_path;
	for ( ; begin_iter != end_iter; ++begin_iter)
	{
		if (boost::filesystem::is_directory(*begin_iter))
			continue;
		else
		{
			std::string strPath = begin_iter->path().string();
			file_path.push_back(strPath);
		}
	}

	for (int i = 0; i < file_path.size(); ++i)
	{
		cout << file_path[i] << endl;
	}

	bool downSampling = false;
	bool normalization = false;

#pragma region load points
	string FILE_NAME = "E:/windows/work/vs2015/LiDARChessBoard/dataset/laser/002.txt";
	string NAME = FILE_NAME.substr(0, FILE_NAME.size() - 4);
	PointCloud::Ptr cloud(new PointCloud());
	if (loadPoints(FILE_NAME, cloud) < 0) 
	{ 
		std::cerr << "error! no such direcctory: " << FILE_NAME << std::endl;  
		return -1; 
	}
#pragma endregion

#pragma region normalization
	if (normalization)
	{
		Eigen::Matrix<double, 3, 1> centroid;
		centroid.setZero();
		int cp = cloud->points.size();
		for (int i = 0; i < cp; ++i)
		{
			centroid[0] += cloud->points[i].x;
			centroid[1] += cloud->points[i].y;
			centroid[2] += cloud->points[i].z;
		}
		centroid /= cp;
		for (int i = 0; i < cp; ++i)
		{
			cloud->points[i].x -= centroid[0];
			cloud->points[i].y -= centroid[1];
			cloud->points[i].z -= centroid[2];
		}
	}
#pragma endregion

#pragma region sampling

	if (downSampling)
	{
		//create the filtering object
		std::cout << "Number of points before downSampling: " << cloud->points.size() << std::endl;
		pcl::VoxelGrid<PointT> sampling;
		sampling.setLeafSize(10, 10, 10);
		sampling.setInputCloud(cloud);
		sampling.filter(*cloud);
		std::cout << "Number of points after downSampling: " << cloud->points.size() << std::endl;
	}
	cout << "cloud size: " << cloud->points.size() << " points." << endl;
	//pcl::io::savePCDFile(NAME + ".pcd", *cloud);
	//pcl::io::savePLYFile(NAME + ".ply", *cloud);
#pragma endregion

	PointCloud::Ptr roi(new PointCloud());
	int resolution = 50;
	int row = 3 * resolution + 1;
	int col = 4 * resolution + 1;
	cv::Mat projection(row, col, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		float x = cloud->points[i].x;
		float y = cloud->points[i].y;
		float z = cloud->points[i].z;
		if (x > 0.5 && x < 3.5 && y > -2 && y < 2)
		{
			int indx = static_cast<int>((x - 0.5) * resolution);
			int indy = static_cast<int>((y + 2) * resolution);
			projection.at<uchar>(indx, indy) = 255;
			roi->points.push_back(cloud->points[i]);
		}
	}
	
	cv::Mat colorProjection;
	cv::cvtColor(projection, colorProjection, CV_GRAY2BGR);
	std::vector< std::vector<cv::Point> > contours;
	findContours(projection, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int requiredIndex = -1;
	double maxArea = 0;
	for (int i = 0; i < contours.size(); ++i)
	{
		double area = cv::contourArea(cv::Mat(contours[i]));
		//cout <<"area: "<< area  << endl;
		if (area > maxArea)
		{
			maxArea = area;
			requiredIndex = i;
		}	
	}

	vector<cv::Point> requiredContour = contours[requiredIndex];

	cv::Mat boundingBox(projection.size(), CV_8UC1, cv::Scalar(0));
	cv::Rect rect = boundingRect(requiredContour);
	cv::rectangle(boundingBox, rect, cv::Scalar(255), CV_FILLED);
	imshow("boundingBox", boundingBox);
	cv::waitKey(1);

	cv::Mat chessProjection(projection.size(), CV_8UC1, cv::Scalar(0));
	cv::drawContours(chessProjection, contours, requiredIndex, cv::Scalar(255), CV_FILLED);
	imshow("chessProjection", chessProjection);
	cv::waitKey(1);


	cv::drawContours(colorProjection, contours, requiredIndex, cv::Scalar(0, 0, 255), 1);
	cv::rectangle(colorProjection, rect, cv::Scalar(0, 255, 0), 1);
	imshow("colorProjection", colorProjection);
	cv::waitKey(1);

	PointCloud::Ptr roughPlane(new PointCloud());
	for (int i = 0; i < roi->points.size(); ++i)
	{
		int indx = static_cast<int>((roi->points[i].x - 0.5) * resolution);
		int indy = static_cast<int>((roi->points[i].y + 2) * resolution);
		if (chessProjection.at<uchar>(indx, indy))
			roughPlane->points.push_back(roi->points[i]);

	}

#pragma region planeSegmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;// Create the segmentation object
	seg.setOptimizeCoefficients(true);// Optional
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud(roughPlane);
	seg.segment(*inliers, *coefficients);

	//cout<<"plane segementation cost "<< tt.toc()<<" ms, inliers = "<< inliers->indices.size ()<<endl;

	//std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
	//                                    << coefficients->values[1] << " "
	//                                    << coefficients->values[2] << " " 
	//                                    << coefficients->values[3] << std::endl;
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	PointCloud::Ptr plane(new PointCloud());
	//for (float xc = -2; xc <= 2; xc += 0.01)
	//{
	//	for (float yc = -2; yc <= 2; yc += 0.01)
	//	{
	//		PointT point;
	//		point.x = xc;
	//		point.y = yc;
	//		point.z = (-d - a*xc - b*yc) / c;
	//		plane->points.push_back(point);
	//	}
	//}
	for (int i = 0; i < roi->points.size(); ++i)
	{
		float x = roi->points[i].x;
		float y = roi->points[i].y;
		float z = roi->points[i].z;
		
		int indx = static_cast<int>((roi->points[i].x - 0.5) * resolution);
		int indy = static_cast<int>((roi->points[i].y + 2) * resolution);

		if (a*x + b*y + c*z + d < 0.1 && boundingBox.at<uchar>(indx, indy) > 0)
			plane->points.push_back(roi->points[i]);
	}
#pragma endregion

	//pcl::io::savePLYFile("E:/windows/work/vs2015/TransferDepthMapToPointCloud/dataset/ply/cloud" + std::to_string(i) + ".ply", *cloud);

#pragma region visualization
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 1, 1, v1);  //显示原点在左下角 (xmin, ymin, xmax, ymax);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("cloud", 10, 10, 20, 1, 1, 1, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> scolor(cloud, 0, 255, 255);
	viewer->addPointCloud<PointT>(cloud, scolor, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud", v1);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> splane(plane, 0, 255, 0);
	viewer->addPointCloud<PointT>(plane, splane, "plane", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane", v1);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#pragma endregion

	return 0;

}