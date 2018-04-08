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
#include "command_args.h"
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#if defined(_WIN32) || defined(WIN32)
	#define blash "//"
#else
	#define blash "/"
#endif

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
				//PointT point;
				//ushort val = src.at<ushort>(i, j);		
				//point.x = (j - cx)*val / f;
				//point.y = (i - cy)*val / f;
				//point.z = val;
				//point.b = 255;
				//point.g = 255;
				//point.r = 0;
				//cloud->points[i*src.cols + j] = point;
			}
		}
	}

	return 0;
}

int main(int argc, char** argv)
{
	CommandArgs arg;
	string CONFIG;
	arg.param("config", CONFIG, "", "files within which directory will be processed");
	arg.parseArgs(argc, argv);
	
	if (CONFIG.empty()) {
		std::cout << "Usage: LiDARChessBoard --config <path for dataset>";
		return -1;
	}
	cout <<"load config file from: "<< CONFIG << endl;
	
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

#pragma region list files within input_dir
	boost::filesystem::path laser_path(input_dir);
	if (!boost::filesystem::exists(laser_path))
	{
		cout << input_dir << " does not exist" << endl;
		return -1;
	}
	boost::filesystem::path result_path(output_dir);
	
	if (!boost::filesystem::exists(result_path))
		boost::filesystem::create_directory(result_path);

	boost::filesystem::recursive_directory_iterator begin_iter(laser_path);
	boost::filesystem::recursive_directory_iterator end_iter;
	std::vector<std::string> file_path;
	std::vector<std::string> file_name;
	for ( ; begin_iter != end_iter; ++begin_iter)
	{
		if (boost::filesystem::is_directory(*begin_iter))
			continue;
		else
		{
			std::string strPath = begin_iter->path().string();		
			int blashPosition = strPath.find_last_of(blash); //windows
			int dotPosition = strPath.find_last_of(".");
			std::string name = strPath.substr(blashPosition + 1, dotPosition - blashPosition - 1);
			file_path.push_back(strPath);
			file_name.push_back(name);
		}
	}
#pragma endregion

	for (int i = 0; i < file_path.size(); ++i)
	{
		cout << file_path[i] << endl;

#pragma region load points
		string FILE_NAME = file_path[i];
		string NAME = FILE_NAME.substr(0, FILE_NAME.size() - 4);
		PointCloud::Ptr cloud(new PointCloud());
		if (loadPoints(FILE_NAME, cloud) < 0)
		{
			std::cerr << "error! no such directory: " << FILE_NAME << std::endl;
			return -1;
		}
#pragma endregion

#pragma region projection
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
#pragma endregion

#pragma region find chessboard
		cv::Mat colorProjection;
		cv::cvtColor(projection, colorProjection, CV_GRAY2BGR);
		std::vector< std::vector<cv::Point> > contours;
		findContours(projection, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		int requiredIndex = -1;
		double maxArea = 0;
		for (int i = 0; i < contours.size(); ++i)
		{
			double area = cv::contourArea(cv::Mat(contours[i]));
			if (area > maxArea)
			{
				maxArea = area;
				requiredIndex = i;
			}
		}
#pragma endregion

#pragma region prepare data
		vector<cv::Point> requiredContour = contours[requiredIndex];
		cv::Rect rect = boundingRect(requiredContour);
		rect.height += 0.40 *  resolution;
		rect.y -= 0.20 *  resolution;
		rect.width += 0.20 *  resolution;
		rect.x -= 0.10 *  resolution;

		cv::Mat boundingBox(projection.size(), CV_8UC1, cv::Scalar(0));
		cv::rectangle(boundingBox, rect, cv::Scalar(255), CV_FILLED);

		cv::Mat chessProjection(projection.size(), CV_8UC1, cv::Scalar(0));
		cv::drawContours(chessProjection, contours, requiredIndex, cv::Scalar(255), CV_FILLED);

		PointCloud::Ptr roughPlane(new PointCloud());
		for (int i = 0; i < roi->points.size(); ++i)
		{
			int indx = static_cast<int>((roi->points[i].x - 0.5) * resolution);
			int indy = static_cast<int>((roi->points[i].y + 2) * resolution);
			if (chessProjection.at<uchar>(indx, indy))
				roughPlane->points.push_back(roi->points[i]);
		}
#pragma endregion

#pragma region planeSegmentation
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;// Create the segmentation object
		seg.setOptimizeCoefficients(true);

		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.1);
		seg.setInputCloud(roughPlane);
		seg.segment(*inliers, *coefficients);

		float a = coefficients->values[0];
		float b = coefficients->values[1];
		float c = coefficients->values[2];
		float d = coefficients->values[3];

		PointCloud::Ptr plane(new PointCloud());
		for (int i = 0; i < roi->points.size(); ++i)
		{
			float x = roi->points[i].x;
			float y = roi->points[i].y;
			float z = roi->points[i].z;

			int indx = static_cast<int>((roi->points[i].x - 0.5) * resolution);
			int indy = static_cast<int>((roi->points[i].y + 2) * resolution);

			if (a*x + b*y + c*z + d < 0.15 && boundingBox.at<uchar>(indx, indy) > 0)
				plane->points.push_back(roi->points[i]);
		}
		cout << "inliers in chessboard: " << plane->points.size() << endl;
#pragma endregion

#pragma region save data

		std::string outTxt = output_dir + file_name[i] + ".txt";
		std::string outPly = output_dir + file_name[i] + ".ply";
		std::fstream outTxtHandle(outTxt, std::fstream::out);
		std::fstream outPlyHandle(outPly, std::fstream::out);
		PointCloud::Ptr combined(new PointCloud());

		outPlyHandle << "ply"
			<< '\n' << "format ascii 1.0"
			<< '\n' << "element vertex " << cloud->points.size() + plane->points.size()
			<< '\n' << "property float x"
			<< '\n' << "property float y"
			<< '\n' << "property float z"
			<< '\n' << "property uchar red"
			<< '\n' << "property uchar green"
			<< '\n' << "property uchar blue"
			<< '\n' << "end_header" << std::endl;
		for (int i = 0; i < plane->points.size(); ++i)
		{
			outPlyHandle << plane->points[i].x << ' ' << plane->points[i].y << ' ' << plane->points[i].z << " 255 0 255\n";
			outTxtHandle << plane->points[i].x << " " << plane->points[i].y << " " << plane->points[i].z << "\n";
		}
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			outPlyHandle << cloud->points[i].x << ' ' << cloud->points[i].y << ' ' << cloud->points[i].z << " 0 255 255\n";
		}

		outPlyHandle.close();
		outTxtHandle.close();
#pragma endregion
	}

	return 0;

}