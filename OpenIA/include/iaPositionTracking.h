#pragma once
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

#include "iaVitruvianAvatar.h"

struct skeleton {

	Eigen::Vector3f BoneJoints[16];
};

class PositionTracking
{
public:
	//skeleton allBoneJoints[2500];
	static bool _StartScan;
	static bool recordData;
	static bool initCalib;
	static bool isCalib;
	static bool readFile;

	static char* fileName;

	static int tFrameIndex;

	
public:
	void LiDARDataReader1();
	void LiDARDataReader2();
	void positionDetection(VitruvianAvatar &vAvatar);
	void getPositionData(float *pose);
	static void resetParameters();
	void updateBoneJoints(int fCount);
	//static void writeJointData(int tIndex);
	static void saveSFQData();

	static void saveSFQuatData(int noOfFrames);
	void saveQautData();
};

