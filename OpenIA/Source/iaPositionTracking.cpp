#include "iaPositionTracking.h"
#include "iaSphereUtility.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include "ouster_client/include/ouster/build.h"
#include "ouster_client/include/ouster/client.h"
#include "ouster_client/include/ouster/lidar_scan.h"
#include "ouster_client/include/ouster/types.h"

// Include VelodyneCapture Header
#include "VelodyneCapture.h"
#include "iaAcquireGesture.h"
//#include "OSLidar.h"

#define CV_PI   3.1415926535897932384626433832795

using namespace std;
using namespace ouster;

const int N_SCANS = 5;
const size_t UDP_BUF_SIZE = 65536;

void FATAL(const char* msg) {
	std::cerr << msg << std::endl;
	std::exit(EXIT_FAILURE);
}

iaAcquireGesture myAcquire;
iaAcquireGesture AcquireSFQ;
//OSLidar osData;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;

Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudViewer1(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudViewer2(new pcl::PointCloud<pcl::PointXYZRGB>);

bool vertical_Up = false;
bool vertical_Down = false;
float i_vary = 0;
//bool _StartScan = false;
//bool recordData = false;
//bool initCalib = false;

vector < pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr>> allClouds;
vector < pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr>> motionClouds;
pcl::PointCloud<pcl::PointXYZ> bodyCenter, bodyCenter_temp, bodyCenter_temp1, bodyCenter_temp2;
pcl::PointXYZ  pelvPosition;
ofstream posDataFile;
char file_Name[1024];
char fileMS[1024];
char dirName[1024] = "LiDARData";
char dirName2[1024] = "PositionData";
int fileIndex = 0;
int frameIndex = -1;
int motionIndex = 0;
int allFrame = 0;
int interpolate = 0;
int framesCountL1 = 0;
int framesCountL2 = 0;
int prevFrameNum = 0;

float xDiff = 0.0;
float yDiff = 0.0;
float zDiff = 0.0;

ofstream yposelidar;
ofstream plvPose;

extern MotionSphere ms;
//float imuLeg = 0.0;

float avatarHeight = 88.0;

bool flag2timeL1 = false;
bool flag1timeL1 = true;

bool flag2timeL2 = false;
bool flag1timeL2 = true;
bool PositionTracking::_StartScan;
bool PositionTracking::initCalib;
bool PositionTracking::recordData;
bool PositionTracking::isCalib;
bool PositionTracking::readFile;
bool PositionTracking::_startLidar;
bool PositionTracking::_initLidar;
unsigned int text_id = 0;
int PositionTracking::tFrameIndex;
int gtFrameIndex = 0;

bool bSaveKeyFrame = false;

//bool XsensConnection::ukeyPressed;
skeleton allBoneJoints[2500];

//Eigen::Vector3f rFoot;
//Eigen::Vector3f lFoot;
//Eigen::Vector3f rKnee;
//Eigen::Vector3f lKnee;
//Eigen::Vector3f cPlv(756.05,	808.213,	34.3041);
//Eigen::Vector3f lPlv;
//Eigen::Vector3f rPlv;
//Eigen::Vector3f cTorso;
//Eigen::Vector3f cSternum;
//Eigen::Vector3f cHead;
//Eigen::Vector3f rShldr;
//Eigen::Vector3f lShldr;
//Eigen::Vector3f rUarm;
//Eigen::Vector3f lUarm;
//Eigen::Vector3f rLarm;
//Eigen::Vector3f lLarm;

Eigen::Vector3f cPlv(822.586, 807.877, 41.1087);
Eigen::Vector3f cSternum(821.249, 652.163, 41.3504);
Eigen::Vector3f cTorso(820.336, 548.355, 40.7681);
Eigen::Vector3f cHead(819.078, 405.171, 39.965);
Eigen::Vector3f rShldr(820.690, 547.669, 162.478);
Eigen::Vector3f rUarm(821.926, 719.490, 163.636);
Eigen::Vector3f rLarm(823.188, 905.634, 163.248);
Eigen::Vector3f lShldr(819.983, 549.041, -80.942);
Eigen::Vector3f lUarm(821.065, 720.866, -81.375);
Eigen::Vector3f lLarm(821.714, 907.013, -80.83);
Eigen::Vector3f rPlv(822.586, 807.877, 112.702);
Eigen::Vector3f rKnee(822.397, 1040.56, 112.258);
Eigen::Vector3f rFoot(821.667, 1268.80, 111.225);
Eigen::Vector3f lPlv(822.586, 807.877, -30.485);
Eigen::Vector3f lKnee(821.921, 1040.56, -31.250);
Eigen::Vector3f lFoot(822.244, 1268.80, -31.923);

std::vector<Eigen::Vector3f> dataBuffer;

skeleton bonePose;

double groundPointY = 0;
bool bOnetime = false;

void writeJointData(int tIndex)
{
	time_t curr_time;
	curr_time = time(NULL);
	tm* tm_local = localtime(&curr_time);

	ofstream skelitonDataFile;
	char fileName[1024];

	sprintf_s(fileName, ".\\SkeletonData\\jointsData-00%d-%d%d%d.txt", fileIndex, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	skelitonDataFile.open(fileName);


	skelitonDataFile << "Frames:" << "\t" << tIndex << "\n" << "Height:" << "\t" << avatarHeight / 10.0 << "\n";

	for (int tCount = 0; tCount < tIndex; tCount++)
	{
		skelitonDataFile
			<< allBoneJoints[tCount].BoneJoints[0][0] << "\t" << allBoneJoints[tCount].BoneJoints[0][1] << "\t" << allBoneJoints[tCount].BoneJoints[0][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[1][0] << "\t" << allBoneJoints[tCount].BoneJoints[1][1] << "\t" << allBoneJoints[tCount].BoneJoints[1][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[2][0] << "\t" << allBoneJoints[tCount].BoneJoints[2][1] << "\t" << allBoneJoints[tCount].BoneJoints[2][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[3][0] << "\t" << allBoneJoints[tCount].BoneJoints[3][1] << "\t" << allBoneJoints[tCount].BoneJoints[3][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[4][0] << "\t" << allBoneJoints[tCount].BoneJoints[4][1] << "\t" << allBoneJoints[tCount].BoneJoints[4][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[5][0] << "\t" << allBoneJoints[tCount].BoneJoints[5][1] << "\t" << allBoneJoints[tCount].BoneJoints[5][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[6][0] << "\t" << allBoneJoints[tCount].BoneJoints[6][1] << "\t" << allBoneJoints[tCount].BoneJoints[6][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[7][0] << "\t" << allBoneJoints[tCount].BoneJoints[7][1] << "\t" << allBoneJoints[tCount].BoneJoints[7][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[8][0] << "\t" << allBoneJoints[tCount].BoneJoints[8][1] << "\t" << allBoneJoints[tCount].BoneJoints[8][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[9][0] << "\t" << allBoneJoints[tCount].BoneJoints[9][1] << "\t" << allBoneJoints[tCount].BoneJoints[9][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[10][0] << "\t" << allBoneJoints[tCount].BoneJoints[10][1] << "\t" << allBoneJoints[tCount].BoneJoints[10][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[11][0] << "\t" << allBoneJoints[tCount].BoneJoints[11][1] << "\t" << allBoneJoints[tCount].BoneJoints[11][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[12][0] << "\t" << allBoneJoints[tCount].BoneJoints[12][1] << "\t" << allBoneJoints[tCount].BoneJoints[12][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[13][0] << "\t" << allBoneJoints[tCount].BoneJoints[13][1] << "\t" << allBoneJoints[tCount].BoneJoints[13][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[14][0] << "\t" << allBoneJoints[tCount].BoneJoints[14][1] << "\t" << allBoneJoints[tCount].BoneJoints[14][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[15][0] << "\t" << allBoneJoints[tCount].BoneJoints[15][1] << "\t" << allBoneJoints[tCount].BoneJoints[15][2] << "\n";
	}

	skelitonDataFile.close();

	fileIndex++;
}

void writekeyframeJointData(int tIndex)
{
	time_t curr_time;
	curr_time = time(NULL);
	tm* tm_local = localtime(&curr_time);

	ofstream skelitonDataFile;
	char fileName[1024];


	sprintf_s(fileName, ".\\SkeletonData\\jointsKeyData-00%d-%d%d%d.txt", fileIndex, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	skelitonDataFile.open(fileName);


	skelitonDataFile << "Frames:" << "\t" << tIndex << "\n";

	int tCount = tIndex - 1;
	//for (int tCount = tIndex; tCount < tIndex; tCount++)
	{
		skelitonDataFile
			<< allBoneJoints[tCount].BoneJoints[0][0] << "\t" << allBoneJoints[tCount].BoneJoints[0][1] << "\t" << allBoneJoints[tCount].BoneJoints[0][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[1][0] << "\t" << allBoneJoints[tCount].BoneJoints[1][1] << "\t" << allBoneJoints[tCount].BoneJoints[1][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[2][0] << "\t" << allBoneJoints[tCount].BoneJoints[2][1] << "\t" << allBoneJoints[tCount].BoneJoints[2][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[3][0] << "\t" << allBoneJoints[tCount].BoneJoints[3][1] << "\t" << allBoneJoints[tCount].BoneJoints[3][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[4][0] << "\t" << allBoneJoints[tCount].BoneJoints[4][1] << "\t" << allBoneJoints[tCount].BoneJoints[4][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[5][0] << "\t" << allBoneJoints[tCount].BoneJoints[5][1] << "\t" << allBoneJoints[tCount].BoneJoints[5][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[6][0] << "\t" << allBoneJoints[tCount].BoneJoints[6][1] << "\t" << allBoneJoints[tCount].BoneJoints[6][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[7][0] << "\t" << allBoneJoints[tCount].BoneJoints[7][1] << "\t" << allBoneJoints[tCount].BoneJoints[7][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[8][0] << "\t" << allBoneJoints[tCount].BoneJoints[8][1] << "\t" << allBoneJoints[tCount].BoneJoints[8][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[9][0] << "\t" << allBoneJoints[tCount].BoneJoints[9][1] << "\t" << allBoneJoints[tCount].BoneJoints[9][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[10][0] << "\t" << allBoneJoints[tCount].BoneJoints[10][1] << "\t" << allBoneJoints[tCount].BoneJoints[10][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[11][0] << "\t" << allBoneJoints[tCount].BoneJoints[11][1] << "\t" << allBoneJoints[tCount].BoneJoints[11][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[12][0] << "\t" << allBoneJoints[tCount].BoneJoints[12][1] << "\t" << allBoneJoints[tCount].BoneJoints[12][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[13][0] << "\t" << allBoneJoints[tCount].BoneJoints[13][1] << "\t" << allBoneJoints[tCount].BoneJoints[13][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[14][0] << "\t" << allBoneJoints[tCount].BoneJoints[14][1] << "\t" << allBoneJoints[tCount].BoneJoints[14][2] << "\t"
			<< allBoneJoints[tCount].BoneJoints[15][0] << "\t" << allBoneJoints[tCount].BoneJoints[15][1] << "\t" << allBoneJoints[tCount].BoneJoints[15][2] << "\n";
	}

	skelitonDataFile.close();

	fileIndex++;
}

void exitAll()
{
	PositionTracking::_startLidar = true;

}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);

	/*time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);*/

	if (event.getKeySym() == "a" && event.keyDown())
	{
		cout << "A Key pressed" << endl;

		VitruvianAvatar::isLoaded = true;
	}

	if (event.getKeySym() == "l" && event.keyDown())
	{
		cout << "l Key pressed" << endl;
		PositionTracking::_startLidar = false;
	}

	if (event.getKeySym() == "i" && event.keyDown())
	{
		cout << "i Key pressed" << endl;
		PositionTracking::_initLidar = true;
	}

	if (event.getKeySym() == "s" && event.keyDown())//Start IMU and LiDAR
	{
		myAcquire.getXsensData();

		PositionTracking::_StartScan = true;
	}
	if (event.getKeySym() == "d" && event.keyDown())//Capture and construct Skeleton 
	{
		cout << "Check LAN Connection!!!" << endl;
		PositionTracking::initCalib = !PositionTracking::initCalib;
	}

	if (event.getKeySym() == "v" && event.keyDown())//Calibrate IMU sesnor
	{
		//anistart = !anistart;
		cout << "v Key pressed" << endl;

		iaAcquireGesture::calibIMU = true;
	}

	if (event.getKeySym() == "b" && event.keyDown())//Reset IMU sensor
	{
		//anistart = !anistart;
		/*for (int i = 0; i < (int)connectXS.mtwDevices.size(); ++i)
		{
			std::cout << "\n reset:" << connectXS.mtwDevices[i]->resetOrientation(XRM_Alignment) << std::endl;
		}
		firstCalib = true;*/
		cout << "b Key pressed" << endl;

		myAcquire.resetIMUSensor();
	}

	if (event.getKeySym() == "f" && event.keyDown())
	{
		cout << "f Key pressed" << endl;
		PositionTracking::readFile = true;//Read TotalCapture data
	}

	if (event.getKeySym() == "k" && event.keyDown())
	{
		cout << "k Key pressed" << endl;
		writekeyframeJointData(PositionTracking::tFrameIndex);

		bSaveKeyFrame = true;
	}

	if (event.getKeySym() == "n" && event.keyDown())
	{
		cout << "n Key pressed" << endl;

		PositionTracking::recordData = !PositionTracking::recordData;

		if (PositionTracking::recordData)
		{
			cout << "Record..." << endl;
			PositionTracking::resetParameters();

		}
		else
		{
			cout << "No More Record..." << endl;
			writeJointData(PositionTracking::tFrameIndex);
			//PositionTracking::saveSFQData();
			PositionTracking::saveSFQuatData(PositionTracking::tFrameIndex);

			// Data extraction
			std::cout << "Data buffer size: " << dataBuffer.size() << std::endl;

			std::ofstream file("D:\\PoseTrack19\\src\\out\\build\\SkeletonData\\test\\output.csv");

			if (!file.is_open()) {
				std::cerr << "Error: Unable to open file " << "output.csv" << std::endl;
				return;
			}

			// Write data to CSV
			for (const auto& vec : dataBuffer) {
				//file << vec[0] << "," << vec[2] << "," << -vec[1] << ", " << vec[3] << ", " << vec[5] << ", " << -vec[4] << ", " << vec[6] << ", " << vec[8] << ", " << -vec[7] << "\n";
				file << vec[0] << "," << vec[1] << "," << vec[2] << ", " << vec[3] << ", " << vec[4] << ", " << vec[5] << ", " << vec[6] << ", " << vec[7] << ", " << vec[8] << "\n";
				//file << vec[0] << "," << vec[1] << "," << vec[2] << "\n";
			}

			file.close();
			std::cout << "Data saved to " << "D:\\PoseTrack19\\src\\out\\build\\SkeletonData\\test\\output.csv" << std::endl;

			dataBuffer.clear(); // 벡터 초기화
			std::cout << "Data buffer has been cleared." << std::endl;
			
		}
		XsensConnection::ukeyPressed = !XsensConnection::ukeyPressed;
	}


	//// Writes data to file.
	//if (event.getKeySym() == "d" && event.keyDown())
	//{
	//	//sprintf_s(file_Name, "CData\\VeloScanData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);

	//	pcl::io::savePCDFileASCII("VeloScanData.pcd", *CloudViewer);
	//	std::cout << "Saving Completed" << std::endl;
	//}

	//if (event.getKeySym() == "a" && event.keyDown())
	//{
	//	std::cout << "Total frames:" << allClouds.size() << std::endl;

	//	for (int i = 0; i < allClouds.size(); i++)
	//	{
	//		//		cout << "Point Cloud " << i << "has got " << allClouds[i]->size() << " Points" << endl;

	//		std::stringstream s;
	//		s << ".\\MotionData\\motionFrame" << i << ".pcd";

	//		std::string savefile = s.str();

	//		if (allClouds[i]->size() > 0) {
	//			pcl::io::savePCDFileASCII(savefile, *allClouds[i]);
	//		}

	//	}

	//	for (int i = 0; i < motionClouds.size(); i++)
	//	{
	//		//			cout << "Point Cloud " << i << "has got " << motionClouds[i]->size() << " Points" << endl;
	//		std::stringstream s;
	//		s << ".\\framePos\\motionPose" << i << ".pcd";

	//		std::string savefile = s.str();

	//		if (motionClouds[i]->size() > 0) {
	//			pcl::io::savePCDFileASCII(savefile, *motionClouds[i]);
	//		}
	//	}


	//	sprintf_s(file_Name, "posdata\\posDataFile.txt");
	//	posDataFile.open(file_Name);

	//	for (int i = 0; i < bodyCenter.size(); i++)
	//	{
	//		posDataFile << bodyCenter.points[i].x << "\t" << bodyCenter.points[i].y << "\t" << bodyCenter.points[i].z << "\n";
	//	}
	//	posDataFile.close();

	//	frameIndex = 0;
	//	motionIndex = 0;
	//	allFrame = 0;
	//	interpolate = 0;
	//	bodyCenter.clear();
	//	allClouds.clear();
	//	motionClouds.clear();
	//}

	//if (event.getKeySym() == "g" && event.keyDown())
	//{
	//	recordData = !recordData;

	//	if (recordData)
	//	{
	//		frameIndex = 0;
	//		motionIndex = 0;
	//		allFrame = 0;
	//		interpolate = 0;
	//		bodyCenter.clear();
	//		allClouds.clear();
	//		motionClouds.clear();
	//	}
	//	/*else
	//	{
	//		posDataFile.close();
	//	}*/
	//}


	//if (event.getKeySym() == "Up" && event.keyDown())
	//{
	//	vertical_Up = true;
	//	vertical_Down = false;
	//	i_vary = 0;
	//}

	//if (event.getKeySym() == "Down" && event.keyDown())
	//{
	//	vertical_Up = false;
	//	vertical_Down = true;
	//	i_vary = 0;
	//}

	////Starts Scan 
	//if (event.getKeySym() == "s" && event.keyDown()) {
	//	_StartScan = !_StartScan;
	//	std::cout << " Scan Started " << std::endl;
	//}

	//if (event.getKeySym() == "v" && event.keyDown())
	//{
	//	i_vary = 0;
	//	CloudViewer->clear();
	//	basic_cloud_ptr->clear();

	//}
}
//
//void mouseEventOccurred(const pcl::visualization::MouseEvent &event, 	void* viewer_void)
//{
//	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
//		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
//	{
//		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;
//
//		char str[512];
//		sprintf(str, "text#%03d", text_id++);
//		viewer->addText("clicked here", event.getX(), event.getY(), str);
//	}
//}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZRGB>);

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPose(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

float prevPelv = 0.0;
int setFlag = 0;

float eEST_x = 2.0;
float eEST_y = 2.0;
float eEST_z = 2.0;

void updateLowerBody(Eigen::Vector3f diffR, Eigen::Vector3f diffL)
{
	rKnee = rKnee + diffR;
	rFoot = rFoot + diffR;
	rPlv = rPlv + diffR;

	lKnee = lKnee + diffL;
	lFoot = lFoot + diffL;
	lPlv = lPlv + diffL;
}



void updateUpperBody(Eigen::Vector3f diffP)
{
	cTorso = cTorso + diffP;
	cHead = cHead + diffP;
	rShldr = rShldr + diffP;
	lShldr = lShldr + diffP;
	rUarm = rUarm + diffP;
	lUarm = lUarm + diffP;
	rLarm = rLarm + diffP;
	lLarm = lLarm + diffP;
}

float KFEstimate(float cMEA, float eMEA, float pEST, float& eEST, float delta)
{

	float KG = eEST / (eEST + eMEA);

	float EST = pEST + KG * (cMEA - pEST);

	//cout << cMEA << "," << EST<<","<< KG <<","<<eEST<< endl;

	if (abs(cMEA - EST) > delta) eEST = 2;
	else eEST = (1 - KG) * (eEST);


	return EST;
}

bool poseDetection(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_A, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_B,
	pcl::PointCloud<pcl::PointXYZRGB>* cloud_out, Eigen::Vector3f& centerPos)
{

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(200.0f);


	std::vector<int> indicies;

	pcl::copyPointCloud(*cloud_A, *cloudA);
	pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indicies);

	// Add points from cloudA to octree
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();

	// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
	octree.switchBuffers();

	pcl::copyPointCloud(*cloud_B, *cloudB);
	pcl::removeNaNFromPointCloud(*cloudB, *cloudB, indicies);

	// Add points from cloudB to octree
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();

	std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);

	pcl::copyPointCloud(*cloudB, newPointIdxVector, *cloudC);


	if (cloudC->size() > 3000)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFilter(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (int i = 0; i < cloudC->size(); i++)
		{
			if (cloudC->points[i].x == 0)
				continue;

			cloudFilter->points.push_back(cloudC->points[i]);

			/*cloudFilter->points[i].x = cloudC->points[i].x;
			cloudFilter->points[i].y = cloudC->points[i].y;
			cloudFilter->points[i].z = cloudC->points[i].z;*/
		}

		pcl::copyPointCloud(*cloudFilter, *cloudC);
	}

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloudC);
	sor.setMeanK(150);
	sor.setStddevMulThresh(0.01);
	sor.filter(*cloudC);



	//Creating the KdTree object for the search method of the extraction
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//tree->setInputCloud(cloudC);

	//std::vector<pcl::PointIndices> cluster_indices;
	//pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	//ec.setClusterTolerance(80.0); 
	//ec.setMinClusterSize(100);
	//ec.setMaxClusterSize(100000);
	//ec.setSearchMethod(tree);
	//ec.setInputCloud(cloudC);
	//ec.extract(cluster_indices);


	//int j = 0;

	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	//{
	//	cloud_cluster->clear();
	//	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	//		cloud_cluster->points.push_back(cloudC->points[*pit]); //*
	//	cloud_cluster->width = cloud_cluster->points.size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;

	//	//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
	//	if (cloud_cluster->points.size() > 0)
	//	{
	//		if (j == 0)
	//		{

	//			pcl::copyPointCloud(*cloud_cluster, *temp_cloud_cluster);

	//			//for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	//			//	cloudPose->points.push_back(cloudC->points[*pit]); //*
	//			//cloudPose->width = cloudPose->points.size();
	//			//cloudPose->height = 1;
	//			//cloudPose->is_dense = true;

	//			break;
	//		}
	//		j++;
	//	}

	//}

	//pcl::copyPointCloud(*temp_cloud_cluster, *cloudC);

	//cloudC->width = cloudC->points.size();
	//cloudC->height = 1;
	//cloudC->is_dense = true;


	if (cloudC->size() <= 0)
	{
		std::cout << "Skipped " << std::endl;

		return false;
	}
	//std::cout << "--------->PointCloud Position " << std::endl;

	//for (int i = 0; i < cloudC->size(); i++)
	//{
	//	cloudC->points[i].x = cloudC->points[i].x;
	//	cloudC->points[i].y = cloudC->points[i].y;
	//	cloudC->points[i].z = cloudC->points[i].z;
	//}

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloudC, centroid);
	//posDataFile << centroid[0] << "\t" << centroid[1] << "\t" << centroid[2] << "\n";

	pcl::PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D(*cloudC, minPt, maxPt);

	//if (motionIndex > 0)
	//{
	//	float direction = 0.0;// = prevPelv - centroid[0];



	//	direction = bodyCenter.points[0].x - centroid[0];


	//	if (direction < -3.0 && direction > 3.0)
	//	{
	//		centroid[0];
	//	}
	//	else if (direction < -0.001)
	//	{
	//		centroid[0] = maxPt.x;
	//	}
	//	else if (direction > 0.001)
	//	{
	//		centroid[0] = maxPt.x;
	//	}


	//	/*if (setFlag > 10)
	//	{
	//		eEST = 2.0;
	//		setFlag = 0;
	//	}
	//	else
	//		setFlag++;*/

	//	centroid[0] = KFEstimate(centroid[0], 5, bodyCenter.points[motionIndex - 1].x, eEST_x, 15.0);

	//}
	//if (motionIndex == 0)
	//	centroid[0] = KFEstimate(centroid[0], 5, centroid[0], eEST_x, 15.0);
	//cout << centroid[0] << endl;
//cout << centroid[0] <<"," << endl;
//centroid[1] = minPt.y;

//std::cout << "Max x: " << maxPt.x << "Max y: " << maxPt.y << "Max z: " << maxPt.z << std::endl;
//std::cout << "Min x: " << maxPt.x <<  "Min y: " << minPt.y << "Min z: " << minPt.z << std::endl;

/*float bodyLength_X = sqrt((maxPt.x - maxPt.x) * (maxPt.x - maxPt.x) + (maxPt.y - maxPt.y) * (maxPt.y - maxPt.y) + (maxPt.z - maxPt.z) * (maxPt.z - maxPt.z));
float bodyLength_Y = sqrt((maxPt.x - maxPt.x) * (maxPt.x - maxPt.x) + (minPt.y - maxPt.y) * (minPt.y - maxPt.y) + (maxPt.z - maxPt.z) * (maxPt.z - maxPt.z));
float bodyLength_Z = sqrt((maxPt.x - maxPt.x) * (maxPt.x - maxPt.x) + (maxPt.y - maxPt.y) * (maxPt.y - maxPt.y) + (minPt.z - maxPt.z) * (minPt.z - maxPt.z));

centroid[0] = centroid[0] - (5.0 - (bodyLength_X / 2));*/
/*centroid[1] = centroid[1] - (47.5 - (bodyLength_Y / 2));*/
//centroid[2] = centroid[2] - (5 - (bodyLength_Z / 2));

/*for (int i = 0; i < cloudC->size(); ++i)
{
	cloudC->points[i].x = cloudC->points[i].x ;
	cloudC->points[i].y = cloudC->points[i].y ;
	cloudC->points[i].z = cloudC->points[i].z ;
}*/
	centerPos[0] = centroid[0];
	centerPos[1] = minPt.y;
	centerPos[2] = centroid[2];
	pcl::copyPointCloud(*cloudC, *cloud_out);

	return true;
}

void PositionTracking::OSlidar()
{
	std::cerr << "Ouster client example " << ouster::CLIENT_VERSION << std::endl;
	/*
	 * The sensor client consists of the network client and a library for
	 * reading and working with data.
	 *
	 * The network client supports reading and writing a limited number of
	 * configuration parameters and receiving data without working directly with
	 * the socket APIs. See the `client.h` for more details. The minimum
	 * required parameters are the sensor hostname/ip and the data destination
	 * hostname/ip.
	 */
	 /* const std::string sensor_hostname = argv[1];
	  const std::string data_destination = argv[2];*/

	  //const std::string sensor_hostname = "os-992037000174"; //OS-0 [32]
	const std::string sensor_hostname = "os-122105000172"; //OS-1 [32] // new lidar
	//const std::string sensor_hostname = "os-122106000321"; //OS-1 [32]
	const std::string data_destination = "192.168.1.27"; //"165.254.219.210"

	//const std::string sensor_hostname = "os-992037000174"; //OS-0 [32]
	//const std::string data_destination = "192.168.1.27";//"192.168.+1.20"

	std::cerr << "Connecting to \"" << sensor_hostname << "\"... " << data_destination << std::endl;

	auto handle = sensor::init_client(sensor_hostname, data_destination);
	std::cout << "handle : " << handle << std::endl; // return shared pointer address

	if (!handle) FATAL("Failed to connect");
	std::cerr << "ok" << std::endl;

	/*
	 * Configuration and calibration parameters can be queried directly from the
	 * sensor. These are required for parsing the packet stream and calculating
	 * accurate point clouds.
	 */
	std::cerr << "Gathering metadata..." << std::endl;
	auto metadata = sensor::get_metadata(*handle);

	// Raw metadata can be parsed into a `sensor_info` struct
	sensor::sensor_info info = sensor::parse_metadata(metadata);

	size_t w = info.format.columns_per_frame;
	size_t h = info.format.pixels_per_column;

	std::cerr << "  Firmware version:  " << info.fw_rev
		<< "\n  Serial number:     " << info.sn
		<< "\n  Product line:      " << info.prod_line
		<< "\n  Scan dimensions:   " << w << " x " << h << std::endl;

	// A LidarScan holds lidar data for an entire rotation of the device
	std::vector<LidarScan> scans{ N_SCANS, LidarScan{w, h} };

	std::unique_ptr<ouster::LidarScan> ls_read(new ouster::LidarScan(w, h));
	std::unique_ptr<ouster::LidarScan> ls_write(new ouster::LidarScan(w, h));

	// A ScanBatcher can be used to batch packets into scans
	sensor::packet_format pf = sensor::get_format(info);
	ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

	/*
	 * The network client provides some convenience wrappers around socket APIs
	 * to facilitate reading lidar and IMU data from the network. It is also
	 * possible to configure the sensor offline and read data directly from a
	 * UDP socket.
	 */
	std::cerr << "Capturing points... ";

	// buffer to store raw packet data
	//std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

	std::unique_ptr<uint8_t[]> lidar_buf(
		new uint8_t[pf.lidar_packet_size + 1]);

	std::mutex swap_mtx;
	int file_ind = 0;

	std::thread poll([&] {
		while (!PositionTracking::_startLidar) {
			//for (int i = 0; i < N_SCANS;) {
				// wait until sensor data is available
			sensor::client_state st = sensor::poll_client(*handle);

			// check for error status
			if (st & sensor::CLIENT_ERROR)
				FATAL("Sensor client returned error state!");

			// check for lidar data, read a packet and add it to the current batch
			if (st & sensor::client_state::LIDAR_DATA) {
				if (!sensor::read_lidar_packet(*handle, lidar_buf.get(), pf))
					FATAL("Failed to read a packet of the expected size!");

				// batcher will return "true" when the current scan is complete
				if (batch_to_scan(lidar_buf.get(), *ls_write)) {

					std::lock_guard<std::mutex> lk(swap_mtx);
					std::swap(ls_read, ls_write);
					// LidarScan provides access to azimuth block data and headers
					auto n_invalid = std::count_if(
						ls_read->headers.begin(), ls_read->headers.end(),
						[](const LidarScan::BlockHeader& h) {
							return h.status != 0xffffffff;
						});
					// retry until we receive a full scan
					//if (n_invalid == 0) i++;
				}
			}

			// check if IMU data is available (but don't do anything with it)
			if (st & sensor::IMU_DATA) {
				sensor::read_imu_packet(*handle, lidar_buf.get(), pf);
			}

			if (st & sensor::EXIT) {
				PositionTracking::_startLidar = true;
				break;
			}
		}
		});
	std::cerr << "ok" << std::endl;

	/*
	 * The example code includes functions for efficiently and accurately
	 * computing point clouds from range measurements. LidarScan data can also
	 * be accessed directly using the Eigen[0] linear algebra library.
	 *
	 * [0] http://eigen.tuxfamily.org
	 */
	std::cerr << "Computing point clouds... " << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> indicies;

	std::thread update_draw([&] {
		while (!PositionTracking::_startLidar) {
			// pre-compute a table for efficiently calculating point clouds from range
			XYZLut lut = ouster::make_xyz_lut(info);
			std::vector<LidarScan::Points> clouds;
			//cout <<"updata"<<endl;
			//for (const LidarScan& scan : ls_read) {
				// compute a point cloud using the lookup table
			clouds.push_back(ouster::cartesian(*ls_read, lut));
			//cout << clouds.size() << endl;
			// channel fields can be queried as well
			auto n_returns = (ls_read->field(LidarScan::Field::RANGE) != 0).count();

			//std::cerr << "  Frame no. " << ls_read->frame_id << " with " << n_returns	<< " returns" << std::endl;

			pcl::PointXYZRGB basic_point;
			for (const LidarScan::Points& cloud : clouds) {

				// write each point, filtering out points without returns
				for (int i = 0; i < cloud.rows(); i++)
				{
					auto xyz = cloud.row(i);
					if (!xyz.isApproxToConstant(0.0))
					{

						basic_point.x = xyz(0) * 1000.0;
						basic_point.y = -xyz(2) * 1000.0; //-xyz(2) * 1000.0; //for os-0-32
						basic_point.z = xyz(1) * 1000.0;

						basic_point.r = 0.9;
						basic_point.g = 0.1;
						basic_point.b = 0.1;

						if (basic_point.x < 6500 && basic_point.y > -2750 && basic_point.y < 600)
							//if (basic_point.x < 6500 && basic_point.y < 2750 && basic_point.y > -600)
							point_cloud_ptr->points.push_back(basic_point);
					}
				}

				*CloudViewer1 = *point_cloud_ptr;
				//cout << CloudViewer1->size() << endl;

				pcl::removeNaNFromPointCloud(*CloudViewer1, *CloudViewer1, indicies);
				pcl::copyPointCloud(*CloudViewer1, *cloudA);
				cloudA->width = cloudA->size();
				cloudA->height = 1;

				//cout << "No. of points" << cloudA->size() << endl;
				if (PositionTracking::_initLidar)
				{
					std::stringstream s;
					s << ".\\Target\\Calib.pcd";

					std::string savefile = s.str();

					flag1timeL1 = false;


					if (cloudA->size() > 2000)
					{
						pcl::io::savePCDFileASCII(savefile, *cloudA);
						cout << "Target saved!!!!!!" << endl;
						flag1timeL1 = true;
						PositionTracking::_initLidar = false;
					}
				}

				if (PositionTracking::recordData)
				{
					std::stringstream s;
					s << ".\\" << "LiDARData" << "\\LiDAR" << framesCountL1 << ".pcd";

					std::string savefile = s.str();

					if (cloudA->size() > 0)
					{
						// 0618
						//pcl::io::savePCDFileASCII(savefile, *cloudA);
						framesCountL1++;
					}
					//else
						//cout << "Framecount" << framesCount << endl;
				}
				else
				{
					framesCountL1 = 0;
				}

				point_cloud_ptr->clear();
			}


			/*
			 * Write output to CSV files. The output can be viewed in a point cloud
			 * viewer like CloudCompare:
			 *
			 * [0] https://github.com/cloudcompare/cloudcompare
			 */
			 //std::cerr << "Writing files... " << std::endl;


			//std::string file_base{ "cloud_" };
			//for (const LidarScan::Points& cloud : clouds) {
			//	std::string filename = file_base + std::to_string(file_ind++) + ".csv";
			//	std::ofstream out;
			//	out.open(filename);
			//	out << std::fixed << std::setprecision(4);

			//	// write each point, filtering out points without returns
			//	for (int i = 0; i < cloud.rows(); i++) {
			//		auto xyz = cloud.row(i);
			//		if (!xyz.isApproxToConstant(0.0))
			//			out << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
			//	}

			//	out.close();
			//	std::cerr << "  Wrote " << filename << std::endl;
			//}

		}
		});

	poll.join();
	update_draw.join();
	//return EXIT_SUCCESS;

	//return 0;
}

void PositionTracking::LiDARDataReader2()
{

	// Open VelodyneCapture that retrieve from Sensor

	const boost::asio::ip::address address = boost::asio::ip::address::from_string("192.168.1.202");
	const unsigned short port = 2468;
	velodyne::VLP16Capture capture(address, port);
	// velodyne::HDL32ECapture capture( address, port );

	// Open VelodyneCapture that retrieve from PCAP
   /* const std::string file_Name = "\\Velodyne-VLP-16-Data-P2.pcap";
	velodyne::VLP16Capture capture( file_Name );*/
	//velodyne::HDL32ECapture capture( file_Name );

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<int> indicies;

	//pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indicies);


	if (!capture.isOpen()) {
		std::cerr << "Can't open VelodyneCapture." << std::endl;
		exit;
		//return -1;
	}
	else
	{
		std::cerr << "Open VelodyneCapture." << std::endl;
	}

	// Capture One Rotation Data
	std::vector<velodyne::Laser> lasers;



	while (capture.isRun())
	{
		lasers.clear();

		if (PositionTracking::_StartScan)
		{
			capture >> lasers;

			if (lasers.empty())
			{
				//std::cerr << "Empty" << std::endl;

				continue;
			}

			//std::cerr << "Point Size1-------------------->:" << lasers.size() << std::endl;
			//lasers.size();
			CloudViewer2->clear();


			for (const velodyne::Laser& laser : lasers)
			{

				const double distance = static_cast<double>(laser.distance);
				const double azimuth = laser.azimuth * CV_PI / 180.0;
				const double vertical = laser.vertical * CV_PI / 180.0;

				float x = static_cast<float>((distance * std::cos(vertical)) * std::sin(azimuth));
				float y = static_cast<float>((distance * std::cos(vertical)) * std::cos(azimuth));
				float z = static_cast<float>((distance * std::sin(vertical)));
				std::cerr << "Point distance:" << distance << std::endl;

				if (x == 0.0f && y == 0.0f && z == 0.0f)
				{
					x = std::numeric_limits<float>::quiet_NaN();
					y = std::numeric_limits<float>::quiet_NaN();
					z = std::numeric_limits<float>::quiet_NaN();
				}

				pcl::PointXYZRGB basic_point;


				basic_point.x = y - 630.0;
				basic_point.y = -z + 1035.0;
				basic_point.z = -x + 140.0;

				basic_point.r = 0.9;
				basic_point.g = 0.1;
				basic_point.b = 0.1;

				if (y < 2800.0)
				{
					point_cloud_ptr->points.push_back(basic_point);
				}
			}//end of for loop

			//logFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << writeindex << "\n";

			*CloudViewer2 += *point_cloud_ptr;

			point_cloud_ptr->clear();

			pcl::removeNaNFromPointCloud(*CloudViewer2, *cloudA, indicies);
			//cout << "size" << cloudA->size() << endl;
			//cout << "LiDAR-2" << cloudA->size() << endl;
			if (PositionTracking::recordData)
			{
				std::stringstream s;
				s << ".\\" << dirName << "\\LiDAR-2-" << framesCountL2 << ".pcd";

				std::string savefile = s.str();

				if (cloudA->size() > 0)
				{

					//pcl::io::savePCDFileASCII(savefile, *cloudA);
					framesCountL2++;
				}
				//else
					//cout << "Framecount" << framesCount << endl;
			}
			else
			{
				framesCountL2 = 0;
			}

			/*time_t curr_time;
			curr_time = time(NULL);
			tm *tm_local = localtime(&curr_time);
			std::cout << "Time:" << tm_local->tm_hour << "\t" << tm_local->tm_min << "\t" << tm_local->tm_sec << std::endl;*/


			if (flag1timeL2)
			{
				std::stringstream s;
				s << ".\\Target\\Calib2.pcd";

				std::string savefile = s.str();

				flag1timeL2 = false;


				if (cloudA->size() > 1800)
				{
					pcl::io::savePCDFileASCII(savefile, *cloudA);

					cout << "Target saved!!!!!!" << endl;

					flag2timeL2 = true;
				}
				else
				{
					flag1timeL2 = true;
					flag2timeL2 = false;
				}
			}
		}
		else
		{
			capture >> lasers;
			lasers.clear();
		}//end of laser scan

	}
}

void PositionTracking::LiDARDataReader1()
{

	// Open VelodyneCapture that retrieve from Sensor
	const boost::asio::ip::address address = boost::asio::ip::address::from_string("192.168.1.201");
	const unsigned short port = 2368;
	velodyne::VLP16Capture capture(address, port);
	// velodyne::HDL32ECapture capture( address, port );

	// Open VelodyneCapture that retrieve from PCAP
   /* const std::string file_Name = "\\Velodyne-VLP-16-Data-P2.pcap";
	velodyne::VLP16Capture capture( file_Name );*/
	//velodyne::HDL32ECapture capture( file_Name );

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZRGB>);

	//std::stringstream s;
	//s << ".\\Target\\Calib.pcd";

	//std::string loadfile = s.str();

	//if (pcl::io::loadPCDFile(loadfile, *cloudA) == -1)
	//{
	//	std::cerr << "Can't load target calib file." << std::endl;
	//	exit;
	//}
	std::vector<int> indicies;

	//pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indicies);


	if (!capture.isOpen()) {
		std::cerr << "Can't open VelodyneCapture." << std::endl;
		exit;
		//return -1;
	}
	else
	{
		std::cerr << "Open VelodyneCapture." << std::endl;
	}

	// Capture One Rotation Data
	std::vector<velodyne::Laser> lasers;


	/*ofstream logFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	logFile.open("logData/lidarlog.txt");*/

	while (capture.isRun())
	{
		lasers.clear();

		if (PositionTracking::_StartScan)
		{
			capture >> lasers;

			if (lasers.empty())
			{
				//std::cerr << "Empty" << std::endl;

				continue;
			}

			//std::cerr << "Point Size1-------------------->:" << lasers.size() << std::endl;
			//lasers.size();
			CloudViewer1->clear();


			for (const velodyne::Laser& laser : lasers)
			{

				const double distance = static_cast<double>(laser.distance);
				const double azimuth = laser.azimuth * CV_PI / 180.0;
				const double vertical = laser.vertical * CV_PI / 180.0;

				float x = static_cast<float>((distance * std::cos(vertical)) * std::sin(azimuth));
				float y = static_cast<float>((distance * std::cos(vertical)) * std::cos(azimuth));
				float z = static_cast<float>((distance * std::sin(vertical)));
				//std::cerr << "Point distance:" << distance << std::endl;

				if (x == 0.0f && y == 0.0f && z == 0.0f) {
					x = std::numeric_limits<float>::quiet_NaN();
					y = std::numeric_limits<float>::quiet_NaN();
					z = std::numeric_limits<float>::quiet_NaN();
				}

				pcl::PointXYZRGB basic_point;
				basic_point.x = x;
				basic_point.y = y;
				basic_point.z = z;

				basic_point.r = 0.9;
				basic_point.g = 0.1;
				basic_point.b = 0.1;

				if (x < 2800.0)
				{
					basic_cloud_ptr->points.push_back(basic_point);
				}
			}//end of for loop

			//logFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << writeindex << "\n";

			*CloudViewer1 += *basic_cloud_ptr;

			basic_cloud_ptr->clear();

			pcl::removeNaNFromPointCloud(*CloudViewer1, *cloudA, indicies);
			//cout << "LiDAR-1" << cloudA->size() << endl;
			if (PositionTracking::recordData)
			{
				std::stringstream s;
				s << ".\\" << dirName << "\\LiDAR-1-" << framesCountL1 << ".pcd";

				std::string savefile = s.str();

				if (cloudA->size() > 0)
				{

					//pcl::io::savePCDFileASCII(savefile, *cloudA);
					framesCountL1++;
				}
				//else
					//cout << "Framecount" << framesCount << endl;
			}
			else
			{
				framesCountL1 = 0;
			}

			/*time_t curr_time;
			curr_time = time(NULL);
			tm *tm_local = localtime(&curr_time);
			std::cout << "Time:" << tm_local->tm_hour << "\t" << tm_local->tm_min << "\t" << tm_local->tm_sec << std::endl;*/


			if (flag1timeL1)
			{
				std::stringstream s;
				s << ".\\Target\\Calib1.pcd";

				std::string savefile = s.str();

				flag1timeL1 = false;


				if (cloudA->size() > 2000)
				{
					pcl::io::savePCDFileASCII(savefile, *cloudA);

					cout << "Target saved!!!!!!" << endl;

					flag2timeL1 = true;
				}
				else
				{
					flag1timeL1 = true;
					flag2timeL1 = false;
				}
			}
		}
		else
		{
			capture >> lasers;
			lasers.clear();
		}//end of laser scan

	}
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr InitCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

float getActulPelivs(Eigen::Vector3f headPos, Eigen::Vector3f centerPos, float height)
{
	//float currentPlv = centroid - head;
	float i = 0.0;
	float point1 = centerPos[0];
	float point2 = centerPos[0];

	for (;;)
	{

		float dist1 = sqrt(pow(headPos[0] - point1, 2) + pow(headPos[1] - centerPos[1], 2) + pow(headPos[2] - centerPos[2], 2));
		float dist2 = sqrt(pow(headPos[0] - point2, 2) + pow(headPos[1] - centerPos[1], 2) + pow(headPos[2] - centerPos[2], 2));

		//cout << "  Point1:" << dist1 << "  Point2:" << dist2 << "  height x 0.4:" << height * 0.4 << "  i:"<< i << endl;

		i += 0.5;

		point1 = centerPos[0] + i;
		point2 = centerPos[0] - i;

		if (dist1 > (height)-2 && (height)+2 > dist1)
		{
			return point1;
		}

		if (dist2 > (height)-2 && (height)+2 > dist2)
		{
			return point2;
		}

		if (i == 25)
			return centerPos[0];

	}
}

void getGroundPoint(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_A, Eigen::Vector3f& centerPos)
{
	//cout << "Get Ground Point!!!" << endl;
	pcl::copyPointCloud(*cloud_A, *InitCloud);
	//pcl::removeNaNFromPointCloud(*InitCloud, *InitCloud, indicies);
	Eigen::Vector3f centroid;

	centroid[0] = 0.0;
	centroid[1] = 0.0;
	centroid[2] = 0.0;

	for (int i = 0; i < InitCloud->size(); i++)
	{
		//The max-Y point exactly below the over head LiDAr sensor
		if (InitCloud->points[i].x  < 10.0 && InitCloud->points[i].z < 30.0 && InitCloud->points[i].x  > -10.0 && InitCloud->points[i].z  > -30.0)
		{
			//cout << "Ground truth-> X:" << InitCloud->points[i].x  << "\t Y:" << InitCloud->points[i].y  << "\t Z:" << InitCloud->points[i].z  << endl;

			if (centroid[1] < InitCloud->points[i].y)
			{
				centroid[0] = InitCloud->points[i].x;
				centroid[1] = InitCloud->points[i].y;
				centroid[2] = InitCloud->points[i].z;
			}
		}
	}
	centerPos = centroid;
}

void updatedriftPosition(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_A)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//cout << "Get Ground Point!!!" << endl;
	pcl::copyPointCloud(*cloud_A, *tempCloud);
	Eigen::Vector3f centroid;

	centroid[0] = 0.0;
	centroid[1] = 0.0;
	centroid[2] = 0.0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

	kdtree.setInputCloud(tempCloud);

	pcl::PointXYZRGB searchPoint;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);


	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << (*tempCloud)[pointIdxRadiusSearch[i]].x
			<< " " << (*tempCloud)[pointIdxRadiusSearch[i]].y
			<< " " << (*tempCloud)[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
}


void DrawBoneCylinder(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, Eigen::Vector3f Point1, Eigen::Vector3f Point2, char* boneID)
{
	//pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);

	cylinder_coeff.values[0] = Point1[0];
	cylinder_coeff.values[1] = Point1[1];
	cylinder_coeff.values[2] = Point1[2];
	cylinder_coeff.values[3] = (Point2[0] - Point1[0]);
	cylinder_coeff.values[4] = (Point2[1] - Point1[1]);
	cylinder_coeff.values[5] = (Point2[2] - Point1[2]);
	cylinder_coeff.values[6] = 10.0;//cylRadius;

	std::string boneName(boneID);
	viewer->removeShape(boneName);
	viewer->addCylinder(cylinder_coeff, boneName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0 /*R,G,B*/, boneName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, boneName);
}

void DrawCylinder(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, Eigen::Vector3f Point1, Eigen::Vector3f Point2, char* boneID)
{
	//pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);

	cylinder_coeff.values[0] = Point1[0];
	cylinder_coeff.values[1] = Point1[1];
	cylinder_coeff.values[2] = Point1[2];
	cylinder_coeff.values[3] = Point2[0];
	cylinder_coeff.values[4] = Point2[1];
	cylinder_coeff.values[5] = Point2[2];
	cylinder_coeff.values[6] = 10.0;//cylRadius;

	std::string boneName(boneID);
	viewer->removeShape(boneName);
	viewer->addCylinder(cylinder_coeff, boneName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0 /*R,G,B*/, boneName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, boneName);
}


void PositionTracking::updateBoneJoints(int fCount)
{
	bonePose.BoneJoints[0] = cPlv;
	bonePose.BoneJoints[1] = cSternum;
	bonePose.BoneJoints[2] = cTorso;
	bonePose.BoneJoints[3] = cHead;
	bonePose.BoneJoints[4] = rShldr;
	bonePose.BoneJoints[5] = rUarm;
	bonePose.BoneJoints[6] = rLarm;
	bonePose.BoneJoints[7] = lShldr;
	bonePose.BoneJoints[8] = lUarm;
	bonePose.BoneJoints[9] = lLarm;
	bonePose.BoneJoints[10] = rPlv;
	bonePose.BoneJoints[11] = rKnee;
	bonePose.BoneJoints[12] = rFoot;
	bonePose.BoneJoints[13] = lPlv;
	bonePose.BoneJoints[14] = lKnee;
	bonePose.BoneJoints[15] = lFoot;

	allBoneJoints[fCount].BoneJoints[0] = bonePose.BoneJoints[0] / 10.0;
	allBoneJoints[fCount].BoneJoints[1] = bonePose.BoneJoints[1] / 10.0;
	allBoneJoints[fCount].BoneJoints[2] = bonePose.BoneJoints[2] / 10.0;
	allBoneJoints[fCount].BoneJoints[3] = bonePose.BoneJoints[3] / 10.0;
	allBoneJoints[fCount].BoneJoints[4] = bonePose.BoneJoints[4] / 10.0;
	allBoneJoints[fCount].BoneJoints[5] = bonePose.BoneJoints[5] / 10.0;
	allBoneJoints[fCount].BoneJoints[6] = bonePose.BoneJoints[6] / 10.0;
	allBoneJoints[fCount].BoneJoints[7] = bonePose.BoneJoints[7] / 10.0;
	allBoneJoints[fCount].BoneJoints[8] = bonePose.BoneJoints[8] / 10.0;
	allBoneJoints[fCount].BoneJoints[9] = bonePose.BoneJoints[9] / 10.0;
	allBoneJoints[fCount].BoneJoints[10] = bonePose.BoneJoints[10] / 10.0;
	allBoneJoints[fCount].BoneJoints[11] = bonePose.BoneJoints[11] / 10.0;
	allBoneJoints[fCount].BoneJoints[12] = bonePose.BoneJoints[12] / 10.0;
	allBoneJoints[fCount].BoneJoints[13] = bonePose.BoneJoints[13] / 10.0;
	allBoneJoints[fCount].BoneJoints[14] = bonePose.BoneJoints[14] / 10.0;
	allBoneJoints[fCount].BoneJoints[15] = bonePose.BoneJoints[15] / 10.0;
}

bool isJointBelowGround(float& hDiff)
{
	float maxYdiff = 0;
	hDiff = groundPointY;

	for (int i = 0; i < 16; i++)
	{
		if (bonePose.BoneJoints[i][1] > maxYdiff)
		{
			maxYdiff = bonePose.BoneJoints[i][1];
		}
		/*if (bonePose.BoneJoints[i][1] > groundPointY)
		{
			maxYdiff = bonePose.BoneJoints[i][1] - groundPointY;
		}
		if(maxYdiff > hDiff)
		hDiff = maxYdiff;*/
	}
	hDiff = maxYdiff - groundPointY;
	if (hDiff < 0)
		return false;
	else
		return true;/*
	if (hDiff > groundPointY)
		return true;
	return false;*/
}



void PositionTracking::saveSFQData()
{
	myAcquire.saveRawQuatData(tFrameIndex);
}


void DrawJointSphere(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, Eigen::Vector3f Point1, char* boneID)
{
	//pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	pcl::PointXYZ spherPoint;
	spherPoint.x = Point1[0];	spherPoint.y = Point1[1];	spherPoint.z = Point1[2];
	std::string jointName(boneID);
	viewer->removeShape(jointName);
	viewer->addSphere(spherPoint, 20.0, jointName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, jointName);
}

void DrawPlvJointSphere(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, Eigen::Vector3f Point1, char* boneID)
{
	//pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	pcl::PointXYZ spherPoint;
	spherPoint.x = Point1[0];	spherPoint.y = Point1[1];	spherPoint.z = Point1[2];
	std::string jointName(boneID);
	viewer->removeShape(jointName);
	viewer->addSphere(spherPoint, 20.0, jointName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, jointName);
}

void DrawrPlvJointSphere(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, Eigen::Vector3f Point1, char* boneID)
{
	//pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	pcl::PointXYZ spherPoint;
	spherPoint.x = Point1[0];	spherPoint.y = Point1[1];	spherPoint.z = Point1[2];
	std::string jointName(boneID);
	viewer->removeShape(jointName);
	viewer->addSphere(spherPoint, 20.0, jointName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, jointName);
}

void DrawlPlvJointSphere(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, Eigen::Vector3f Point1, char* boneID)
{
	//pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

	pcl::PointXYZ spherPoint;
	spherPoint.x = Point1[0];	spherPoint.y = Point1[1];	spherPoint.z = Point1[2];
	std::string jointName(boneID);
	viewer->removeShape(jointName);
	viewer->addSphere(spherPoint, 20.0, jointName);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, jointName);
}


void PositionTracking::positionDetection(VitruvianAvatar& vAvatar)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr real_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr real_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudL2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr calibL1L2(new pcl::PointCloud<pcl::PointXYZRGB>);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	int v1(0);
	//int v2(1);
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(1, 1, 1);

	std::stringstream s;
	s << ".\\Target\\Calib.pcd";

	std::string loadfile = s.str();

	if (pcl::io::loadPCDFile(loadfile, *cloudA) == -1)
	{
		std::cerr << "Can't load target calib file." << std::endl;
		exit;
	}
	std::vector<int> indicies;

	pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indicies);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudA, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.1, 0.9, "cloud", v1);

	/*viewer->addPointCloud<pcl::PointXYZRGB>(cloudA, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.1, 0.9, "cloud");*/

	/*std::stringstream s2;
	s2 << ".\\Target\\Calib2.pcd";

	std::string loadfile2 = s2.str();

	if (pcl::io::loadPCDFile(loadfile2, *cloudA) == -1)
	{
		std::cerr << "Can't load target calib file." << std::endl;
		exit;
	}

	pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indicies);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudA, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.2, 0.5, "cloud2");*/


	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 50, 0, -1, 0); //Set camera position
	//viewer->initCameraParameters();
	//viewer->setCameraPosition(-50, 0, 0, 0, -1, 0, v2);
	//viewer->addCoordinateSystem(300.0);
	viewer->resetCamera();

	//pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	viewer->addPointCloud(cloudB, "targetcloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "targetcloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "targetcloud", v1);
	/*viewer->addPointCloud(cloudB, "targetcloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "targetcloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, "targetcloud");*/


	pcl::PointXYZ o, o1;
	o.x = 0.0;
	o.y = 0.0;
	o.z = 0.0;
	/*viewer->addSphere(o, 0.2, "sphere");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sphere", v1);

	o1.x = 3000.0;
	o1.y = 0.0;
	o1.z = 0.0;

	viewer->addLine(o, o1, "bodycenterline");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bodycenterline", v1);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, "bodycenterline", v1);*/

	//o1.x = 0.0;
	//o1.y = 0.0;
	//o1.z = 0.0;
	//viewer->addSphere(o1, 0.2, "sphereG");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sphereG");

	//viewer->addSphere(o1, 0.2, "sphereMX");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sphereMX");


	//viewer->addSphere(o1, 0.2, "sphereMI");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sphereMI");

	//viewer->addSphere(o1, 0.2, "sphereMir");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sphereMir");

	//viewer->addSphere(o1, 0.4, "sphereMid");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "sphereMid");

	//viewer->addSphere(o1, 25, "spherePelv");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1.0, 0.5, "spherePelv");

	//viewer->addSphere(o1, 25, "spherefinalPelv");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1.0, 0.5, "spherefinalPelv");

	char fileName[1024];


	bool er_flag = false;

	Eigen::Vector3f centroid;
	Eigen::Vector3f groundPoint;
	Eigen::Vector3f groundPoint2;
	Eigen::Vector3f prevcPelv;
	Eigen::Vector3f footRPoint, footLPoint;
	Eigen::Vector4f centroidX;


	bool foundPos = false;
	pcl::PointCloud<pcl::PointXYZRGB> tempCloud;

	bool atFrameInit = true;

	float iHeadHeight = 0.0;
	float iShldHeight = 0.0;
	float iPelvHeight = 0.0;
	float iKneeHeight = 0.0;
	float iFootPose = 0.0;
	float  deltaXY = 0.0;
	float  deltaXY2 = 0.0;
	float prevxDiff = 0.0;
	float prevyDiff = 0.0;
	float acctHeight = 0.0;


	float lLegLength, uLegLength, uBodyLength, uArmLength, lArmLength, neckLength, rShldrLength, lShldrLength, sPlvLength;

	pcl::PointXYZRGB minPt, maxPt;

	int quatFrame = 0;

	ofstream RAboneDataFile;
	ofstream LAboneDataFile;
	ofstream RLboneDataFile;
	ofstream LLboneDataFile;

	ofstream FRAboneDataFile;
	ofstream FLAboneDataFile;
	ofstream FRLboneDataFile;
	ofstream FLLboneDataFile;

	ofstream rawDataFile;



	while (!viewer->wasStopped())
	{
		if (_StartScan && flag1timeL1)
		{
			flag1timeL1 = false;
			std::stringstream s;
			s << ".\\Target\\Calib.pcd";

			std::string loadfile = s.str();

			if (pcl::io::loadPCDFile(loadfile, *cloudA) == -1)
			{
				std::cerr << "Can't load target calib file." << std::endl;
				exit;
			}
			std::vector<int> indicies;

			pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indicies);
			*calibL1L2 += *cloudA;

			viewer->updatePointCloud(cloudA, "cloud");
			//viewer->resetCamera();
			flag2timeL1 = false;

			pcl::getMinMax3D(*cloudA, minPt, maxPt);

			//getGroundPoint(cloudA, groundPoint);
			//deltaXY = ((maxPt.y ) - groundPoint[1]) / ((maxPt.x ) - groundPoint[0]);
			//groundPoint[1] = groundPoint[1] - 40.0; // -40 To adjust the thikness of the ground 
			groundPoint[1] = maxPt.y - 50;
			cout << "Max truth-> X:" << maxPt.x << "\t Y:" << maxPt.y << "\t Z:" << maxPt.z << endl;

			cout << "Ground truth-> X:" << groundPoint[0] << "\t Y:" << groundPoint[1] << "\t Z:" << groundPoint[2] << "\t Delta:" << deltaXY << endl;


			//---------------------
			//std::stringstream s2;
			//s2 << ".\\Target\\Calib2.pcd";

			//std::string loadfile2 = s2.str();

			//if (pcl::io::loadPCDFile(loadfile2, *cloudL2) == -1)
			//{
			//	std::cerr << "Can't load target calib file." << std::endl;
			//	exit;
			//}
			////std::vector<int> indicies;

			//pcl::removeNaNFromPointCloud(*cloudL2, *cloudL2, indicies);

			//*calibL1L2 += *cloudL2;

			////viewer->updatePointCloud(cloudL2, "cloud");
			////viewer->resetCamera();
			//flag2timeL2 = false;

			//pcl::getMinMax3D(*cloudL2, minPt, maxPt);

			////getGroundPoint(cloudL2, groundPoint2);

			////deltaXY2 = ((maxPt.y ) - groundPoint2[1]) / ((maxPt.x ) - groundPoint2[0]);

			//cout << "Max truth2-> X:" << maxPt.x  << "\t Y:" << maxPt.y  << "\t Z:" << maxPt.z  << endl;

			//cout << "Max truth2-> X:" << minPt.x  << "\t Y:" << minPt.y  << "\t Z:" << minPt.z  << endl;

			////cout << "Ground truth2-> X:" << groundPoint2[0] << "\t Y:" << groundPoint2[1] << "\t Z:" << groundPoint2[2] << "\t Delta:" << deltaXY2 << endl;

			//cout << "Ground truth-> X:" << abs(((maxPt.x ) +65) - groundPoint[0]) << "\t Y:" << abs((maxPt.y )- groundPoint[1]) << "\t Z:" << abs((maxPt.z )- groundPoint[2])  << endl;

		}

		pcl::removeNaNFromPointCloud(*CloudViewer1, *CloudViewer1, indicies);
		//pcl::removeNaNFromPointCloud(*CloudViewer2, *CloudViewer2, indicies);

		real_cloud1->clear();

		*real_cloud1 = *CloudViewer1;
		//*real_cloud1 += *CloudViewer2;

		viewer->updatePointCloud(real_cloud1, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "cloud", v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.1, 0.9, "cloud", v1);
		/*for (int i = 0; i < CloudViewer2->size(); ++i)
		{
			CloudViewer2->points[i].x = CloudViewer2->points[i].x - (65.0);
			CloudViewer2->points[i].y = CloudViewer2->points[i].y - (100.0);
			CloudViewer2->points[i].z = CloudViewer2->points[i].z ;
		}*/

		if (PositionTracking::initCalib)
		{
			if (real_cloud1->size() > 2000)
				foundPos = poseDetection(calibL1L2, real_cloud1, &tempCloud, centroid);
			else foundPos = false;

			if (foundPos)
			{
				acctHeight = ((groundPoint[1] - centroid[1]) /*+ (centroid[0] * deltaXY)*/);

				cout << "groundpoint:" << groundPoint[1] << "\tHeadpoint:" << centroid[1] << endl;

				iHeadHeight = centroid[1];

				iShldHeight = iHeadHeight + acctHeight * 0.16;
				iPelvHeight = iHeadHeight + acctHeight * 0.45;
				iKneeHeight = iHeadHeight + acctHeight * 0.71;
				iFootPose = groundPoint[1];//iHeadHeight + acctHeight;

				float headlength = sqrt(pow((iHeadHeight - iShldHeight), 2));


				avatarHeight = iPelvHeight;

				o.x = 0.0;
				o.y = iHeadHeight;
				o.z = 0.0;

				o1.x = 3000.0;
				o1.y = iHeadHeight;
				o1.z = 0.0;

				viewer->removeAllShapes();

				//viewer->addLine(o, o1, "headline", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "headline", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, "headline", v1);

				o.x = 0.0;
				o.y = iPelvHeight;
				o.z = 0.0;

				o1.x = 3000.0;
				o1.y = iPelvHeight;
				o1.z = 0.0;

				//viewer->addLine(o, o1, "pelvline", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "pelvline", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, "pelvline", v1);

				o.x = 0.0;
				o.y = iKneeHeight;
				o.z = 0.0;

				o1.x = 3000.0;
				o1.y = iKneeHeight;
				o1.z = 0.0;

				//viewer->addLine(o, o1, "kneeline", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "kneeline", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, "kneeline", v1);

				o.x = 0.0;
				o.y = iFootPose;
				o.z = 0.0;

				o1.x = 3000.0;
				o1.y = iFootPose + 20; //To adjust the foot sphere on ground
				o1.z = 0.0;

				//viewer->addLine(o, o1, "GroundLine", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "GroundLine", v1);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10.0, "GroundLine", v1);

				groundPointY = iFootPose + 20;//Fixed ground 

				cout << "\t pHeight:" << acctHeight / 10.0 << "\t iHeadHeight:" << iHeadHeight / 10.0 << "\t iShldHeight:" << iShldHeight / 10.0 << "\t iPelvHeight:" << iPelvHeight / 10.0 << "\t iKneeHeight:" << iKneeHeight / 10.0 << endl;


				VitruvianAvatar::humanHeight = (acctHeight / 10.0);
				vAvatar.initializeVetruvianVtkAvatar();

				std::stringstream s;
				s << ".\\Target\\test.pcd";

				std::string savefile = s.str();

				flag1timeL2 = false;


				if (cloudA->size() > 0)
				{
					pcl::io::savePCDFileASCII(savefile, tempCloud);

					cout << "Cloud saved!!!!!!" << endl;
				}

				pcl::copyPointCloud(tempCloud, *cloudB);
				cout << "Cloud Size1:" << cloudB->size() << endl;
				pcl::getMinMax3D(*cloudB, minPt, maxPt);
				pcl::PassThrough<pcl::PointXYZRGB> pass_foot;
				pass_foot.setInputCloud(cloudB);
				pass_foot.setFilterFieldName("y");
				pass_foot.setFilterLimits(minPt.y - 50, minPt.y + 50);
				//pass2.setFilterLimitsNegative (true);
				pass_foot.filter(*cloudB);
				pcl::getMinMax3D(*cloudB, minPt, maxPt);

				cout << "Cloud Size2:" << cloudB->size() << endl;

				//------Skeleton-------------//
				//pcl::ModelCoefficients cylinder_coeff;
				//cylinder_coeff.values.resize(7);


				//Left Lower Leg and foot
				//ordered pair
				float xLKnee = maxPt.x - maxPt.x;
				float yLKnee = iKneeHeight - iFootPose;
				float zLKnee = minPt.z - minPt.z;
				float fNorm = sqrt(pow(xLKnee, 2) + pow(yLKnee, 2) + pow(zLKnee, 2));
				//Orderpair normalize
				xLKnee = (xLKnee / fNorm);
				yLKnee = (yLKnee / fNorm);
				zLKnee = (zLKnee / fNorm);

				//Knee Length
				float kneeLength = sqrt(pow((maxPt.x - maxPt.x), 2) + pow((iKneeHeight - iFootPose), 2) + pow((maxPt.z - maxPt.z), 2));

				lFoot[0] = maxPt.x;// +(maxPt.x - minPt.x) / 2;
				lFoot[1] = iFootPose;
				lFoot[2] = minPt.z;

				lLegLength = kneeLength;

				//Right Lower Leg and Foot
				float xRKnee = maxPt.x - maxPt.x;
				float yRKnee = iKneeHeight - iFootPose;
				float zRKnee = minPt.z - minPt.z;
				fNorm = sqrt(pow(xRKnee, 2) + pow(yRKnee, 2) + pow(zRKnee, 2));
				//Orderpair normalize
				xRKnee = (xRKnee / fNorm);
				yRKnee = (yRKnee / fNorm);
				zRKnee = (zRKnee / fNorm);


				rFoot[0] = maxPt.x;// +(maxPt.x - minPt.x) / 2;
				rFoot[1] = iFootPose;
				rFoot[2] = maxPt.z;

				/*DrawCylinder(viewer, { maxPt.x, iFootPose, maxPt.z }, { (xRKnee)*kneeLength, (yRKnee)*kneeLength, (zRKnee)*kneeLength }, "RLLeg");
				DrawJointSphere(viewer, { maxPt.x, iFootPose,maxPt.z }, "RLLegSphere");*/

				float pelvLength = sqrt(pow((iPelvHeight - iKneeHeight), 2));

				/////////Shouldr position and length/////////////
				lKnee[0] = maxPt.x;
				lKnee[1] = iKneeHeight;
				lKnee[2] = minPt.z;

				uLegLength = pelvLength;
				sPlvLength = (headlength * 0.5);

				rKnee[0] = maxPt.x;
				rKnee[1] = iKneeHeight;
				rKnee[2] = maxPt.z;

				float xplv = maxPt.x - maxPt.x;
				float yplv = lPlv[1] - lKnee[1];
				float zplv = minPt.z - minPt.z;
				fNorm = sqrt(pow(xplv, 2) + pow(yplv, 2) + pow(zplv, 2));

				float lpelvLength = sqrt(pow((lPlv[1] - lKnee[1]), 2));

				/*DrawCylinder(viewer, { lKnee[0], lKnee[1], lKnee[2] }, { (xplv / fNorm)*lpelvLength, (yplv / fNorm)*lpelvLength, (zplv / fNorm)*lpelvLength }, "LULeg");
				DrawJointSphere(viewer, { lKnee[0], lKnee[1], lKnee[2] }, "LeftKneeSphere");*/

				//Right Upper Leg
				xplv = maxPt.x - maxPt.x;
				yplv = rPlv[1] - rKnee[1];
				zplv = maxPt.z - maxPt.z;

				fNorm = sqrt(pow(xplv, 2) + pow(yplv, 2) + pow(zplv, 2));
				float rpelvLength = sqrt(pow((lPlv[1] - lKnee[1]), 2));

				/*DrawCylinder(viewer, { rKnee[0], rKnee[1], rKnee[2] }, { (xplv / fNorm)*rpelvLength, (yplv / fNorm)*rpelvLength, (zplv / fNorm)*rpelvLength }, "RULeg");
				DrawJointSphere(viewer, { rKnee[0], rKnee[1], rKnee[2] }, "RightKneeSphere");*/

				/////////Head position/////////////
				cHead[0] = maxPt.x;
				cHead[1] = iHeadHeight;
				cHead[2] = (maxPt.z + (minPt.z - maxPt.z) / 2);

				/////////Pelvise position/////////////
				cPlv[0] = cHead[0];
				cPlv[1] = iPelvHeight;
				cPlv[2] = cHead[2];

				cout << "Pelvis-> X:" << cPlv[0] << "\t Y:" << cPlv[1] << "\t Z:" << cPlv[2] << endl;

				zDiff = cPlv[2];// to adjust intial position drift

				////Left Pelvise Position/////
				lPlv[0] = cPlv[0];
				lPlv[1] = cPlv[1];
				lPlv[2] = cPlv[2] + sPlvLength;

				////Right Pelvise Position/////
				rPlv[0] = cPlv[0];
				rPlv[1] = cPlv[1];
				rPlv[2] = cPlv[2] - sPlvLength;

				////Left Knee Position/////
				lKnee[0] = lPlv[0];
				lKnee[1] = iKneeHeight;
				lKnee[2] = lPlv[2];
				////Right Knee Position/////
				rKnee[0] = rPlv[0];
				rKnee[1] = iKneeHeight;
				rKnee[2] = rPlv[2];
				////Left foot Position/////
				lFoot[0] = lKnee[0];
				lFoot[1] = iFootPose;
				lFoot[2] = lKnee[2];
				////Right foot Position/////
				rFoot[0] = rKnee[0];
				rFoot[1] = iFootPose;
				rFoot[2] = rKnee[2];


				xplv = maxPt.x - maxPt.x;
				yplv = cPlv[1] - cPlv[1];
				zplv = cPlv[2] - rPlv[2];

				fNorm = sqrt(pow(xplv, 2) + pow(yplv, 2) + pow(zplv, 2));

				DrawBoneCylinder(viewer, { cPlv[0], cPlv[1], cPlv[2] }, { rPlv[0], rPlv[1], rPlv[2] }, "RPelvis");

				xplv = maxPt.x - maxPt.x;
				yplv = cPlv[1] - cPlv[1];
				zplv = cPlv[2] - lPlv[2];

				fNorm = sqrt(pow(xplv, 2) + pow(yplv, 2) + pow(zplv, 2));

				DrawBoneCylinder(viewer, cPlv, lPlv, "LPelvis");

				DrawJointSphere(viewer, { cPlv[0], cPlv[1], cPlv[2] }, "PelvSphere");
				DrawJointSphere(viewer, { rPlv[0], rPlv[1], rPlv[2] }, "rPelvSphere");
				DrawJointSphere(viewer, { lPlv[0], lPlv[1], lPlv[2] }, "lPelvSphere");


				DrawBoneCylinder(viewer, lKnee, lPlv, "LULeg");
				DrawJointSphere(viewer, { lKnee[0], lKnee[1], lKnee[2] }, "LeftKneeSphere");

				DrawBoneCylinder(viewer, rKnee, rPlv, "RULeg");
				DrawJointSphere(viewer, { rKnee[0], rKnee[1], rKnee[2] }, "RightKneeSphere");

				DrawBoneCylinder(viewer, lFoot, lKnee, "LLLeg");
				DrawJointSphere(viewer, lFoot, "LLLegSphere");

				DrawBoneCylinder(viewer, rFoot, rKnee, "RLLeg");
				DrawJointSphere(viewer, rFoot, "RLLegSphere");

				//Upper Body - Pelivse to Shoulder
				float xshldr = maxPt.x - maxPt.x;
				float yshldr = iShldHeight - iPelvHeight;
				float zshldr = (minPt.z - (minPt.z - maxPt.z) / 2) - (minPt.z - (minPt.z - maxPt.z) / 2);
				fNorm = sqrt(pow(xshldr, 2) + pow(yshldr, 2) + pow(zshldr, 2));

				float length = sqrt(pow((maxPt.x - maxPt.x), 2) + pow((iShldHeight - iPelvHeight), 2) + pow((maxPt.z - maxPt.z), 2));


				/////////Shouldr position and length/////////////
				cTorso[0] = maxPt.x;
				cTorso[1] = iShldHeight;
				cTorso[2] = (maxPt.z + (minPt.z - maxPt.z) / 2);


				DrawCylinder(viewer, { maxPt.x, iPelvHeight, (minPt.z - (minPt.z - maxPt.z) / 2) }, { (xshldr / fNorm) * length, (yshldr / fNorm) * length, (zshldr / fNorm) * length }, "UpperBody");
				DrawJointSphere(viewer, { cTorso[0], cTorso[1], cTorso[2] }, "SholderSphere");

				cSternum[0] = cTorso[0] + ((cPlv[0] - cTorso[0]) / 2);
				cSternum[1] = cTorso[1] + ((cPlv[1] - cTorso[1]) / 2);
				cSternum[2] = cTorso[2] + ((cPlv[2] - cTorso[2]) / 2);

				uBodyLength = length;

				DrawJointSphere(viewer, { cSternum[0], cSternum[1], cSternum[2] }, "sternumSphere");


				//Neck - Shouder to Head
				float xNeck = maxPt.x - maxPt.x;
				float yNeck = iHeadHeight - iShldHeight;
				float zNeck = (minPt.z - (minPt.z - maxPt.z) / 2) - (minPt.z - (minPt.z - maxPt.z) / 2);
				fNorm = sqrt(pow(xNeck, 2) + pow(yNeck, 2) + pow(zNeck, 2));



				neckLength = headlength;

				DrawCylinder(viewer, { maxPt.x, iShldHeight, (minPt.z - (minPt.z - maxPt.z) / 2) }, { (xNeck / fNorm) * headlength, (yNeck / fNorm) * headlength, (zNeck / fNorm) * headlength }, "Neck");
				DrawJointSphere(viewer, { cHead[0], cHead[1], cHead[2] }, "HeadSphere");


				//-----Right shoulder----

				xshldr = maxPt.x;
				yshldr = iShldHeight;
				zshldr = (maxPt.z + (minPt.z - maxPt.z) / 2);

				float xRshldr = xshldr;
				float yRshldr = yshldr;
				float zRshldr = zshldr + ((headlength) * 0.85);
				//ordered pair
				xRshldr = xRshldr - xshldr;
				yRshldr = yRshldr - yshldr;
				zRshldr = zRshldr - zshldr;
				//normalize ordered pair
				float vecLength = sqrt(pow(xRshldr, 2) + pow(yRshldr, 2) + pow(zRshldr, 2));
				xRshldr = xRshldr / vecLength;
				yRshldr = yRshldr / vecLength;
				zRshldr = (zRshldr / vecLength) * (headlength * 0.85);

				//Right Shoulder point

				DrawCylinder(viewer, { xshldr, yshldr, zshldr }, { xRshldr, yRshldr, zRshldr }, "rightShoulder");

				xRshldr = xshldr;
				yRshldr = yshldr;
				zRshldr = zshldr + ((headlength) * 0.85);

				rShldr[0] = xRshldr;
				rShldr[1] = yRshldr;
				rShldr[2] = zRshldr;
				rShldrLength = (headlength * 0.85);


				DrawJointSphere(viewer, { xRshldr, yRshldr, zRshldr }, "rightShoulderSphere");

				//-----Right upper Arm----

				//ordered pair
				float xRElb = xRshldr;
				float yRElb = yRshldr + ((headlength) * 1.2);
				float zRElb = zRshldr;
				//ordered pair
				xRElb = xRElb - xRshldr;
				yRElb = yRElb - yRshldr;
				zRElb = zRElb - zRshldr;

				//normalize ordered pair
				vecLength = sqrt(pow(xRElb, 2) + pow(yRElb, 2) + pow(zRElb, 2));
				xRElb = xRElb / vecLength;
				yRElb = (yRElb / vecLength) * (headlength * 1.2);
				zRElb = zRElb / vecLength;

				//Right Shoulder point

				DrawCylinder(viewer, { xRshldr, yRshldr, zRshldr }, { xRElb, yRElb, zRElb }, "rightUpperArm");
				xRElb = xRshldr;
				yRElb = yRshldr + ((headlength) * 1.2);
				zRElb = zRshldr;

				rUarm[0] = xRElb;
				rUarm[1] = yRElb;
				rUarm[2] = zRElb;
				uArmLength = (headlength * 1.2);

				DrawJointSphere(viewer, { xRElb, yRElb, zRElb }, "rightUpperArmSphere");

				//-----Right Lower Arm----

				float xRhand = xRElb;
				float yRhand = yRElb + ((headlength) * 1.3);
				float zRhand = zRElb;

				//ordered pair
				xRhand = xRhand - xRElb;
				yRhand = yRhand - yRElb;
				zRhand = zRhand - zRElb;

				//normalize ordered pair
				vecLength = sqrt(pow(xRhand, 2) + pow(yRhand, 2) + pow(zRhand, 2));
				xRhand = xRhand / vecLength;
				yRhand = (yRhand / vecLength) * (headlength * 1.3);
				zRhand = zRhand / vecLength;


				DrawCylinder(viewer, { xRElb, yRElb, zRElb }, { xRhand, yRhand, zRhand }, "rightLowerArm");
				//Right Shoulder point
				xRhand = xRElb;
				yRhand = yRElb + ((headlength) * 1.3);
				zRhand = zRElb;

				rLarm[0] = xRhand;
				rLarm[1] = yRhand;
				rLarm[2] = zRhand;
				lArmLength = (headlength * 1.3);

				cout << "Right Arm-> X:" << rLarm[0] << "\t Y:" << rLarm[1] << "\t Z:" << rLarm[2] << endl;

				DrawJointSphere(viewer, { xRhand, yRhand, zRhand }, "rightLowerArmSphere");

				//-----Left shoulder----

				float xLshldr = xshldr;
				float yLshldr = yshldr;
				float zLshldr = zshldr - ((headlength) * 0.85);
				//ordered pair
				xLshldr = xLshldr - xshldr;
				yLshldr = yLshldr - yshldr;
				zLshldr = zLshldr - zshldr;
				//normalize ordered pair
				vecLength = sqrt(pow(xLshldr, 2) + pow(yLshldr, 2) + pow(zLshldr, 2));
				xLshldr = xLshldr / vecLength;
				yLshldr = yLshldr / vecLength;
				zLshldr = (zLshldr / vecLength) * (headlength * 0.85);


				DrawCylinder(viewer, { xshldr, yshldr, zshldr }, { xLshldr, yLshldr, zLshldr }, "leftShoulder");
				//Left Shoulder Point
				xLshldr = xshldr;
				yLshldr = yshldr;
				zLshldr = zshldr - ((headlength) * 0.85);

				lShldr[0] = xLshldr;
				lShldr[1] = yLshldr;
				lShldr[2] = zLshldr;


				DrawJointSphere(viewer, { xLshldr, yLshldr, zLshldr }, "leftShoulderSphere");

				//-----Left upper Arm----

				//ordered pair
				float xLElb = xLshldr;
				float yLElb = yLshldr + ((headlength) * 1.2);
				float zLElb = zLshldr;

				//ordered pair
				xLElb = xLElb - xLshldr;
				yLElb = yLElb - yLshldr;
				zLElb = zLElb - zLshldr;

				//normalize ordered pair
				vecLength = sqrt(pow(xLElb, 2) + pow(yLElb, 2) + pow(zLElb, 2));
				xLElb = xLElb / vecLength;
				yLElb = (yLElb / vecLength) * (headlength * 1.2);
				zLElb = zLElb / vecLength;

				DrawCylinder(viewer, { xLshldr, yLshldr, zLshldr }, { xLElb, yLElb, zLElb }, "leftUpperArm");
				//Right Shoulder point
				xLElb = xLshldr;
				yLElb = yLshldr + ((headlength) * 1.2);
				zLElb = zLshldr;

				lUarm[0] = xLElb;
				lUarm[1] = yLElb;
				lUarm[2] = zLElb;

				DrawJointSphere(viewer, { xLElb, yLElb, zLElb }, "leftUpperArmSphere");

				//-----Left Lower Arm----

				float xLhand = xLElb;
				float yLhand = yLElb + ((headlength) * 1.3);
				float zLhand = zLElb;

				//ordered pair
				xLhand = xLhand - xLElb;
				yLhand = yLhand - yLElb;
				zLhand = zLhand - zLElb;

				//normalize ordered pair
				vecLength = sqrt(pow(xLhand, 2) + pow(yLhand, 2) + pow(zLhand, 2));
				xLhand = xLhand / vecLength;
				yLhand = (yLhand / vecLength) * (headlength * 1.3);
				zLhand = zLhand / vecLength;

				DrawCylinder(viewer, { xLElb, yLElb, zLElb }, { xLhand, yLhand, zLhand }, "leftLowerArm");
				//Right Shoulder point
				xLhand = xLElb;
				yLhand = yLElb + ((headlength) * 1.3);
				zLhand = zLElb;

				lLarm[0] = xLhand;
				lLarm[1] = yLhand;
				lLarm[2] = zLhand;

				cout << "Left Arm-> X:" << lLarm[0] << "\t Y:" << lLarm[1] << "\t Z:" << lLarm[2] << endl;

				DrawJointSphere(viewer, { xLhand, yLhand, zLhand }, "leftLowerArmSphere");

				PositionTracking::initCalib = false;
				PositionTracking::isCalib = true;

				bOnetime = true;

			}
		}

		//IMU Calibration
		if (myAcquire.calibIMU)
		{
			myAcquire.caliberateQSF();
			AcquireSFQ.caliberateQSF();
			myAcquire.calibIMU = false;
		}

		if (PositionTracking::isCalib && framesCountL1 > 1 /*&& framesCountL2 > 1*/)
		{
			if (real_cloud1->size() > 0)
			{
				foundPos = poseDetection(calibL1L2, real_cloud1, &tempCloud, centroid);

				if (foundPos)
				{
					pcl::copyPointCloud(tempCloud, *cloudB);

					//if (bSaveKeyFrame)
					{
						std::stringstream s;
						s << ".\\PositionData\\BodyFrameCloud" << tFrameIndex << ".pcd";

						std::string savefile = s.str();

						if (cloudA->size() > 0)
						{
							// 0618 data save stop
							//pcl::io::savePCDFileASCII(savefile, *cloudB);
						}
						bSaveKeyFrame = false;
					}

					if (!er_flag)
					{
						viewer->updatePointCloud(cloudB, "targetcloud");
					}
					else
					{
						//viewer->removePointCloud("targetcloud");
						viewer->addPointCloud(cloudB, "targetcloud", v1);
						er_flag = false;
					}

					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "targetcloud", v1);
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "targetcloud", v1);

					pcl::getMinMax3D(*cloudB, minPt, maxPt);
					pcl::PassThrough<pcl::PointXYZRGB> pass_foot;
					pass_foot.setInputCloud(cloudB);
					pass_foot.setFilterFieldName("y");
					pass_foot.setFilterLimits(maxPt.y - 35, maxPt.y + 10);
					//pass2.setFilterLimitsNegative (true);
					pass_foot.filter(*cloudB);
					pcl::getMinMax3D(*cloudB, minPt, maxPt);
					pcl::compute3DCentroid(*cloudB, centroidX);

					footRPoint[0] = maxPt.x;
					footRPoint[1] = groundPoint[1];
					footRPoint[2] = maxPt.z;

					footLPoint[0] = maxPt.x;
					footLPoint[1] = groundPoint[1];
					footLPoint[2] = minPt.z;

				}
				else foundPos = false;
			}

			footLPoint[0] = lFoot[0];
			footLPoint[2] = lFoot[2];
			footRPoint[0] = rFoot[0];
			footRPoint[2] = rFoot[2];

			frameIndex++;

			//if (frameIndex == 0)
			//{
			//	
			//	time_t curr_time;
			//	curr_time = time(NULL);
			//	tm *tm_local = localtime(&curr_time);
			//	
			//	//sprintf_s(fileName, ".\\BoneData\\RAboneData-%d%d%d.txt",  tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			//	//RAboneDataFile.open(fileName);	
			//	//sprintf_s(fileName, ".\\BoneData\\LAboneData-%d%d%d.txt", tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			//	//LAboneDataFile.open(fileName);
			//	//sprintf_s(fileName, ".\\BoneData\\RLboneData-%d%d%d.txt", tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			//	//RLboneDataFile.open(fileName);
			//	//sprintf_s(fileName, ".\\BoneData\\LLboneData-%d%d%d.txt", tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			//	//LLboneDataFile.open(fileName);

			//	sprintf_s(fileName, ".\\SkeletonData\\RawIMUData-00%d-%d%d%d.txt", 0, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			//	rawDataFile.open(fileName);
			//	rawDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << 0 << "\n";

			//}

			//Translate to both the legs to ground point
			Eigen::Vector3f diffR = footRPoint - rFoot; //Update new ground points 

			Eigen::Vector3f diffL = footLPoint - lFoot;

			updateLowerBody(diffR, diffL); // Lower body traslation to the ground point

			//Save raw quaternion data to file
			Avatar getRaw = myAcquire.getRawQ();


			RawData[frameIndex][0] = getRaw.b0;
			RawData[frameIndex][1] = getRaw.b1;
			RawData[frameIndex][2] = getRaw.b2;
			RawData[frameIndex][3] = getRaw.b3;
			RawData[frameIndex][4] = getRaw.b4;
			RawData[frameIndex][5] = getRaw.b5;
			RawData[frameIndex][6] = getRaw.b6;
			RawData[frameIndex][7] = getRaw.b7;
			RawData[frameIndex][8] = getRaw.b8;
			RawData[frameIndex][9] = getRaw.b9;
			/*
			rawDataFile
				<< getRaw.b0.mData[3] << "\t" << getRaw.b0.mData[0] << "\t" << getRaw.b0.mData[1] << "\t" << getRaw.b0.mData[2] << "\t"
				<< getRaw.b1.mData[3] << "\t" << getRaw.b1.mData[0] << "\t" << getRaw.b1.mData[1] << "\t" << getRaw.b1.mData[2] << "\t"
				<< getRaw.b2.mData[3] << "\t" << getRaw.b2.mData[0] << "\t" << getRaw.b2.mData[1] << "\t" << getRaw.b2.mData[2] << "\t"
				<< getRaw.b3.mData[3] << "\t" << getRaw.b3.mData[0] << "\t" << getRaw.b3.mData[1] << "\t" << getRaw.b3.mData[2] << "\t"
				<< getRaw.b4.mData[3] << "\t" << getRaw.b4.mData[0] << "\t" << getRaw.b4.mData[1] << "\t" << getRaw.b4.mData[2] << "\t"
				<< getRaw.b5.mData[3] << "\t" << getRaw.b5.mData[0] << "\t" << getRaw.b5.mData[1] << "\t" << getRaw.b5.mData[2] << "\t"
				<< getRaw.b6.mData[3] << "\t" << getRaw.b6.mData[0] << "\t" << getRaw.b6.mData[1] << "\t" << getRaw.b6.mData[2] << "\t"
				<< getRaw.b7.mData[3] << "\t" << getRaw.b7.mData[0] << "\t" << getRaw.b7.mData[1] << "\t" << getRaw.b7.mData[2] << "\t"
				<< getRaw.b8.mData[3] << "\t" << getRaw.b8.mData[0] << "\t" << getRaw.b8.mData[1] << "\t" << getRaw.b8.mData[2] << "\t"
				<< getRaw.b9.mData[3] << "\t" << getRaw.b9.mData[0] << "\t" << getRaw.b9.mData[1] << "\t" << getRaw.b9.mData[2] << "\n";*/


				////To save SFQ data
				//AcquireSFQ.getRawQ();
			Avatar getSfq = AcquireSFQ.getSFQ();

			sfqData[frameIndex][0] = getSfq.b0;
			sfqData[frameIndex][1] = getSfq.b1;
			sfqData[frameIndex][2] = getSfq.b2;
			sfqData[frameIndex][3] = getSfq.b3;
			sfqData[frameIndex][4] = getSfq.b4;
			sfqData[frameIndex][5] = getSfq.b5;
			sfqData[frameIndex][6] = getSfq.b6;
			sfqData[frameIndex][7] = getSfq.b7;
			sfqData[frameIndex][8] = getSfq.b8;
			sfqData[frameIndex][9] = getSfq.b9;


			Avatar currentOri = (myAcquire.getFirstInvQuat());
			VitruvianAvatar::vitruvianAvatarUpdate = currentOri;
			double vector[3] = { 0,	0,	1 };

			/*currentOri.b0 = currentOri.b0.Inverse();
			currentOri.b1 = currentOri.b1.Inverse();
			currentOri.b2 = currentOri.b2.Inverse();
			currentOri.b3 = currentOri.b3.Inverse();
			currentOri.b4 = currentOri.b4.Inverse();
			currentOri.b5 = currentOri.b5.Inverse();
			currentOri.b6 = currentOri.b6.Inverse();
			currentOri.b7 = currentOri.b7.Inverse();
			currentOri.b8 = currentOri.b8.Inverse();
			currentOri.b9 = currentOri.b9.Inverse();*/


			//Right Foot <- Lower Right Leg orientation			
			quaternion rFootQuat(vector[0], vector[1], vector[2], 0);
			rFootQuat = currentOri.b7.mutiplication(rFootQuat.mutiplication(currentOri.b7.Inverse()));

			rKnee[0] = rFoot[0] + (rFootQuat.mData[1] * lLegLength);
			rKnee[1] = rFoot[1] + (-rFootQuat.mData[2] * lLegLength);
			rKnee[2] = rFoot[2] + (-rFootQuat.mData[0] * lLegLength);

			//Left Foot <- Lower Left Leg orientation		
			quaternion lFootQuat(vector[0], vector[1], vector[2], 0);

			lFootQuat = currentOri.b9.mutiplication(lFootQuat.mutiplication(currentOri.b9.Inverse()));

			lKnee[0] = lFoot[0] + (lFootQuat.mData[1] * lLegLength);
			lKnee[1] = lFoot[1] + (-lFootQuat.mData[2] * lLegLength);
			lKnee[2] = lFoot[2] + (-lFootQuat.mData[0] * lLegLength);

			//Right Kneepoint <- upper Right Leg orientation

			quaternion rKneeQuat(vector[0], vector[1], vector[2], 0);

			rKneeQuat = currentOri.b6.mutiplication(rKneeQuat.mutiplication(currentOri.b6.Inverse()));

			rPlv[0] = rKnee[0] + (rKneeQuat.mData[1] * uLegLength);
			rPlv[1] = rKnee[1] + (-rKneeQuat.mData[2] * uLegLength);
			rPlv[2] = rKnee[2] + (-rKneeQuat.mData[0] * uLegLength);

			//Left Kneepoint <- upper Left Leg orientation
			quaternion lKneeQuat(vector[0], vector[1], vector[2], 0);

			lKneeQuat = currentOri.b8.mutiplication(lKneeQuat.mutiplication(currentOri.b8.Inverse()));

			lPlv[0] = lKnee[0] + (lKneeQuat.mData[1] * uLegLength);
			lPlv[1] = lKnee[1] + (-lKneeQuat.mData[2] * uLegLength);
			lPlv[2] = lKnee[2] + (-lKneeQuat.mData[0] * uLegLength);


			//Compute new Pelvis by comparing Right and Left Pelvis position
			if (lPlv[1] > rPlv[1])//Considering right leg 
			{

				quaternion plvQuat(vector[2], vector[1], vector[0], 0);
				//quaternion plvQuat(0, 0, 1, 0);
				plvQuat = currentOri.b0.mutiplication(plvQuat.mutiplication(currentOri.b0.Inverse()));

				cPlv[0] = (rPlv[0] + (plvQuat.mData[1] * sPlvLength));
				cPlv[1] = rPlv[1] + (-plvQuat.mData[2] * sPlvLength);
				cPlv[2] = rPlv[2] + (-plvQuat.mData[0] * sPlvLength);
			}

			if (rPlv[1] >= lPlv[1])//Considering leg leg 
			{

				quaternion plvQuat(-vector[2], vector[1], vector[0], 0);
				//quaternion plvQuat(0, 0, -1, 0);
				plvQuat = currentOri.b0.mutiplication(plvQuat.mutiplication(currentOri.b0.Inverse()));

				cPlv[0] = (lPlv[0] + (plvQuat.mData[1] * sPlvLength));
				cPlv[1] = lPlv[1] + (-plvQuat.mData[2] * sPlvLength);
				cPlv[2] = lPlv[2] + (-plvQuat.mData[0] * sPlvLength);
			}

			if (bOnetime)
			{
				zDiff = (cPlv[2] - zDiff);
				cPlv[2] = cPlv[2] - zDiff;
				bOnetime = false;
			}

			//cout<< one_line_data <<endl;

			//to adjust intial drift position

		   //Translate upper body to match pelvise height
		   //prevcPelv = cPlv;
		   //cPlv =(rPlv + lPlv) / 2;

		   //cPlv[0] = centroidX[0]; //Center of Max-X and Min-X

		   //updateUpperBody(prevcPelv - cPlv);

		   //-----------------------------------------------------------------------------------------
		   //cPlv[0] = -cPlv[0];

		   //Right Pelvis <- Pelvis orientation			
			quaternion rplvQuat1(-vector[2], vector[1], vector[0], 0);
			//quaternion rplvQuat1(0, 0, -1, 0);
			rplvQuat1 = currentOri.b0.mutiplication(rplvQuat1.mutiplication(currentOri.b0.Inverse()));

			rPlv[0] = cPlv[0] + (rplvQuat1.mData[1] * sPlvLength);//Changes in Lower body with pelvis rotation and model moves continously in x axis
			rPlv[1] = cPlv[1] + (-rplvQuat1.mData[2] * sPlvLength);//Changes in Lower body with pelvis rotation and model moves continously in x axis
			rPlv[2] = cPlv[2] + (-rplvQuat1.mData[0] * sPlvLength);

			//Left Pelvis <- Pelvis orientation			
			quaternion lplvQuat1(vector[2], vector[1], vector[0], 0);
			//quaternion lplvQuat1(0, 0, 1, 0);
			lplvQuat1 = currentOri.b0.mutiplication(lplvQuat1.mutiplication(currentOri.b0.Inverse()));

			lPlv[0] = cPlv[0] + (lplvQuat1.mData[1] * sPlvLength);//Changes in Lower body with pelvis rotation and model moves continously in x axis
			lPlv[1] = cPlv[1] + (-lplvQuat1.mData[2] * sPlvLength);//Changes in Lower body with pelvis rotation and model moves continously in x axis
			lPlv[2] = cPlv[2] + (-lplvQuat1.mData[0] * sPlvLength);

			//Recomputing left leg orientation by changing the intial vector direction
			//Right Foot <- Lower Right Leg orientation			
			quaternion rFootQuat1(vector[0], vector[1], -vector[2], 0);
			//quaternion rFootQuat1(0, -1, 0, 0);
			rFootQuat1 = currentOri.b6.mutiplication(rFootQuat1.mutiplication(currentOri.b6.Inverse()));

			rKnee[0] = rPlv[0] + (rFootQuat1.mData[1] * uLegLength);
			rKnee[1] = rPlv[1] + (-rFootQuat1.mData[2] * uLegLength);
			rKnee[2] = rPlv[2] + (-rFootQuat1.mData[0] * uLegLength);

			rFootQuat1.normalize();
			//RLboneDataFile << rFootQuat1.mData[3] << "\t" << rFootQuat1.mData[0] << "\t" << rFootQuat1.mData[1] << "\t" << rFootQuat1.mData[2] << "\t";

			//Right Kneepoint <- upper Right Leg orientation

			quaternion rKneeQuat1(vector[0], vector[1], -vector[2], 0);
			//quaternion rKneeQuat1(0, -1, 0, 0);
			rKneeQuat1 = currentOri.b7.mutiplication(rKneeQuat1.mutiplication(currentOri.b7.Inverse()));

			rFoot[0] = rKnee[0] + (rKneeQuat1.mData[1] * lLegLength);
			rFoot[1] = rKnee[1] + (-rKneeQuat1.mData[2] * lLegLength);
			rFoot[2] = rKnee[2] + (-rKneeQuat1.mData[0] * lLegLength);

			rKneeQuat1.normalize();
			//RLboneDataFile << rKneeQuat1.mData[3] << "\t" << rKneeQuat1.mData[0] << "\t" << rKneeQuat1.mData[1] << "\t" << rKneeQuat1.mData[2] << "\n";

				//Left Foot <- Lower Left Leg orientation		
			quaternion lFootQuat1(vector[0], vector[1], -vector[2], 0);
			//quaternion lFootQuat1(0, -1, 0, 0);
			lFootQuat1 = currentOri.b8.mutiplication(lFootQuat1.mutiplication(currentOri.b8.Inverse()));

			lKnee[0] = lPlv[0] + (lFootQuat1.mData[1] * uLegLength);
			lKnee[1] = lPlv[1] + (-lFootQuat1.mData[2] * uLegLength);
			lKnee[2] = lPlv[2] + (-lFootQuat1.mData[0] * uLegLength);

			lFootQuat1.normalize();
			//LLboneDataFile << lFootQuat1.mData[3] << "\t" << lFootQuat1.mData[0] << "\t" << lFootQuat1.mData[1] << "\t" << lFootQuat1.mData[2] << "\t";

			//Left Kneepoint <- upper Left Leg orientation
			quaternion lKneeQuat1(vector[0], vector[1], -vector[2], 0);
			//quaternion lKneeQuat1(0, -1, 0, 0);
			lKneeQuat1 = currentOri.b9.mutiplication(lKneeQuat1.mutiplication(currentOri.b9.Inverse()));

			lFoot[0] = lKnee[0] + (lKneeQuat1.mData[1] * lLegLength);
			lFoot[1] = lKnee[1] + (-lKneeQuat1.mData[2] * lLegLength);
			lFoot[2] = lKnee[2] + (-lKneeQuat1.mData[0] * lLegLength);

			lKneeQuat1.normalize();
			//LLboneDataFile << lKneeQuat1.mData[3] << "\t" << lKneeQuat1.mData[0] << "\t" << lKneeQuat1.mData[1] << "\t" << lKneeQuat1.mData[2] << "\n";

		//-------------------------------------------------------------------------------------------------------

			//Upper body sternum rotation 
			quaternion cSternumQuat(vector[0], vector[1], vector[2], 0);
			//quaternion cSternumQuat(0, 1, 0, 0);
			cSternumQuat = currentOri.b0.mutiplication(cSternumQuat.mutiplication(currentOri.b0.Inverse()));

			cSternum[0] = cPlv[0] + (cSternumQuat.mData[1] * uBodyLength * 0.6);
			cSternum[1] = cPlv[1] + (-cSternumQuat.mData[2] * uBodyLength * 0.6);
			cSternum[2] = cPlv[2] + (-cSternumQuat.mData[0] * uBodyLength * 0.6);

			//Upper body torso rotation 
			quaternion cTorsoQuat(vector[0], vector[1], vector[2], 0);
			//quaternion cTorsoQuat(0, 1, 0, 0);
			cTorsoQuat = currentOri.b1.mutiplication(cTorsoQuat.mutiplication(currentOri.b1.Inverse()));

			cTorso[0] = cSternum[0] + (cTorsoQuat.mData[1] * uBodyLength * 0.4);
			cTorso[1] = cSternum[1] + (-cTorsoQuat.mData[2] * uBodyLength * 0.4);
			cTorso[2] = cSternum[2] + (-cTorsoQuat.mData[0] * uBodyLength * 0.4);

			//Translate Head
			cHead[0] = cTorso[0] + (cTorsoQuat.mData[1] * neckLength);
			cHead[1] = cTorso[1] + (-cTorsoQuat.mData[2] * neckLength);
			cHead[2] = cTorso[2] + (-cTorsoQuat.mData[0] * neckLength);

			//Rotate right shoulder
			quaternion rShlrdQuat(-vector[2], vector[1], vector[0], 0);
			//quaternion rShlrdQuat(0, 0, -1, 0);
			rShlrdQuat = currentOri.b1.mutiplication(rShlrdQuat.mutiplication(currentOri.b1.Inverse()));
			rShldr[0] = cTorso[0] + (rShlrdQuat.mData[1] * rShldrLength);
			rShldr[1] = cTorso[1] + (-rShlrdQuat.mData[2] * rShldrLength);
			rShldr[2] = cTorso[2] + (-rShlrdQuat.mData[0] * rShldrLength);

			//Rotate left shoulder
			quaternion lShlrdQuat(vector[2], vector[1], vector[0], 0);
			//quaternion lShlrdQuat(0, 0, 1, 0);
			lShlrdQuat = currentOri.b1.mutiplication(lShlrdQuat.mutiplication(currentOri.b1.Inverse()));
			lShldr[0] = cTorso[0] + (lShlrdQuat.mData[1] * rShldrLength);
			lShldr[1] = cTorso[1] + (-lShlrdQuat.mData[2] * rShldrLength);
			lShldr[2] = cTorso[2] + (-lShlrdQuat.mData[0] * rShldrLength);

			//Rotate right upperArm
			quaternion rUarmQuat(vector[0], vector[1], -vector[2], 0);
			//quaternion rUarmQuat(0, -1, 0, 0);
			rUarmQuat = currentOri.b2.mutiplication(rUarmQuat.mutiplication(currentOri.b2.Inverse()));
			rUarm[0] = rShldr[0] + (rUarmQuat.mData[1] * uArmLength);
			rUarm[1] = rShldr[1] + (-rUarmQuat.mData[2] * uArmLength);
			rUarm[2] = rShldr[2] + (-rUarmQuat.mData[0] * uArmLength);

			rUarmQuat.normalize();
			//RAboneDataFile << rUarmQuat.mData[0] << "\t" << rUarmQuat.mData[1] << "\t" << rUarmQuat.mData[2] << "\t" << rUarmQuat.mData[3] << "\t";

			//Rotate Left upperArm
			quaternion lUarmQuat(vector[0], vector[1], -vector[2], 0);
			//quaternion lUarmQuat(0, -1, 0, 0);
			lUarmQuat = currentOri.b4.mutiplication(lUarmQuat.mutiplication(currentOri.b4.Inverse()));
			lUarm[0] = lShldr[0] + (lUarmQuat.mData[1] * uArmLength);
			lUarm[1] = lShldr[1] + (-lUarmQuat.mData[2] * uArmLength);
			lUarm[2] = lShldr[2] + (-lUarmQuat.mData[0] * uArmLength);

			lUarmQuat.normalize();
			//LAboneDataFile  << lUarmQuat.mData[0] << "\t" << lUarmQuat.mData[1] << "\t" << lUarmQuat.mData[2] << "\t" << lUarmQuat.mData[3] << "\t";

			//Rotate Right LowerArm
			quaternion rLarmQuat(vector[0], vector[1], -vector[2], 0);
			//quaternion rLarmQuat(0, -1, 0, 0);
			rLarmQuat = currentOri.b3.mutiplication(rLarmQuat.mutiplication(currentOri.b3.Inverse()));
			rLarm[0] = rUarm[0] + (rLarmQuat.mData[1] * lArmLength);
			rLarm[1] = rUarm[1] + (-rLarmQuat.mData[2] * lArmLength);
			rLarm[2] = rUarm[2] + (-rLarmQuat.mData[0] * lArmLength);

			rLarmQuat.normalize();
			//RAboneDataFile  << rLarmQuat.mData[0] << "\t" << rLarmQuat.mData[1] << "\t" << rLarmQuat.mData[2] << "\t" << rLarmQuat.mData[3] << "\n";

			//Rotate Left LowerArm
			quaternion lLarmQuat(vector[0], vector[1], -vector[2], 0);
			//quaternion lLarmQuat(0, -1, 0, 0);
			lLarmQuat = currentOri.b5.mutiplication(lLarmQuat.mutiplication(currentOri.b5.Inverse()));
			lLarm[0] = lUarm[0] + (lLarmQuat.mData[1] * lArmLength);
			lLarm[1] = lUarm[1] + (-lLarmQuat.mData[2] * lArmLength);
			lLarm[2] = lUarm[2] + (-lLarmQuat.mData[0] * lArmLength);

			lLarmQuat.normalize();
			//LAboneDataFile  << lLarmQuat.mData[0] << "\t" << lLarmQuat.mData[1] << "\t" << lLarmQuat.mData[2] << "\t" << lLarmQuat.mData[3] << "\n";

			updateBoneJoints(frameIndex);//Update Bone jounts in array to save to file

			cout << "center_X: " << cPlv[0] << "\tcenter_Y: " << cPlv[1] << "\tcenter_Z: " << cPlv[2] << endl;
			cout << "Right Arm_X:" << rLarm[0] << "\t Right Arm_Y:" << rLarm[1] << "\t Right Arm_Z:" << rLarm[2] << endl;
			cout << "Left Arm_X:" << lLarm[0] << "\t Left Arm_Y:" << lLarm[1] << "\t Left Arm_Z:" << lLarm[2] << endl;

			// 파일로 저장
			dataBuffer.push_back(cPlv);
			dataBuffer.push_back(rLarm);
			dataBuffer.push_back(lLarm);

			//
			//float yGroundDiff = 0;
			//if (isJointBelowGround(yGroundDiff))
			//{
			//	//calcuate and Update bone joints above ground
			//	rFoot[1] = rFoot[1] - yGroundDiff;
			//	lFoot[1] = lFoot[1] - yGroundDiff;
			//	rKnee[1] = rKnee[1] - yGroundDiff;
			//	lKnee[1] = lKnee[1] - yGroundDiff;
			//	cPlv[1] = cPlv[1] - yGroundDiff;
			//	lPlv[1] = lPlv[1] - yGroundDiff;
			//	rPlv[1] = rPlv[1] - yGroundDiff;
			//	cTorso[1] = cTorso[1] - yGroundDiff;
			//	cSternum[1] = cSternum[1] - yGroundDiff;
			//	cHead[1] = cHead[1] - yGroundDiff;
			//	rShldr[1] = rShldr[1] - yGroundDiff;
			//	lShldr[1] = lShldr[1] - yGroundDiff;
			//	rUarm[1] = rUarm[1] + yGroundDiff;
			//	lUarm[1] = lUarm[1] + yGroundDiff;
			//	rLarm[1] = rLarm[1] + yGroundDiff;
			//	lLarm[1] = lLarm[1] + yGroundDiff;
			//}

			//updateBoneJoints(frameIndex);//Update Bone jounts in array to save to file

			//cout << currentOri.b2 << "/t" << currentOri.b3 << endl;

			//Update skeliton Joint and bones
			DrawBoneCylinder(viewer, rFoot, rKnee, "RLLeg");//Right Foot to Right Knee joint
			DrawJointSphere(viewer, rFoot, "RLLegSphere");//Right Foot

			DrawBoneCylinder(viewer, lFoot, lKnee, "LLLeg");//Left Foot to Left Knee joint
			DrawJointSphere(viewer, lFoot, "LLLegSphere");//Left Foot

			DrawBoneCylinder(viewer, rKnee, rPlv, "RULeg");//Right Knee to Pelvis 
			DrawJointSphere(viewer, rKnee, "RightKneeSphere");//Right Knee

			DrawBoneCylinder(viewer, lKnee, lPlv, "LULeg");//Left Knee to Pelvis 
			DrawJointSphere(viewer, lKnee, "LeftKneeSphere");//Left Knee

			DrawBoneCylinder(viewer, cPlv, cSternum, "UpperSternum");//Pelvis to sternum
			DrawPlvJointSphere(viewer, cPlv, "PelvSphere");//Pelvis

			DrawBoneCylinder(viewer, cSternum, cTorso, "UpperBody");//Sternum to torso
			DrawJointSphere(viewer, cSternum, "sternumSphere");//Sternum

			DrawBoneCylinder(viewer, cPlv, rPlv, "RPelvis");//Pelvis to right pelvis
			DrawJointSphere(viewer, rPlv, "rPelvSphere");//Right Pelvis

			DrawBoneCylinder(viewer, cPlv, lPlv, "LPelvis");//Pelvis to left pelvis
			DrawJointSphere(viewer, lPlv, "lPelvSphere");//Left Pelvis

			DrawBoneCylinder(viewer, cTorso, cHead, "Neck");//Torso to head
			DrawJointSphere(viewer, cTorso, "SholderSphere");//Torso
			DrawJointSphere(viewer, cHead, "HeadSphere");//Head

			DrawBoneCylinder(viewer, cTorso, rShldr, "rightShoulder");//Torso to Right Shoulder
			DrawJointSphere(viewer, rShldr, "rightShoulderSphere");//Right Shoulder

			DrawBoneCylinder(viewer, cTorso, lShldr, "leftShoulder");//Torso to Left Shoulder
			DrawJointSphere(viewer, lShldr, "leftShoulderSphere");//Left Shoulder

			DrawBoneCylinder(viewer, rShldr, rUarm, "rightUpperArm");//Right Shoulder to Right Upper arm
			DrawJointSphere(viewer, rUarm, "rightUpperArmSphere");//Right Elbow joint

			DrawBoneCylinder(viewer, lShldr, lUarm, "leftUpperArm");//Left Shoulder to Left Upper arm
			DrawJointSphere(viewer, lUarm, "leftUpperArmSphere");//Left Elbow joint

			DrawBoneCylinder(viewer, rUarm, rLarm, "rightLowerArm");//Right Upper arm to Right lower arm
			DrawJointSphere(viewer, rLarm, "rightLowerArmSphere");//Right lower arm

			DrawBoneCylinder(viewer, lUarm, lLarm, "leftLowerArm");//Left Upper arm to right lower arm
			DrawJointSphere(viewer, lLarm, "leftLowerArmSphere");//right lower arm

			//cout << "IMU + Render" << frameIndex << endl;
		}

		tFrameIndex = frameIndex;
		gtFrameIndex = frameIndex;


		if (!PositionTracking::recordData)
		{

			//RAboneDataFile.close();
			//LAboneDataFile.close();
			//RLboneDataFile.close();
			//LLboneDataFile.close();
			if (frameIndex > 0)
			{
				saveFullBodySFQ(frameIndex);
				saveFullBodyRawdata(frameIndex); // 0721 update
			}
			frameIndex = -1;

			//rawDataFile.close();
		}

		//Read from file

		if (PositionTracking::readFile)//Read from file
		{
			//Draw skeliton Joint and bones
			DrawBoneCylinder(viewer, rFoot, rKnee, "RLLeg");//Right Foot to Right Knee joint
			DrawJointSphere(viewer, rFoot, "RLLegSphere");//Right Foot

			DrawBoneCylinder(viewer, lFoot, lKnee, "LLLeg");//Left Foot to Left Knee joint
			DrawJointSphere(viewer, lFoot, "LLLegSphere");//Left Foot

			DrawBoneCylinder(viewer, rKnee, rPlv, "RULeg");//Right Knee to Pelvis 
			DrawJointSphere(viewer, rKnee, "RightKneeSphere");//Right Knee

			DrawBoneCylinder(viewer, lKnee, lPlv, "LULeg");//Left Knee to Pelvis 
			DrawJointSphere(viewer, lKnee, "LeftKneeSphere");//Left Knee

			DrawBoneCylinder(viewer, cPlv, cSternum, "UpperSternum");//Pelvis to sternum
			DrawPlvJointSphere(viewer, cPlv, "PelvSphere");//Pelvis

			DrawBoneCylinder(viewer, cSternum, cTorso, "UpperBody");//Sternum to torso
			DrawJointSphere(viewer, cSternum, "sternumSphere");//Sternum

			DrawBoneCylinder(viewer, cPlv, rPlv, "RPelvis");//Pelvis to right pelvis
			DrawJointSphere(viewer, rPlv, "rPelvSphere");//Right Pelvis

			DrawBoneCylinder(viewer, cPlv, lPlv, "LPelvis");//Pelvis to left pelvis
			DrawJointSphere(viewer, lPlv, "lPelvSphere");//Left Pelvis

			DrawBoneCylinder(viewer, cTorso, cHead, "Neck");//Torso to head
			DrawJointSphere(viewer, cTorso, "SholderSphere");//Torso
			DrawJointSphere(viewer, cHead, "HeadSphere");//Head

			DrawBoneCylinder(viewer, cTorso, rShldr, "rightShoulder");//Torso to Right Shoulder
			DrawJointSphere(viewer, rShldr, "rightShoulderSphere");//Right Shoulder

			DrawBoneCylinder(viewer, cTorso, lShldr, "leftShoulder");//Torso to Left Shoulder
			DrawJointSphere(viewer, lShldr, "leftShoulderSphere");//Left Shoulder

			DrawBoneCylinder(viewer, rShldr, rUarm, "rightUpperArm");//Right Shoulder to Right Upper arm
			DrawJointSphere(viewer, rUarm, "rightUpperArmSphere");//Right Elbow joint

			DrawBoneCylinder(viewer, lShldr, lUarm, "leftUpperArm");//Left Shoulder to Left Upper arm
			DrawJointSphere(viewer, lUarm, "leftUpperArmSphere");//Left Elbow joint

			DrawBoneCylinder(viewer, rUarm, rLarm, "rightLowerArm");//Right Upper arm to Right lower arm
			DrawJointSphere(viewer, rLarm, "rightLowerArmSphere");//Right lower arm

			DrawBoneCylinder(viewer, lUarm, lLarm, "leftLowerArm");//Left Upper arm to right lower arm
			DrawJointSphere(viewer, lLarm, "leftLowerArmSphere");//right lower arm

			footLPoint[0] = lFoot[0];
			//footLPoint[1] = lFoot[1];
			footLPoint[2] = lFoot[2];
			footRPoint[0] = rFoot[0];
			//footRPoint[1] = rFoot[1];
			footRPoint[2] = rFoot[2];

			//Translate to both the legs to ground point
			Eigen::Vector3f diffR = footRPoint - rFoot; //Update new ground points 

			Eigen::Vector3f diffL = footLPoint - lFoot;

			updateLowerBody(diffR, diffL); // Lower body traslation to the ground point

			if (quatFrame == 0)
			{
				myAcquire.readFileQuatData("TotalCapture.txt");//read Rawquat form file

				//Save bone vector to a file
				/*time_t curr_time;
				curr_time = time(NULL);
				tm *tm_local = localtime(&curr_time);

				sprintf_s(fileName, ".\\BoneData\\FRAboneData-%d%d%d.txt", tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
				FRAboneDataFile.open(fileName);
				sprintf_s(fileName, ".\\BoneData\\FLAboneData-%d%d%d.txt", tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
				FLAboneDataFile.open(fileName);
				sprintf_s(fileName, ".\\BoneData\\FRLboneData-%d%d%d.txt", tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
				FRLboneDataFile.open(fileName);
				sprintf_s(fileName, ".\\BoneData\\FLLboneData-%d%d%d.txt", tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
				FLLboneDataFile.open(fileName);*/
			}


			if (quatFrame > myAcquire.sUtility.noOfFrames)
			{
				PositionTracking::readFile = false;
				quatFrame = 0;

				FRAboneDataFile.close();
				FLAboneDataFile.close();
				FRLboneDataFile.close();
				FLLboneDataFile.close();
			}

			Avatar currentOri = myAcquire.getFRQuatdata(quatFrame);
			quatFrame++;
			VitruvianAvatar::vitruvianAvatarUpdate = currentOri;

			double vector[3] = { 0,	0,	1 };

			//Right Foot <- Lower Right Leg orientation			
			quaternion rFootQuat(vector[0], vector[1], vector[2], 0);
			rFootQuat = currentOri.b7.mutiplication(rFootQuat.mutiplication(currentOri.b7.Inverse()));

			rKnee[0] = rFoot[0] + (rFootQuat.mData[1] * lLegLength);
			rKnee[1] = rFoot[1] + (-rFootQuat.mData[2] * lLegLength);
			rKnee[2] = rFoot[2] + (-rFootQuat.mData[0] * lLegLength);

			//Left Foot <- Lower Left Leg orientation		
			quaternion lFootQuat(vector[0], vector[1], vector[2], 0);

			lFootQuat = currentOri.b9.mutiplication(lFootQuat.mutiplication(currentOri.b9.Inverse()));

			lKnee[0] = lFoot[0] + (lFootQuat.mData[1] * lLegLength);
			lKnee[1] = lFoot[1] + (-lFootQuat.mData[2] * lLegLength);
			lKnee[2] = lFoot[2] + (-lFootQuat.mData[0] * lLegLength);

			//Right Kneepoint <- upper Right Leg orientation

			quaternion rKneeQuat(vector[0], vector[1], vector[2], 0);

			rKneeQuat = currentOri.b6.mutiplication(rKneeQuat.mutiplication(currentOri.b6.Inverse()));

			rPlv[0] = rKnee[0] + (rKneeQuat.mData[1] * uLegLength);
			rPlv[1] = rKnee[1] + (-rKneeQuat.mData[2] * uLegLength);
			rPlv[2] = rKnee[2] + (-rKneeQuat.mData[0] * uLegLength);

			//Left Kneepoint <- upper Left Leg orientation
			quaternion lKneeQuat(vector[0], vector[1], vector[2], 0);

			lKneeQuat = currentOri.b8.mutiplication(lKneeQuat.mutiplication(currentOri.b8.Inverse()));

			lPlv[0] = lKnee[0] + (lKneeQuat.mData[1] * uLegLength);
			lPlv[1] = lKnee[1] + (-lKneeQuat.mData[2] * uLegLength);
			lPlv[2] = lKnee[2] + (-lKneeQuat.mData[0] * uLegLength);


			////Compute new Pelvis by comparing Right and Left Pelvis position
			//if (rPlv[1] <= lPlv[1])//Considering right leg 
			//{
			//	//updateLowerBody({ 0,0,0 }, (lPlv - rPlv)); // translation left leg to pelvise

			//	float diffL = (rPlv[1] - lPlv[1]);

			//	lKnee[1] = lKnee[1] + diffL;
			//	lFoot[1] = lFoot[1] + diffL;
			//	lPlv[1] = lPlv[1] + diffL;

			//	diffL = -(lPlv[0] - rPlv[0]);

			//	lKnee[0] = lKnee[0] + diffL;
			//	lFoot[0] = lFoot[0] + diffL;
			//	lPlv[0] = lPlv[0] + diffL;
			//}

			//if (lPlv[1] < rPlv[1])//Considering leg leg 
			//{
			//	//updateLowerBody((rPlv - lPlv), { 0,0,0 }); // translation right leg to pelvise
			//	float diffR = (lPlv[1] - rPlv[1]);
			//	rKnee[1] = rKnee[1] + diffR;
			//	rFoot[1] = rFoot[1] + diffR;
			//	rPlv[1] = rPlv[1] + diffR;

			//	diffR = -(rPlv[0] - lPlv[0]);

			//	rKnee[0] = rKnee[0] + diffR;
			//	rFoot[0] = rFoot[0] + diffR;
			//	rPlv[0] = rPlv[0] + diffR;
			//}

			////Translate upper body to match pelvise height
			//prevcPelv = cPlv;
			//cPlv = (rPlv + lPlv) / 2;

			////cPlv[0] = centroidX[0]; //Center of Max-X and Min-X

			//updateUpperBody(prevcPelv - cPlv);

			//Compute new Pelvis by comparing Right and Left Pelvis position
			if (lPlv[1] > rPlv[1])//Considering right leg 
			{
				//updateLowerBody({ 0,0,0 }, (lPlv - rPlv)); // translation left leg to pelvise

				/*float diffL = (rPlv[1] - lPlv[1]);

				lKnee[1] = lKnee[1] + diffL;
				lFoot[1] = lFoot[1] + diffL;
				lPlv[1] = lPlv[1] + diffL;

				diffL = -(lPlv[0] - rPlv[0]);

				lKnee[0] = lKnee[0] + diffL;
				lFoot[0] = lFoot[0] + diffL;
				lPlv[0] = lPlv[0] + diffL;*/

				quaternion plvQuat(vector[2], vector[1], vector[0], 0);

				plvQuat = currentOri.b0.mutiplication(plvQuat.mutiplication(currentOri.b0.Inverse()));

				cPlv[0] = rPlv[0] + (plvQuat.mData[1] * sPlvLength);
				cPlv[1] = rPlv[1] + (-plvQuat.mData[2] * sPlvLength);
				cPlv[2] = rPlv[2] + (-plvQuat.mData[0] * sPlvLength);

			}

			if (rPlv[1] >= lPlv[1])//Considering leg leg 
			{
				//updateLowerBody((rPlv - lPlv), { 0,0,0 }); // translation right leg to pelvise
				/*float diffR = (lPlv[1] - rPlv[1]);
				rKnee[1] = rKnee[1] + diffR;
				rFoot[1] = rFoot[1] + diffR;
				rPlv[1] = rPlv[1] + diffR;

				diffR = -(rPlv[0] - lPlv[0]);

				rKnee[0] = rKnee[0] + diffR;
				rFoot[0] = rFoot[0] + diffR;
				rPlv[0] = rPlv[0] + diffR;*/

				quaternion plvQuat(-vector[2], vector[1], vector[0], 0);

				plvQuat = currentOri.b0.mutiplication(plvQuat.mutiplication(currentOri.b0.Inverse()));

				cPlv[0] = lPlv[0] + (plvQuat.mData[1] * sPlvLength);
				cPlv[1] = lPlv[1] + (-plvQuat.mData[2] * sPlvLength);
				cPlv[2] = lPlv[2] + (-plvQuat.mData[0] * sPlvLength);
			}

			//-----------------------------------------------------------------------------------------

			//Right Pelvis <- Pelvis orientation			
			quaternion rplvQuat1(-vector[2], vector[1], vector[0], 0);
			rplvQuat1 = currentOri.b0.mutiplication(rplvQuat1.mutiplication(currentOri.b0.Inverse()));

			rPlv[0] = cPlv[0] /*+ (rplvQuat1.mData[1] * sPlvLength)*/;
			rPlv[1] = cPlv[1] /*+ (-rplvQuat1.mData[2] * sPlvLength)*/;
			rPlv[2] = cPlv[2] + (-rplvQuat1.mData[0] * sPlvLength);

			//Left Pelvis <- Pelvis orientation			
			quaternion lplvQuat1(vector[2], vector[1], vector[0], 0);
			lplvQuat1 = currentOri.b0.mutiplication(lplvQuat1.mutiplication(currentOri.b0.Inverse()));

			lPlv[0] = cPlv[0] /*+ (lplvQuat1.mData[1] * sPlvLength) */;
			lPlv[1] = cPlv[1] /*+ (-lplvQuat1.mData[2] * sPlvLength)*/;
			lPlv[2] = cPlv[2] + (-lplvQuat1.mData[0] * sPlvLength);

			//Recomputing left leg orientation by changing the intial vector direction
			//Right Foot <- Lower Right Leg orientation			
			quaternion rFootQuat1(vector[0], vector[1], -vector[2], 0);
			rFootQuat1 = currentOri.b6.mutiplication(rFootQuat1.mutiplication(currentOri.b6.Inverse()));

			rKnee[0] = rPlv[0] + (rFootQuat1.mData[1] * uLegLength);
			rKnee[1] = rPlv[1] + (-rFootQuat1.mData[2] * uLegLength);
			rKnee[2] = rPlv[2] + (-rFootQuat1.mData[0] * uLegLength);

			//Right Kneepoint <- upper Right Leg orientation

			quaternion rKneeQuat1(vector[0], vector[1], -vector[2], 0);

			rKneeQuat1 = currentOri.b7.mutiplication(rKneeQuat1.mutiplication(currentOri.b7.Inverse()));

			rFoot[0] = rKnee[0] + (rKneeQuat1.mData[1] * lLegLength);
			rFoot[1] = rKnee[1] + (-rKneeQuat1.mData[2] * lLegLength);
			rFoot[2] = rKnee[2] + (-rKneeQuat1.mData[0] * lLegLength);

			//Left Foot <- Lower Left Leg orientation		
			quaternion lFootQuat1(vector[0], vector[1], -vector[2], 0);

			lFootQuat1 = currentOri.b8.mutiplication(lFootQuat1.mutiplication(currentOri.b8.Inverse()));

			lKnee[0] = lPlv[0] + (lFootQuat1.mData[1] * uLegLength);
			lKnee[1] = lPlv[1] + (-lFootQuat1.mData[2] * uLegLength);
			lKnee[2] = lPlv[2] + (-lFootQuat1.mData[0] * uLegLength);

			//Left Kneepoint <- upper Left Leg orientation
			quaternion lKneeQuat1(vector[0], vector[1], -vector[2], 0);

			lKneeQuat1 = currentOri.b9.mutiplication(lKneeQuat1.mutiplication(currentOri.b9.Inverse()));

			lFoot[0] = lKnee[0] + (lKneeQuat1.mData[1] * lLegLength);
			lFoot[1] = lKnee[1] + (-lKneeQuat1.mData[2] * lLegLength);
			lFoot[2] = lKnee[2] + (-lKneeQuat1.mData[0] * lLegLength);

			//-------------------------------------------------------------------------------------------------------

			//Upper body sternum rotation 
			quaternion cSternumQuat(vector[0], vector[1], vector[2], 0);
			cSternumQuat = currentOri.b0.mutiplication(cSternumQuat.mutiplication(currentOri.b0.Inverse()));

			cSternum[0] = cPlv[0] + (cSternumQuat.mData[1] * uBodyLength * 0.6);
			cSternum[1] = cPlv[1] + (-cSternumQuat.mData[2] * uBodyLength * 0.6);
			cSternum[2] = cPlv[2] + (-cSternumQuat.mData[0] * uBodyLength * 0.6);

			//Upper body torso rotation 
			quaternion cTorsoQuat(vector[0], vector[1], vector[2], 0);
			cTorsoQuat = currentOri.b1.mutiplication(cTorsoQuat.mutiplication(currentOri.b1.Inverse()));

			cTorso[0] = cSternum[0] + (cTorsoQuat.mData[1] * uBodyLength * 0.4);
			cTorso[1] = cSternum[1] + (-cTorsoQuat.mData[2] * uBodyLength * 0.4);
			cTorso[2] = cSternum[2] + (-cTorsoQuat.mData[0] * uBodyLength * 0.4);

			//Translate Head
			cHead[0] = cTorso[0] + (cTorsoQuat.mData[1] * neckLength);
			cHead[1] = cTorso[1] + (-cTorsoQuat.mData[2] * neckLength);
			cHead[2] = cTorso[2] + (-cTorsoQuat.mData[0] * neckLength);

			//Rotate right shoulder
			quaternion rShlrdQuat(-vector[2], vector[1], vector[0], 0);
			rShlrdQuat = currentOri.b1.mutiplication(rShlrdQuat.mutiplication(currentOri.b1.Inverse()));
			rShldr[0] = cTorso[0] + (-rShlrdQuat.mData[1] * rShldrLength);
			rShldr[1] = cTorso[1] + (-rShlrdQuat.mData[2] * rShldrLength);
			rShldr[2] = cTorso[2] + (-rShlrdQuat.mData[0] * rShldrLength);

			//Rotate right upperArm
			quaternion rUarmQuat(vector[0], vector[1], -vector[2], 0);
			rUarmQuat = currentOri.b2.mutiplication(rUarmQuat.mutiplication(currentOri.b2.Inverse()));
			rUarm[0] = rShldr[0] + (-rUarmQuat.mData[1] * uArmLength);
			rUarm[1] = rShldr[1] + (-rUarmQuat.mData[2] * uArmLength);
			rUarm[2] = rShldr[2] + (-rUarmQuat.mData[0] * uArmLength);

			//Rotate Right LowerArm
			quaternion rLarmQuat(vector[0], vector[1], -vector[2], 0);
			rLarmQuat = currentOri.b3.mutiplication(rLarmQuat.mutiplication(currentOri.b3.Inverse()));
			rLarm[0] = rUarm[0] + (-rLarmQuat.mData[1] * lArmLength);
			rLarm[1] = rUarm[1] + (-rLarmQuat.mData[2] * lArmLength);
			rLarm[2] = rUarm[2] + (-rLarmQuat.mData[0] * lArmLength);

			//Rotate left shoulder
			quaternion lShlrdQuat(vector[2], vector[1], vector[0], 0);
			lShlrdQuat = currentOri.b1.mutiplication(lShlrdQuat.mutiplication(currentOri.b1.Inverse()));
			lShldr[0] = cTorso[0] + (-lShlrdQuat.mData[1] * rShldrLength);
			lShldr[1] = cTorso[1] + (-lShlrdQuat.mData[2] * rShldrLength);
			lShldr[2] = cTorso[2] + (-lShlrdQuat.mData[0] * rShldrLength);

			//Rotate Left upperArm
			quaternion lUarmQuat(vector[0], vector[1], -vector[2], 0);
			lUarmQuat = currentOri.b4.mutiplication(lUarmQuat.mutiplication(currentOri.b4.Inverse()));
			lUarm[0] = lShldr[0] + (-lUarmQuat.mData[1] * uArmLength);
			lUarm[1] = lShldr[1] + (-lUarmQuat.mData[2] * uArmLength);
			lUarm[2] = lShldr[2] + (-lUarmQuat.mData[0] * uArmLength);

			//Rotate Left LowerArm
			quaternion lLarmQuat(vector[0], vector[1], -vector[2], 0);
			lLarmQuat = currentOri.b5.mutiplication(lLarmQuat.mutiplication(currentOri.b5.Inverse()));
			lLarm[0] = lUarm[0] + (-lLarmQuat.mData[1] * lArmLength);
			lLarm[1] = lUarm[1] + (-lLarmQuat.mData[2] * lArmLength);
			lLarm[2] = lUarm[2] + (-lLarmQuat.mData[0] * lArmLength);

			//Update skeliton Joint and bones
			DrawBoneCylinder(viewer, rFoot, rKnee, "RLLeg");//Right Foot to Right Knee joint
			DrawJointSphere(viewer, rFoot, "RLLegSphere");//Right Foot

			DrawBoneCylinder(viewer, lFoot, lKnee, "LLLeg");//Left Foot to Left Knee joint
			DrawJointSphere(viewer, lFoot, "LLLegSphere");//Left Foot

			DrawBoneCylinder(viewer, rKnee, rPlv, "RULeg");//Right Knee to Pelvis 
			DrawJointSphere(viewer, rKnee, "RightKneeSphere");//Right Knee

			DrawBoneCylinder(viewer, lKnee, lPlv, "LULeg");//Left Knee to Pelvis 
			DrawJointSphere(viewer, lKnee, "LeftKneeSphere");//Left Knee

			DrawBoneCylinder(viewer, cPlv, cSternum, "UpperSternum");//Pelvis to sternum
			DrawPlvJointSphere(viewer, cPlv, "PelvSphere");//Pelvis

			DrawBoneCylinder(viewer, cSternum, cTorso, "UpperBody");//Sternum to torso
			DrawJointSphere(viewer, cSternum, "sternumSphere");//Sternum

			DrawBoneCylinder(viewer, cPlv, rPlv, "RPelvis");//Pelvis to right pelvis
			DrawJointSphere(viewer, rPlv, "rPelvSphere");//Right Pelvis

			DrawBoneCylinder(viewer, cPlv, lPlv, "LPelvis");//Pelvis to left pelvis
			DrawJointSphere(viewer, lPlv, "lPelvSphere");//Left Pelvis

			DrawBoneCylinder(viewer, cTorso, cHead, "Neck");//Torso to head
			DrawJointSphere(viewer, cTorso, "SholderSphere");//Torso
			DrawJointSphere(viewer, cHead, "HeadSphere");//Head

			DrawBoneCylinder(viewer, cTorso, rShldr, "rightShoulder");//Torso to Right Shoulder
			DrawJointSphere(viewer, rShldr, "rightShoulderSphere");//Right Shoulder

			DrawBoneCylinder(viewer, cTorso, lShldr, "leftShoulder");//Torso to Left Shoulder
			DrawJointSphere(viewer, lShldr, "leftShoulderSphere");//Left Shoulder

			DrawBoneCylinder(viewer, rShldr, rUarm, "rightUpperArm");//Right Shoulder to Right Upper arm
			DrawJointSphere(viewer, rUarm, "rightUpperArmSphere");//Right Elbow joint

			DrawBoneCylinder(viewer, lShldr, lUarm, "leftUpperArm");//Left Shoulder to Left Upper arm
			DrawJointSphere(viewer, lUarm, "leftUpperArmSphere");//Left Elbow joint

			DrawBoneCylinder(viewer, rUarm, rLarm, "rightLowerArm");//Right Upper arm to Right lower arm
			DrawJointSphere(viewer, rLarm, "rightLowerArmSphere");//Right lower arm

			DrawBoneCylinder(viewer, lUarm, lLarm, "leftLowerArm");//Left Upper arm to right lower arm
			DrawJointSphere(viewer, lLarm, "leftLowerArmSphere");//right lower arm


		}
		else
		{
			quatFrame = 0;
		}

		try
		{
			viewer->spinOnce();
		}
		catch (const std::length_error& le)
		{
			viewer->removeAllPointClouds();

			viewer->addPointCloud(cloudA, "cloud", v1);
			/*viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "cloud");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.9, "cloud");*/
			er_flag = true;
			std::cerr << "Length error: " << le.what() << '\n';
		}
	}
}

void PositionTracking::getPositionData(float* pose)
{
	if (tFrameIndex > 0)
	{
		pose[0] = (allBoneJoints->BoneJoints[0][0] - bonePose.BoneJoints[0][0]) / 10.0;
		pose[1] = (allBoneJoints->BoneJoints[0][1] - bonePose.BoneJoints[0][1]) / 10.0;
		pose[2] = (allBoneJoints->BoneJoints[0][2] - bonePose.BoneJoints[0][2]) / 10.0;
	}
	else
	{
		pose[0] = 0.0;
		pose[1] = 0.0;
		pose[2] = 0.0;
	}
}

void PositionTracking::resetParameters()
{
	frameIndex = -1;
	motionIndex = 0;
	allFrame = 0;
	interpolate = 0;
	bodyCenter.clear();
	allClouds.clear();
	motionClouds.clear();
	bodyCenter_temp.clear();
	flag2timeL1 = true;
	framesCountL1 = 0;
	framesCountL2 = 0;
	gtFrameIndex = 0;
	bOnetime = true;
}

void PositionTracking::saveSFQuatData(int noOfFrames)
{
	time_t curr_time;
	curr_time = time(NULL);
	tm* tm_local = localtime(&curr_time);

	ofstream avatarDataFile;

	char fileName[1024];


	sprintf_s(fileName, ".\\SkeletonData\\RawIMUData-00%d-%d%d%d.txt", 0, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatarDataFile.open(fileName);

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << noOfFrames << "\n";


	for (int tCount = 0; tCount < noOfFrames; tCount++)
	{
		avatarDataFile
			<< myAcquire.sUtility.avatarData[tCount].b0.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b0.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b0.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b0.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b1.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b1.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b1.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b1.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b2.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b2.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b2.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b2.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b3.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b3.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b3.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b3.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b4.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b4.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b4.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b4.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b5.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b5.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b5.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b5.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b6.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b6.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b6.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b6.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b7.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b7.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b7.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b7.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b8.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b8.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b8.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b8.mData[2] << "\t"
			<< myAcquire.sUtility.avatarData[tCount].b9.mData[3] << "\t" << myAcquire.sUtility.avatarData[tCount].b9.mData[0] << "\t" << myAcquire.sUtility.avatarData[tCount].b9.mData[1] << "\t" << myAcquire.sUtility.avatarData[tCount].b9.mData[2] << "\n";
	}
	avatarDataFile.close();
}

void PositionTracking::saveFullBodySFQ(int noOfFrames)
{
	time_t curr_time;
	curr_time = time(NULL);
	tm* tm_local = localtime(&curr_time);

	ofstream avatarDataFile;

	char fileName[1024];

	sprintf_s(fileName, ".\\SkeletonData\\FB-SFQ-Data-00%d-%d%d%d.txt", 0, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatarDataFile.open(fileName);



	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << noOfFrames << "\n";



	for (int tCount = 0; tCount < noOfFrames; tCount++)
	{
		avatarDataFile
			<< sfqData[tCount][0].mData[3] << "\t" << sfqData[tCount][0].mData[0] << "\t" << sfqData[tCount][0].mData[1] << "\t" << sfqData[tCount][0].mData[2] << "\t"
			<< sfqData[tCount][1].mData[3] << "\t" << sfqData[tCount][1].mData[0] << "\t" << sfqData[tCount][1].mData[1] << "\t" << sfqData[tCount][1].mData[2] << "\t"
			<< sfqData[tCount][2].mData[3] << "\t" << sfqData[tCount][2].mData[0] << "\t" << sfqData[tCount][2].mData[1] << "\t" << sfqData[tCount][2].mData[2] << "\t"
			<< sfqData[tCount][3].mData[3] << "\t" << sfqData[tCount][3].mData[0] << "\t" << sfqData[tCount][3].mData[1] << "\t" << sfqData[tCount][3].mData[2] << "\t"
			<< sfqData[tCount][4].mData[3] << "\t" << sfqData[tCount][4].mData[0] << "\t" << sfqData[tCount][4].mData[1] << "\t" << sfqData[tCount][4].mData[2] << "\t"
			<< sfqData[tCount][5].mData[3] << "\t" << sfqData[tCount][5].mData[0] << "\t" << sfqData[tCount][5].mData[1] << "\t" << sfqData[tCount][5].mData[2] << "\t"
			<< sfqData[tCount][6].mData[3] << "\t" << sfqData[tCount][6].mData[0] << "\t" << sfqData[tCount][6].mData[1] << "\t" << sfqData[tCount][6].mData[2] << "\t"
			<< sfqData[tCount][7].mData[3] << "\t" << sfqData[tCount][7].mData[0] << "\t" << sfqData[tCount][7].mData[1] << "\t" << sfqData[tCount][7].mData[2] << "\t"
			<< sfqData[tCount][8].mData[3] << "\t" << sfqData[tCount][8].mData[0] << "\t" << sfqData[tCount][8].mData[1] << "\t" << sfqData[tCount][8].mData[2] << "\t"
			<< sfqData[tCount][9].mData[3] << "\t" << sfqData[tCount][9].mData[0] << "\t" << sfqData[tCount][9].mData[1] << "\t" << sfqData[tCount][9].mData[2] << "\n";
	}


	//실시간 진행중 -
	for (int tCount = 0; tCount < noOfFrames; tCount++)
	{
		ms.su->avatarData[tCount].b0 = sfqData[tCount][0];
		ms.su->avatarData[tCount].b1 = sfqData[tCount][1];
		ms.su->avatarData[tCount].b2 = sfqData[tCount][2];
		ms.su->avatarData[tCount].b3 = sfqData[tCount][3];
		ms.su->avatarData[tCount].b4 = sfqData[tCount][4];
		ms.su->avatarData[tCount].b5 = sfqData[tCount][5];
		ms.su->avatarData[tCount].b6 = sfqData[tCount][6];
		ms.su->avatarData[tCount].b7 = sfqData[tCount][7];
		ms.su->avatarData[tCount].b8 = sfqData[tCount][8];
		ms.su->avatarData[tCount].b9 = sfqData[tCount][9];

		//std::cout << "Frame " << tCount << " Data:" << std::endl;
		//std::cout << "b0: " << ms.su->avatarData[tCount].b0.mData[3] << ", " << ms.su->avatarData[tCount].b0.mData[0] << ", " << ms.su->avatarData[tCount].b0.mData[1] << ", " << ms.su->avatarData[tCount].b0.mData[2] << std::endl;
		//std::cout << "b1: " << ms.su->avatarData[tCount].b1.mData[3] << ", " << ms.su->avatarData[tCount].b1.mData[0] << ", " << ms.su->avatarData[tCount].b1.mData[1] << ", " << ms.su->avatarData[tCount].b1.mData[2] << std::endl;
		//std::cout << "b2: " << ms.su->avatarData[tCount].b2.mData[3] << ", " << ms.su->avatarData[tCount].b2.mData[0] << ", " << ms.su->avatarData[tCount].b2.mData[1] << ", " << ms.su->avatarData[tCount].b2.mData[2] << std::endl;
		//std::cout << "b3: " << ms.su->avatarData[tCount].b3.mData[3] << ", " << ms.su->avatarData[tCount].b3.mData[0] << ", " << ms.su->avatarData[tCount].b3.mData[1] << ", " << ms.su->avatarData[tCount].b3.mData[2] << std::endl;
		//std::cout << "b4: " << ms.su->avatarData[tCount].b4.mData[3] << ", " << ms.su->avatarData[tCount].b4.mData[0] << ", " << ms.su->avatarData[tCount].b4.mData[1] << ", " << ms.su->avatarData[tCount].b4.mData[2] << std::endl;
		//std::cout << "b5: " << ms.su->avatarData[tCount].b5.mData[3] << ", " << ms.su->avatarData[tCount].b5.mData[0] << ", " << ms.su->avatarData[tCount].b5.mData[1] << ", " << ms.su->avatarData[tCount].b5.mData[2] << std::endl;
		//std::cout << "b6: " << ms.su->avatarData[tCount].b6.mData[3] << ", " << ms.su->avatarData[tCount].b6.mData[0] << ", " << ms.su->avatarData[tCount].b6.mData[1] << ", " << ms.su->avatarData[tCount].b6.mData[2] << std::endl;
		//std::cout << "b7: " << ms.su->avatarData[tCount].b7.mData[3] << ", " << ms.su->avatarData[tCount].b7.mData[0] << ", " << ms.su->avatarData[tCount].b7.mData[1] << ", " << ms.su->avatarData[tCount].b7.mData[2] << std::endl;
		//std::cout << "b8: " << ms.su->avatarData[tCount].b7.mData[3] << ", " << ms.su->avatarData[tCount].b7.mData[0] << ", " << ms.su->avatarData[tCount].b7.mData[1] << ", " << ms.su->avatarData[tCount].b7.mData[2] << std::endl;
		//std::cout << "b9: " << ms.su->avatarData[tCount].b7.mData[3] << ", " << ms.su->avatarData[tCount].b7.mData[0] << ", " << ms.su->avatarData[tCount].b7.mData[1] << ", " << ms.su->avatarData[tCount].b7.mData[2] << std::endl;
	}

	avatarDataFile.close();

	//delete sfqData;
}

void PositionTracking::saveFullBodyRawdata(int noOfFrames) // 21. 07 21. update
{
	time_t curr_time;
	curr_time = time(NULL);
	tm* tm_local = localtime(&curr_time);

	ofstream avatarDataFile;

	char fileName[1024];


	sprintf_s(fileName, ".\\SkeletonData\\FB-Raw-Data-00%d-%d%d%d.txt", 0, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatarDataFile.open(fileName);

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << noOfFrames << "\n";

	for (int tCount = 0; tCount < noOfFrames; tCount++)
	{
		avatarDataFile
			<< RawData[tCount][0].mData[3] << "\t" << RawData[tCount][0].mData[0] << "\t" << RawData[tCount][0].mData[1] << "\t" << RawData[tCount][0].mData[2] << "\t"
			<< RawData[tCount][1].mData[3] << "\t" << RawData[tCount][1].mData[0] << "\t" << RawData[tCount][1].mData[1] << "\t" << RawData[tCount][1].mData[2] << "\t"
			<< RawData[tCount][2].mData[3] << "\t" << RawData[tCount][2].mData[0] << "\t" << RawData[tCount][2].mData[1] << "\t" << RawData[tCount][2].mData[2] << "\t"
			<< RawData[tCount][3].mData[3] << "\t" << RawData[tCount][3].mData[0] << "\t" << RawData[tCount][3].mData[1] << "\t" << RawData[tCount][3].mData[2] << "\t"
			<< RawData[tCount][4].mData[3] << "\t" << RawData[tCount][4].mData[0] << "\t" << RawData[tCount][4].mData[1] << "\t" << RawData[tCount][4].mData[2] << "\t"
			<< RawData[tCount][5].mData[3] << "\t" << RawData[tCount][5].mData[0] << "\t" << RawData[tCount][5].mData[1] << "\t" << RawData[tCount][5].mData[2] << "\t"
			<< RawData[tCount][6].mData[3] << "\t" << RawData[tCount][6].mData[0] << "\t" << RawData[tCount][6].mData[1] << "\t" << RawData[tCount][6].mData[2] << "\t"
			<< RawData[tCount][7].mData[3] << "\t" << RawData[tCount][7].mData[0] << "\t" << RawData[tCount][7].mData[1] << "\t" << RawData[tCount][7].mData[2] << "\t"
			<< RawData[tCount][8].mData[3] << "\t" << RawData[tCount][8].mData[0] << "\t" << RawData[tCount][8].mData[1] << "\t" << RawData[tCount][8].mData[2] << "\t"
			<< RawData[tCount][9].mData[3] << "\t" << RawData[tCount][9].mData[0] << "\t" << RawData[tCount][9].mData[1] << "\t" << RawData[tCount][9].mData[2] << "\n";
	}

	avatarDataFile.close();

	//delete sfqData;
}


void PositionTracking::saveQautData()
{
	if (PositionTracking::isCalib && framesCountL1 > 1 && framesCountL2 > 1)
	{
		myAcquire.saveRawQDataInRealTime(frameIndex, false);
	}
	else
	{
		myAcquire.saveRawQDataInRealTime(frameIndex, true);
	}
}

