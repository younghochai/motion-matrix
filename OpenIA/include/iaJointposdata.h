#pragma once


#include "iaquaternion.h"
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include "iaPositionTracking.h"
struct JointPos {
	
	Eigen::Vector3f Jointspos[16];
	
};
struct JointQuat {

	quaternion JointsQuat[10];

};

namespace gst{
struct Bodypos {

	double x, y, z;

};
}

class GenData {
	
public:

	//~GenData() {
	//	//cout << "[°´Ã¼°¡ ¼Ò¸êµË´Ï´Ù.]\n";
	//}
	JointQuat QuatData[9000];
	GenData* EditData;
	double jointPoint[17][3];
	double jointPoint_backup[17][3];

	double Eulerangle[100][24][3];

	int maxframe;
	string poseName;
	int motion_idx;

	void genQuaternion();
	void ExtractSeed();
	void testSLERPFT();
	void FindInitEpt();
	void FindthrePoint();
	void GenEditSlerp();
	
	void data_save();
	void euler_save();
	void euler_toquat();

	void data_sample();
	void pos_check();
};

class DynamicData {

public:
	JointPos posData[2000];
	JointPos NorData[2000];
	vector<vector<int>> keyframenum;
	int keyframecnt = 5;
	vector<int> movingjoint;
	int motionclass;
	int maxframe;
	string posename;
	
	double tjtrdis;

	double avg[16][3];
	double variance[16][3];
	vector<vector<vector<double>>> compareData;

	void calvar();
	void Searchmove();
	void ExtractKeyframe(int keyposenum);
	void ExtractData();
	void removePelvis();
	void calDist();

};

class MHADdata {

public:
	std::vector<std::vector<gst::Bodypos>> MotionsphereD;
	
	std::vector<double> testAngle;
	std::vector<double> testAngle2;
	std::vector<double> testAngle3;
	std::vector<double> testAngle4;
	std::vector<double> testAngle5;

	std::vector<gst::Bodypos> testAxis;
	std::vector<gst::Bodypos> testAxis2;
	std::vector<gst::Bodypos> testAxis3;
	std::vector<gst::Bodypos> testAxis4;


	std::vector<std::vector<gst::Bodypos>> trainingData;
	int maxframe;
	string posename;
	int training_data_frame;
	vector<int> movingjoint;
	vector<int> A_group = {1,2,3,4,7,8,15,16};
	vector<int> B_group = {9,11,12,13,14,17};
	vector<int> C_group = {10,18};
	vector<int> D_group = {5,6};
	

};


class JointPosition {
public:
	JointPos posData[2000];
	JointPos loadData[2000];

	bool dataGet = false;
	int maxframe;
	double avg[16][3];
	double variance[16][3];
	double Tposdata[16][3];
	double Bothhandup[16][3];

	vector<DynamicData*> Dynamicdatacontrol;
	vector<DynamicData*> Dynamicreferdatacontrol;
	vector<DynamicData*> MSDatacontrol;
	vector<DynamicData*> MSreferDatacontrol;

	vector<GenData*> GenDatacontrol;
	int Genrefernum = 5;
	int Genrefercnt = 0;


	vector<MHADdata*> MHADatacontrol;
	vector<MHADdata*> MHAreferDatacontrol;        // A                    //13
	vector<char> group = { 'A','A','A','A','D','D','E','A','B','C','E','B','F','B','A','A','B','D' };
	int MHADdatanum = 576;
	int MHADreferdatanum = 36;
	int action_number = 18;


	int MSdatanum=50;
	int Msdatacnt = 0;
	int MSreferdatanum = 10;
	int MSreferdatacnt=0;


	int Dydatatotalnum = 50;
	int Dydataloadnum=0;
	int Dycompdataloadnum =50;
	int Dyreferdataloadnum = 0;
	int Dyreferdataloadtotnum = 10;
	vector<vector<int>> keyframecomparenum;


	double referencepose[19][16][3];
	string posename[19];
	int referFramenum[19];
	int loadingfilecount = 0;
	int loadingfilenum;

	double Test[16][3];

	
	int tposeframe;
	int bothposeframe;
	int testposeframe;
public:

	void calavg();
	void calvar();
	void loaddataT();
	void loaddataB();
	void loadtestdata();

	void loadDydata();
	void loadDyreferdata();
	void loadMSdata();
	void loadMSreferdata();

	// Test
	void referCalvecangle();
	void Calvecangle();

	// bvh loader
	void loadbvhdata();


	// gen quat
	void loadGeNQuatdata();
	void GeNQuatdata();

	void loadMHADdata();
	void loadMHADreferdata();
	void trainingReferData();
	void detectMovingjoint();
	void detecttestMovingjoint();
	void setloadingfilenum(int i)
	{
		this->loadingfilenum = i;
	}
	void checkposename()
	{
		for(int i=0;i<this->loadingfilenum;i++)
		cout <<"file "<<i<<" pose : "<<this->posename[i] <<endl;

	}
	void loadreferencedata();
	
	void calloadavg(int maxframe, double data[][3]);

};