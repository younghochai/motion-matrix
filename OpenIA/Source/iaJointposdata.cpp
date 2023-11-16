#pragma once

/*
*This work is dual-licensed under BSD-3 and Apache License 2.0. 

*You can choose between one of them if you use this work.

*SPDX-License-Identifier: BSD-3-Clause OR Apache License 2.0

*/
#include "iaJointposdata.h"
//#include "mat.h"
#include <cmath>
#include <random>

double CalculatePointDistance(const Eigen::Vector3f start, const Eigen::Vector3f current) {
	//return abs(x - y);
	double check;
	check = std::sqrt(std::pow((start[0] - current[0]), 2) + std::pow((start[1] - current[1]), 2) + std::pow((start[2] - current[2]), 2));
	//cout<<"dist value : " <<check <<endl;
	return check;
}

void DynamicData::removePelvis()
{
	int jointnum = 16;
	double Pelvis[3] = { 0,0,0 };

		Pelvis[0] = this->posData[0].Jointspos[0][0];
		Pelvis[1] = this->posData[1].Jointspos[0][1];
		Pelvis[2] = this->posData[2].Jointspos[0][2];
	for (int i = 0; i < jointnum; i++)
	{
		for (int j = 0; j < this->maxframe; j++)
		{
			//double scale = sqrt(pow(this->posData[j].Jointspos[i][0], 2) + pow(this->posData[j].Jointspos[i][1], 2) + pow(this->posData[j].Jointspos[i][2], 2));
			this->posData[j].Jointspos[i][0] = this->posData[j].Jointspos[i][0]- Pelvis[0];
			this->posData[j].Jointspos[i][1] = this->posData[j].Jointspos[i][1] - Pelvis[1];
			this->posData[j].Jointspos[i][2] = this->posData[j].Jointspos[i][2] - Pelvis[2];
			/*jpos.posData[tCount].Jointspos[0][0]*/
		}
	}	
}

void DynamicData::calDist()
{
	int jointnum = 16;
	
	double buf = 0;
	for (int i = 0; i < jointnum; i++)
	{
		double result = 0;
		for (int j = 0; j < this->maxframe - 1; j++)
		{
			result += CalculatePointDistance(this->posData[j].Jointspos[i], this->posData[j + 1].Jointspos[i]);
		}
		buf += result;
	}
	this->tjtrdis = buf;
}


void DynamicData::calvar()
{ 
	int jointnum = 16;
	double sum[3] = { 0,0,0 };

	for (int i = 0; i < jointnum; i++)
	{
		for (int j = 0; j < this->maxframe; j++)
		{
			sum[0] += this->posData[j].Jointspos[i][0];
			sum[1] += this->posData[j].Jointspos[i][1];
			sum[2] += this->posData[j].Jointspos[i][2];
			/*jpos.posData[tCount].Jointspos[0][0]*/
		}
		this->avg[i][0] = sum[0] / this->maxframe;
		this->avg[i][1] = sum[1] / this->maxframe;
		this->avg[i][2] = sum[2] / this->maxframe;

		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
	}


	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
	
	for (int i = 0; i < jointnum; i++)
	{
		for (int j = 0; j < this->maxframe; j++)
		{
			sum[0] += pow((this->posData[j].Jointspos[i][0] - this->avg[i][0]),2);
			sum[1] += pow((this->posData[j].Jointspos[i][1] - this->avg[i][1]),2);
			sum[2] += pow((this->posData[j].Jointspos[i][2] - this->avg[i][2]),2);
			
		}
		this->variance[i][0] = sum[0] / this->maxframe;
		this->variance[i][1] = sum[1] / this->maxframe;
		this->variance[i][2] = sum[2] / this->maxframe;
		//cout << "this maxframe " << sum[0] << endl;
		//cout << "this maxframe "<< this->maxframe << endl;
		//cout << this->variance[i][0]<<" ," << this->variance[i][1] << " ," << this->variance[i][2] <<endl;

		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
	}
}

void DynamicData::Searchmove()
{
	int jointnum = 16;
	double sum=0;

	for (int i = 0; i < jointnum; i++)
	{
		for (int j = 0; j < 3; j++)
			sum += this->variance[i][j];

		if (sum > 60)
		{
			this->movingjoint.push_back(i);
		}
		
		sum = 0;
	}

	/*if (movingjoint.back() < 7)
	{
		this->motionclass = 0;

	}
	else {

		this->motionclass = 1;
	}*/

	//cout << "confirm" << endl;
}

void DynamicData::ExtractData()
{
	for (auto jointidx = this->movingjoint.begin(); jointidx != this->movingjoint.end(); ++jointidx)
	{
		int k = 0;
		
		vector<vector<double>>  inputDatajointbuf;
		
		for (int frame=0; frame <this->keyframecnt; ++frame)
		{
			vector<double>  inputDataframebuf;
			
			for (int coord = 0; coord < 3; coord++)
			{
				inputDataframebuf.push_back(this->posData[keyframenum[k][frame]].Jointspos[*jointidx][coord]);
			}
			inputDatajointbuf.push_back(inputDataframebuf);
		}

		this->compareData.push_back(inputDatajointbuf);

		k++;
	}
}


void DynamicData::ExtractKeyframe(int keyposenum)
{
	
	int divide = (keyposenum + 1) / 2;
	
	for (auto jointidx = this->movingjoint.begin(); jointidx != this->movingjoint.end(); ++jointidx)
	{
		int maxdistframe = 0;
		vector<int> keyframe;
		double buf = CalculatePointDistance(this->posData[0].Jointspos[*jointidx], this->posData[1].Jointspos[*jointidx]);

		for (int i = 2; i < this->maxframe; i++)
		{
			double cal= CalculatePointDistance(this->posData[0].Jointspos[*jointidx], this->posData[i].Jointspos[*jointidx]);
			if (buf <cal)
			{
				//cout << "!!!!!!";
				maxdistframe = i;

				buf = cal;
			}
		}
		
		cout <<"max dist idx : "<< maxdistframe <<endl;
		for (int posnum=0;posnum<keyposenum/2;posnum++)
		{

			keyframe.push_back((maxdistframe / divide + 0.5)*(posnum + 1));

		}
		keyframe.push_back(maxdistframe);

		for (int posnum = keyposenum/2+1; posnum < keyposenum ; posnum++)
		{

			keyframe.push_back((maxdistframe / divide + 0.5)*(posnum + 1));

		}

		this->keyframenum.push_back(keyframe);
	}
}

void JointPosition::calavg()
{
	
	int jointnum = 16;
	double sum[3] = {0,0,0};
	
	for (int i = 0; i < jointnum; i++)
	{
		for (int j = 0; j < this->maxframe; j++)
		{
			sum[0] += this->posData[j].Jointspos[i][0];
			sum[1] += this->posData[j].Jointspos[i][1];
			sum[2] += this->posData[j].Jointspos[i][2];
				/*jpos.posData[tCount].Jointspos[0][0]*/
		}
		this->avg[i][0]= sum[0] / this->maxframe;
		this->avg[i][1]= sum[1] / this->maxframe;
		this->avg[i][2]= sum[2] / this->maxframe;


		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
	}
}

void JointPosition::calloadavg(int maxframe, double data[][3])
{

	int jointnum = 16;
	double sum[3] = { 0,0,0 };
	double scale_buf[3];
	for (int i = 0; i < jointnum; i++)
	{
		for (int j = 0; j < maxframe; j++)
		{
			sum[0] += this->loadData[j].Jointspos[i][0];
			sum[1] += this->loadData[j].Jointspos[i][1];
			sum[2] += this->loadData[j].Jointspos[i][2];
			/*jpos.posData[tCount].Jointspos[0][0]*/
		}

		if (i == 0)
		{
			scale_buf[0] = sum[0] / maxframe;
			scale_buf[1] = sum[1] / maxframe;
			scale_buf[2] = sum[2] / maxframe;
		}
		data[i][0] = sum[0] / maxframe- scale_buf[0] +10;
		data[i][1] = sum[1] / maxframe- scale_buf[1]+10;
		data[i][2] = sum[2] / maxframe- scale_buf[2]+10;

		//cout <<"data : "<< data[i][0] << endl;

		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
	}
}

void JointPosition::calvar()
{

	int jointnum = 16;
	double sum[3] = { 0,0,0 };

	for (int i = 0; i < jointnum; i++)
	{
		for (int j = 0; j < this->maxframe; j++)
		{
			sum[0] += (this->posData[j].Jointspos[i][0] - this->avg[i][0])*(this->posData[j].Jointspos[i][0] - this->avg[i][0]);
			sum[1] += (this->posData[j].Jointspos[i][1] - this->avg[i][1])*(this->posData[j].Jointspos[i][1] - this->avg[i][1]);
			sum[2] += (this->posData[j].Jointspos[i][2] - this->avg[i][2])*(this->posData[j].Jointspos[i][2] - this->avg[i][2]);
			
		}
		this->variance[i][0] = sum[0] / this->maxframe;
		this->variance[i][1] = sum[1] / this->maxframe;
		this->variance[i][2] = sum[2] / this->maxframe;


		sum[0] = 0;
		sum[1] = 0;
		sum[2] = 0;
	}
}


void JointPosition::loaddataT()
{
	int count = 0;
	int tCount = 0;

	std::ifstream _filestream(".\\GestureData\\reference\\T-Pose.txt");
	std::string _line;
	int _option;
	std::string _dummy;
	int lineCount = 0;

	while (std::getline(_filestream, _line))
	{
		
		std::stringstream _linestream;
		_linestream << _line;
	
		if (count == 0)
		{
			
			_linestream >> _line >> this->tposeframe; count++;
			//cout <<" tpose frame " <<this->tposeframe << endl;
			continue;

			
		}

		_linestream
			>> this->loadData[lineCount].Jointspos[0][0] >> this->loadData[lineCount].Jointspos[0][1] >> this->loadData[lineCount].Jointspos[0][2]
			>> this->loadData[lineCount].Jointspos[1][0] >> this->loadData[lineCount].Jointspos[1][1] >> this->loadData[lineCount].Jointspos[1][2]
			>> this->loadData[lineCount].Jointspos[2][0] >> this->loadData[lineCount].Jointspos[2][1] >> this->loadData[lineCount].Jointspos[2][2]
			>> this->loadData[lineCount].Jointspos[3][0] >> this->loadData[lineCount].Jointspos[3][1] >> this->loadData[lineCount].Jointspos[3][2]
			>> this->loadData[lineCount].Jointspos[4][0] >> this->loadData[lineCount].Jointspos[4][1] >> this->loadData[lineCount].Jointspos[4][2]
			>> this->loadData[lineCount].Jointspos[5][0] >> this->loadData[lineCount].Jointspos[5][1] >> this->loadData[lineCount].Jointspos[5][2]
			>> this->loadData[lineCount].Jointspos[6][0] >> this->loadData[lineCount].Jointspos[6][1] >> this->loadData[lineCount].Jointspos[6][2]
			>> this->loadData[lineCount].Jointspos[7][0] >> this->loadData[lineCount].Jointspos[7][1] >> this->loadData[lineCount].Jointspos[7][2]
			>> this->loadData[lineCount].Jointspos[8][0] >> this->loadData[lineCount].Jointspos[8][1] >> this->loadData[lineCount].Jointspos[8][2]
			>> this->loadData[lineCount].Jointspos[9][0] >> this->loadData[lineCount].Jointspos[9][1] >> this->loadData[lineCount].Jointspos[9][2]
			>> this->loadData[lineCount].Jointspos[10][0] >> this->loadData[lineCount].Jointspos[10][1] >> this->loadData[lineCount].Jointspos[10][2]
			>> this->loadData[lineCount].Jointspos[11][0] >> this->loadData[lineCount].Jointspos[11][1] >> this->loadData[lineCount].Jointspos[11][2]
			>> this->loadData[lineCount].Jointspos[12][0] >> this->loadData[lineCount].Jointspos[12][1] >> this->loadData[lineCount].Jointspos[12][2]
			>> this->loadData[lineCount].Jointspos[13][0] >> this->loadData[lineCount].Jointspos[13][1] >> this->loadData[lineCount].Jointspos[13][2]
			>> this->loadData[lineCount].Jointspos[14][0] >> this->loadData[lineCount].Jointspos[14][1] >> this->loadData[lineCount].Jointspos[14][2]
			>> this->loadData[lineCount].Jointspos[15][0] >> this->loadData[lineCount].Jointspos[15][1] >> this->loadData[lineCount].Jointspos[15][2];
		lineCount++;
	}
	//cout <<"load data" <<this->loadData[tCount].Jointspos[0][0] << endl;
	calloadavg(this->tposeframe,this->Tposdata);
	//cout <<"cal data "<< this->Tposdata[0][0] << endl;
	memset(this->loadData, NULL, sizeof(this->loadData));
}

void JointPosition::loaddataB()
{
	int count = 0;
	int tCount = 0;

	std::ifstream _filestream(".\\GestureData\\reference\\Both-hand.txt");
	std::string _line;
	int _option;
	std::string _dummy;
	int lineCount = 0;

	while (std::getline(_filestream, _line))
	{
		std::stringstream _linestream;
		_linestream << _line;

		if (count == 0)
		{
			_linestream >> _line >> this->bothposeframe; count++; continue;
		}


		_linestream
			>> this->loadData[lineCount].Jointspos[0][0] >> this->loadData[lineCount].Jointspos[0][1] >> this->loadData[lineCount].Jointspos[0][2]
			>> this->loadData[lineCount].Jointspos[1][0] >> this->loadData[lineCount].Jointspos[1][1] >> this->loadData[lineCount].Jointspos[1][2]
			>> this->loadData[lineCount].Jointspos[2][0] >> this->loadData[lineCount].Jointspos[2][1] >> this->loadData[lineCount].Jointspos[2][2]
			>> this->loadData[lineCount].Jointspos[3][0] >> this->loadData[lineCount].Jointspos[3][1] >> this->loadData[lineCount].Jointspos[3][2]
			>> this->loadData[lineCount].Jointspos[4][0] >> this->loadData[lineCount].Jointspos[4][1] >> this->loadData[lineCount].Jointspos[4][2]
			>> this->loadData[lineCount].Jointspos[5][0] >> this->loadData[lineCount].Jointspos[5][1] >> this->loadData[lineCount].Jointspos[5][2]
			>> this->loadData[lineCount].Jointspos[6][0] >> this->loadData[lineCount].Jointspos[6][1] >> this->loadData[lineCount].Jointspos[6][2]
			>> this->loadData[lineCount].Jointspos[7][0] >> this->loadData[lineCount].Jointspos[7][1] >> this->loadData[lineCount].Jointspos[7][2]
			>> this->loadData[lineCount].Jointspos[8][0] >> this->loadData[lineCount].Jointspos[8][1] >> this->loadData[lineCount].Jointspos[8][2]
			>> this->loadData[lineCount].Jointspos[9][0] >> this->loadData[lineCount].Jointspos[9][1] >> this->loadData[lineCount].Jointspos[9][2]
			>> this->loadData[lineCount].Jointspos[10][0] >> this->loadData[lineCount].Jointspos[10][1] >> this->loadData[lineCount].Jointspos[10][2]
			>> this->loadData[lineCount].Jointspos[11][0] >> this->loadData[lineCount].Jointspos[11][1] >> this->loadData[lineCount].Jointspos[11][2]
			>> this->loadData[lineCount].Jointspos[12][0] >> this->loadData[lineCount].Jointspos[12][1] >> this->loadData[lineCount].Jointspos[12][2]
			>> this->loadData[lineCount].Jointspos[13][0] >> this->loadData[lineCount].Jointspos[13][1] >> this->loadData[lineCount].Jointspos[13][2]
			>> this->loadData[lineCount].Jointspos[14][0] >> this->loadData[lineCount].Jointspos[14][1] >> this->loadData[lineCount].Jointspos[14][2]
			>> this->loadData[lineCount].Jointspos[15][0] >> this->loadData[lineCount].Jointspos[15][1] >> this->loadData[lineCount].Jointspos[15][2];
		lineCount++;
	}

	calloadavg(this->bothposeframe, this->Bothhandup);
	memset(this->loadData, NULL, sizeof(this->loadData));
}

void JointPosition::loadtestdata()
{

	int count = 0;
	int tCount = 0;

	std::ifstream _filestream(".\\GestureData\\reference\\test.txt");
	std::string _line;
	int _option;
	std::string _dummy;
	int lineCount = 0;

	while (std::getline(_filestream, _line))
	{
		std::stringstream _linestream;
		_linestream << _line;

		if (count == 0)
		{
			_linestream >> _line >> this->testposeframe; count++; continue;
		}
			_linestream
			>> this->loadData[lineCount].Jointspos[0][0] >> this->loadData[lineCount].Jointspos[0][1] >> this->loadData[lineCount].Jointspos[0][2]
			>> this->loadData[lineCount].Jointspos[1][0] >> this->loadData[lineCount].Jointspos[1][1] >> this->loadData[lineCount].Jointspos[1][2]
			>> this->loadData[lineCount].Jointspos[2][0] >> this->loadData[lineCount].Jointspos[2][1] >> this->loadData[lineCount].Jointspos[2][2]
			>> this->loadData[lineCount].Jointspos[3][0] >> this->loadData[lineCount].Jointspos[3][1] >> this->loadData[lineCount].Jointspos[3][2]
			>> this->loadData[lineCount].Jointspos[4][0] >> this->loadData[lineCount].Jointspos[4][1] >> this->loadData[lineCount].Jointspos[4][2]
			>> this->loadData[lineCount].Jointspos[5][0] >> this->loadData[lineCount].Jointspos[5][1] >> this->loadData[lineCount].Jointspos[5][2]
			>> this->loadData[lineCount].Jointspos[6][0] >> this->loadData[lineCount].Jointspos[6][1] >> this->loadData[lineCount].Jointspos[6][2]
			>> this->loadData[lineCount].Jointspos[7][0] >> this->loadData[lineCount].Jointspos[7][1] >> this->loadData[lineCount].Jointspos[7][2]
			>> this->loadData[lineCount].Jointspos[8][0] >> this->loadData[lineCount].Jointspos[8][1] >> this->loadData[lineCount].Jointspos[8][2]
			>> this->loadData[lineCount].Jointspos[9][0] >> this->loadData[lineCount].Jointspos[9][1] >> this->loadData[lineCount].Jointspos[9][2]
			>> this->loadData[lineCount].Jointspos[10][0] >> this->loadData[lineCount].Jointspos[10][1] >> this->loadData[lineCount].Jointspos[10][2]
			>> this->loadData[lineCount].Jointspos[11][0] >> this->loadData[lineCount].Jointspos[11][1] >> this->loadData[lineCount].Jointspos[11][2]
			>> this->loadData[lineCount].Jointspos[12][0] >> this->loadData[lineCount].Jointspos[12][1] >> this->loadData[lineCount].Jointspos[12][2]
			>> this->loadData[lineCount].Jointspos[13][0] >> this->loadData[lineCount].Jointspos[13][1] >> this->loadData[lineCount].Jointspos[13][2]
			>> this->loadData[lineCount].Jointspos[14][0] >> this->loadData[lineCount].Jointspos[14][1] >> this->loadData[lineCount].Jointspos[14][2]
			>> this->loadData[lineCount].Jointspos[15][0] >> this->loadData[lineCount].Jointspos[15][1] >> this->loadData[lineCount].Jointspos[15][2];
		lineCount++;
	}

	calloadavg(this->testposeframe, this->Test);
	memset(this->loadData, NULL, sizeof(this->loadData));
}


void JointPosition::loadreferencedata()
{

	//int count = 0;
	int tCount = 0;
	string fileName;
	
	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;

	for (int i = 0; i < this->loadingfilenum; i++) {
		int lineCount = 0;
		int count = 0;
		
		fileName = ".\\GestureData\\reference\\loadfile" + to_string(this->loadingfilecount+1) + ".txt";
		_filestream.open(fileName);
		cout << "check" << fileName << endl;

		while (std::getline(_filestream, _line))
		{
			std::stringstream _linestream;
			_linestream << _line;
			//cout << "gesture working" << endl;
			if (count == 0)
			{
				_linestream >> this->posename[this->loadingfilecount]; count++;
				
				cout <<"check"<<_line <<endl;
				
				continue;

				
			}
			if (count == 1)
			{
				_linestream >> _line >> this->referFramenum[this->loadingfilecount]; count++; 
				
				cout << "check" << this->referFramenum[this->loadingfilecount] << endl;
				
				continue;
			}

			_linestream
				>> this->loadData[lineCount].Jointspos[0][0] >> this->loadData[lineCount].Jointspos[0][1] >> this->loadData[lineCount].Jointspos[0][2]
				>> this->loadData[lineCount].Jointspos[1][0] >> this->loadData[lineCount].Jointspos[1][1] >> this->loadData[lineCount].Jointspos[1][2]
				>> this->loadData[lineCount].Jointspos[2][0] >> this->loadData[lineCount].Jointspos[2][1] >> this->loadData[lineCount].Jointspos[2][2]
				>> this->loadData[lineCount].Jointspos[3][0] >> this->loadData[lineCount].Jointspos[3][1] >> this->loadData[lineCount].Jointspos[3][2]
				>> this->loadData[lineCount].Jointspos[4][0] >> this->loadData[lineCount].Jointspos[4][1] >> this->loadData[lineCount].Jointspos[4][2]
				>> this->loadData[lineCount].Jointspos[5][0] >> this->loadData[lineCount].Jointspos[5][1] >> this->loadData[lineCount].Jointspos[5][2]
				>> this->loadData[lineCount].Jointspos[6][0] >> this->loadData[lineCount].Jointspos[6][1] >> this->loadData[lineCount].Jointspos[6][2]
				>> this->loadData[lineCount].Jointspos[7][0] >> this->loadData[lineCount].Jointspos[7][1] >> this->loadData[lineCount].Jointspos[7][2]
				>> this->loadData[lineCount].Jointspos[8][0] >> this->loadData[lineCount].Jointspos[8][1] >> this->loadData[lineCount].Jointspos[8][2]
				>> this->loadData[lineCount].Jointspos[9][0] >> this->loadData[lineCount].Jointspos[9][1] >> this->loadData[lineCount].Jointspos[9][2]
				>> this->loadData[lineCount].Jointspos[10][0] >> this->loadData[lineCount].Jointspos[10][1] >> this->loadData[lineCount].Jointspos[10][2]
				>> this->loadData[lineCount].Jointspos[11][0] >> this->loadData[lineCount].Jointspos[11][1] >> this->loadData[lineCount].Jointspos[11][2]
				>> this->loadData[lineCount].Jointspos[12][0] >> this->loadData[lineCount].Jointspos[12][1] >> this->loadData[lineCount].Jointspos[12][2]
				>> this->loadData[lineCount].Jointspos[13][0] >> this->loadData[lineCount].Jointspos[13][1] >> this->loadData[lineCount].Jointspos[13][2]
				>> this->loadData[lineCount].Jointspos[14][0] >> this->loadData[lineCount].Jointspos[14][1] >> this->loadData[lineCount].Jointspos[14][2]
				>> this->loadData[lineCount].Jointspos[15][0] >> this->loadData[lineCount].Jointspos[15][1] >> this->loadData[lineCount].Jointspos[15][2];
			lineCount++;
		}
		_filestream.close();


		calloadavg(this->referFramenum[i], this->referencepose[i]);
		memset(this->loadData, NULL, sizeof(this->loadData));

		this->loadingfilecount++;
	}
}

void JointPosition::loadDydata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());

	//int count = 0;
	int tCount = 0;
	string fileName;

	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;

	for (int i = 0; i < this->Dycompdataloadnum; i++) {
		int lineCount = 0;
		int count = 0;

		this->Dynamicdatacontrol.push_back(new DynamicData());
		fileName = ".\\GestureData\\reference\\loadDYfile" + to_string(this->Dydataloadnum + 1) + ".txt";
		_filestream.open(fileName);
		cout << "check" << fileName << endl;

		while (std::getline(_filestream, _line))
		{
			std::stringstream _linestream;
			_linestream << _line;
			//cout << "gesture working" << endl;
			if (count == 0)
			{
				_linestream >> this->Dynamicdatacontrol.back()->posename; count++;
				//cout << "check" << _line << endl;
				continue;
			}
			if (count == 1)
			{
				_linestream >> _line >> this->Dynamicdatacontrol.back()->maxframe; count++;

				//cout << "check" << this->referFramenum[this->loadingfilecount] << endl;

				continue;
			}

			_linestream
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[0][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[0][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[0][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[1][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[1][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[1][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[2][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[2][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[2][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[3][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[3][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[3][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[4][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[4][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[4][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[5][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[5][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[5][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[6][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[6][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[6][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[7][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[7][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[7][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[8][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[8][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[8][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[9][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[9][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[9][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[10][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[10][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[10][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[11][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[11][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[11][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[12][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[12][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[12][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[13][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[13][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[13][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[14][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[14][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[14][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[15][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[15][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[15][2];
			lineCount++;
		}
		_filestream.close();

		//cout << "readfinish" << endl;
		//this->Dynamicdatacontrol.back()->calDist();
		this->Dynamicdatacontrol.back()->removePelvis();
		//cout << "readfinish" << endl;
		this->Dynamicdatacontrol.back()->calvar();
		//cout << "readfinish" << endl;
		this->Dynamicdatacontrol.back()->Searchmove();
		//cout << "readfinish" << endl;
		this->Dynamicdatacontrol.back()->ExtractKeyframe(this->Dynamicdatacontrol.back()->keyframecnt);
		this->Dynamicdatacontrol.back()->ExtractData();
		//calloadavg(this->referFramenum[i], this->referencepose[i]);
		//memset(this->loadData, NULL, sizeof(this->loadData));

		this->Dydataloadnum++;
	}
}

void JointPosition::loadDyreferdata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());
//int count = 0;
	int tCount = 0;
	string fileName;

	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;

	for (int i = 0; i < this->Dycompdataloadnum; i++) {
		int lineCount = 0;
		int count = 0;

		this->Dynamicdatacontrol.push_back(new DynamicData());
		fileName = ".\\GestureData\\reference\\loadDYfile" + to_string(this->Dydataloadnum + 1) + ".txt";
		_filestream.open(fileName);
		cout << "check" << fileName << endl;

		while (std::getline(_filestream, _line))
		{
			std::stringstream _linestream;
			_linestream << _line;
			//cout << "gesture working" << endl;
			if (count == 0)
			{
				_linestream >> this->Dynamicdatacontrol.back()->posename; count++;

				//cout << "check" << _line << endl;

				continue;


			}
			if (count == 1)
			{
				_linestream >> _line >> this->Dynamicdatacontrol.back()->maxframe; count++;

				//cout << "check" << this->referFramenum[this->loadingfilecount] << endl;

				continue;
			}

			_linestream
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[0][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[0][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[0][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[1][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[1][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[1][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[2][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[2][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[2][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[3][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[3][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[3][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[4][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[4][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[4][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[5][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[5][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[5][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[6][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[6][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[6][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[7][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[7][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[7][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[8][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[8][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[8][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[9][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[9][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[9][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[10][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[10][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[10][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[11][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[11][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[11][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[12][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[12][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[12][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[13][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[13][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[13][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[14][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[14][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[14][2]
				>> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[15][0] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[15][1] >> this->Dynamicdatacontrol.back()->posData[lineCount].Jointspos[15][2];
			lineCount++;
		}
		_filestream.close();

		//cout << "readfinish" << endl;
		//this->Dynamicdatacontrol.back()->calDist();
		this->Dynamicdatacontrol.back()->removePelvis();
		//cout << "readfinish" << endl;
		this->Dynamicdatacontrol.back()->calvar();
		//cout << "readfinish" << endl;
		this->Dynamicdatacontrol.back()->Searchmove();
		//cout << "readfinish" << endl;
		this->Dynamicdatacontrol.back()->ExtractKeyframe(this->Dynamicdatacontrol.back()->keyframecnt);
		this->Dynamicdatacontrol.back()->ExtractData();
		//calloadavg(this->referFramenum[i], this->referencepose[i]);
		//memset(this->loadData, NULL, sizeof(this->loadData));

		this->Dydataloadnum++;
	}
		

}
//
//if (count == 0)
//{
//	_linestream >> _line >> this->GenDatacontrol.back()->maxframe; count++;
//
//	cout << "check" << this->GenDatacontrol.back()->maxframe << endl;
//
//	continue;
//}
//
//_linestream
//>> buf >> buf >> buf >> buf >> buf >> buf           //1
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][3]		   //2   pelvis
//>> buf >> buf >> buf >> buf >> buf >> buf		   //3
//>> buf >> buf >> buf >> buf >> buf >> buf		   //4
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][3]		   //5   spine
//>> buf >> buf >> buf >> buf >> buf >> buf		   //6
//>> buf >> buf >> buf >> buf >> buf >> buf		   //7
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][3]		   //8   LUA
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][3]		   //9   LLA
//>> buf >> buf >> buf >> buf >> buf >> buf          //10
//>> buf >> buf >> buf >> buf >> buf >> buf          //11
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][3]		   //2   RUA
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][3]	   //3   RLA
//>> buf >> buf >> buf >> buf >> buf >> buf		   //4
//>> buf >> buf >> buf >> buf >> buf >> buf		   //5
//>> buf >> buf >> buf >> buf >> buf >> buf		   //6
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][3]		   //7   Lhip
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][3]		   //8   Lknee
//>> buf >> buf >> buf >> buf >> buf >> buf		   //9
//>> buf >> buf >> buf >> buf >> buf >> buf          //10
//>> buf >> buf >> buf >> buf >> buf >> buf          //21
//>> buf >> buf >> buf >> buf >> buf >> buf		   //2
//>> buf >> buf >> buf >> buf >> buf >> buf		   //3
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][3]		   //4   Rhip
//>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][3]		   //5   Rknee
//>> buf >> buf >> buf >> buf >> buf >> buf		   //6
//>> buf >> buf >> buf >> buf >> buf >> buf		   //7
//>> buf >> buf >> buf >> buf >> buf >> buf		   //8
//>> buf >> buf >> buf >> buf >> buf >> buf		   //9
//>> buf >> buf >> buf >> buf >> buf >> buf;     //30


void JointPosition::loadbvhdata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());

	//int count = 0;
	int tCount = 0;
	string fileName;

	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;


	//string fileName[] = { ".\\GenData\\Refer\\R_Hook1.txt" ,".\\GenData\\Refer\\R_Hook1.txt",".\\GenData\\Refer\\0.5 straight 2.txt",".\\GenData\\Refer\\SLERP_data.txt" ,".\\GenData\\Refer\\Hook_std1.txt" };

	//// back up
	////string fileName[]= { ".\\GenData\\Refer\\R_Straight1.txt",".\\GenData\\Refer\\R_Hook1.txt",".\\GenData\\Refer\\0.5 straight 2.txt",".\\GenData\\Refer\\SLERP_data.txt" ,".\\GenData\\Refer\\SEED_data.txt" };
	////string fileName[] = { ".\\GenData\\Refer\\Trj Range\\0.5 straight 2.txt" ,".\\GenData\\Refer\\R_Hook1.txt",".\\GenData\\Refer\\R_Hook3.txt",".\\GenData\\Refer\\SLERP_data.txt" ,".\\GenData\\Refer\\SEED_data.txt" };
	//std::ifstream _filestream;
	//std::string _line;
	//int _option;
	//std::string _dummy;
	////fileName = 
	//for (int i = 0; i < this->Genrefernum; i++) {
	//	int lineCount = 0;
	//	int count = 0;

	//	this->GenDatacontrol.push_back(new GenData());

	//	_filestream.open(fileName[i]);
	//	cout << "check" << fileName[i] << endl;

	//	while (std::getline(_filestream, _line))
	//	{
	//		std::stringstream _linestream;
	//		_linestream << _line;
	//		//cout << "gesture working" << endl;
	//		if (count == 0)
	//		{
	//			//cout << "check" << endl;
	//			_linestream >> this->GenDatacontrol.back()->poseName;

	//			//cout << "check" << _line << endl;
	//			count++;
	//			continue;


	//		}
	//		if (count == 1)
	//		{
	//			_linestream >> _line >> this->GenDatacontrol.back()->maxframe; count++;

	//			cout << "check" << this->GenDatacontrol.back()->maxframe << endl;

	//			continue;
	//		}


	
	int lineCount = 0;
	int count = 0;

	//".\\GenData\\Refer\\R_Hook1.txt"
	fileName = ".\\GenData\\Refer\\Hookmotion.bvh";
	_filestream.open(fileName);
	//cout << "check" << fileName << endl;

	this->GenDatacontrol.push_back(new GenData());

	
	cout << "check" << fileName << endl;
	double buf = 0;
	while (std::getline(_filestream, _line))
	{
		std::stringstream _linestream;
		_linestream << _line;
		//cout << "gesture working" << endl;

		if (count == 0)
		{
			_linestream >> _line >> this->GenDatacontrol.back()->maxframe; count++;

			cout << "check !!!!" << this->GenDatacontrol.back()->maxframe << endl;

			continue;
		}

		_linestream
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][2]           //1
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][2]		   //2   pelvis
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][2]		   //3																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][2]		   //4																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][2]		   //5   spine
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][2]		   //6																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][2]	   //7																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][2]		   //8   LUA
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][2]		   //9   LLA
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][2]         //10																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][10][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][10][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][10][2]        //11																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][11][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][11][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][11][2]		   //2   RUA
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][12][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][12][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][12][2]	   //3   RLA
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][13][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][13][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][13][2]	   //4																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][14][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][14][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][14][2]		   //5																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][15][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][15][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][15][2]		   //6																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][16][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][16][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][16][2]		   //7   Lhip
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][17][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][17][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][17][2]		   //8   Lknee
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][18][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][18][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][18][2]	   //9																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][19][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][19][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][19][2]         //10																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][20][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][20][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][20][2]        //21																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][21][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][21][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][21][2]	   //2																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][22][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][22][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][22][2]	   //3																																				  
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][23][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][23][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][23][2]		   //4   Rhip
			>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][24][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][24][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][24][2]		   //5   Rknee
			>> buf >> buf >> buf >> buf >> buf >> buf		   //6
			>> buf >> buf >> buf >> buf >> buf >> buf		   //7
			>> buf >> buf >> buf >> buf >> buf >> buf		   //8
			>> buf >> buf >> buf >> buf >> buf >> buf		   //9
			>> buf >> buf >> buf >> buf >> buf >> buf;     //30uatData[lineCount].JointsQuat[9].mData[2];
		lineCount++;
	}
	_filestream.close();

	//_linestream
	//	>> buf >> buf >> buf >> buf >> buf >> buf           //1
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][0][2]		   //2   pelvis
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //3																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //4																																				  
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][1][2]		   //5   spine
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //6																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //7																																				  
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][2][2]		   //8   LUA
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][3][2]		   //9   LLA
	//	>> buf >> buf >> buf >> buf >> buf >> buf          //10																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf          //11																																				  
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][4][2]		   //2   RUA
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][5][2]	   //3   RLA
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //4																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //5																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //6																																				  
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][6][2]		   //7   Lhip
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][7][2]		   //8   Lknee
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //9																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf          //10																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf          //21																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //2																																				  
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //3																																				  
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][8][2]		   //4   Rhip
	//	>> buf >> buf >> buf >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][0] >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][1] >> this->GenDatacontrol.back()->Eulerangle[lineCount][9][2]		   //5   Rknee
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //6
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //7
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //8
	//	>> buf >> buf >> buf >> buf >> buf >> buf		   //9
	//	>> buf >> buf >> buf >> buf >> buf >> buf;     //30uatData[lineCount].JointsQuat[9].mData[2];


	//this->Genrefercnt++;

	

}


void JointPosition::loadMSdata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());

	//int count = 0;
	int tCount = 0;
	string fileName;

	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;

	for (int i = 0; i < this->MSdatanum; i++) {
		int lineCount = 0;
		int count = 0;

		this->MSDatacontrol.push_back(new DynamicData());
		fileName = ".\\GestureData\\reference\\MS\\loadMSfile" + to_string(this->Msdatacnt + 1) + ".txt";
		_filestream.open(fileName);
		//cout << "check" << fileName << endl;

		while (std::getline(_filestream, _line))
		{
			std::stringstream _linestream;
			_linestream << _line;
			//cout << "gesture working" << endl;
			if (count == 0)
			{
				_linestream >> this->MSDatacontrol.back()->posename; count++;

				//cout << "check" << _line << endl;

				continue;


			}
			if (count == 1)
			{
				_linestream >> _line >> this->MSDatacontrol.back()->maxframe; count++;

				//cout << "check" << this->referFramenum[this->loadingfilecount] << endl;

				continue;
			}

			_linestream
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[0][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[0][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[0][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[1][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[1][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[1][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[2][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[2][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[2][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[3][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[3][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[3][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[4][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[4][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[4][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[5][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[5][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[5][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[6][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[6][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[6][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[7][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[7][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[7][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[8][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[8][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[8][2]
				>> this->MSDatacontrol.back()->posData[lineCount].Jointspos[9][0] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[9][1] >> this->MSDatacontrol.back()->posData[lineCount].Jointspos[9][2];
				lineCount++;
		}
		_filestream.close();
		this->MSDatacontrol.back()->calDist();
		//this->MSDatacontrol.back()->removePelvis();
		this->MSDatacontrol.back()->calvar();
		this->MSDatacontrol.back()->Searchmove();
		//this->MSDatacontrol.back()->ExtractKeyframe(this->MSDatacontrol.back()->keyframecnt);
		//this->MSDatacontrol.back()->ExtractData();
		//calloadavg(this->referFramenum[i], this->referencepose[i]);
		//memset(this->loadData, NULL, sizeof(this->loadData));

		this->Msdatacnt++;
	}

}




void JointPosition::loadMSreferdata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());

	//int count = 0;
	int tCount = 0;
	string fileName;

	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;

	for (int i = 0; i < this->MSreferdatanum; i++) {
		int lineCount = 0;
		int count = 0;

		this->MSreferDatacontrol.push_back(new DynamicData());
		fileName = ".\\GestureData\\reference\\MS\\loadreferMSfile" + to_string(this->MSreferdatacnt + 1) + ".txt";
		_filestream.open(fileName);
		//cout << "check" << fileName << endl;

		while (std::getline(_filestream, _line))
		{
			std::stringstream _linestream;
			_linestream << _line;
			//cout << "gesture working" << endl;
			if (count == 0)
			{
				_linestream >> this->MSreferDatacontrol.back()->posename; count++;

				//cout << "check" << _line << endl;

				continue;


			}
			if (count == 1)
			{
				_linestream >> _line >> this->MSreferDatacontrol.back()->maxframe; count++;

				//cout << "check" << this->referFramenum[this->loadingfilecount] << endl;

				continue;
			}

			_linestream
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[0][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[0][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[0][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[1][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[1][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[1][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[2][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[2][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[2][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[3][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[3][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[3][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[4][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[4][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[4][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[5][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[5][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[5][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[6][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[6][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[6][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[7][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[7][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[7][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[8][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[8][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[8][2]
				>> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[9][0] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[9][1] >> this->MSreferDatacontrol.back()->posData[lineCount].Jointspos[9][2];
			lineCount++;
		}
		_filestream.close();
		this->MSreferDatacontrol.back()->calDist();
		//this->MSreferDatacontrol.back()->removePelvis();
		this->MSreferDatacontrol.back()->calvar();
		this->MSreferDatacontrol.back()->Searchmove();
		//this->MSreferDatacontrol.back()->ExtractKeyframe(this->MSreferDatacontrol.back()->keyframecnt);
		//this->MSreferDatacontrol.back()->ExtractData();
		//calloadavg(this->referFramenum[i], this->referencepose[i]);
		//memset(this->loadData, NULL, sizeof(this->loadData));

		this->MSreferdatacnt++;
	}

}

namespace  gst {
	void cal_end_Pos(const GenData* rawdata, double in[][3]);
	void matread(const char *file, std::vector<double>& v)
	{
		//// open MAT-file
		//MATFile *pmat = matOpen(file, "r");
		//if (pmat == NULL) return;

		//// extract the specified variable
		//mxArray *arr = matGetVariable(pmat, "d_skel");
		//if (arr != NULL && mxIsDouble(arr) && !mxIsEmpty(arr)) {
		//	// copy data
		//	mwSize num = mxGetNumberOfElements(arr);
		//	double *pr = mxGetPr(arr);
		//	if (pr != NULL) {
		//		v.reserve(num); //is faster than resize :-)
		//		v.assign(pr, pr + num);
		//	}
		//}

		//// cleanup
		//mxDestroyArray(arr);
		//matClose(pmat);
	}
	Bodypos vector_minus(const Bodypos lower, const Bodypos upper)
	{

		Bodypos result;

		result.x = upper.x - lower.x;
		result.y = upper.y - lower.y;
		result.z = upper.z - lower.z;

		double scale;
		scale = sqrt(pow(result.x, 2) + pow(result.y, 2) + pow(result.z, 2));
		result.x = result.x / scale;
		result.y = result.y / scale;
		result.z = result.z / scale;

		return result;
	}
	TVec3 vector_minus(const TVec3 upper, const TVec3 lower)
	{

		TVec3 result;

		result._x = upper._x - lower._x;
		result._y = upper._y - lower._y;
		result._z = upper._z - lower._z;

		double scale;
		scale = sqrt(pow(result._x, 2) + pow(result._y, 2) + pow(result._z, 2));
		result._x = result._x / scale;
		result._y = result._y / scale;
		result._z = result._z / scale;

		return result;
	}

	Bodypos vector_minus_t(const Bodypos Current, const Bodypos Next)
	{

		Bodypos result;

		result.x = Next.x - Current.x;
		result.y = Next.y - Current.y;
		result.z = Next.z - Current.z;


		return result;
	}
	Bodypos vector_cross(const Bodypos A, const Bodypos B)
	{

		Bodypos result;

		result.x = A.y*B.z - A.z*B.y;
		result.y = A.z*B.x - A.x*B.z;
		result.z = A.x*B.y - A.y*B.x;


		return result;
	}
	TVec3 vector_cross(const TVec3 A, const TVec3 B)
	{

		TVec3 result;

		result._x = A._x*B._z - A._z*B._y;
		result._y = A._z*B._x - A._x*B._z;
		result._z = A._x*B._y - A._y*B._x;


		return result;
	}
	double vector_dot(const TVec3 lower, const TVec3 upper)
	{

		double result;

		double dotpro, scale_l, scale_u;
		dotpro = (lower._x*upper._x) + (lower._y*upper._y) + (lower._z*upper._z);
		scale_l = sqrt(pow(lower._x, 2) + pow(lower._y, 2) + pow(lower._z, 2));
		scale_u = sqrt(pow(upper._x, 2) + pow(upper._y, 2) + pow(upper._z, 2));

		double x = dotpro / (scale_l*scale_u);

		result = acos(x);
		if (x > 1 || x < -1)
		{
			cout << "inf" << result << endl;
			result = 0;

		}
		else
		{

			result = acos(x);

		}

		return result;
	}
	double vector_Angle(const Bodypos lower, const Bodypos upper)
	{

		double result;

		double dotpro, scale_l, scale_u;
		dotpro = (lower.x*upper.x) + (lower.y*upper.y) + (lower.z*upper.z);
		scale_l = sqrt(pow(lower.x, 2) + pow(lower.y, 2) + pow(lower.z, 2));
		scale_u = sqrt(pow(upper.x, 2) + pow(upper.y, 2) + pow(upper.z, 2));

		double x = dotpro / (scale_l*scale_u);

		result = acos(x);
		if (x > 1 || x < -1)
		{
			cout << "inf" << result << endl;
			result = 0;

		}
		else
		{

			result = acos(x);

		}

		return result;
	}


	double vector_distance(const Bodypos lower, const Bodypos upper)
	{

		Bodypos result;

		result.x = upper.x - lower.x;
		result.y = upper.y - lower.y;
		result.z = upper.z - lower.z;

		double scale;
		scale = sqrt(pow(result.x, 2) + pow(result.y, 2) + pow(result.z, 2));

		return scale;
	}

	double vector_Sph_distance(const Bodypos lower, const Bodypos upper)
	{

		Bodypos result;

		result.x = upper.x * lower.x;
		result.y = upper.y * lower.y;
		result.z = upper.z * lower.z;

		double scale;
		scale = acos(result.x+ result.y+ result.z);

		return scale;
	}


	double vector_avg(const Bodypos lower, const Bodypos upper)
	{

		Bodypos result;

		result.x = upper.x - lower.x;
		result.y = upper.y - lower.y;
		result.z = upper.z - lower.z;

		double scale;
		scale = sqrt(pow(result.x, 2) + pow(result.y, 2) + pow(result.z, 2));

		return scale;
	}

	Bodypos vector_middle(const Bodypos lower, const Bodypos upper)
	{

		Bodypos result;

		result.x = (upper.x + lower.x) / 2;
		result.y = (upper.y + lower.y) / 2;
		result.z = (upper.z + lower.z) / 2;



		return result;
	}
	quaternion AxisagltoQuat(const int Axis_num, const double angle)
	{

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */

		quaternion result;
		TVec3 Axis[4];

		Axis[0] = { 0.18,0,0.00000005 }; // X -axis up and down
	
		Axis[1] = { 0,0.00000005,0.18 };  // Z -axis Right and left
		Axis[2] = { 0,0.00000005,-0.18 };
		Axis[3] = { -0.18,0,0.00000005 };
		result.mData[3] = cos(angle / 2);
		result.mData[0] = Axis[Axis_num]._x*sin(angle / 2);
		result.mData[1] = Axis[Axis_num]._y*sin(angle / 2);
		result.mData[2] = Axis[Axis_num]._z*sin(angle / 2);
			   		 
		return result;
	}
	quaternion AxisagltoQuat(const TVec3 Axis, const double angle)
	{

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */

		quaternion result;
		
		result.mData[3] = cos(angle );
		result.mData[0] = Axis._x*sin(angle );
		result.mData[1] = Axis._y*sin(angle );
		result.mData[2] = Axis._z*sin(angle );

		return result;
	}

	quaternion EulertoQuat(double angle[3])
	{

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */

		quaternion Q_x,Q_y,Q_z;
		quaternion result;
		//quaternion(double x, double y, double z, double w);
		Q_x = quaternion(sin(angle[1] * PI / 180 / 2), 0, 0, cos(angle[1]*PI/180/2));
		Q_y = quaternion(0, sin(angle[2] * PI / 180 / 2), 0, cos(angle[2] * PI / 180 / 2));
		Q_z = quaternion(0, 0, sin(-angle[0] * PI / 180 / 2), cos(angle[0] * PI / 180 / 2));

		result = Q_y.mutiplication(Q_x.mutiplication(Q_z));
		return result;
	}

	vector<double> vec_Theta(const TVec3 Axis)
	{

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */

		vector<double> result;
		result.push_back( acos(Axis._z)*180/PI); // Theta 
		result.push_back(atan(Axis._x / -Axis._y) * 180 / PI); // PI

		return result;
	}
	TVec3 Ang_deg_Vec(const vector<double> angle_deg)
	{

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */

		TVec3 result;

		result._y = -sin(angle_deg[0] * PI / 180)*cos(angle_deg[1] * PI / 180);
		result._x = sin(angle_deg[0] * PI / 180)*sin(angle_deg[1] * PI / 180);
		result._z= cos(angle_deg[0] * PI / 180);

		/*double scale;
		scale = sqrt(pow(result._x, 2) + pow(result._y, 2) + pow(result._z, 2));
		result._x = result._x / scale;
		result._y = result._y / scale;
		result._z = result._z / scale;*/



		return result;
	}

	quaternion get_pure_qu(quaternion in_qu)
	{
		quaternion q_out;
		double angle;
		TVec3 pure_axis;
		angle = acos(in_qu.mData[3]);

		pure_axis._x = in_qu.mData[0] / sin(angle);
		pure_axis._y = in_qu.mData[1] / sin(angle);
		pure_axis._z = in_qu.mData[2] / sin(angle);
		q_out = quaternion(pure_axis, angle);

		return q_out;
	}
	quaternion get_smt_qu(quaternion pure_qu,int cur_frm, int tot_frm)
	{
		quaternion q_out;
		double angle, real;
		TVec3 edit_axis;
		//cal_quat = gst::get_smt_qu(pure_qu,i-1, middleidx-1);
		double ratio = (double)cur_frm / tot_frm;
		//cout <<"angle ratio : " <<ratio <<endl;
		angle = pure_qu.mData[3]*(ratio*0.3+0.7);
		real = cos(angle);
		edit_axis._x = pure_qu.mData[0]* sin(angle);
		edit_axis._y = pure_qu.mData[1]* sin(angle);
		edit_axis._z = pure_qu.mData[2]* sin(angle);


		q_out = quaternion(edit_axis, real);

		return q_out;
	}

	quaternion rnd_mix_quat(int vertical, int horizion)
	{
		quaternion q_out;

		q_out = quaternion(0, 0, 0, 1);
		quaternion q_aply;

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */
		
		
		if (vertical > 0)
			q_aply = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		else
			q_aply = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962);

		for (int i = 0; i < abs(vertical); i++)
		{
			
			q_out = q_aply.mutiplication(q_out);
			//cout << q_out << endl;

		}
		cout << q_aply <<endl;
		if (horizion > 0)
			q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
		else
			q_aply = quaternion(0, -0.0087155743, 5.33894E-18, 0.999962);
		for (int i = 0; i < abs(horizion); i++)
		{
			q_out = q_aply.mutiplication(q_out);

		}
		cout << q_aply << endl;

		return q_out;
	}
	//quaternion dir_mix_quat(quaternion q_aply, quaternion in_put, int cycle)
	//{
	//	quaternion result;
	//	if (cycle == 0)
	//	{
	//		
	//		return q_aply.mutiplication(in_put);
	//	}
	//	else
	//	{
	//		dir_mix_quat(q_aply, in_put, cycle-1);
	//		

	//	}
	//	
	//}

	quaternion q_mult_test(quaternion q_aply, quaternion in_put)
	{
		quaternion result;
		
		result=q_aply.mutiplication(in_put);


		return result;
		
	}


	quaternion dir_mix_quat(const quaternion data, int vertical, int horizion)
	{
		quaternion q_out=data;
		//quaternion q_out;
		//q_out = quaternion(0, 0, 0, 1);
		quaternion q_aply;

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
			q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */


		/*if (vertical > 0)
			q_aply = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		else
			q_aply = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962);
*/
		if (vertical > 0)
			q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
		else
			q_aply = quaternion(0, -0.0087155743, 5.33894E-18, 0.999962);

		for (int i = 0; i < abs(vertical); i++)
		{

			//q_out = q_mult_test(q_aply, q_out);
			q_out = q_aply.mutiplication(q_out);
			//cout << q_out << endl;
		}

		if (horizion > 0)
			q_aply = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		else
			q_aply = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962);
		for (int i = 0; i < abs(horizion); i++)
		{

			//q_out = q_mult_test(q_aply, q_out);
			q_out = q_aply.mutiplication(q_out);
			//cout << q_out << endl;
		}
		/*q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);


		q_out = q_aply.mutiplication(q_aply.mutiplication(q_aply.mutiplication(data)));*/

		return q_out;
	}
	quaternion rnd_mix_fix_quat(int vertical, int horizion)
	{
		quaternion q_out;

		q_out = quaternion(0, 0, 0, 1);
		quaternion q_aply;

		/*q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
			q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); */


		if (vertical > 0)
			q_aply = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
		else
			q_aply = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);

		for (int i = 0; i < abs(vertical); i++)
		{

			q_out = q_aply.mutiplication(q_out);


		}

		if (horizion > 0)
			q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
		else
			q_aply = quaternion(0, -0.0087155743, 5.33894E-18, 0.999962);
		for (int i = 0; i < abs(horizion); i++)
		{
			q_out = q_aply.mutiplication(q_out);

		}


		return q_out;
	}
	double quaternion_distance(const quaternion trj1, const quaternion trj2)
	{
		TVec3 startingVector = { 0,0,-1 };

		quaternion vQuat = quaternion(startingVector._x, startingVector._y, startingVector._z, 0);
		quaternion vQuat1 = quaternion(startingVector._x, startingVector._y, startingVector._z, 0);
		quaternion q_buf1 = trj1;
		quaternion q_buf2 = trj2;


		quaternion trj_qvec1 = q_buf1.mutiplication(vQuat.mutiplication(q_buf1.Inverse()));
		quaternion trj_qvec2 = q_buf2.mutiplication(vQuat1.mutiplication(q_buf2.Inverse()));


		Bodypos trj_vec1, trj_vec2;

		trj_vec1.x = trj_qvec1.mData[0];
		trj_vec1.y = trj_qvec1.mData[1];
		trj_vec1.z = trj_qvec1.mData[2];


		trj_vec2.x = trj_qvec2.mData[0];
		trj_vec2.y = trj_qvec2.mData[1];
		trj_vec2.z = trj_qvec2.mData[2];


		//cout << "check Point :  " << trj_vec1.x << " , " << trj_vec1.y <<" , "<< trj_vec1.z <<"  /   " << trj_vec2.x << " , " << trj_vec2.y << " , " << trj_vec2.z <<endl;
		double scale;
		scale = vector_Sph_distance(trj_vec1, trj_vec2);

		return scale;
	}
	TVec3 quaternion_to_trajectory(const quaternion trj1)
	{
		TVec3 startingVector = { 0,0,-1 };

		quaternion vQuat = quaternion(startingVector._x, startingVector._y, startingVector._z, 0);
		
		quaternion q_buf1 = trj1;
		quaternion trj_qvec1 = q_buf1.mutiplication(vQuat.mutiplication(q_buf1.Inverse()));

		TVec3 trj_vec1;

		trj_vec1._x = trj_qvec1.mData[0];
		trj_vec1._y = trj_qvec1.mData[1];
		trj_vec1._z = trj_qvec1.mData[2];

		return trj_vec1;
	}

	vector<TVec3> caltrajectory(const GenData* trj1)
	{
		int trj1_frm = trj1->maxframe;
		
		vector<TVec3> trajectory;


		for (int trj1_idx = 0; trj1_idx < trj1_frm; trj1_idx++)
		{
			quaternion first_q1, q1;

			first_q1 = trj1->QuatData[0].JointsQuat[2];
			q1 = trj1->QuatData[trj1_idx].JointsQuat[2];

			TVec3 buf;

			buf = quaternion_to_trajectory(q1.mutiplication(first_q1.Inverse()));			
			trajectory.push_back(buf);
		}				
		return trajectory;

	}
	int find_middlePI_range(const GenData* trj1,int init_frm,int End_frm)
	{
		int trj1_frm = trj1->maxframe;
		int middlept_idx;

		quaternion first_q1[2];
		first_q1[0] = trj1->QuatData[0].JointsQuat[1];
		first_q1[1] = trj1->QuatData[0].JointsQuat[2];

		vector<int> idx_buf;
		idx_buf.push_back(init_frm);
		idx_buf.push_back(End_frm-1);
		cout<< " buf idx check "<<idx_buf[0]<<", "<< idx_buf[1] << endl;
		//else
		//	vQuat = quaternion(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

		//quaternion lw = lq.mutiplication(vQuat.mutiplication(lq.Inverse()));//QVQ-1
		double mid_pi=0;
		double pi_buf[2];
		quaternion q1_cal[2];
		quaternion buf_q_cal[2];
		quaternion Hieq1_cal;
		TVec3 trj1_cal;
		vector<double> cal_theta;

		for (int i = 0; i < idx_buf.size(); i++) {

		
		q1_cal[0] = trj1->QuatData[idx_buf[i]].JointsQuat[1];
		q1_cal[1] = trj1->QuatData[idx_buf[i]].JointsQuat[2];

		buf_q_cal[0] = q1_cal[0].mutiplication(first_q1[0].Inverse());
		buf_q_cal[1] = q1_cal[1].mutiplication(first_q1[1].Inverse());
		
		Hieq1_cal = buf_q_cal[0].Inverse().mutiplication(buf_q_cal[1]);

		trj1_cal = gst::quaternion_to_trajectory(Hieq1_cal);
		
		cal_theta = gst::vec_Theta(trj1_cal);
		mid_pi += cal_theta[1];
		//cout<<"PI test  " << cal_theta[1] <<endl;
		}
		mid_pi = mid_pi / 2;
		//cout << "mid PI test  " << mid_pi << endl;
		double trj1_theta, trj2_theta;
		double dist_buf = 100;
		double theta_buf;
		int keep_idx = 0;
		for (int trj1_idx = init_frm; trj1_idx < End_frm; trj1_idx++)
		{




			quaternion q1[2];
			q1[0] = trj1->QuatData[trj1_idx].JointsQuat[1];
			q1[1] = trj1->QuatData[trj1_idx].JointsQuat[2];

			quaternion buf_q[2];

			buf_q[0] = q1[0].mutiplication(first_q1[0].Inverse());
			buf_q[1] = q1[1].mutiplication(first_q1[1].Inverse());

			quaternion Hieq1, Hieq2;
			Hieq1 = buf_q[0].Inverse().mutiplication(buf_q[1]);

			TVec3 trj1 = gst::quaternion_to_trajectory(Hieq1);

			vector<double> cal_theta;

			cal_theta = gst::vec_Theta(trj1);

			//cout << "pi test Value " << cal_theta[1] <<endl;
			if (dist_buf > abs(mid_pi-cal_theta[1]))
			{
				dist_buf = abs(mid_pi - cal_theta[1]);
				
				keep_idx = trj1_idx;

			}
								   			 		  		  
		}
		middlept_idx = keep_idx;
		//cout << "idx result " << keep_idx << "     " << theta_buf << endl;
		return middlept_idx;
	}
	JointQuat get_firsinv(const JointQuat* first, const JointQuat* cur_frm )
	{
	
		JointQuat data;
		JointQuat first_buf = *first;
		JointQuat cur_buf = *cur_frm;


		for (int i = 0; i < 10; i++) {

			data.JointsQuat[i]= cur_buf.JointsQuat[i].mutiplication(first_buf.JointsQuat[i].Inverse());
		}

		
		return data;
	}
	
	void joint_pos_backup(double in[][3],double (*data)[3])
	{
		for (int i = 0; i < 17; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				in[i][j] = data[i][j];
			}
		}
	}
	void quat_smt_aply(GenData* rawdata,quaternion q_apply, int cnt, int joint_idx)
	{
		int start_point =10;
		int buf= start_point;
		for (int q_cnt = 0; q_cnt < cnt; q_cnt++)
		{
			
			for (int i = buf; i < rawdata->maxframe-1; i++)
			{
				rawdata->QuatData[i].JointsQuat[joint_idx] = q_apply.mutiplication(rawdata->QuatData[i].JointsQuat[joint_idx]);
			}

			//buf = start_point + (int)((rawdata->maxframe-1 - start_point) / (double)cnt)*q_cnt;
			cout <<"idx check" <<buf<<endl;;
		}
	
	
	
	}

	void cal_edit_Pos(GenData* rawdata,double target[3], double joint[3],double sensitivity)
	{

		double differ[3];
		double jointPoint[17][3];
		JointQuat af_firdata;
		//double sensitivity=5;

		for (int i = 0; i < 3; i++)
		{
			differ[i] = joint[i] - target[i];
		}

		quaternion q_vertical,q_horizon, q_lower;
		quaternion q_aply;
		int horizon_cnt=0;
		int vertical_cnt = 0;
		int depth_cnt = 0;
		cout <<"differ"<< differ[0]<< endl;

		if (differ[0] < 0)
			q_aply = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);
		else
			q_aply = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962);

		while(abs(differ[0])> sensitivity){

			//q_out = q_mult_test(q_aply, q_out);
			rawdata->QuatData[rawdata->maxframe-1].JointsQuat[2] = q_aply.mutiplication(rawdata->QuatData[rawdata->maxframe-1].JointsQuat[2]);
			//cout << q_out << endl;
			gst::cal_end_Pos(rawdata, jointPoint);

			differ[0] = jointPoint[15][0] - target[0];
			//if (horizon_cnt > 70)
			//{
			//	break;
			//	/*q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
			//	wheel_cnt++;*/

			//}
		horizon_cnt++;
		}
		q_horizon = q_aply;
		gst::quat_smt_aply(rawdata, q_horizon, horizon_cnt,2);

		cout <<"horzion time : " <<horizon_cnt << endl;

		if (differ[1] < 0)

			q_aply = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962);
		else
			q_aply = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962);

			/*q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
		else
			q_aply = quaternion(0, -0.0087155743, 5.33894E-18, 0.999962);*/
		int switching_idx = 0;
		quaternion Hieq1;
		vector<double> cal_theta;
		TVec3 trj1;
		int wheel_cnt=0;
		q_vertical = q_aply;
		quaternion q_wheel= quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
		cout << "differ" << differ[1] << endl;
		while (abs(differ[1])> sensitivity) {
			
					   		
			rawdata->QuatData[rawdata->maxframe - 1].JointsQuat[2] = q_aply.mutiplication(rawdata->QuatData[rawdata->maxframe - 1].JointsQuat[2]);
			
			
		/*	int keep_idx = 0;
			quaternion first_q1[2];
			first_q1[0] = trj1->QuatData[0].JointsQuat[1];
			first_q1[1] = trj1->QuatData[0].JointsQuat[2];

			quaternion q1[2];
			q1[0] = trj1->QuatData[trj1_idx].JointsQuat[1];
			q1[1] = trj1->QuatData[trj1_idx].JointsQuat[2];

			quaternion buf_q[2];

			buf_q[0] = q1[0].mutiplication(first_q1[0].Inverse());
			buf_q[1] = q1[1].mutiplication(first_q1[1].Inverse());

			quaternion Hieq1, Hieq2;
			Hieq1 = buf_q[1].Inverse().mutiplication(buf_q[2]);

			TVec3 trj1 = gst::quaternion_to_trajectory(Hieq1);*/

			gst::cal_end_Pos(rawdata, jointPoint);

			differ[1] = jointPoint[15][1] - target[1];
		
			af_firdata = gst::get_firsinv(&rawdata->QuatData[0], &rawdata->QuatData[rawdata->maxframe - 1]);
			Hieq1 = af_firdata.JointsQuat[1].Inverse().mutiplication(af_firdata.JointsQuat[2]);

			trj1 = gst::quaternion_to_trajectory(Hieq1);
			//cout << "wheel act " << trj1._z << endl;
			cal_theta = gst::vec_Theta(trj1);
			//cout << "wheel act " << cal_theta[0] << endl;
			//if (vertical_cnt>60)
			//{
			//	break;
			//	/*q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
			//	wheel_cnt++;*/

			//}


			vertical_cnt++;
			}
		
		gst::quat_smt_aply(rawdata, q_vertical, vertical_cnt - wheel_cnt,2);
		cout << "vertical time : " << vertical_cnt - wheel_cnt << endl;

		gst::quat_smt_aply(rawdata, q_wheel, wheel_cnt, 2);
		cout << "vertical wheel time : " << wheel_cnt << endl;


		//if (differ[2] > 0)
		//	q_aply = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);// decrease angle -z
		//else
		//	q_aply = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); // increase angle +z

		//while (abs(differ[2]) > 3.5) {


		//	rawdata->QuatData[rawdata->maxframe - 1].JointsQuat[3] = q_aply.mutiplication(rawdata->QuatData[rawdata->maxframe - 1].JointsQuat[3]);


		//	gst::cal_end_Pos(rawdata, jointPoint);

		//	differ[2] = jointPoint[15][2] - target[2];

		//	//if (depth_cnt > 60)
		//	//{
		//	//	break;
		//	//	/*q_aply = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
		//	//	wheel_cnt++;*/

		//	//}

		//	depth_cnt++;
		//}
		//q_lower = q_aply;
		//gst::quat_smt_aply(rawdata, q_lower, depth_cnt,3);

		//cout << "depth time : " << depth_cnt << endl;





	}

	void cal_edit_Pos(const GenData* rawdata, double in[][3])
	{
		enum {
			rightFootLowerLeg,   // 0
			leftFootLowerLeg,		 // 1
			rightKneeUpperLeg,		 // 2
			rightHipSegment,		 // 3
			leftKneeUpperLeg,		 // 4
			leftHipSegment,			 // 5
			pelvisStomach,			 // 6
			sternumChest, 			 // 7
			neckToChin, 			 // 8
			chinToHead, 			 // 9
			head, 					 // 10
			rightShoulderSegment, 	 // 11
			leftShoulderSegment, 	 // 12
			rightElbowUpperArm,		 // 13
			leftElbowUpperArm, 		 // 14
			rightHandLowerArm, 		 // 15
			leftHandLowerArm		 // 16

		};

		enum {
			FootLowerLeglength,              // 0
			KneeUpperLeglength,				 // 1
			HipSegmentlength,				 // 2
			pelvisStomachlength,				 // 3
			ShoulderSegmentlength,			 // 4
			ElbowUpperArmlength,			 // 5
			HandLowerArmlength,				 // 6
			neckToChinlength
		};



		double jointPoint[17][3] = { 0 };
		double bonelength[8];
		JointQuat af_firdata;
		

		for (int i = 0; i < 8; i++)
		{
			bonelength[i] = VitruvianAvatar::model_length[i + 2];

		}
		
			af_firdata = gst::get_firsinv(&rawdata->QuatData[0], &rawdata->QuatData[rawdata->maxframe-1]);

			//cout << VitruvianAvatar::model_length[0] << endl;   // rightFootLowerLeg.jointPoint[0];
			//cout << VitruvianAvatar::model_length[1] << endl;		  // leftFootLowerLeg.jointPoint[0];

			//cout << VitruvianAvatar::model_length[2] << endl;		  // bonelength[FootLowerLeglength];
			//cout << VitruvianAvatar::model_length[3] << endl;		  // bonelength[KneeUpperLeglength];
			//cout << VitruvianAvatar::model_length[4] << endl;		  // bonelength[HipSegmentlength];
			//cout << VitruvianAvatar::model_length[5] << endl;		  // bonelength[pelvisStomachlength];
			//cout << VitruvianAvatar::model_length[6] << endl;		  // bonelength[ShoulderSegmentlength];
			//cout << VitruvianAvatar::model_length[7] << endl;		  // bonelength[ElbowUpperArmlength];
			//cout << VitruvianAvatar::model_length[8] << endl;		  // bonelength[HandLowerArmlength];
			//cout << VitruvianAvatar::model_length[9] << endl;
			////VitruvianAvatar::model_length[9]



			jointPoint[rightFootLowerLeg][0] = rawdata->jointPoint[rightFootLowerLeg][0];
			jointPoint[rightFootLowerLeg][1] = 0;
			jointPoint[rightFootLowerLeg][2] = rawdata->jointPoint[rightFootLowerLeg][2];

			jointPoint[leftFootLowerLeg][0] = rawdata->jointPoint[leftFootLowerLeg][0];
			jointPoint[leftFootLowerLeg][1] = 0;
			jointPoint[leftFootLowerLeg][2] = rawdata->jointPoint[leftFootLowerLeg][2];

			//cout << "right  Check  : z " << jointPoint[rightFootLowerLeg][2] << "   z " << jointPoint[leftFootLowerLeg][2] << endl;
			double diffR = jointPoint[rightFootLowerLeg][1];
			double diffL = jointPoint[leftFootLowerLeg][1];



			//}

			jointPoint[rightKneeUpperLeg][1] = jointPoint[rightKneeUpperLeg][1] - diffR;
			jointPoint[rightHipSegment][1] = jointPoint[rightHipSegment][1] - diffR;

			jointPoint[leftKneeUpperLeg][1] = jointPoint[leftKneeUpperLeg][1] - diffL;
			jointPoint[leftHipSegment][1] = jointPoint[leftHipSegment][1] - diffL;


			//---------------------------------------- Bottom-Up Update -----------------------------------------------
			//Finding right knee joint from the fixed right foot.
			quaternion q = af_firdata.JointsQuat[7];// .Inverse().mutiplication(af_firdata.JointsQuat[7]);
			quaternion vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[rightKneeUpperLeg][0] = jointPoint[rightFootLowerLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
			jointPoint[rightKneeUpperLeg][1] = 0 + vec.mData[2] * bonelength[FootLowerLeglength];
			jointPoint[rightKneeUpperLeg][2] = jointPoint[rightFootLowerLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];

			//Finding left knee joint from the fixed left foot.
			q = af_firdata.JointsQuat[9];// .Inverse().mutiplication(af_firdata.JointsQuat[9]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[leftKneeUpperLeg][0] = jointPoint[leftFootLowerLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
			jointPoint[leftKneeUpperLeg][1] = 0 + vec.mData[2] * bonelength[FootLowerLeglength];
			jointPoint[leftKneeUpperLeg][2] = jointPoint[leftFootLowerLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];

			//Finding right pelvis joint from the right knee.
			q = af_firdata.JointsQuat[6];// .Inverse().mutiplication(af_firdata.JointsQuat[6]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[rightHipSegment][0] = jointPoint[rightKneeUpperLeg][0] + vec.mData[0] * bonelength[KneeUpperLeglength];
			jointPoint[rightHipSegment][1] = jointPoint[rightKneeUpperLeg][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
			jointPoint[rightHipSegment][2] = jointPoint[rightKneeUpperLeg][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];

			//Finding left pelvis joint from the left knee.
			q = af_firdata.JointsQuat[8];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[leftHipSegment][0] = jointPoint[leftKneeUpperLeg][0] + vec.mData[0] * bonelength[KneeUpperLeglength];
			jointPoint[leftHipSegment][1] = jointPoint[leftKneeUpperLeg][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
			jointPoint[leftHipSegment][2] = jointPoint[leftKneeUpperLeg][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];

			//Finding pelvis joint from the left and right pelvis.
			if (jointPoint[leftHipSegment][1] > jointPoint[rightHipSegment][1])
			{
				//cout <<"left " <<endl;
				q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
				vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

				jointPoint[pelvisStomach][0] = jointPoint[leftHipSegment][0] + vec.mData[0] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][1] = jointPoint[leftHipSegment][1] + vec.mData[2] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][2] = jointPoint[leftHipSegment][2] - vec.mData[1] * bonelength[HipSegmentlength];

				/*float diffR = jointPoint[leftHipSegment][1] - jointPoint[rightHipSegment][1];
				jointPoint[rightKneeUpperLeg][1] = jointPoint[rightKneeUpperLeg][1] + diffR;
				jointPoint[rightFootLowerLeg][1] = jointPoint[rightFootLowerLeg][1] + diffR;
				jointPoint[rightHipSegment][1] = jointPoint[rightHipSegment][1] + diffR;

				diffR = jointPoint[leftHipSegment][2] - jointPoint[rightHipSegment][2];;
				jointPoint[rightKneeUpperLeg][2] = jointPoint[rightKneeUpperLeg][2] + diffR;
				jointPoint[rightFootLowerLeg][2] = jointPoint[rightFootLowerLeg][2] + diffR;
				jointPoint[rightHipSegment][2] = jointPoint[rightHipSegment][2] + diffR;*/

				//	isEqual = false;

			}
			else 	if (jointPoint[rightHipSegment][1] >= jointPoint[leftHipSegment][1])
			{
				//cout << "right " << endl;
				q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
				vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

				jointPoint[pelvisStomach][0] = jointPoint[rightHipSegment][0] + vec.mData[0] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][1] = jointPoint[rightHipSegment][1] + vec.mData[2] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][2] = jointPoint[rightHipSegment][2] - vec.mData[1] * bonelength[HipSegmentlength];
				//cout << jointPoint[pelvisStomach][0] << "," << jointPoint[pelvisStomach][1] << "," << jointPoint[pelvisStomach][2] << ",";

				/*float diffR = jointPoint[rightHipSegment][1] - jointPoint[leftHipSegment][1];
				jointPoint[leftKneeUpperLeg][1]	= jointPoint[leftKneeUpperLeg][1] + diffR;
				jointPoint[leftFootLowerLeg][1]	= jointPoint[leftFootLowerLeg][1] + diffR;
				jointPoint[leftHipSegment][1]	= jointPoint[leftHipSegment][1] + diffR;

				jointPoint[leftKneeUpperLeg][2]	= jointPoint[leftKneeUpperLeg][2] + diffR;
				jointPoint[leftFootLowerLeg][2]	= jointPoint[leftFootLowerLeg][2] + diffR;
				jointPoint[leftHipSegment][2]	= jointPoint[leftHipSegment][2]   + diffR;*/
				// isEqual = false;
			}
			/*else
			{
				jointPoint[pelvisStomach][0] = (jointPoint[rightHipSegment][0] + jointPoint[leftHipSegment][0]) / 2;
				jointPoint[pelvisStomach][1] = (jointPoint[rightHipSegment][1] + jointPoint[leftHipSegment][1]) / 2;
				jointPoint[pelvisStomach][2] = (jointPoint[rightHipSegment][2] + jointPoint[leftHipSegment][2]) / 2;
				diffR = jointPoint[rightHipSegment][2] - jointPoint[leftHipSegment][2];
				isEqual = true;
			}*/
			//cout << jointPoint[pelvisStomach][0] << "," << jointPoint[pelvisStomach][1] << "," << jointPoint[pelvisStomach][2]<<endl;
		//----------------------------------------Top-Down Update---------------------------------------------------------------------------------------
				//Finding right pelvis joint from the pelvis.
			q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[0]);
			vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[rightHipSegment][0] = jointPoint[pelvisStomach][0] + vec.mData[0] * bonelength[HipSegmentlength];
			jointPoint[rightHipSegment][1] = jointPoint[pelvisStomach][1] + vec.mData[2] * bonelength[HipSegmentlength]; //Changes in Lower body with pelvis rotation and model moves continously in z axis
			jointPoint[rightHipSegment][2] = jointPoint[pelvisStomach][2] - vec.mData[1] * bonelength[HipSegmentlength];//Changes in Lower body with pelvis rotation and model moves continously in z axis
			//cout << jointPoint[rightHipSegment][0] << "," << jointPoint[rightHipSegment][1] << "," << jointPoint[rightHipSegment][2] << ",";

			//Finding left pelvis joint from the pelvis.
			q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[0]);
			vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[leftHipSegment][0] = jointPoint[pelvisStomach][0] + vec.mData[0] * bonelength[HipSegmentlength];
			jointPoint[leftHipSegment][1] = jointPoint[pelvisStomach][1] + vec.mData[2] * bonelength[HipSegmentlength];  //Changes in Lower body with pelvis rotation and model moves continously in z axis
			jointPoint[leftHipSegment][2] = jointPoint[pelvisStomach][2] - vec.mData[1] * bonelength[HipSegmentlength];	//Changes in Lower body with pelvis rotation and model moves continously in z axis
			//cout << jointPoint[leftHipSegment][0] << "," << jointPoint[leftHipSegment][1] << "," << jointPoint[leftHipSegment][2] << ",";

		//if (!isEqual) //Execute topdown approach only when left and right hips are unequal
			{
				//Finding right knee from the right pelvis point
				q = af_firdata.JointsQuat[6];// .Inverse().mutiplication(af_firdata.JointsQuat[6]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[rightKneeUpperLeg][0] = (jointPoint[rightHipSegment][0] + vec.mData[0] * bonelength[KneeUpperLeglength]);
				jointPoint[rightKneeUpperLeg][1] = jointPoint[rightHipSegment][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
				jointPoint[rightKneeUpperLeg][2] = jointPoint[rightHipSegment][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];
				//cout << jointPoint[rightKneeUpperLeg][0] << "," << jointPoint[rightKneeUpperLeg][1] << "," << jointPoint[rightKneeUpperLeg][2] << ",";

				//Finding left knee from the left pelvis point
				q = af_firdata.JointsQuat[8];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[leftKneeUpperLeg][0] = jointPoint[leftHipSegment][0] + vec.mData[0] * bonelength[KneeUpperLeglength];
				jointPoint[leftKneeUpperLeg][1] = jointPoint[leftHipSegment][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
				jointPoint[leftKneeUpperLeg][2] = jointPoint[leftHipSegment][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];
				//cout << jointPoint[leftKneeUpperLeg][0] << "," << jointPoint[leftKneeUpperLeg][1] << "," << jointPoint[leftKneeUpperLeg][2] << ",";

				//Finding right foot from the right knee point
				q = af_firdata.JointsQuat[7];// .Inverse().mutiplication(af_firdata.JointsQuat[7]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[rightFootLowerLeg][0] = jointPoint[rightKneeUpperLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
				jointPoint[rightFootLowerLeg][1] = jointPoint[rightKneeUpperLeg][1] + vec.mData[2] * bonelength[FootLowerLeglength];
				jointPoint[rightFootLowerLeg][2] = jointPoint[rightKneeUpperLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];
				//cout << jointPoint[rightFootLowerLeg][0] << "," << jointPoint[rightFootLowerLeg][1] << "," << jointPoint[rightFootLowerLeg][2] << ",";

				//Finding left foot from the left knee point
				q = af_firdata.JointsQuat[9];// .Inverse().mutiplication(af_firdata.JointsQuat[9]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[leftFootLowerLeg][0] = jointPoint[leftKneeUpperLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
				jointPoint[leftFootLowerLeg][1] = jointPoint[leftKneeUpperLeg][1] + vec.mData[2] * bonelength[FootLowerLeglength];;
				jointPoint[leftFootLowerLeg][2] = jointPoint[leftKneeUpperLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];
				//cout << jointPoint[leftFootLowerLeg][0] << "," << jointPoint[leftFootLowerLeg][1] << "," << jointPoint[leftFootLowerLeg][2] << ",";
			}
			//---------------------------------------------Updating UpperBody starting from pelvis position --------------------
			//Finding sternum from the pelvis point
			q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[0]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[sternumChest][0] = jointPoint[pelvisStomach][0] + vec.mData[0] * bonelength[pelvisStomachlength];
			jointPoint[sternumChest][1] = jointPoint[pelvisStomach][1] + vec.mData[2] * bonelength[pelvisStomachlength];
			jointPoint[sternumChest][2] = jointPoint[pelvisStomach][2] + -vec.mData[1] * bonelength[pelvisStomachlength];
			//cout << jointPoint[sternumChest][0] << "," << jointPoint[sternumChest][1] << "," << jointPoint[sternumChest][2] << ",";
			//Finding neck from the sternum point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));


			jointPoint[neckToChin][0] = jointPoint[sternumChest][0] + vec.mData[0] * bonelength[pelvisStomachlength];
			jointPoint[neckToChin][1] = jointPoint[sternumChest][1] + vec.mData[2] * bonelength[pelvisStomachlength];
			jointPoint[neckToChin][2] = jointPoint[sternumChest][2] + -vec.mData[1] * bonelength[pelvisStomachlength];
			//cout << jointPoint[neckToChin][0] << "," << jointPoint[neckToChin][1] << "," << jointPoint[neckToChin][2] << ",";
			//headQuat = af_firdata.JointsQuat[1]; //assigning headQuat for head rotation

			//Finding head from the neck point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[chinToHead][0] = jointPoint[neckToChin][0] + vec.mData[0] * bonelength[neckToChinlength];
			jointPoint[chinToHead][1] = jointPoint[neckToChin][1] + vec.mData[2] * bonelength[neckToChinlength];
			jointPoint[chinToHead][2] = jointPoint[neckToChin][2] + -vec.mData[1] * bonelength[neckToChinlength];

			jointPoint[head][0] = jointPoint[chinToHead][0] + vec.mData[0] * bonelength[neckToChinlength];
			jointPoint[head][1] = jointPoint[chinToHead][1] + vec.mData[2] * bonelength[neckToChinlength];
			jointPoint[head][2] = jointPoint[chinToHead][2] + -vec.mData[1] * bonelength[neckToChinlength];
			//cout << jointPoint[head][0] << "," << jointPoint[head][1] << "," << jointPoint[head][2] << ",";
			//---------------------------------------------- Hands ----------------------------------------------------
				//Finding rightShoulder from the neck point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[rightShoulderSegment][0] = jointPoint[neckToChin][0] + vec.mData[0] * bonelength[ShoulderSegmentlength];
			jointPoint[rightShoulderSegment][1] = jointPoint[neckToChin][1] + vec.mData[2] * bonelength[ShoulderSegmentlength];
			jointPoint[rightShoulderSegment][2] = jointPoint[neckToChin][2] + -vec.mData[1] * bonelength[ShoulderSegmentlength];
			//cout << jointPoint[rightShoulderSegment][0] << "," << jointPoint[rightShoulderSegment][1] << "," << jointPoint[rightShoulderSegment][2] << ",";
			//Finding leftShoulder from the neck point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[leftShoulderSegment][0] = jointPoint[neckToChin][0] + vec.mData[0] * bonelength[ShoulderSegmentlength];
			jointPoint[leftShoulderSegment][1] = jointPoint[neckToChin][1] + vec.mData[2] * bonelength[ShoulderSegmentlength];
			jointPoint[leftShoulderSegment][2] = jointPoint[neckToChin][2] + -vec.mData[1] * bonelength[ShoulderSegmentlength];
			//cout << jointPoint[leftShoulderSegment][0] << "," << jointPoint[leftShoulderSegment][1] << "," << jointPoint[leftShoulderSegment][2] << ",";
			//Finding right elbow from the right shoulder point
			q = af_firdata.JointsQuat[2];// .Inverse().mutiplication(af_firdata.JointsQuat[2]);
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention Pose
			//vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse())); // T-Pose

			jointPoint[rightElbowUpperArm][0] = jointPoint[rightShoulderSegment][0] + vec.mData[0] * bonelength[ElbowUpperArmlength];
			jointPoint[rightElbowUpperArm][1] = jointPoint[rightShoulderSegment][1] + vec.mData[2] * bonelength[ElbowUpperArmlength];
			jointPoint[rightElbowUpperArm][2] = jointPoint[rightShoulderSegment][2] + -vec.mData[1] * bonelength[ElbowUpperArmlength];
			//cout << jointPoint[rightElbowUpperArm][0] << "," << jointPoint[rightElbowUpperArm][1] << "," << jointPoint[rightElbowUpperArm][2] << ",";
			//Finding left elbow from the left shoulder point
			q = af_firdata.JointsQuat[4];// .Inverse().mutiplication(af_firdata.JointsQuat[4]);
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention Pose
			//vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse())); // T-Pose

			jointPoint[leftElbowUpperArm][0] = jointPoint[leftShoulderSegment][0] + vec.mData[0] * bonelength[ElbowUpperArmlength];
			jointPoint[leftElbowUpperArm][1] = jointPoint[leftShoulderSegment][1] + vec.mData[2] * bonelength[ElbowUpperArmlength];
			jointPoint[leftElbowUpperArm][2] = jointPoint[leftShoulderSegment][2] + -vec.mData[1] * bonelength[ElbowUpperArmlength];
			//cout << jointPoint[leftElbowUpperArm][0] << "," << jointPoint[leftElbowUpperArm][1] << "," << jointPoint[leftElbowUpperArm][2] << ",";
			//Finding right hand from the right elbow point
			q = af_firdata.JointsQuat[3];// .Inverse().mutiplication(af_firdata.JointsQuat[3]);
			//rightHand.rotation = q;
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention pose
			//vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse())); // t-Pose

			jointPoint[rightHandLowerArm][0] = jointPoint[rightElbowUpperArm][0] + vec.mData[0] * bonelength[HandLowerArmlength];
			jointPoint[rightHandLowerArm][1] = jointPoint[rightElbowUpperArm][1] + vec.mData[2] * bonelength[HandLowerArmlength];
			jointPoint[rightHandLowerArm][2] = jointPoint[rightElbowUpperArm][2] + -vec.mData[1] * bonelength[HandLowerArmlength];
			//cout << jointPoint[rightHandLowerArm][0] << "," << jointPoint[rightHandLowerArm][1] << "," << jointPoint[rightHandLowerArm][2]<<",";
			//Finding left hand from the left elbow point
			q = af_firdata.JointsQuat[5];// .Inverse().mutiplication(af_firdata.JointsQuat[5]);
			//leftHand.rotation = q;
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention Pose
			//vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse())); // T-pose

			jointPoint[leftHandLowerArm][0] = jointPoint[leftElbowUpperArm][0] + vec.mData[0] * bonelength[HandLowerArmlength];
			jointPoint[leftHandLowerArm][1] = jointPoint[leftElbowUpperArm][1] + vec.mData[2] * bonelength[HandLowerArmlength];
			jointPoint[leftHandLowerArm][2] = jointPoint[leftElbowUpperArm][2] + -vec.mData[1] * bonelength[HandLowerArmlength];

		
		//backup
		joint_pos_backup(in, jointPoint);

		//cout << "right Hand CAL Position Check  : x " << af_firdata.JointsQuat[3].mData[0] << " , y   " << af_firdata.JointsQuat[3].mData[1] << " , z  " << af_firdata.JointsQuat[3].mData[2] << " , z  " << af_firdata.JointsQuat[3].mData[3] << endl;
		//cout << "right Hand CAL Position Check  : x " << jointPoint[rightHandLowerArm][0] << " , y   " << jointPoint[rightHandLowerArm][1] << " , z  " << jointPoint[rightHandLowerArm][2] << endl;


	}


	void cal_end_Pos(const GenData* rawdata, double in[][3])
	{
		enum { rightFootLowerLeg ,   // 0
			leftFootLowerLeg,		 // 1
			rightKneeUpperLeg,		 // 2
			rightHipSegment,		 // 3
			leftKneeUpperLeg,		 // 4
			leftHipSegment,			 // 5
			pelvisStomach,			 // 6
			sternumChest, 			 // 7
			neckToChin, 			 // 8
			chinToHead, 			 // 9
			head, 					 // 10
			rightShoulderSegment, 	 // 11
			leftShoulderSegment, 	 // 12
			rightElbowUpperArm,		 // 13
			leftElbowUpperArm, 		 // 14
			rightHandLowerArm, 		 // 15
			leftHandLowerArm		 // 16

	};

		enum {
			FootLowerLeglength,              // 0
			KneeUpperLeglength,				 // 1
			HipSegmentlength,				 // 2
			pelvisStomachlength,				 // 3
			ShoulderSegmentlength,			 // 4
			ElbowUpperArmlength,			 // 5
			HandLowerArmlength,				 // 6
			neckToChinlength
		};									 

		/*first_X[0] = rightFootLowerLeg.jointPoint[0];
		first_X[1] = leftFootLowerLeg.jointPoint[0];

		first_Z[0] = rightFootLowerLeg.jointPoint[2];
		first_Z[1] = leftFootLowerLeg.jointPoint[2];*/

		double jointPoint[17][3] = {0};
		double bonelength[8];
		JointQuat af_firdata;
		jointPoint[rightFootLowerLeg][0] = 0;// VitruvianAvatar::model_length[0];
		jointPoint[rightFootLowerLeg][1] = 0;
		jointPoint[rightFootLowerLeg][2] = 0;
		jointPoint[leftFootLowerLeg][0] = 22.933;//VitruvianAvatar::model_length[2];
		jointPoint[leftFootLowerLeg][1] = 0;
		jointPoint[leftFootLowerLeg][2] = 0;

		for (int i = 0; i < 8; i++)
		{
			bonelength[i] = VitruvianAvatar::model_length[i + 2];

		}
		for (int frm = 1; frm < rawdata->maxframe; frm++)
		{
			af_firdata = gst::get_firsinv(&rawdata->QuatData[0], &rawdata->QuatData[frm]);

			//cout << VitruvianAvatar::model_length[0] << endl;   // rightFootLowerLeg.jointPoint[0];
			//cout << VitruvianAvatar::model_length[1] << endl;		  // leftFootLowerLeg.jointPoint[0];

			//cout << VitruvianAvatar::model_length[2] << endl;		  // bonelength[FootLowerLeglength];
			//cout << VitruvianAvatar::model_length[3] << endl;		  // bonelength[KneeUpperLeglength];
			//cout << VitruvianAvatar::model_length[4] << endl;		  // bonelength[HipSegmentlength];
			//cout << VitruvianAvatar::model_length[5] << endl;		  // bonelength[pelvisStomachlength];
			//cout << VitruvianAvatar::model_length[6] << endl;		  // bonelength[ShoulderSegmentlength];
			//cout << VitruvianAvatar::model_length[7] << endl;		  // bonelength[ElbowUpperArmlength];
			//cout << VitruvianAvatar::model_length[8] << endl;		  // bonelength[HandLowerArmlength];
			//cout << VitruvianAvatar::model_length[9] << endl;
			////VitruvianAvatar::model_length[9]

			

			jointPoint[rightFootLowerLeg][0] = jointPoint[rightFootLowerLeg][0];
			jointPoint[rightFootLowerLeg][1] = 0;
			jointPoint[rightFootLowerLeg][2] = jointPoint[rightFootLowerLeg][2];

			jointPoint[leftFootLowerLeg][0] = jointPoint[leftFootLowerLeg][0];
			jointPoint[leftFootLowerLeg][1] = 0;
			jointPoint[leftFootLowerLeg][2] = jointPoint[leftFootLowerLeg][2];

			//cout << "right  Check  : z " << jointPoint[rightFootLowerLeg][2] << "   z " << jointPoint[leftFootLowerLeg][2] << endl;
			double diffR = jointPoint[rightFootLowerLeg][1];
			double diffL = jointPoint[leftFootLowerLeg][1];



			//}

			jointPoint[rightKneeUpperLeg][1] = jointPoint[rightKneeUpperLeg][1] - diffR;
			jointPoint[rightHipSegment][1] = jointPoint[rightHipSegment][1] - diffR;

			jointPoint[leftKneeUpperLeg][1] = jointPoint[leftKneeUpperLeg][1] - diffL;
			jointPoint[leftHipSegment][1] = jointPoint[leftHipSegment][1] - diffL;


			//---------------------------------------- Bottom-Up Update -----------------------------------------------
			//Finding right knee joint from the fixed right foot.
			quaternion q = af_firdata.JointsQuat[7];// .Inverse().mutiplication(af_firdata.JointsQuat[7]);
			quaternion vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[rightKneeUpperLeg][0] = jointPoint[rightFootLowerLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
			jointPoint[rightKneeUpperLeg][1] = 0 + vec.mData[2] * bonelength[FootLowerLeglength];
			jointPoint[rightKneeUpperLeg][2] = jointPoint[rightFootLowerLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];

			//Finding left knee joint from the fixed left foot.
			q = af_firdata.JointsQuat[9];// .Inverse().mutiplication(af_firdata.JointsQuat[9]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[leftKneeUpperLeg][0] = jointPoint[leftFootLowerLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
			jointPoint[leftKneeUpperLeg][1] = 0 + vec.mData[2] * bonelength[FootLowerLeglength];
			jointPoint[leftKneeUpperLeg][2] = jointPoint[leftFootLowerLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];

			//Finding right pelvis joint from the right knee.
			q = af_firdata.JointsQuat[6];// .Inverse().mutiplication(af_firdata.JointsQuat[6]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[rightHipSegment][0] = jointPoint[rightKneeUpperLeg][0] + vec.mData[0] * bonelength[KneeUpperLeglength];
			jointPoint[rightHipSegment][1] = jointPoint[rightKneeUpperLeg][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
			jointPoint[rightHipSegment][2] = jointPoint[rightKneeUpperLeg][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];

			//Finding left pelvis joint from the left knee.
			q = af_firdata.JointsQuat[8];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[leftHipSegment][0] = jointPoint[leftKneeUpperLeg][0] + vec.mData[0] * bonelength[KneeUpperLeglength];
			jointPoint[leftHipSegment][1] = jointPoint[leftKneeUpperLeg][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
			jointPoint[leftHipSegment][2] = jointPoint[leftKneeUpperLeg][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];

			//Finding pelvis joint from the left and right pelvis.
			if (jointPoint[leftHipSegment][1] > jointPoint[rightHipSegment][1])
			{
				//cout <<"left " <<endl;
				q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
				vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

				jointPoint[pelvisStomach][0] = jointPoint[leftHipSegment][0] + vec.mData[0] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][1] = jointPoint[leftHipSegment][1] + vec.mData[2] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][2] = jointPoint[leftHipSegment][2] - vec.mData[1] * bonelength[HipSegmentlength];

				/*float diffR = jointPoint[leftHipSegment][1] - jointPoint[rightHipSegment][1];
				jointPoint[rightKneeUpperLeg][1] = jointPoint[rightKneeUpperLeg][1] + diffR;
				jointPoint[rightFootLowerLeg][1] = jointPoint[rightFootLowerLeg][1] + diffR;
				jointPoint[rightHipSegment][1] = jointPoint[rightHipSegment][1] + diffR;

				diffR = jointPoint[leftHipSegment][2] - jointPoint[rightHipSegment][2];;
				jointPoint[rightKneeUpperLeg][2] = jointPoint[rightKneeUpperLeg][2] + diffR;
				jointPoint[rightFootLowerLeg][2] = jointPoint[rightFootLowerLeg][2] + diffR;
				jointPoint[rightHipSegment][2] = jointPoint[rightHipSegment][2] + diffR;*/

				//	isEqual = false;

			}
			else 	if (jointPoint[rightHipSegment][1] >= jointPoint[leftHipSegment][1])
			{
				//cout << "right " << endl;
				q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
				vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

				jointPoint[pelvisStomach][0] = jointPoint[rightHipSegment][0] + vec.mData[0] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][1] = jointPoint[rightHipSegment][1] + vec.mData[2] * bonelength[HipSegmentlength];
				jointPoint[pelvisStomach][2] = jointPoint[rightHipSegment][2] - vec.mData[1] * bonelength[HipSegmentlength];
				//cout << jointPoint[pelvisStomach][0] << "," << jointPoint[pelvisStomach][1] << "," << jointPoint[pelvisStomach][2] << ",";

				/*float diffR = jointPoint[rightHipSegment][1] - jointPoint[leftHipSegment][1];
				jointPoint[leftKneeUpperLeg][1]	= jointPoint[leftKneeUpperLeg][1] + diffR;
				jointPoint[leftFootLowerLeg][1]	= jointPoint[leftFootLowerLeg][1] + diffR;
				jointPoint[leftHipSegment][1]	= jointPoint[leftHipSegment][1] + diffR;

				jointPoint[leftKneeUpperLeg][2]	= jointPoint[leftKneeUpperLeg][2] + diffR;
				jointPoint[leftFootLowerLeg][2]	= jointPoint[leftFootLowerLeg][2] + diffR;
				jointPoint[leftHipSegment][2]	= jointPoint[leftHipSegment][2]   + diffR;*/
				// isEqual = false;
			}
			/*else
			{
				jointPoint[pelvisStomach][0] = (jointPoint[rightHipSegment][0] + jointPoint[leftHipSegment][0]) / 2;
				jointPoint[pelvisStomach][1] = (jointPoint[rightHipSegment][1] + jointPoint[leftHipSegment][1]) / 2;
				jointPoint[pelvisStomach][2] = (jointPoint[rightHipSegment][2] + jointPoint[leftHipSegment][2]) / 2;
				diffR = jointPoint[rightHipSegment][2] - jointPoint[leftHipSegment][2];
				isEqual = true;
			}*/
			//cout << jointPoint[pelvisStomach][0] << "," << jointPoint[pelvisStomach][1] << "," << jointPoint[pelvisStomach][2]<<endl;
		//----------------------------------------Top-Down Update---------------------------------------------------------------------------------------
				//Finding right pelvis joint from the pelvis.
			q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[0]);
			vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[rightHipSegment][0] = jointPoint[pelvisStomach][0] + vec.mData[0] * bonelength[HipSegmentlength];
			jointPoint[rightHipSegment][1] = jointPoint[pelvisStomach][1] + vec.mData[2] * bonelength[HipSegmentlength]; //Changes in Lower body with pelvis rotation and model moves continously in z axis
			jointPoint[rightHipSegment][2] = jointPoint[pelvisStomach][2] - vec.mData[1] * bonelength[HipSegmentlength];//Changes in Lower body with pelvis rotation and model moves continously in z axis
			//cout << jointPoint[rightHipSegment][0] << "," << jointPoint[rightHipSegment][1] << "," << jointPoint[rightHipSegment][2] << ",";

			//Finding left pelvis joint from the pelvis.
			q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[0]);
			vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[leftHipSegment][0] = jointPoint[pelvisStomach][0] + vec.mData[0] * bonelength[HipSegmentlength];
			jointPoint[leftHipSegment][1] = jointPoint[pelvisStomach][1] + vec.mData[2] * bonelength[HipSegmentlength];  //Changes in Lower body with pelvis rotation and model moves continously in z axis
			jointPoint[leftHipSegment][2] = jointPoint[pelvisStomach][2] - vec.mData[1] * bonelength[HipSegmentlength];	//Changes in Lower body with pelvis rotation and model moves continously in z axis
			//cout << jointPoint[leftHipSegment][0] << "," << jointPoint[leftHipSegment][1] << "," << jointPoint[leftHipSegment][2] << ",";

		//if (!isEqual) //Execute topdown approach only when left and right hips are unequal
			{
				//Finding right knee from the right pelvis point
				q = af_firdata.JointsQuat[6];// .Inverse().mutiplication(af_firdata.JointsQuat[6]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[rightKneeUpperLeg][0] = (jointPoint[rightHipSegment][0] + vec.mData[0] * bonelength[KneeUpperLeglength]);
				jointPoint[rightKneeUpperLeg][1] = jointPoint[rightHipSegment][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
				jointPoint[rightKneeUpperLeg][2] = jointPoint[rightHipSegment][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];
				//cout << jointPoint[rightKneeUpperLeg][0] << "," << jointPoint[rightKneeUpperLeg][1] << "," << jointPoint[rightKneeUpperLeg][2] << ",";

				//Finding left knee from the left pelvis point
				q = af_firdata.JointsQuat[8];// .Inverse().mutiplication(af_firdata.JointsQuat[8]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[leftKneeUpperLeg][0] = jointPoint[leftHipSegment][0] + vec.mData[0] * bonelength[KneeUpperLeglength];
				jointPoint[leftKneeUpperLeg][1] = jointPoint[leftHipSegment][1] + vec.mData[2] * bonelength[KneeUpperLeglength];
				jointPoint[leftKneeUpperLeg][2] = jointPoint[leftHipSegment][2] + -vec.mData[1] * bonelength[KneeUpperLeglength];
				//cout << jointPoint[leftKneeUpperLeg][0] << "," << jointPoint[leftKneeUpperLeg][1] << "," << jointPoint[leftKneeUpperLeg][2] << ",";

				//Finding right foot from the right knee point
				q = af_firdata.JointsQuat[7];// .Inverse().mutiplication(af_firdata.JointsQuat[7]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[rightFootLowerLeg][0] = jointPoint[rightKneeUpperLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
				jointPoint[rightFootLowerLeg][1] = jointPoint[rightKneeUpperLeg][1] + vec.mData[2] * bonelength[FootLowerLeglength];
				jointPoint[rightFootLowerLeg][2] = jointPoint[rightKneeUpperLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];
				//cout << jointPoint[rightFootLowerLeg][0] << "," << jointPoint[rightFootLowerLeg][1] << "," << jointPoint[rightFootLowerLeg][2] << ",";

				//Finding left foot from the left knee point
				q = af_firdata.JointsQuat[9];// .Inverse().mutiplication(af_firdata.JointsQuat[9]);
				vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse()));

				jointPoint[leftFootLowerLeg][0] = jointPoint[leftKneeUpperLeg][0] + vec.mData[0] * bonelength[FootLowerLeglength];
				jointPoint[leftFootLowerLeg][1] = jointPoint[leftKneeUpperLeg][1] + vec.mData[2] * bonelength[FootLowerLeglength];;
				jointPoint[leftFootLowerLeg][2] = jointPoint[leftKneeUpperLeg][2] + -vec.mData[1] * bonelength[FootLowerLeglength];
				//cout << jointPoint[leftFootLowerLeg][0] << "," << jointPoint[leftFootLowerLeg][1] << "," << jointPoint[leftFootLowerLeg][2] << ",";
			}
			//---------------------------------------------Updating UpperBody starting from pelvis position --------------------
			//Finding sternum from the pelvis point
			q = af_firdata.JointsQuat[0];// .Inverse().mutiplication(af_firdata.JointsQuat[0]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[sternumChest][0] = jointPoint[pelvisStomach][0] + vec.mData[0] * bonelength[pelvisStomachlength];
			jointPoint[sternumChest][1] = jointPoint[pelvisStomach][1] + vec.mData[2] * bonelength[pelvisStomachlength];
			jointPoint[sternumChest][2] = jointPoint[pelvisStomach][2] + -vec.mData[1] * bonelength[pelvisStomachlength];
			//cout << jointPoint[sternumChest][0] << "," << jointPoint[sternumChest][1] << "," << jointPoint[sternumChest][2] << ",";
			//Finding neck from the sternum point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));


			jointPoint[neckToChin][0] = jointPoint[sternumChest][0] + vec.mData[0] * bonelength[pelvisStomachlength];
			jointPoint[neckToChin][1] = jointPoint[sternumChest][1] + vec.mData[2] * bonelength[pelvisStomachlength];
			jointPoint[neckToChin][2] = jointPoint[sternumChest][2] + -vec.mData[1] * bonelength[pelvisStomachlength];
			//cout << jointPoint[neckToChin][0] << "," << jointPoint[neckToChin][1] << "," << jointPoint[neckToChin][2] << ",";
			//headQuat = af_firdata.JointsQuat[1]; //assigning headQuat for head rotation

			//Finding head from the neck point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ 0,0,1,0 }.mutiplication(q.Inverse()));

			jointPoint[chinToHead][0] = jointPoint[neckToChin][0] + vec.mData[0] * bonelength[neckToChinlength];
			jointPoint[chinToHead][1] = jointPoint[neckToChin][1] + vec.mData[2] * bonelength[neckToChinlength];
			jointPoint[chinToHead][2] = jointPoint[neckToChin][2] + -vec.mData[1] * bonelength[neckToChinlength];

			jointPoint[head][0] = jointPoint[chinToHead][0] + vec.mData[0] * bonelength[neckToChinlength];
			jointPoint[head][1] = jointPoint[chinToHead][1] + vec.mData[2] * bonelength[neckToChinlength];
			jointPoint[head][2] = jointPoint[chinToHead][2] + -vec.mData[1] * bonelength[neckToChinlength];
			//cout << jointPoint[head][0] << "," << jointPoint[head][1] << "," << jointPoint[head][2] << ",";
			//---------------------------------------------- Hands ----------------------------------------------------
				//Finding rightShoulder from the neck point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[rightShoulderSegment][0] = jointPoint[neckToChin][0] + vec.mData[0] * bonelength[ShoulderSegmentlength];
			jointPoint[rightShoulderSegment][1] = jointPoint[neckToChin][1] + vec.mData[2] * bonelength[ShoulderSegmentlength];
			jointPoint[rightShoulderSegment][2] = jointPoint[neckToChin][2] + -vec.mData[1] * bonelength[ShoulderSegmentlength];
			//cout << jointPoint[rightShoulderSegment][0] << "," << jointPoint[rightShoulderSegment][1] << "," << jointPoint[rightShoulderSegment][2] << ",";
			//Finding leftShoulder from the neck point
			q = af_firdata.JointsQuat[1];// .Inverse().mutiplication(af_firdata.JointsQuat[1]);
			vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse()));

			jointPoint[leftShoulderSegment][0] = jointPoint[neckToChin][0] + vec.mData[0] * bonelength[ShoulderSegmentlength];
			jointPoint[leftShoulderSegment][1] = jointPoint[neckToChin][1] + vec.mData[2] * bonelength[ShoulderSegmentlength];
			jointPoint[leftShoulderSegment][2] = jointPoint[neckToChin][2] + -vec.mData[1] * bonelength[ShoulderSegmentlength];
			//cout << jointPoint[leftShoulderSegment][0] << "," << jointPoint[leftShoulderSegment][1] << "," << jointPoint[leftShoulderSegment][2] << ",";
			//Finding right elbow from the right shoulder point
			q = af_firdata.JointsQuat[2];// .Inverse().mutiplication(af_firdata.JointsQuat[2]);
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention Pose
			//vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse())); // T-Pose

			jointPoint[rightElbowUpperArm][0] = jointPoint[rightShoulderSegment][0] + vec.mData[0] * bonelength[ElbowUpperArmlength];
			jointPoint[rightElbowUpperArm][1] = jointPoint[rightShoulderSegment][1] + vec.mData[2] * bonelength[ElbowUpperArmlength];
			jointPoint[rightElbowUpperArm][2] = jointPoint[rightShoulderSegment][2] + -vec.mData[1] * bonelength[ElbowUpperArmlength];
			//cout << jointPoint[rightElbowUpperArm][0] << "," << jointPoint[rightElbowUpperArm][1] << "," << jointPoint[rightElbowUpperArm][2] << ",";
			//Finding left elbow from the left shoulder point
			q = af_firdata.JointsQuat[4];// .Inverse().mutiplication(af_firdata.JointsQuat[4]);
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention Pose
			//vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse())); // T-Pose

			jointPoint[leftElbowUpperArm][0] = jointPoint[leftShoulderSegment][0] + vec.mData[0] * bonelength[ElbowUpperArmlength];
			jointPoint[leftElbowUpperArm][1] = jointPoint[leftShoulderSegment][1] + vec.mData[2] * bonelength[ElbowUpperArmlength];
			jointPoint[leftElbowUpperArm][2] = jointPoint[leftShoulderSegment][2] + -vec.mData[1] * bonelength[ElbowUpperArmlength];
			//cout << jointPoint[leftElbowUpperArm][0] << "," << jointPoint[leftElbowUpperArm][1] << "," << jointPoint[leftElbowUpperArm][2] << ",";
			//Finding right hand from the right elbow point
			q = af_firdata.JointsQuat[3];// .Inverse().mutiplication(af_firdata.JointsQuat[3]);
			//rightHand.rotation = q;
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention pose
			//vec = q.mutiplication(quaternion{ -1,0,0,0 }.mutiplication(q.Inverse())); // t-Pose

			jointPoint[rightHandLowerArm][0] = jointPoint[rightElbowUpperArm][0] + vec.mData[0] * bonelength[HandLowerArmlength];
			jointPoint[rightHandLowerArm][1] = jointPoint[rightElbowUpperArm][1] + vec.mData[2] * bonelength[HandLowerArmlength];
			jointPoint[rightHandLowerArm][2] = jointPoint[rightElbowUpperArm][2] + -vec.mData[1] * bonelength[HandLowerArmlength];
			//cout << jointPoint[rightHandLowerArm][0] << "," << jointPoint[rightHandLowerArm][1] << "," << jointPoint[rightHandLowerArm][2]<<",";
			//Finding left hand from the left elbow point
			q = af_firdata.JointsQuat[5];// .Inverse().mutiplication(af_firdata.JointsQuat[5]);
			//leftHand.rotation = q;
			vec = q.mutiplication(quaternion{ 0,0,-1,0 }.mutiplication(q.Inverse())); // Attention Pose
			//vec = q.mutiplication(quaternion{ 1,0,0,0 }.mutiplication(q.Inverse())); // T-pose

			jointPoint[leftHandLowerArm][0] = jointPoint[leftElbowUpperArm][0] + vec.mData[0] * bonelength[HandLowerArmlength];
			jointPoint[leftHandLowerArm][1] = jointPoint[leftElbowUpperArm][1] + vec.mData[2] * bonelength[HandLowerArmlength];
			jointPoint[leftHandLowerArm][2] = jointPoint[leftElbowUpperArm][2] + -vec.mData[1] * bonelength[HandLowerArmlength];

		}
		//backup
		joint_pos_backup(in, jointPoint);

		//cout << "right Hand CAL Position Check  : x " << af_firdata.JointsQuat[3].mData[0] << " , y   " << af_firdata.JointsQuat[3].mData[1] << " , z  " << af_firdata.JointsQuat[3].mData[2] << " , z  " << af_firdata.JointsQuat[3].mData[3] << endl;
		//cout << "right Hand CAL Position Check  : x " << jointPoint[rightHandLowerArm][0] << " , y   " << jointPoint[rightHandLowerArm][1] << " , z  " << jointPoint[rightHandLowerArm][2] << endl;
	
		
     }


	int find_middlept(const GenData* trj1)
	{
		int trj1_frm = trj1->maxframe;
		int middlept_idx;

		quaternion first_q1[2];
		first_q1[0] = trj1->QuatData[0].JointsQuat[1];
		first_q1[1] = trj1->QuatData[0].JointsQuat[2];

		//else
		//	vQuat = quaternion(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

		//quaternion lw = lq.mutiplication(vQuat.mutiplication(lq.Inverse()));//QVQ-1

		double trj1_theta, trj2_theta;
		double dist_buf=0;
		double theta_buf;
		int keep_idx = 0;
		for (int trj1_idx = 0; trj1_idx < trj1_frm; trj1_idx++)
		{
			
			
			

			quaternion q1[2];
			q1[0] = trj1->QuatData[trj1_idx].JointsQuat[1];
			q1[1] = trj1->QuatData[trj1_idx].JointsQuat[2];

			quaternion buf_q[2];

			buf_q[0] = q1[0].mutiplication(first_q1[0].Inverse());
			buf_q[1] = q1[1].mutiplication(first_q1[1].Inverse());

			quaternion Hieq1, Hieq2;
			Hieq1 = buf_q[0].Inverse().mutiplication(buf_q[1]);

			TVec3 trj1 = gst::quaternion_to_trajectory(Hieq1);

			vector<double> cal_theta;

			cal_theta = gst::vec_Theta(trj1);

			//cout << "pi Value " << cal_theta[1]  << "theta Value " << cal_theta[0] <<endl;
			if (dist_buf > cal_theta[1])
			{
				dist_buf = cal_theta[1];
				theta_buf = cal_theta[0];
				keep_idx = trj1_idx;

			}
			else if (dist_buf == cal_theta[1] && theta_buf < cal_theta[0])
			{

				dist_buf = cal_theta[1];
				theta_buf = cal_theta[0];
				keep_idx = trj1_idx;


			}
			
		

			
					

		}
		middlept_idx = keep_idx;
		cout << "idx result " << keep_idx << "     "<< theta_buf << endl;
		return middlept_idx;
	}
	vector<int> idxSort_theta(const GenData* trj1, const GenData* trj2)
	{
		int trj1_frm = trj1->maxframe;
		int trj2_frm = trj2->maxframe;
		vector<int> close_idx;


		//else
		//	vQuat = quaternion(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

		//quaternion lw = lq.mutiplication(vQuat.mutiplication(lq.Inverse()));//QVQ-1

		for (int trj1_idx = 0; trj1_idx < trj1_frm; trj1_idx++)
		{
			double dist_buf;
			int keep_idx = 0;
			quaternion first_q1[2];
			first_q1[0] = trj1->QuatData[0].JointsQuat[1];
			first_q1[1] = trj1->QuatData[0].JointsQuat[2];

			quaternion q1[2];
			q1[0] = trj1->QuatData[trj1_idx].JointsQuat[1];
			q1[1] = trj1->QuatData[trj1_idx].JointsQuat[2];

			quaternion buf_q[2];

			buf_q[0] = q1[0].mutiplication(first_q1[0].Inverse());
			buf_q[1] = q1[1].mutiplication(first_q1[1].Inverse());

			quaternion Hieq1, Hieq2;
			Hieq1 = buf_q[1].Inverse().mutiplication(buf_q[2]);

			TVec3 trj1 = gst::quaternion_to_trajectory(Hieq1);

			vector<double> cal_theta;

			cal_theta = gst::vec_Theta(trj1);

			double trj1_theta, trj2_theta;
			trj1_theta = cal_theta[0];

			dist_buf = trj1_theta;
			for (int trj2_idx = 0; trj2_idx < trj2_frm; trj2_idx++)
			{
				double buf;
				
				quaternion first_q2[2];
				
				
				first_q2[0] = trj2->QuatData[0].JointsQuat[1];
				first_q2[1] = trj2->QuatData[0].JointsQuat[2];

				quaternion q2[2];
				
				
				q2[0] = trj2->QuatData[trj2_idx].JointsQuat[1];
				q2[1] = trj2->QuatData[trj2_idx].JointsQuat[2];

				// result.b0 = mySu->avatarData[index].b0.mutiplication(mySu->avatarData[0].b0.Inverse());
				//buf = quaternion_distance(q1.mutiplication(first_q1.Inverse()), q2.mutiplication(first_q2.Inverse()));

				//Init_q = buf[1].Inverse().mutiplication(buf[2]);
				

				//buf[i] = this->QuatData[Prev_frame].JointsQuat[i].mutiplication(firstquat[i]);
				

				buf_q[0] = q2[0].mutiplication(first_q2[0].Inverse());
				buf_q[1] = q2[1].mutiplication(first_q2[1].Inverse());
				Hieq2 = buf_q[1].Inverse().mutiplication(buf_q[2]);
						
				TVec3 trj2 = gst::quaternion_to_trajectory(Hieq2);
				
				cal_theta = gst::vec_Theta(trj2);
				trj2_theta = cal_theta[0];
				buf = abs(trj1_theta - trj2_theta);
				if (dist_buf > buf)
				{
					dist_buf = buf;
					keep_idx = trj2_idx;
				}
			}
			//cout << "Checkidx : "<< trj1_idx  <<" ,   "<< keep_idx <<endl;
			close_idx.push_back(keep_idx);

		}

		cout << "Checkidx :" << close_idx.size() << endl;
		return close_idx;
	}
	vector<int> idxSort(const GenData* trj1, const GenData* trj2)
	{
		int trj1_frm = trj1->maxframe;
		int trj2_frm = trj2->maxframe;
		vector<int> close_idx;


		//else
		//	vQuat = quaternion(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

		//quaternion lw = lq.mutiplication(vQuat.mutiplication(lq.Inverse()));//QVQ-1

		for (int trj1_idx = 0; trj1_idx < trj1_frm; trj1_idx++)
		{
			double dist_buf = 10;
			int keep_idx = 0;

			for (int trj2_idx = 0; trj2_idx < trj2_frm; trj2_idx++)
			{
				double buf;
				quaternion first_q1, first_q2;
				first_q1 =trj1->QuatData[0].JointsQuat[2];
				first_q2= trj2->QuatData[0].JointsQuat[2];
				quaternion q1, q2;
				q1 = trj1->QuatData[trj1_idx].JointsQuat[2];
				q2 = trj2->QuatData[trj2_idx].JointsQuat[2];
				// result.b0 = mySu->avatarData[index].b0.mutiplication(mySu->avatarData[0].b0.Inverse());
				buf = quaternion_distance(q1.mutiplication(first_q1.Inverse()), q2.mutiplication(first_q2.Inverse()));
				
				if (dist_buf > buf)
				{
					dist_buf = buf;
					keep_idx = trj2_idx;
				}
			}
			//cout << "Checkidx : "<< trj1_idx  <<" ,   "<< keep_idx <<endl;
			close_idx.push_back(keep_idx);
					   					   			 
		}

		cout << "Checkidx :" << close_idx .size()<<endl;
		return close_idx;
	}

	GenData generateIntermediateFrames(int totalIntermediateFrames, const GenData* motion_data)
	{
		int noOfKeyFrames = motion_data->maxframe - 1;
		int intermediateFrameCount = totalIntermediateFrames / (noOfKeyFrames - 1);
		vector<JointQuat> Databuf(1000);
		//Avatar GeneratedData[500];
		float tStep = 1 / (float)intermediateFrameCount;
		int dataCount = 0;
		cout << "shift S :  " << noOfKeyFrames << "  intermediateFrameCount  :    " << intermediateFrameCount << endl;
		Databuf[dataCount] = motion_data->QuatData[dataCount];
		dataCount++;
		for (int i = 1; i < noOfKeyFrames; i++)
		{
			JointQuat from = motion_data->QuatData[i];
			JointQuat to = motion_data->QuatData[i + 1];

			for (float j = 0; j <= 1; j = j + tStep)
			{
				Databuf[dataCount].JointsQuat[0] = Databuf[dataCount].JointsQuat[0].SLERP(from.JointsQuat[0], to.JointsQuat[0], j); if (isnan(Databuf[dataCount].JointsQuat[0].mData[3])) Databuf[dataCount].JointsQuat[0] = from.JointsQuat[0];
				Databuf[dataCount].JointsQuat[1] = Databuf[dataCount].JointsQuat[1].SLERP(from.JointsQuat[1], to.JointsQuat[1], j); if (isnan(Databuf[dataCount].JointsQuat[1].mData[3])) Databuf[dataCount].JointsQuat[1] = from.JointsQuat[1];
				Databuf[dataCount].JointsQuat[2] = Databuf[dataCount].JointsQuat[2].SLERP(from.JointsQuat[2], to.JointsQuat[2], j); if (isnan(Databuf[dataCount].JointsQuat[2].mData[3])) Databuf[dataCount].JointsQuat[2] = from.JointsQuat[2];
				Databuf[dataCount].JointsQuat[3] = Databuf[dataCount].JointsQuat[3].SLERP(from.JointsQuat[3], to.JointsQuat[3], j); if (isnan(Databuf[dataCount].JointsQuat[3].mData[3])) Databuf[dataCount].JointsQuat[3] = from.JointsQuat[3];
				Databuf[dataCount].JointsQuat[4] = Databuf[dataCount].JointsQuat[4].SLERP(from.JointsQuat[4], to.JointsQuat[4], j); if (isnan(Databuf[dataCount].JointsQuat[4].mData[3])) Databuf[dataCount].JointsQuat[4] = from.JointsQuat[4];
				Databuf[dataCount].JointsQuat[5] = Databuf[dataCount].JointsQuat[5].SLERP(from.JointsQuat[5], to.JointsQuat[5], j); if (isnan(Databuf[dataCount].JointsQuat[5].mData[3])) Databuf[dataCount].JointsQuat[5] = from.JointsQuat[5];
				Databuf[dataCount].JointsQuat[6] = Databuf[dataCount].JointsQuat[6].SLERP(from.JointsQuat[6], to.JointsQuat[6], j); if (isnan(Databuf[dataCount].JointsQuat[6].mData[3])) Databuf[dataCount].JointsQuat[6] = from.JointsQuat[6];
				Databuf[dataCount].JointsQuat[7] = Databuf[dataCount].JointsQuat[7].SLERP(from.JointsQuat[7], to.JointsQuat[7], j); if (isnan(Databuf[dataCount].JointsQuat[7].mData[3])) Databuf[dataCount].JointsQuat[7] = from.JointsQuat[7];
				Databuf[dataCount].JointsQuat[8] = Databuf[dataCount].JointsQuat[8].SLERP(from.JointsQuat[8], to.JointsQuat[8], j); if (isnan(Databuf[dataCount].JointsQuat[8].mData[3])) Databuf[dataCount].JointsQuat[8] = from.JointsQuat[8];
				Databuf[dataCount].JointsQuat[9] = Databuf[dataCount].JointsQuat[9].SLERP(from.JointsQuat[9], to.JointsQuat[9], j); if (isnan(Databuf[dataCount].JointsQuat[9].mData[3])) Databuf[dataCount].JointsQuat[9] = from.JointsQuat[9];
				dataCount++;
			}
		}
		
		//su->noOfFrames = dataCount;
		GenData Newdata;
		Newdata.maxframe = dataCount;
		cout << "  dataCount  :    " << dataCount << endl;
		for (int i = 0; i < dataCount; i++)
		{
			Newdata.QuatData[i] = Databuf[i];
		}

		return Newdata;
	}
	GenData generateSmoothchange(const GenData* motion_data)
	{

		GenData Newdata;
		int vertical = 0;
		int horizon = 10;
		Newdata = *motion_data;
		int middleidx = gst::find_middlept(motion_data);
		quaternion edit_qu;
		quaternion pure_qu;
		edit_qu = gst::dir_mix_quat(Newdata.QuatData[middleidx].JointsQuat[2], vertical,horizon);
		//cout << "middle point idx check:" << gst::find_middlePI_range(&Newdata, middleidx, Newdata.maxframe) << endl;

		//edit_qu = gst::rnd_mix_fix_quat(3, -10); // vertical , horizontal // -10 ,10
		//cout << "middle point idx check:" << gst::find_middlePI_range(&Newdata, middleidx, Newdata.maxframe) << endl;
		//edit_qu = edit_qu.mutiplication(edit_qu);
		//Newdata.QuatData[Newdata.maxframe-1].JointsQuat[2] = gst::dir_mix_quat(Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2], 1, 10);;
		Newdata.QuatData[middleidx].JointsQuat[2] =edit_qu;
		//edit_qu = gst::rnd_mix_quat(0, -10);

		//Newdata.QuatData[middleidx].JointsQuat[2] = edit_qu.mutiplication(Newdata.QuatData[middleidx].JointsQuat[2]);
		//edit_qu = gst::rnd_mix_quat(0,-10);


		pure_qu = gst::get_pure_qu(edit_qu);

		///////////////////////////////////////////////// before middle
		for (int i = 2; i < middleidx; i++)
		{
			quaternion cal_quat;
			cal_quat = gst::get_smt_qu(pure_qu,i-1, middleidx-1);

			//Newdata.QuatData[i].JointsQuat[2] = gst::dir_mix_quat(Newdata.QuatData[i].JointsQuat[2], int((-5)*(i-1)/(middleidx -1)), int((10)*(i - 1) / (middleidx - 1)));
			
			int vertical_step = floor(vertical*i / (middleidx - 3.0));
			int horizon_step = floor(horizon*i / (middleidx - 3.0));


			
			Newdata.QuatData[i].JointsQuat[2] = gst::dir_mix_quat(Newdata.QuatData[i].JointsQuat[2], vertical_step, horizon_step);

		
		}

		///////////////////////////////////////////// after middle
		for (int i = middleidx+1; i < Newdata.maxframe; i++)
		{
			quaternion cal_quat;
			cal_quat = gst::get_smt_qu(pure_qu, Newdata.maxframe-i-1, Newdata.maxframe- middleidx-1);

			int vertical_step = floor(vertical*(1.0-((i- middleidx+0.0) /(Newdata.maxframe- middleidx))));
			int horizon_step = floor(horizon*(1.0 - ((i - middleidx + 0.0) / (Newdata.maxframe - middleidx))));
			//cout << " step check : " << vertical_step << "," << horizon_step << endl;
			Newdata.QuatData[i].JointsQuat[2] = gst::dir_mix_quat(Newdata.QuatData[i].JointsQuat[2], vertical_step, horizon_step);
		}


		/////////////////////////////////////////////////// before middle
		//for (int i = 2; i < middleidx; i++)
		//{
		//	quaternion cal_quat;
		//	cal_quat = gst::get_smt_qu(pure_qu,i-1, middleidx-1);

		//	Newdata.QuatData[i].JointsQuat[2] = cal_quat.mutiplication(Newdata.QuatData[i].JointsQuat[2]);
		//}

		///////////////////////////////////////////////// after middle
		//for (int i = middleidx+1; i < Newdata.maxframe; i++)
		//{
		//	quaternion cal_quat;
		//	cal_quat = gst::get_smt_qu(pure_qu, Newdata.maxframe-i-1, Newdata.maxframe- middleidx-1);

		//	Newdata.QuatData[i].JointsQuat[2] = cal_quat.mutiplication(Newdata.QuatData[i].JointsQuat[2]);
		//}

		


		return Newdata;
	}
	GenData endptSmoothchange(const GenData* motion_data)
	{

		GenData Newdata;

		Newdata = *motion_data;
		int middleidx = gst::find_middlept(motion_data);
		quaternion edit_qu;
		quaternion pure_qu;

		quaternion first_q1[2];
		first_q1[0] = Newdata.QuatData[0].JointsQuat[1];
		first_q1[1] = Newdata.QuatData[0].JointsQuat[2];

		//else
		//	vQuat = quaternion(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

		//quaternion lw = lq.mutiplication(vQuat.mutiplication(lq.Inverse()));//QVQ-1

		quaternion q1[2];
		q1[0] = Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[1];
		q1[1] = Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2];

		quaternion buf_q[2];

		buf_q[0] = q1[0].mutiplication(first_q1[0].Inverse());
		buf_q[1] = q1[1].mutiplication(first_q1[1].Inverse());

		quaternion Hieq1, Hieq2;
		Hieq1 = buf_q[0].Inverse().mutiplication(buf_q[1]);

		TVec3 trj1 = gst::quaternion_to_trajectory(Hieq1);

		vector<double> cal_theta;

		cal_theta = gst::vec_Theta(trj1);
		vector<double> edit_angle;
		edit_angle.push_back(cal_theta[0]+20); // vertical
		edit_angle.push_back(cal_theta[1]-10 );// horizon

		

		TVec3 NewVector = gst::Ang_deg_Vec(edit_angle);

		//quaternion calQuat;
		TVec3 cal_axis;
		double cal_angle;
		cal_axis = gst::vector_cross(trj1, NewVector);
		cal_angle = gst::vector_dot(trj1, NewVector);

		edit_qu = gst::AxisagltoQuat(cal_axis, cal_angle);
		

		//quaternion q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962);

		//edit_qu = gst::rnd_mix_quat(0, 20); // vertical , horizontal // -10 ,10
		//cout << "middle point idx check:" << gst::find_middlePI_range(&Newdata, middleidx, Newdata.maxframe) << endl;
		
		//Newdata.QuatData[Newdata.maxframe-1].JointsQuat[2] = gst::dir_mix_quat(Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2], 1, 10);;
		//Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2] = edit_qu.mutiplication(edit_qu.mutiplication(Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2]));
		Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2] = edit_qu.mutiplication(Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2]);
		
		//Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2] = q.mutiplication(q.mutiplication(Newdata.QuatData[Newdata.maxframe - 1].JointsQuat[2]));
		pure_qu = gst::get_pure_qu(edit_qu);
		
		cout << "middle point idx check:" << Newdata.maxframe - 1 << endl;
		///////////////////////////////////////////////// after middle
		for (int i = 1 ; i < Newdata.maxframe-1; i++)
		{
			quaternion cal_quat;
			cal_quat = gst::get_smt_qu(pure_qu, i- middleidx, Newdata.maxframe - middleidx);

			Newdata.QuatData[i].JointsQuat[2] = cal_quat.mutiplication(Newdata.QuatData[i].JointsQuat[2]);
		}




		return Newdata;
	}

	void addMidFrame(int selectedFrame, GenData *&motion_data)
	{
		
		JointQuat Databuf;
		
		
		
		Databuf.JointsQuat[0] = Databuf.JointsQuat[0].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[0], motion_data->QuatData[selectedFrame].JointsQuat[0], 0.5); if (isnan(Databuf.JointsQuat[0].mData[3])) Databuf.JointsQuat[0] = motion_data->QuatData[selectedFrame-1].JointsQuat[0];
		Databuf.JointsQuat[1] = Databuf.JointsQuat[1].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[1], motion_data->QuatData[selectedFrame].JointsQuat[1], 0.5); if (isnan(Databuf.JointsQuat[1].mData[3])) Databuf.JointsQuat[1] = motion_data->QuatData[selectedFrame-1].JointsQuat[1];
		Databuf.JointsQuat[2] = Databuf.JointsQuat[2].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[2], motion_data->QuatData[selectedFrame].JointsQuat[2], 0.5); if (isnan(Databuf.JointsQuat[2].mData[3])) Databuf.JointsQuat[2] = motion_data->QuatData[selectedFrame-1].JointsQuat[2];
		Databuf.JointsQuat[3] = Databuf.JointsQuat[3].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[3], motion_data->QuatData[selectedFrame].JointsQuat[3], 0.5); if (isnan(Databuf.JointsQuat[3].mData[3])) Databuf.JointsQuat[3] = motion_data->QuatData[selectedFrame-1].JointsQuat[3];
		Databuf.JointsQuat[4] = Databuf.JointsQuat[4].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[4], motion_data->QuatData[selectedFrame].JointsQuat[4], 0.5); if (isnan(Databuf.JointsQuat[4].mData[3])) Databuf.JointsQuat[4] = motion_data->QuatData[selectedFrame-1].JointsQuat[4];
		Databuf.JointsQuat[5] = Databuf.JointsQuat[5].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[5], motion_data->QuatData[selectedFrame].JointsQuat[5], 0.5); if (isnan(Databuf.JointsQuat[5].mData[3])) Databuf.JointsQuat[5] = motion_data->QuatData[selectedFrame-1].JointsQuat[5];
		Databuf.JointsQuat[6] = Databuf.JointsQuat[6].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[6], motion_data->QuatData[selectedFrame].JointsQuat[6], 0.5); if (isnan(Databuf.JointsQuat[6].mData[3])) Databuf.JointsQuat[6] = motion_data->QuatData[selectedFrame-1].JointsQuat[6];
		Databuf.JointsQuat[7] = Databuf.JointsQuat[7].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[7], motion_data->QuatData[selectedFrame].JointsQuat[7], 0.5); if (isnan(Databuf.JointsQuat[7].mData[3])) Databuf.JointsQuat[7] = motion_data->QuatData[selectedFrame-1].JointsQuat[7];
		Databuf.JointsQuat[8] = Databuf.JointsQuat[8].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[8], motion_data->QuatData[selectedFrame].JointsQuat[8], 0.5); if (isnan(Databuf.JointsQuat[8].mData[3])) Databuf.JointsQuat[8] = motion_data->QuatData[selectedFrame-1].JointsQuat[8];
		Databuf.JointsQuat[9] = Databuf.JointsQuat[9].SLERP(motion_data->QuatData[selectedFrame-1].JointsQuat[9], motion_data->QuatData[selectedFrame].JointsQuat[9], 0.5); if (isnan(Databuf.JointsQuat[9].mData[3])) Databuf.JointsQuat[9] = motion_data->QuatData[selectedFrame-1].JointsQuat[9];
			
		motion_data->maxframe++;

		for (int i = motion_data->maxframe; i >= selectedFrame; i--)
		{
			motion_data->QuatData[i] = motion_data->QuatData[i - 1];
		}
		motion_data->QuatData[selectedFrame] = Databuf;

		
	}
	void EditMidFrame(int selectedFrame, GenData *&motion_data)
	{
		motion_data->maxframe++;

		for (int i = motion_data->maxframe; i >= selectedFrame; i--)
		{
			motion_data->QuatData[i] = motion_data->QuatData[i - 1];
		}
		motion_data->QuatData[selectedFrame] = motion_data->QuatData[selectedFrame+1];

		JointQuat Databuf;


		quaternion first_q2[2];


		first_q2[0] = motion_data->QuatData[0].JointsQuat[1];
		first_q2[1] = motion_data->QuatData[0].JointsQuat[2];

		quaternion q2[2];
		quaternion q2_end[2];

		q2[0] = motion_data->QuatData[selectedFrame-1].JointsQuat[1];
		q2[1] = motion_data->QuatData[selectedFrame-1].JointsQuat[2];

		q2_end[0] = motion_data->QuatData[selectedFrame+2 ].JointsQuat[1];
		q2_end[1] = motion_data->QuatData[selectedFrame+2 ].JointsQuat[2];

		// result.b0 = mySu->avatarData[index].b0.mutiplication(mySu->avatarData[0].b0.Inverse());
		//buf = quaternion_distance(q1.mutiplication(first_q1.Inverse()), q2.mutiplication(first_q2.Inverse()));

		//Init_q = buf[1].Inverse().mutiplication(buf[2]);


		//buf[i] = this->QuatData[Prev_frame].JointsQuat[i].mutiplication(firstquat[i]);
		quaternion buf_q[2];
		quaternion Hieq[2];
		buf_q[0] = q2[0].mutiplication(first_q2[0].Inverse());
		buf_q[1] = q2[1].mutiplication(first_q2[1].Inverse());


		Hieq[0] = buf_q[0].Inverse().mutiplication(buf_q[1]);

		buf_q[0] = q2_end[0].mutiplication(first_q2[0].Inverse());
		buf_q[1] = q2_end[1].mutiplication(first_q2[1].Inverse());

		Hieq[1] = buf_q[0].Inverse().mutiplication(buf_q[1]);

		TVec3 init_vec = gst::quaternion_to_trajectory(Hieq[0]);
		TVec3 end_vec = gst::quaternion_to_trajectory(Hieq[1]);
		
		TVec3 result_aixs = gst::vector_minus(end_vec, init_vec);


		///////////////////////////////////////////// 위치 조정


		/*quaternion first_q2[2];
		quaternion q2[2];
		quaternion buf_q[2];
		quaternion Hieq[2];*/
		first_q2[0] = motion_data->QuatData[0].JointsQuat[1];
		first_q2[1] = motion_data->QuatData[0].JointsQuat[2];

		
		

		q2[0] = motion_data->QuatData[selectedFrame].JointsQuat[1];
		q2[1] = motion_data->QuatData[selectedFrame].JointsQuat[2];
		
		buf_q[0] = q2[0].mutiplication(first_q2[0].Inverse());
		buf_q[1] = q2[1].mutiplication(first_q2[1].Inverse());


		Hieq[0] = buf_q[0].Inverse().mutiplication(buf_q[1]);

		TVec3 edit_trj;
		quaternion calQuat;
		TVec3 cal_axis;
		double cal_angle;
		TVec3 NewVector;
		edit_trj = gst::quaternion_to_trajectory(Hieq[0]);

		vector<double> edti_bf;
		edti_bf = gst::vec_Theta(edit_trj);

		//vector<double> edit_angle;
		//edit_angle.push_back(edti_bf[0] - 5);
		//edit_angle.push_back(edti_bf[1] +10);
		//double vertical_ang = 10;
		//double hori_ang = 20;

		//;

		//NewVector = gst::Ang_deg_Vec(edit_angle);
		//cal_axis = gst::vector_cross(edit_trj, NewVector);
		/*cout << " vector check : " << NewVector._x << " ," << NewVector._y << " ," << NewVector._z << endl;

		cout << " vector// check : " << cal_axis._x << " ," << cal_axis._y << " ," << cal_axis._z << endl;*/
		vector<double> edit_angle_ck;
		edit_angle_ck.push_back(edti_bf[0]+10);
		edit_angle_ck.push_back(edti_bf[1]-5);
		NewVector = gst::Ang_deg_Vec(edit_angle_ck);
		
		cal_axis = gst::vector_cross( edit_trj, NewVector);
		cal_angle = gst::vector_dot(edit_trj, NewVector);
		cout << " vector// check : " << edit_trj._x << " ," << edit_trj._y << " ," << edit_trj._z << endl;
		cout << " vector check : " << result_aixs._x << " ," << result_aixs._y << " ," << result_aixs._z << endl;
		cout << " vector check : " << cal_axis._x << " ," << cal_axis._y << " ," << cal_axis._z << endl;
		cout << " vector angle : " << edti_bf[0] <<"  , "<< edti_bf[1] << endl;

		cout << " vector angle : " << edit_angle_ck[0] << "  , " << edit_angle_ck[1] << endl;
		///////////////////////////////////////////// 위치 조정
		

		quaternion cal_quat;
		//cout << " vector// check : " << cal_axis._x << " ," << cal_axis._y << " ," << cal_axis._z << "  init " << result_aixs._x << " ," << result_aixs._y << " ," << result_aixs._z << endl;
		//cal_quat = gst::AxisagltoQuat(result_aixs,-10*PI/180);
		cal_quat = gst::rnd_mix_quat(-3,-3);
		//cal_quat = gst::AxisagltoQuat(cal_axis, cal_angle);
		motion_data->QuatData[selectedFrame].JointsQuat[2] = cal_quat.mutiplication(motion_data->QuatData[selectedFrame].JointsQuat[2]);
	}

}

void JointPosition::loadMHADdata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());

	//int count = 0;
	string fileName;

	string PoseName;

	int s_num = 0;
	int t_num = 0;
	int a_num = 0;
	int joint_num = 20;
	int motionsphere_num = 10;
	for (int i = 0; i < this->MHADdatanum; i++) {
		
		
		this->MHADatacontrol.push_back(new MHADdata());
		fileName = ".\\Matdata\\a" + to_string(a_num + 1)+"_s" + to_string(s_num+ 1) +"_t"+ to_string(t_num + 1) + "_skeleton.mat";
		std::vector<double> v;
		gst::matread(fileName.c_str(), v);

		int frame_num = v.size() / (joint_num * 3);
		std::vector<std::vector<gst::Bodypos>> Body3dpos(frame_num, std::vector<gst::Bodypos>(joint_num));


		int joint = 0;
		int frame = 0;
		for (size_t i = 0; i < v.size(); i++)
		{
			Body3dpos[frame][joint].x = v[i];
			Body3dpos[frame][joint].y = v[i + 20];
			Body3dpos[frame][joint].z = v[i + 40];

			joint++;
			if (joint == 20)
			{
				i = i + 40;
				joint = 0;
				frame++;
			}

		}

		std::vector<std::vector<gst::Bodypos>> MotionsphereD(frame_num, std::vector<gst::Bodypos>(motionsphere_num));

		for (size_t i = 0; i < frame_num; i++)
		{
			//                                   upper				lower
			
			MotionsphereD[i][0] = gst::vector_minus(Body3dpos[i][3], Body3dpos[i][2]);
			MotionsphereD[i][1] = gst::vector_minus(Body3dpos[i][2], Body3dpos[i][1]);

			MotionsphereD[i][2] = gst::vector_minus(Body3dpos[i][8], Body3dpos[i][9]);
			MotionsphereD[i][3] = gst::vector_minus(Body3dpos[i][9], Body3dpos[i][10]);
			MotionsphereD[i][4] = gst::vector_minus(Body3dpos[i][4], Body3dpos[i][5]);
			MotionsphereD[i][5] = gst::vector_minus(Body3dpos[i][5], Body3dpos[i][6]);

			MotionsphereD[i][6] = gst::vector_minus(Body3dpos[i][16], Body3dpos[i][17]);
			MotionsphereD[i][7] = gst::vector_minus(Body3dpos[i][17], Body3dpos[i][18]);
			MotionsphereD[i][8] = gst::vector_minus(Body3dpos[i][12], Body3dpos[i][13]);
			MotionsphereD[i][9] = gst::vector_minus(Body3dpos[i][13], Body3dpos[i][14]);
		}
		PoseName = "a" + to_string(a_num);
		this->MHADatacontrol.back()->MotionsphereD = MotionsphereD;
		this->MHADatacontrol.back()->maxframe = frame_num;
		this->MHADatacontrol.back()->posename = PoseName;
		
		t_num++;


		//if (s_num == 3&&t_num==2)
		//{
		//	t_num++;

		//}
		if (t_num == 4)
		{
			t_num = 0;
			s_num++;
			/*if (s_num == 3)
			{
				t_num = 1;

			}*/

			if (s_num == 8)
			{
				a_num++;
				if (a_num == 12 || a_num == 18)
				{
					a_num++;

				}

				s_num = 0;

			}


		}

	}
	
}

void JointPosition::loadMHADreferdata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());

	//int count = 0;
	string fileName;

	string PoseName;                              // A          //          //13  
	vector<char> group = { 'A','A','A','A','D','D','D','A','B','H','E','B','B','A','A','B','D' ,'D'};
	//vector<char> group = { 'A','A','A','A','D','D','A','B','H','E','B','B','A','A','B','D' ,'D' ,'D'};
	int s_num = 3;//1
	int t_num = 1;
	int a_num = 0;
	//int pose_a=0;
	int joint_num = 20;
	int motionsphere_num = 10;
	int actionloop_cnt = 0;
	int group_cnt = 0;
	bool secondr_refer = false;
	for (int i = 0; i < this->MHADreferdatanum; i++) {
		

		this->MHAreferDatacontrol.push_back(new MHADdata());
		fileName = ".\\Matdata\\a" + to_string(a_num + 1) + "_s" + to_string(s_num + 1) + "_t" + to_string(t_num + 1) + "_skeleton.mat";
		std::vector<double> v;
		gst::matread(fileName.c_str(), v);

		int frame_num = v.size() / (joint_num * 3);
		std::vector<std::vector<gst::Bodypos>> Body3dpos(frame_num, std::vector<gst::Bodypos>(joint_num));


		int joint = 0;
		int frame = 0;
		for (size_t i = 0; i < v.size(); i++)
		{
			Body3dpos[frame][joint].x = v[i];
			Body3dpos[frame][joint].y = v[i + 20];
			Body3dpos[frame][joint].z = v[i + 40];

			joint++;
			if (joint == 20)
			{
				i = i + 40;
				joint = 0;
				frame++;
			}

		}

		std::vector<std::vector<gst::Bodypos>> MotionsphereD(frame_num, std::vector<gst::Bodypos>(motionsphere_num));

		for (size_t i = 0; i < frame_num; i++)
		{
			//                                   upper				lower

			MotionsphereD[i][0] = gst::vector_minus(Body3dpos[i][3], Body3dpos[i][2]);
			MotionsphereD[i][1] = gst::vector_minus(Body3dpos[i][2], Body3dpos[i][1]);

			MotionsphereD[i][2] = gst::vector_minus(Body3dpos[i][8], Body3dpos[i][9]);
			MotionsphereD[i][3] = gst::vector_minus(Body3dpos[i][9], Body3dpos[i][10]);
			MotionsphereD[i][4] = gst::vector_minus(Body3dpos[i][4], Body3dpos[i][5]);
			MotionsphereD[i][5] = gst::vector_minus(Body3dpos[i][5], Body3dpos[i][6]);

			MotionsphereD[i][6] = gst::vector_minus(Body3dpos[i][16], Body3dpos[i][17]);
			MotionsphereD[i][7] = gst::vector_minus(Body3dpos[i][17], Body3dpos[i][18]);
			MotionsphereD[i][8] = gst::vector_minus(Body3dpos[i][12], Body3dpos[i][13]);
			MotionsphereD[i][9] = gst::vector_minus(Body3dpos[i][13], Body3dpos[i][14]);
		}
		this->MHAreferDatacontrol.back()->MotionsphereD = MotionsphereD;
		this->MHAreferDatacontrol.back()->maxframe = frame_num;

		PoseName = "a" + to_string(a_num);
		//cout << PoseName<< endl;
		this->MHAreferDatacontrol.back()->posename = PoseName;
		
		a_num++;
		if (a_num == 12|| a_num == 18)
		{
			a_num++;

		}
		if (a_num == this->action_number+2|| secondr_refer)
		{
			switch (group[group_cnt])
			{
			case 'A':
				s_num = 0;//3
				t_num = 2;
				cout << this->group[group_cnt] << endl;
				break;
			case 'B':
				s_num = 4;//3
				t_num = 0;
				cout << this->group[group_cnt] << endl;
				break;
			case 'D':
				s_num = 2;//3
				t_num = 0;
				cout << this->group[group_cnt] << endl;
				break;
			case 'C':
				s_num = 6;//3
				t_num = 3;
				cout << this->group[group_cnt] << endl;
				break;
			case 'E':
				s_num = 1;//3 0-1
				t_num = 3;
				cout << this->group[group_cnt] << endl;
				break;
			case 'F':
				s_num = 4;//3
				t_num = 1;
				cout << this->group[group_cnt] << endl;
				break;
			case 'H':
				s_num = 1;//3  1-3
				t_num = 3;
				cout << this->group[group_cnt] << endl;
				break;
		
			}

			if (!secondr_refer)
			{
				a_num = 0;
			}
			secondr_refer = true;
			group_cnt++;
			//cout << this->group[group_cnt] << endl;
		}
		actionloop_cnt++;
	}
	cout << actionloop_cnt++ << endl;
}

 //Refer load ft Back Up
//void JointPosition::loadMHADreferdata()
//{
//	//DynamicData *newNode = new DynamicData();
//	//this->Dynamicdatacontrol.push_back(new DynamicData());
//
//	//int count = 0;
//	string fileName;
//
//	string PoseName;
//
//
//	int s_num = 4;//1
//	int t_num = 0;
//	int a_num = 0;
//	int joint_num = 20;
//	int motionsphere_num = 10;
//	for (int i = 0; i < this->MHADreferdatanum; i++) {
//
//
//		this->MHAreferDatacontrol.push_back(new MHADdata());
//		fileName = ".\\Matdata\\a" + to_string(a_num + 1) + "_s" + to_string(s_num + 1) + "_t" + to_string(t_num + 1) + "_skeleton.mat";
//		std::vector<double> v;
//		gst::matread(fileName.c_str(), v);
//
//		int frame_num = v.size() / (joint_num * 3);
//		std::vector<std::vector<gst::Bodypos>> Body3dpos(frame_num, std::vector<gst::Bodypos>(joint_num));
//
//
//		int joint = 0;
//		int frame = 0;
//		for (size_t i = 0; i < v.size(); i++)
//		{
//			Body3dpos[frame][joint].x = v[i];
//			Body3dpos[frame][joint].y = v[i + 20];
//			Body3dpos[frame][joint].z = v[i + 40];
//
//			joint++;
//			if (joint == 20)
//			{
//				i = i + 40;
//				joint = 0;
//				frame++;
//			}
//
//		}
//
//		std::vector<std::vector<gst::Bodypos>> MotionsphereD(frame_num, std::vector<gst::Bodypos>(motionsphere_num));
//
//		for (size_t i = 0; i < frame_num; i++)
//		{
//			//                                   upper				lower
//
//			MotionsphereD[i][0] = gst::vector_minus(Body3dpos[i][3], Body3dpos[i][2]);
//			MotionsphereD[i][1] = gst::vector_minus(Body3dpos[i][2], Body3dpos[i][1]);
//
//			MotionsphereD[i][2] = gst::vector_minus(Body3dpos[i][8], Body3dpos[i][9]);
//			MotionsphereD[i][3] = gst::vector_minus(Body3dpos[i][9], Body3dpos[i][10]);
//			MotionsphereD[i][4] = gst::vector_minus(Body3dpos[i][4], Body3dpos[i][5]);
//			MotionsphereD[i][5] = gst::vector_minus(Body3dpos[i][5], Body3dpos[i][6]);
//
//			MotionsphereD[i][6] = gst::vector_minus(Body3dpos[i][16], Body3dpos[i][17]);
//			MotionsphereD[i][7] = gst::vector_minus(Body3dpos[i][17], Body3dpos[i][18]);
//			MotionsphereD[i][8] = gst::vector_minus(Body3dpos[i][12], Body3dpos[i][13]);
//			MotionsphereD[i][9] = gst::vector_minus(Body3dpos[i][13], Body3dpos[i][14]);
//		}
//		this->MHAreferDatacontrol.back()->MotionsphereD = MotionsphereD;
//		this->MHAreferDatacontrol.back()->maxframe = frame_num;
//
//		PoseName = "a" + to_string(a_num);
//		//cout << PoseName<< endl;
//		this->MHAreferDatacontrol.back()->posename = PoseName;
//
//		a_num++;
//		if (a_num == 12 || a_num == 18)
//		{
//			a_num++;
//
//		}
//		if (a_num == this->action_number+2)
//		{
//			t_num = 1;
//			a_num = 0;
//			s_num = 3;//3
//			//cout <<"achange" <<endl;
//		}
//	}
//
//}

void JointPosition::trainingReferData()
{
	int motionsphere_num = 10;
	int action_num = this->action_number;

	for (int iter = 0; iter < action_num; iter++) // action 0-27
	{
		int max_idx, min_idx;
		int ref2 = this->MHAreferDatacontrol[iter]->maxframe;
		int ref1 = this->MHAreferDatacontrol[iter+action_num]->maxframe;

		
		if (ref1 > ref2)
		{
			max_idx = iter + action_num; min_idx = iter;
		}
		else
		{
			max_idx = iter; min_idx = iter + action_num;

		}
		cout << ref1 << "  ,  " << ref2 << endl;
		int max_frame = this->MHAreferDatacontrol[max_idx]->maxframe;
		int min_frame = this->MHAreferDatacontrol[min_idx]->maxframe;
		std::vector<std::vector<gst::Bodypos>> trainingDatabuf(max_frame, std::vector<gst::Bodypos>(motionsphere_num));
		std::vector<std::vector<gst::Bodypos>> minDatabuf(max_frame, std::vector<gst::Bodypos>(motionsphere_num));
		cout << ref1 << "  ,  " << ref2 << endl;
		//minDatabuf = this->MHAreferDatacontrol[min_idx]->MotionsphereD;
		
		//vector<int> result(max_frame - min_frame);
		int gap = max_frame - min_frame;
		int stride = min_frame / (gap+1);
		cout << ref1 << "  ,  " << ref2 << endl;
		cout << gap <<endl;
		//cout << "check" << endl;
		for (int i = 0, j = 0,jump=stride; i < max_frame; i++)
		{
			if (jump == 0&&gap!=0)
			{
				minDatabuf[i][0] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][0], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][0]);
				minDatabuf[i][1] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][1], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][1]);
				minDatabuf[i][2] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][2], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][2]);
				minDatabuf[i][3] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][3], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][3]);
				minDatabuf[i][4] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][4], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][4]);
				minDatabuf[i][5] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][5], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][5]);
				minDatabuf[i][6] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][6], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][6]);
				minDatabuf[i][7] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][7], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][7]);
				minDatabuf[i][8] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][8], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][8]);
				minDatabuf[i][9] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][9], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j-1][9]);
				gap--;
				jump == stride;

			}
			else {
				minDatabuf[i][0] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][0];
				minDatabuf[i][1] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][1];
				minDatabuf[i][2] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][2];
				minDatabuf[i][3] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][3];
				minDatabuf[i][4] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][4];
				minDatabuf[i][5] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][5];
				minDatabuf[i][6] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][6];
				minDatabuf[i][7] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][7];
				minDatabuf[i][8] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][8];
				minDatabuf[i][9] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][9];
				j++;

				jump--;
			}
		}
	

			for (size_t i = 0; i < max_frame; i++)
			{
				//                                   upper				lower

				trainingDatabuf[i][0] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][0], minDatabuf[i][0]);
				trainingDatabuf[i][1] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][1], minDatabuf[i][1]);
				trainingDatabuf[i][2] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][2], minDatabuf[i][2]);
				trainingDatabuf[i][3] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][3], minDatabuf[i][3]);
				trainingDatabuf[i][4] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][4], minDatabuf[i][4]);
				trainingDatabuf[i][5] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][5], minDatabuf[i][5]);
				trainingDatabuf[i][6] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][6], minDatabuf[i][6]);
				trainingDatabuf[i][7] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][7], minDatabuf[i][7]);
				trainingDatabuf[i][8] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][8], minDatabuf[i][8]);
				trainingDatabuf[i][9] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][9], minDatabuf[i][9]);
			}											   


			this->MHAreferDatacontrol[iter]->trainingData = trainingDatabuf;

			this->MHAreferDatacontrol[iter]->training_data_frame = max_frame;


	}
	// train + 3rd refer mix

	//cout << "3rd chedk"<<endl;
	//for (int iter = 0; iter < action_num; iter++) // action 0-27
	//{
	//	int max_idx, min_idx;
	//	int max_frame;
	//	int min_frame;
	//	if (this->MHAreferDatacontrol[iter]->training_data_frame > this->MHAreferDatacontrol[iter + action_num*2]->maxframe)
	//	{
	//		max_frame = this->MHAreferDatacontrol[iter]->training_data_frame;
	//		min_frame = this->MHAreferDatacontrol[iter + action_num * 2]->maxframe;
	//	}
	//	else
	//	{
	//		max_frame = this->MHAreferDatacontrol[iter + action_num * 2]->maxframe;
	//		min_frame = this->MHAreferDatacontrol[iter]->training_data_frame;
	//		

	//	}
	//	
	//	
	//	std::vector<std::vector<gst::Bodypos>> trainingDatabuf(max_frame, std::vector<gst::Bodypos>(motionsphere_num));
	//	std::vector<std::vector<gst::Bodypos>> minDatabuf(max_frame, std::vector<gst::Bodypos>(motionsphere_num));

	//	std::vector<std::vector<gst::Bodypos>> minMHAD(min_frame, std::vector<gst::Bodypos>(motionsphere_num));
	//	std::vector<std::vector<gst::Bodypos>> maxMHAD(max_frame, std::vector<gst::Bodypos>(motionsphere_num));


	//	if (min_frame == this->MHAreferDatacontrol[iter]->training_data_frame)
	//	{
	//		minMHAD = this->MHAreferDatacontrol[iter]->trainingData;
	//		minMHAD=this->MHAreferDatacontrol[iter + action_num * 2]->MotionsphereD;
	//	}
	//	else
	//	{
	//		minMHAD = this->MHAreferDatacontrol[iter + action_num * 2]->MotionsphereD;
	//		maxMHAD = this->MHAreferDatacontrol[iter]->trainingData;
	//	}
	//	//minDatabuf = this->MHAreferDatacontrol[min_idx]->MotionsphereD;

	//	vector<int> result(max_frame - min_frame);
	//	int gap = max_frame - min_frame;
	//	int stride = min_frame / (gap + 1);

	//	//cout << "check" << endl;
	//	for (int i = 0, j = 0, jump = stride; i < max_frame; i++)
	//	{
	//		if (jump == 0 && gap != 0)
	//		{
	//			minDatabuf[i][0] = gst::vector_middle(minMHAD[j][0], minMHAD[j-1][0]);
	//			minDatabuf[i][1] = gst::vector_middle(minMHAD[j][1], minMHAD[j-1][1]);
	//			minDatabuf[i][2] = gst::vector_middle(minMHAD[j][2], minMHAD[j-1][2]);
	//			minDatabuf[i][3] = gst::vector_middle(minMHAD[j][3], minMHAD[j-1][3]);
	//			minDatabuf[i][4] = gst::vector_middle(minMHAD[j][4], minMHAD[j-1][4]);
	//			minDatabuf[i][5] = gst::vector_middle(minMHAD[j][5], minMHAD[j-1][5]);
	//			minDatabuf[i][6] = gst::vector_middle(minMHAD[j][6], minMHAD[j-1][6]);
	//			minDatabuf[i][7] = gst::vector_middle(minMHAD[j][7], minMHAD[j-1][7]);
	//			minDatabuf[i][8] = gst::vector_middle(minMHAD[j][8], minMHAD[j-1][8]);
	//			minDatabuf[i][9] = gst::vector_middle(minMHAD[j][9], minMHAD[j-1][9]);
	//			gap--;
	//			jump == stride;

	//		}
	//		else {
	//			minDatabuf[i][0] = minMHAD[j][0];
	//			minDatabuf[i][1] = minMHAD[j][1];
	//			minDatabuf[i][2] = minMHAD[j][2];
	//			minDatabuf[i][3] = minMHAD[j][3];
	//			minDatabuf[i][4] = minMHAD[j][4];
	//			minDatabuf[i][5] = minMHAD[j][5];
	//			minDatabuf[i][6] = minMHAD[j][6];
	//			minDatabuf[i][7] = minMHAD[j][7];
	//			minDatabuf[i][8] = minMHAD[j][8];
	//			minDatabuf[i][9] = minMHAD[j][9];
	//			j++;

	//			jump--;
	//		}
	//	}


	//	for (size_t i = 0; i < max_frame; i++)
	//	{
	//		//                                   upper				lower

	//		trainingDatabuf[i][0] = gst::vector_middle(maxMHAD[i][0], minDatabuf[i][0]);
	//		trainingDatabuf[i][1] = gst::vector_middle(maxMHAD[i][1], minDatabuf[i][1]);
	//		trainingDatabuf[i][2] = gst::vector_middle(maxMHAD[i][2], minDatabuf[i][2]);
	//		trainingDatabuf[i][3] = gst::vector_middle(maxMHAD[i][3], minDatabuf[i][3]);
	//		trainingDatabuf[i][4] = gst::vector_middle(maxMHAD[i][4], minDatabuf[i][4]);
	//		trainingDatabuf[i][5] = gst::vector_middle(maxMHAD[i][5], minDatabuf[i][5]);
	//		trainingDatabuf[i][6] = gst::vector_middle(maxMHAD[i][6], minDatabuf[i][6]);
	//		trainingDatabuf[i][7] = gst::vector_middle(maxMHAD[i][7], minDatabuf[i][7]);
	//		trainingDatabuf[i][8] = gst::vector_middle(maxMHAD[i][8], minDatabuf[i][8]);
	//		trainingDatabuf[i][9] = gst::vector_middle(maxMHAD[i][9], minDatabuf[i][9]);
	//	}


	//	this->MHAreferDatacontrol[iter]->trainingData = trainingDatabuf;

	//	this->MHAreferDatacontrol[iter]->training_data_frame = max_frame;


	//}
	//cout << "3rd chedk" << endl;
}



// 2021.01.20 Test


void JointPosition::Calvecangle()
{
	
	int action_num = this->MHADdatanum;
	//vector<gst::Bodypos> UpperVector;
	//vector<gst::Bodypos> LowerVector;
	//vector_minus_t
	for (int iter = 0; iter < action_num; iter++) // action 0-27
	{

		int max_frame = this->MHADatacontrol[iter]->maxframe;

		//cout << "check" << endl;
		for (int i = 0; i < max_frame - 1; i++)
		{

			/*UpperVector.push_back(vector_minus_t(this->MHAreferDatacontrol[iter]->MotionsphereD[i][2],
				this->MHAreferDatacontrol[iter]->MotionsphereD[i+1][2]));

			LowerVector.push_back(vector_minus_t(this->MHAreferDatacontrol[iter]->MotionsphereD[i][3],
				this->MHAreferDatacontrol[iter]->MotionsphereD[i + 1][3]));*/

			this->MHADatacontrol[iter]->testAngle.push_back(vector_Angle(this->MHADatacontrol[iter]->MotionsphereD[i][2], this->MHADatacontrol[iter]->MotionsphereD[i][3]));
			this->MHADatacontrol[iter]->testAngle2.push_back(vector_Angle(this->MHADatacontrol[iter]->MotionsphereD[i][1], this->MHADatacontrol[iter]->MotionsphereD[i][2]));
			this->MHADatacontrol[iter]->testAxis.push_back(vector_cross(this->MHADatacontrol[iter]->MotionsphereD[i][2], this->MHADatacontrol[iter]->MotionsphereD[i][4]));
			this->MHADatacontrol[iter]->testAxis2.push_back(vector_cross(this->MHADatacontrol[iter]->MotionsphereD[i][0], this->MHADatacontrol[iter]->MotionsphereD[i][2]));
			this->MHADatacontrol[iter]->testAxis3.push_back(vector_cross(this->MHADatacontrol[iter]->MotionsphereD[i][0], this->MHADatacontrol[iter]->MotionsphereD[i][3]));
			this->MHADatacontrol[iter]->testAxis4.push_back(vector_cross(this->MHADatacontrol[iter]->MotionsphereD[i][6], this->MHADatacontrol[iter]->MotionsphereD[i][8]));
			//this->MHADatacontrol[iter]->testAngle2.push_back(vector_Angle(this->MHADatacontrol[iter]->MotionsphereD[i][2], this->MHADatacontrol[iter]->MotionsphereD[i][4]));
			//this->MHADatacontrol[iter]->testAngle3.push_back(vector_Angle(this->MHADatacontrol[iter]->MotionsphereD[i][1], this->MHADatacontrol[iter]->MotionsphereD[i][4]));

			//this->MHADatacontrol[iter]->testAngle4.push_back(vector_Angle(this->MHADatacontrol[iter]->MotionsphereD[i][1], this->MHADatacontrol[iter]->MotionsphereD[i][2]));

			//this->MHADatacontrol[iter]->testAngle5.push_back(vector_Angle(this->MHADatacontrol[iter]->MotionsphereD[i][6], this->MHADatacontrol[iter]->MotionsphereD[i][8]));
			/*this->MHADatacontrol[iter]->testAngle.push_back(vector_Angle(vector_minus_t(this->MHADatacontrol[iter]->MotionsphereD[i][2],
				this->MHADatacontrol[iter]->MotionsphereD[i + 1][2]), vector_minus_t(this->MHADatacontrol[iter]->MotionsphereD[i][4],
					this->MHADatacontrol[iter]->MotionsphereD[i + 1][4])));*/
		}


	}

}

void JointPosition::referCalvecangle()
{
	
	int action_num = this->action_number;
	//vector<gst::Bodypos> UpperVector;
	//vector<gst::Bodypos> LowerVector;
	//vector_minus_t
	for (int iter = 0; iter < action_num; iter++) // action 0-27
	{
	
		int max_frame = this->MHAreferDatacontrol[iter]->training_data_frame;
		
		//cout << "check" << endl;
		for (int i = 0 ; i < max_frame-1; i++)
		{
			
			/*UpperVector.push_back(vector_minus_t(this->MHAreferDatacontrol[iter]->MotionsphereD[i][2],
				this->MHAreferDatacontrol[iter]->MotionsphereD[i+1][2]));
				
			LowerVector.push_back(vector_minus_t(this->MHAreferDatacontrol[iter]->MotionsphereD[i][3],
				this->MHAreferDatacontrol[iter]->MotionsphereD[i + 1][3]));*/
			this->MHAreferDatacontrol[iter]->testAngle.push_back(vector_Angle(this->MHAreferDatacontrol[iter]->trainingData[i][2], this->MHAreferDatacontrol[iter]->trainingData[i][3]));
			this->MHAreferDatacontrol[iter]->testAngle2.push_back(vector_Angle(this->MHAreferDatacontrol[iter]->trainingData[i][1], this->MHAreferDatacontrol[iter]->trainingData[i][2]));
			this->MHAreferDatacontrol[iter]->testAxis.push_back(vector_cross(this->MHAreferDatacontrol[iter]->trainingData[i][2], this->MHAreferDatacontrol[iter]->trainingData[i][4]));
			this->MHAreferDatacontrol[iter]->testAxis2.push_back(vector_cross(this->MHAreferDatacontrol[iter]->trainingData[i][0], this->MHAreferDatacontrol[iter]->trainingData[i][2]));
			this->MHAreferDatacontrol[iter]->testAxis3.push_back(vector_cross(this->MHAreferDatacontrol[iter]->trainingData[i][0], this->MHAreferDatacontrol[iter]->trainingData[i][3]));
			this->MHAreferDatacontrol[iter]->testAxis4.push_back(vector_cross(this->MHAreferDatacontrol[iter]->trainingData[i][6], this->MHAreferDatacontrol[iter]->trainingData[i][8]));

			/*this->MHAreferDatacontrol[iter]->testAxis.push_back(vector_cross(this->MHAreferDatacontrol[iter]->trainingData[i][2], this->MHAreferDatacontrol[iter]->MotionsphereD[i][4]));
			this->MHAreferDatacontrol[iter]->testAxis2.push_back(vector_cross(this->MHAreferDatacontrol[iter]->MotionsphereD[i][0], this->MHAreferDatacontrol[iter]->MotionsphereD[i][4]));
			this->MHAreferDatacontrol[iter]->testAxis3.push_back(vector_cross(this->MHAreferDatacontrol[iter]->MotionsphereD[i][2], this->MHAreferDatacontrol[iter]->MotionsphereD[i][3]));
			this->MHAreferDatacontrol[iter]->testAxis4.push_back(vector_cross(this->MHAreferDatacontrol[iter]->MotionsphereD[i][0], this->MHAreferDatacontrol[iter]->MotionsphereD[i][2]));
*/


			//this->MHAreferDatacontrol[iter]->testAngle2.push_back(vector_Angle(this->MHAreferDatacontrol[iter]->MotionsphereD[i][2], this->MHAreferDatacontrol[iter]->MotionsphereD[i][4]));
			//this->MHAreferDatacontrol[iter]->testAngle3.push_back(vector_Angle(this->MHAreferDatacontrol[iter]->MotionsphereD[i][1], this->MHAreferDatacontrol[iter]->MotionsphereD[i][4]));

			//this->MHAreferDatacontrol[iter]->testAngle4.push_back(vector_Angle(this->MHAreferDatacontrol[iter]->MotionsphereD[i][1], this->MHAreferDatacontrol[iter]->MotionsphereD[i][2]));

			//this->MHAreferDatacontrol[iter]->testAngle5.push_back(vector_Angle(this->MHAreferDatacontrol[iter]->MotionsphereD[i][6], this->MHAreferDatacontrol[iter]->MotionsphereD[i][8]));
			/*this->MHAreferDatacontrol[iter]->testAngle.push_back (vector_Angle(vector_minus_t(this->MHAreferDatacontrol[iter]->MotionsphereD[i][2],
				this->MHAreferDatacontrol[iter]->MotionsphereD[i + 1][2]),vector_minus_t(this->MHAreferDatacontrol[iter]->MotionsphereD[i][4],
				this->MHAreferDatacontrol[iter]->MotionsphereD[i + 1][4])));*/
		}


	}

}
// training ft Back Up
//void JointPosition::trainingReferData()
//{
//	int motionsphere_num = 10;
//	int action_num = this->action_number;
//
//	for (int iter = 0; iter < action_num; iter++) // action 0-27
//	{
//		int max_idx, min_idx;
//		if (this->MHAreferDatacontrol[iter]->maxframe > this->MHAreferDatacontrol[iter + action_num]->maxframe)
//		{
//			max_idx = iter; min_idx = iter + action_num;
//		}
//		else
//		{
//			max_idx = iter + action_num; min_idx = iter;
//
//		}
//
//		int max_frame = this->MHAreferDatacontrol[max_idx]->maxframe;
//		int min_frame = this->MHAreferDatacontrol[min_idx]->maxframe;
//		std::vector<std::vector<gst::Bodypos>> trainingDatabuf(max_frame, std::vector<gst::Bodypos>(motionsphere_num));
//		std::vector<std::vector<gst::Bodypos>> minDatabuf(max_frame, std::vector<gst::Bodypos>(motionsphere_num));
//
//		//minDatabuf = this->MHAreferDatacontrol[min_idx]->MotionsphereD;
//
//		vector<int> result(max_frame - min_frame);
//		int gap = max_frame - min_frame;
//		int stride = min_frame / (gap + 1);
//
//		//cout << "check" << endl;
//		for (int i = 0, j = 0, jump = stride; i < max_frame; i++)
//		{
//			if (jump == 0 && gap != 0)
//			{
//				minDatabuf[i][0] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][0], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][0]);
//				minDatabuf[i][1] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][1], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][1]);
//				minDatabuf[i][2] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][2], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][2]);
//				minDatabuf[i][3] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][3], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][3]);
//				minDatabuf[i][4] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][4], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][4]);
//				minDatabuf[i][5] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][5], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][5]);
//				minDatabuf[i][6] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][6], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][6]);
//				minDatabuf[i][7] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][7], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][7]);
//				minDatabuf[i][8] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][8], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][8]);
//				minDatabuf[i][9] = gst::vector_middle(this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][9], this->MHAreferDatacontrol[min_idx]->MotionsphereD[j - 1][9]);
//				gap--;
//				jump == stride;
//
//			}
//			else {
//				minDatabuf[i][0] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][0];
//				minDatabuf[i][1] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][1];
//				minDatabuf[i][2] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][2];
//				minDatabuf[i][3] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][3];
//				minDatabuf[i][4] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][4];
//				minDatabuf[i][5] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][5];
//				minDatabuf[i][6] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][6];
//				minDatabuf[i][7] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][7];
//				minDatabuf[i][8] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][8];
//				minDatabuf[i][9] = this->MHAreferDatacontrol[min_idx]->MotionsphereD[j][9];
//				j++;
//
//				jump--;
//			}
//		}
//
//
//		for (size_t i = 0; i < max_frame; i++)
//		{
//			//                                   upper				lower
//
//			trainingDatabuf[i][0] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][0], minDatabuf[i][0]);
//			trainingDatabuf[i][1] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][1], minDatabuf[i][1]);
//			trainingDatabuf[i][2] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][2], minDatabuf[i][2]);
//			trainingDatabuf[i][3] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][3], minDatabuf[i][3]);
//			trainingDatabuf[i][4] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][4], minDatabuf[i][4]);
//			trainingDatabuf[i][5] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][5], minDatabuf[i][5]);
//			trainingDatabuf[i][6] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][6], minDatabuf[i][6]);
//			trainingDatabuf[i][7] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][7], minDatabuf[i][7]);
//			trainingDatabuf[i][8] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][8], minDatabuf[i][8]);
//			trainingDatabuf[i][9] = gst::vector_middle(this->MHAreferDatacontrol[max_idx]->MotionsphereD[i][9], minDatabuf[i][9]);
//		}
//
//
//		this->MHAreferDatacontrol[iter]->trainingData = trainingDatabuf;
//
//		this->MHAreferDatacontrol[iter]->training_data_frame = max_frame;
//
//
//	}
//
//
//}



void JointPosition::detecttestMovingjoint()
{
	int motionsphere_num = 10;
	int action_num = this->MHADdatanum;

	for (int iter = 0; iter < action_num; iter++) // action 0-27
	{
		int maxframe = this->MHADatacontrol[iter]->maxframe;


		double buf = 0;
		for (int joint = 0; joint < motionsphere_num; joint++)
		{
			double result = 0;
			for (int frame = 0; frame < maxframe - 1; frame++)
			{
				result += gst::vector_distance(this->MHADatacontrol[iter]->MotionsphereD[frame][joint], this->MHADatacontrol[iter]->MotionsphereD[frame + 1][joint]);
				//cout << result << endl;
			}
		/*	if (iter == 0)
			{
				cout << result << endl;

			}*/
			if (result > 3)
			{
				this->MHADatacontrol[iter]->movingjoint.push_back(joint);

			}
		}

		/*cout << "action : " << iter + 1 << "moving joint ";
		for (int i = 0; i < this->MHADatacontrol[iter]->movingjoint.size(); i++)
			cout << this->MHADatacontrol[iter]->movingjoint[i] << " ,";
		cout << endl;*/
		//cout << "finish" << endl;
	}



	




}

//void JointPosition::detectMovingjoint()
//{
//	int motionsphere_num = 10;
//	int action_num = this->action_number;
//
//	for (int iter = 0; iter < action_num; iter++) // action 0-27
//	{
//		int maxframe = this->MHAreferDatacontrol[iter]->training_data_frame;
//
//
//		double buf = 0;
//		for (int joint = 0; joint < motionsphere_num; joint++)
//		{
//			double result = 0;
//			for (int frame = 0; frame < maxframe - 1; frame++)
//			{
//
//
//				result += gst::vector_distance(this->MHAreferDatacontrol[iter]->trainingData[frame][joint], this->MHAreferDatacontrol[iter]->trainingData[frame + 1][joint]);
//				//cout << result << endl;
//			}
//			if (iter == 0)
//			{
//				cout << result << endl;
//
//			}
//			if (result > 2)
//			{
//				this->MHAreferDatacontrol[iter]->movingjoint.push_back(joint);
//
//			}
//		}
//
//		cout << "action : " << iter + 1 << "moving joint ";
//		for (int i = 0; i < this->MHAreferDatacontrol[iter]->movingjoint.size(); i++)
//			cout << this->MHAreferDatacontrol[iter]->movingjoint[i] << " ,";
//		cout << endl;
//
//	}
//
//
//
//
//
//
//
//
//}



void JointPosition::detectMovingjoint()
{
	int motionsphere_num = 10;
	int action_num = this->action_number;

	for (int iter = 0; iter < action_num; iter++) // action 0-27
	{
		int maxframe = this->MHAreferDatacontrol[iter]->training_data_frame;
		

		double buf = 0;
		for (int joint = 0; joint < motionsphere_num; joint++)
		{
			double result = 0;
			for (int frame = 0; frame < maxframe - 1; frame++)
			{
				result += gst::vector_distance(this->MHAreferDatacontrol[iter]->trainingData[frame][joint], this->MHAreferDatacontrol[iter]->trainingData[frame+1][joint]);
				//cout << result << endl;
			}
			if (iter == 0)
			{
				cout << result << endl;

			}
			if (result >3)
			{
				this->MHAreferDatacontrol[iter]->movingjoint.push_back(joint);

			}
		}
		
		cout << "action : " << iter + 1 << "moving joint " ;
		for (int i = 0; i < this->MHAreferDatacontrol[iter]->movingjoint.size(); i++)
			cout << this->MHAreferDatacontrol[iter]->movingjoint[i] << " ,";
		cout<< endl;

	}


}


void JointPosition::loadGeNQuatdata()
{
	//DynamicData *newNode = new DynamicData();
	//this->Dynamicdatacontrol.push_back(new DynamicData());

	//int count = 0;
	int tCount = 0;
	
	string fileName[] = {".\\GenData\\Refer\\test1.txt" ,".\\GenData\\Refer\\test2.txt",".\\GenData\\Refer\\test3.txt",".\\GenData\\Refer\\test4.txt" ,".\\GenData\\Refer\\test5.txt" };
	
	// back up
	//string fileName[]= { ".\\GenData\\Refer\\R_Straight1.txt",".\\GenData\\Refer\\R_Hook1.txt",".\\GenData\\Refer\\0.5 straight 2.txt",".\\GenData\\Refer\\SLERP_data.txt" ,".\\GenData\\Refer\\SEED_data.txt" };
	//string fileName[] = { ".\\GenData\\Refer\\Trj Range\\0.5 straight 2.txt" ,".\\GenData\\Refer\\R_Hook1.txt",".\\GenData\\Refer\\R_Hook3.txt",".\\GenData\\Refer\\SLERP_data.txt" ,".\\GenData\\Refer\\SEED_data.txt" };
	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;
	//fileName = 
	for (int i = 0; i < this->Genrefernum; i++) {
		int lineCount = 0;
		int count = 0;

		this->GenDatacontrol.push_back(new GenData());
		
		_filestream.open(fileName[i]);
		cout << "check" << fileName[i] << endl;

		while (std::getline(_filestream, _line))
		{
			std::stringstream _linestream;
			_linestream << _line;
			//cout << "gesture working" << endl;
			if (count == 0)
			{
				//cout << "check" << endl;
				_linestream >> this->GenDatacontrol.back()->poseName; 

				//cout << "check" << _line << endl;
				count++;
				continue;


			}
			if (count == 1)
			{
				_linestream >> _line >> this->GenDatacontrol.back()->maxframe; count++;

				cout << "check"<< this->GenDatacontrol.back()->maxframe << endl;

				continue;
			}

			_linestream
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[0].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[0].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[0].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[0].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[1].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[1].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[1].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[1].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[2].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[2].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[2].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[2].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[3].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[3].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[3].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[3].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[4].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[4].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[4].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[4].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[5].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[5].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[5].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[5].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[6].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[6].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[6].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[6].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[7].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[7].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[7].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[7].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[8].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[8].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[8].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[8].mData[2]
				>> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[9].mData[3] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[9].mData[0] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[9].mData[1] >> this->GenDatacontrol.back()->QuatData[lineCount].JointsQuat[9].mData[2];
			lineCount++;
		}
		_filestream.close();


		
		this->GenDatacontrol.back()->motion_idx = i;
		this->Genrefercnt++;
	}

}

void JointPosition::GeNQuatdata()
{
	//int GenNum = 50;
	float Slerp_ration = 0.5;

	int motion_idx = 2;
		
	int big_maxfrm, sm_maxfrm;
	int big_idx, sm_idx;

	if (this->GenDatacontrol[0]->maxframe > this->GenDatacontrol[motion_idx]->maxframe)
	{
		big_maxfrm = this->GenDatacontrol[0]->maxframe;
		big_idx = 0;
		sm_maxfrm = this->GenDatacontrol[motion_idx]->maxframe;
		sm_idx = 1;
	}
	else
	{
		big_maxfrm = this->GenDatacontrol[motion_idx]->maxframe;
		big_idx =1;
		sm_maxfrm = this->GenDatacontrol[0]->maxframe;
		sm_idx = 0;
	}
	vector<JointQuat> Databuf(sm_maxfrm);
	vector<int> close_idx;

	close_idx = gst::idxSort(this->GenDatacontrol[0], this->GenDatacontrol[2]);
	//cout <<"idx check  : " <<close_idx.size() << endl;
	for (int i = 0; i < sm_maxfrm; i++)
	{
		int p_frame = i;
	

		Databuf[i].JointsQuat[0] = Databuf[i].JointsQuat[0].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[0], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[0], Slerp_ration);
		Databuf[i].JointsQuat[1] = Databuf[i].JointsQuat[1].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[1], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[1], Slerp_ration);
		Databuf[i].JointsQuat[2] = Databuf[i].JointsQuat[2].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[2], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[2], Slerp_ration);
		Databuf[i].JointsQuat[3] = Databuf[i].JointsQuat[3].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[3], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[3], Slerp_ration);
		Databuf[i].JointsQuat[4] = Databuf[i].JointsQuat[4].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[4], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[4], Slerp_ration);
		Databuf[i].JointsQuat[5] = Databuf[i].JointsQuat[5].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[5], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[5], Slerp_ration);
		Databuf[i].JointsQuat[6] = Databuf[i].JointsQuat[6].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[6], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[6], Slerp_ration);
		Databuf[i].JointsQuat[7] = Databuf[i].JointsQuat[7].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[7], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[7], Slerp_ration);
		Databuf[i].JointsQuat[8] = Databuf[i].JointsQuat[8].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[8], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[8], Slerp_ration);
		Databuf[i].JointsQuat[9] = Databuf[i].JointsQuat[9].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[9], this->GenDatacontrol[motion_idx]->QuatData[close_idx[i]].JointsQuat[9], Slerp_ration);

		//cout << p_frame << endl;
	}
	

	//for (int i=0 ,j = 0; i < sm_maxfrm; i++)
	//{
	//	j = int(big_maxfrm / sm_maxfrm * i);

	//	int p_frame, n_frame;
	//	if (sm_idx == 0)
	//	{
	//		p_frame = i;
	//		n_frame = j;
	//	}
	//	else
	//	{
	//		p_frame = j;
	//		n_frame = i;

	//	}


	//	Databuf[i].JointsQuat[0] = Databuf[i].JointsQuat[0].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[0], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[0], Slerp_ration);
	//	Databuf[i].JointsQuat[1] = Databuf[i].JointsQuat[1].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[1], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[1], Slerp_ration);
	//	Databuf[i].JointsQuat[2] = Databuf[i].JointsQuat[2].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[2], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[2], Slerp_ration);
	//	Databuf[i].JointsQuat[3] = Databuf[i].JointsQuat[3].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[3], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[3], Slerp_ration);
	//	Databuf[i].JointsQuat[4] = Databuf[i].JointsQuat[4].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[4], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[4], Slerp_ration);
	//	Databuf[i].JointsQuat[5] = Databuf[i].JointsQuat[5].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[5], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[5], Slerp_ration);
	//	Databuf[i].JointsQuat[6] = Databuf[i].JointsQuat[6].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[6], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[6], Slerp_ration);
	//	Databuf[i].JointsQuat[7] = Databuf[i].JointsQuat[7].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[7], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[7], Slerp_ration);
	//	Databuf[i].JointsQuat[8] = Databuf[i].JointsQuat[8].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[8], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[8], Slerp_ration);
	//	Databuf[i].JointsQuat[9] = Databuf[i].JointsQuat[9].SLERP(this->GenDatacontrol[0]->QuatData[p_frame].JointsQuat[9], this->GenDatacontrol[motion_idx]->QuatData[n_frame].JointsQuat[9], Slerp_ration);

	//	//cout << p_frame << endl;
	//}
	// 캘리브레이션 저장
	Databuf[0].JointsQuat[0] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[0];
	Databuf[0].JointsQuat[1] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[1];
	Databuf[0].JointsQuat[2] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[2];
	Databuf[0].JointsQuat[3] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[3];
	Databuf[0].JointsQuat[4] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[4];
	Databuf[0].JointsQuat[5] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[5];
	Databuf[0].JointsQuat[6] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[6];
	Databuf[0].JointsQuat[7] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[7];
	Databuf[0].JointsQuat[8] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[8];
	Databuf[0].JointsQuat[9] = this->GenDatacontrol[0]->QuatData[0].JointsQuat[9];

	// check nan value
	for (int joint = 0; joint < 10; joint++)
	{


		for (int i = 0; i < sm_maxfrm; i++)
		{

			if (isnan(Databuf[i].JointsQuat[joint].mData[3])!=0)
			{
				Databuf[i].JointsQuat[joint] = Databuf[i - 1].JointsQuat[joint];

			}
			

			
		}

	}

	// save new data set
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/GenerateSLERPData-%dmotion-ratio%f.txt",motion_idx,Slerp_ration);
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	//avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << maxFrame << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];

	cout << "pp" << sm_maxfrm << endl;
	for (int tCount = 0; tCount < sm_maxfrm; tCount++)
	{

		avatarDataFile
			<< Databuf[tCount].JointsQuat[0].mData[3] << "\t" << Databuf[tCount].JointsQuat[0].mData[0] << "\t" << Databuf[tCount].JointsQuat[0].mData[1] << "\t" << Databuf[tCount].JointsQuat[0].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[1].mData[3] << "\t" << Databuf[tCount].JointsQuat[1].mData[0] << "\t" << Databuf[tCount].JointsQuat[1].mData[1] << "\t" << Databuf[tCount].JointsQuat[1].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[2].mData[3] << "\t" << Databuf[tCount].JointsQuat[2].mData[0] << "\t" << Databuf[tCount].JointsQuat[2].mData[1] << "\t" << Databuf[tCount].JointsQuat[2].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[3].mData[3] << "\t" << Databuf[tCount].JointsQuat[3].mData[0] << "\t" << Databuf[tCount].JointsQuat[3].mData[1] << "\t" << Databuf[tCount].JointsQuat[3].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[4].mData[3] << "\t" << Databuf[tCount].JointsQuat[4].mData[0] << "\t" << Databuf[tCount].JointsQuat[4].mData[1] << "\t" << Databuf[tCount].JointsQuat[4].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[5].mData[3] << "\t" << Databuf[tCount].JointsQuat[5].mData[0] << "\t" << Databuf[tCount].JointsQuat[5].mData[1] << "\t" << Databuf[tCount].JointsQuat[5].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[6].mData[3] << "\t" << Databuf[tCount].JointsQuat[6].mData[0] << "\t" << Databuf[tCount].JointsQuat[6].mData[1] << "\t" << Databuf[tCount].JointsQuat[6].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[7].mData[3] << "\t" << Databuf[tCount].JointsQuat[7].mData[0] << "\t" << Databuf[tCount].JointsQuat[7].mData[1] << "\t" << Databuf[tCount].JointsQuat[7].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[8].mData[3] << "\t" << Databuf[tCount].JointsQuat[8].mData[0] << "\t" << Databuf[tCount].JointsQuat[8].mData[1] << "\t" << Databuf[tCount].JointsQuat[8].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[9].mData[3] << "\t" << Databuf[tCount].JointsQuat[9].mData[0] << "\t" << Databuf[tCount].JointsQuat[9].mData[1] << "\t" << Databuf[tCount].JointsQuat[9].mData[2] << "\n";

		
	}
	

	avatarDataFile.close();
	Databuf.clear();


}
void GenData::testSLERPFT()
{
	int Genframe = 200;
	//int maxFrame = this->maxframe;
	GenData Databuf;
	
	Databuf = gst::generateIntermediateFrames(Genframe, this);

	// save new data set
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/SeedtoFullData index%d.txt", this->motion_idx);
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Databuf.maxframe << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < Databuf.maxframe; tCount++)
	{

		avatarDataFile
			<< Databuf.QuatData[tCount].JointsQuat[0].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[1].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[2].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[3].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[4].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[5].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[6].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[7].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[8].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[9].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[2] << "\n";


	}


	avatarDataFile.close();
	//Databuf.clear();





}

void GenData::euler_toquat() {

	
		this->QuatData[0].JointsQuat[0] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[1] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[2] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[3] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[4] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[5] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[6] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[7] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[8] = quaternion(0,0,0,1);
		this->QuatData[0].JointsQuat[9] = quaternion(0,0,0,1);





		//this->QuatData[i].JointsQuat[0] = gst::EulertoQuat(this->Eulerangle[i][0]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][0]));// 0 - 1
		//this->QuatData[i].JointsQuat[1] = gst::EulertoQuat(this->Eulerangle[i][4]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][3]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][2]).mutiplication(this->QuatData[i].JointsQuat[0])));  //0 1/ 2 3 4
		//this->QuatData[i].JointsQuat[2] = gst::EulertoQuat(this->Eulerangle[i][4]);
		//this->QuatData[i].JointsQuat[3] = gst::EulertoQuat(this->Eulerangle[i][5]);
		//this->QuatData[i].JointsQuat[4] = gst::EulertoQuat(this->Eulerangle[i][2]);
		//this->QuatData[i].JointsQuat[5] = gst::EulertoQuat(this->Eulerangle[i][3]);
		//this->QuatData[i].JointsQuat[6] = gst::EulertoQuat(this->Eulerangle[i][8]);
		//this->QuatData[i].JointsQuat[7] = gst::EulertoQuat(this->Eulerangle[i][9]);
		//this->QuatData[i].JointsQuat[8] = gst::EulertoQuat(this->Eulerangle[i][6]);
		//this->QuatData[i].JointsQuat[9] = gst::EulertoQuat(this->Eulerangle[i][7]);

		//this->QuatData[i].JointsQuat[0] = gst::EulertoQuat(this->Eulerangle[i][0]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][0]));// 0 - 1
		//this->QuatData[i].JointsQuat[1] = gst::EulertoQuat(this->Eulerangle[i][4]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][3]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][2]).mutiplication(this->QuatData[i].JointsQuat[0])));  //0 1/ 2 3 4
		//this->QuatData[i].JointsQuat[2] = gst::EulertoQuat(this->Eulerangle[i][12]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][6]).mutiplication(this->QuatData[i].JointsQuat[1]));
		//this->QuatData[i].JointsQuat[3] = gst::EulertoQuat(this->Eulerangle[i][13]).mutiplication(this->QuatData[i].JointsQuat[2]);
		//this->QuatData[i].JointsQuat[4] = gst::EulertoQuat(this->Eulerangle[i][7]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][6]).mutiplication(this->QuatData[i].JointsQuat[1]));
		//this->QuatData[i].JointsQuat[5] = gst::EulertoQuat(this->Eulerangle[i][8]).mutiplication(this->QuatData[i].JointsQuat[2]);
		//this->QuatData[i].JointsQuat[6] = gst::EulertoQuat(this->Eulerangle[i][23]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][2]).mutiplication(this->QuatData[i].JointsQuat[0]));
		//this->QuatData[i].JointsQuat[7] = gst::EulertoQuat(this->Eulerangle[i][24]).mutiplication(this->QuatData[i].JointsQuat[6]);
		//this->QuatData[i].JointsQuat[8] = gst::EulertoQuat(this->Eulerangle[i][16]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][2]).mutiplication(this->QuatData[i].JointsQuat[0]));
		//this->QuatData[i].JointsQuat[9] = gst::EulertoQuat(this->Eulerangle[i][17]).mutiplication(this->QuatData[i].JointsQuat[8]);
	

	for (int i = 1; i < this->maxframe+1; i++)
	{
		this->QuatData[i].JointsQuat[0] = gst::EulertoQuat(this->Eulerangle[i][0]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][0]).Inverse());// 0 - 1
		this->QuatData[i].JointsQuat[1] = gst::EulertoQuat(this->Eulerangle[i][4]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][3]).Inverse().mutiplication(gst::EulertoQuat(this->Eulerangle[i][2]).Inverse().mutiplication(this->QuatData[i].JointsQuat[0].Inverse())));  //0 1/ 2 3 4
		this->QuatData[i].JointsQuat[2] = gst::EulertoQuat(this->Eulerangle[i][12]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][6]).mutiplication(this->QuatData[i].JointsQuat[1]));
		this->QuatData[i].JointsQuat[3] = gst::EulertoQuat(this->Eulerangle[i][13]).mutiplication(this->QuatData[i].JointsQuat[2]);
		this->QuatData[i].JointsQuat[4] = gst::EulertoQuat(this->Eulerangle[i][7]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][6]).mutiplication(this->QuatData[i].JointsQuat[1]));
		this->QuatData[i].JointsQuat[5] = gst::EulertoQuat(this->Eulerangle[i][8]).mutiplication(this->QuatData[i].JointsQuat[2]);
		this->QuatData[i].JointsQuat[6] = gst::EulertoQuat(this->Eulerangle[i][23]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][2]).mutiplication(this->QuatData[i].JointsQuat[0]));
		this->QuatData[i].JointsQuat[7] = gst::EulertoQuat(this->Eulerangle[i][24]).mutiplication(this->QuatData[i].JointsQuat[6]);
		this->QuatData[i].JointsQuat[8] = gst::EulertoQuat(this->Eulerangle[i][16]).mutiplication(gst::EulertoQuat(this->Eulerangle[i][2]).mutiplication(this->QuatData[i].JointsQuat[0]));
		this->QuatData[i].JointsQuat[9] = gst::EulertoQuat(this->Eulerangle[i][17]).mutiplication(this->QuatData[i].JointsQuat[8]);






	}

	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/Euler to Quat.txt");
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << this->maxframe+1 << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < this->maxframe+1; tCount++)
	{

		avatarDataFile
			<< this->QuatData[tCount].JointsQuat[0].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[0].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[0].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[0].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[1].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[1].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[1].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[1].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[2].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[2].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[2].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[2].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[3].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[3].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[3].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[3].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[4].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[4].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[4].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[4].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[5].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[5].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[5].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[5].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[6].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[6].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[6].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[6].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[7].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[7].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[7].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[7].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[8].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[8].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[8].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[8].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[9].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[9].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[9].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[9].mData[2] << "\n";


	}


	avatarDataFile.close();







}
void GenData::pos_check()
{
	double jointPoint[17][3];
	//double target1[3] = { -15.7647, 123.46, 66.13581 }; // right hand joint idx 15 


	// Hook target
	//    {4.2353, 123.46, 66.13581},
	//    { 24.2353, 123.46, 66.13581 },
	//    { -15.7647, 123.46, 66.13581 },
	//    
	//    { 4.2353,  103.46, 66.13581 },
	//    { 24.2353, 103.46, 66.13581 },
	//    { -15.7647,103.46, 66.13581 },
	//    
	//    { 4.2353,  83.46, 66.13581 },
	//    { 24.2353, 83.46, 66.13581 },
	//    { -15.7647,83.46, 66.13581 }


	
	//    {4.2353, 123.46, 85.6437},
	//    { 24.2353, 123.46, 85.6437 },
	//    { -15.7647, 123.46, 85.6437 },
	//    
	//    { 4.2353,  103.46, 85.6437 },
	//    { 24.2353, 103.46, 85.6437 },
	//    { -15.7647,103.46, 85.6437 },
	//    
	//    { 4.2353,  83.46, 85.6437 },
	//    { 24.2353, 83.46, 85.6437 },
	//    { -15.7647,83.46, 85.6437 }

	// Straight Tatget _author

	//    {4.2353, 123.46, 85.6437},
	//    { 24.2353, 123.46, 85.6437 },
	//    { 44.2353, 123.46, 85.6437 },
	//    
	//    { 4.2353,  103.46, 85.6437 },
	//    { 24.2353, 103.46, 85.6437 },
	//    { 44.2353,103.46, 85.6437 },
	//    
	//    { 4.2353,  83.46, 85.6437 },
	//    { 24.2353, 83.46, 85.6437 },
	//    { 44.2353,83.46, 85.6437 }

	//{14.2353, 113.46, 58.13581},
	//{ 34.2353, 113.46, 58.13581 },
	//{ 54.2353, 113.46, 58.13581 },

	//{ 14.2353,  93.46, 58.13581 },
	//{ 34.2353, 93.46, 58.13581 },
	//{ 54.2353,93.46,  58.13581 },

	//{ 14.2353,  73.46, 58.13581 },
	//{ 34.2353, 73.46, 58.13581 },
	//{ 54.2353,73.46,  58.13581 }


	//    {4.2353, 123.46, 77.6437},
	//    { 24.2353, 123.46, 77.6437 },
	//    { 44.2353, 123.46, 77.6437 },
	//    
	//    { 4.2353,  103.46, 77.6437 },
	//    { 24.2353, 103.46, 77.6437 },
	//    { 44.2353,103.46, 77.6437 },
	//    
	//    { 4.2353,  83.46, 77.6437 },
	//    { 24.2353, 83.46, 77.6437 },
	//    { 44.2353,83.46, 77.6437 }

			  //    {4.2353, 123.46, 85.6437},
	//    { 24.2353, 123.46, 85.6437 },
	//    { -15.7647, 123.46, 85.6437 },
	//    
	//    { 4.2353,  103.46, 85.6437 },
	//    { 24.2353, 103.46, 85.6437 },
	//    { -15.7647,103.46, 85.6437 },
	//    
	//    { 4.2353,  83.46, 85.6437 },
	//    { 24.2353, 83.46, 85.6437 },
	//    { -15.7647,83.46, 85.6437 }

	/*{4.2353, 123.46, 80.6437},
	{ 24.2353, 123.46, 80.6437 },
	{ 44.2353, 123.46, 80.6437 },

	{ 4.2353,  103.46, 80.6437 },
	{ 24.2353, 103.46, 80.6437 },
	{ 44.2353,103.46, 80.6437 },

	{ 4.2353,  83.46, 80.6437 },
	{ 24.2353, 83.46, 80.6437 },
	{ 44.2353,83.46, 80.6437 }*/

	double target1[3] = {11,123.46, 86.6437 };
	
	gst::cal_end_Pos(this,jointPoint);

	gst::joint_pos_backup(this->jointPoint,jointPoint);

	cout << "right Hand CAL Position Check  : x " << this->jointPoint[15][0] << " , y   " << this->jointPoint[15][1] << " , z  " << this->jointPoint[15][2] << endl;
	
	//gst::cal_edit_Pos(this, target1, this->jointPoint[15],5.0);
	////gst::cal_edit_Pos(this, target1);
	//gst::cal_end_Pos(this, jointPoint);

	//gst::cal_edit_Pos(this, target1, this->jointPoint[15], 5.0);
	////gst::cal_edit_Pos(this, target1);
	//gst::cal_end_Pos(this, jointPoint);
	//gst::cal_edit_Pos(this, target1, this->jointPoint[15], 5.0);
	////gst::cal_edit_Pos(this, target1);
	//gst::cal_end_Pos(this, jointPoint);
	//gst::cal_edit_Pos(this, target1, this->jointPoint[15], 5.0);
	////gst::cal_edit_Pos(this, target1);
	//gst::cal_end_Pos(this, jointPoint);
	gst::cal_edit_Pos(this, target1, this->jointPoint[15],2.0);
	//gst::cal_edit_Pos(this, target1);
	gst::cal_end_Pos(this, jointPoint);

	gst::cal_edit_Pos(this, target1, jointPoint[15],1);
	gst::cal_end_Pos(this, jointPoint);
	gst::cal_edit_Pos(this, target1, jointPoint[15], 1);
	gst::cal_end_Pos(this, jointPoint);
	//gst::cal_edit_Pos(this, target1, jointPoint[15], 1);
	//gst::cal_end_Pos(this, jointPoint);
	//gst::cal_edit_Pos(this, target1, jointPoint[15], 1);
	//gst::cal_end_Pos(this, jointPoint);
	//gst::cal_edit_Pos(this, target1, jointPoint[15], 1);
	//gst::cal_end_Pos(this, jointPoint);
	/*gst::cal_edit_Pos(this, target1, jointPoint[15], 0.8);
	gst::cal_end_Pos(this, jointPoint);*/
	/*gst::cal_edit_Pos(this, target1, jointPoint[15], 0.1);
	gst::cal_end_Pos(this, jointPoint);*/
	/*gst::cal_edit_Pos(this, target1, jointPoint[15], 5.0);
	gst::cal_end_Pos(this, jointPoint);*/

	cout << "right Hand CAL Position Check  : x " << jointPoint[15][0] << " , y   " << jointPoint[15][1] << " , z  " << jointPoint[15][2] << endl;

	GenData Databuf;

	Databuf = *this;
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "Author_Q/MC_Straight_edit/Straight_edit_MC.txt");
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Databuf.maxframe << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < Databuf.maxframe; tCount++)
	{

		avatarDataFile
			<< Databuf.QuatData[tCount].JointsQuat[0].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[1].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[2].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[3].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[4].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[5].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[6].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[7].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[8].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[9].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[2] << "\n";


	}


	avatarDataFile.close();








}


void GenData::euler_save()
{


	
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/Trajec Smt Edit.txt");
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << this->maxframe << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < this->maxframe; tCount++)
	{

		avatarDataFile
			 <<this->Eulerangle[tCount][0][0] << "\t" << this->Eulerangle[tCount][0][1] << "\t" << this->Eulerangle[tCount][0][2] <<  "\t"
			<< this->Eulerangle[tCount][1][0] << "\t" << this->Eulerangle[tCount][1][1] << "\t" << this->Eulerangle[tCount][1][2] << "\t"
			<< this->Eulerangle[tCount][2][0] << "\t" << this->Eulerangle[tCount][2][1] << "\t" << this->Eulerangle[tCount][2][2] << "\t"
			<< this->Eulerangle[tCount][3][0] << "\t" << this->Eulerangle[tCount][3][1] << "\t" << this->Eulerangle[tCount][3][2] << "\t"
			<< this->Eulerangle[tCount][4][0] << "\t" << this->Eulerangle[tCount][4][1] << "\t" << this->Eulerangle[tCount][4][2] << "\t"
			<< this->Eulerangle[tCount][5][0] << "\t" << this->Eulerangle[tCount][5][1] << "\t" << this->Eulerangle[tCount][5][2] << "\t"
			<< this->Eulerangle[tCount][6][0] << "\t" << this->Eulerangle[tCount][6][1] << "\t" << this->Eulerangle[tCount][6][2] << "\t"
			<< this->Eulerangle[tCount][7][0] << "\t" << this->Eulerangle[tCount][7][1] << "\t" << this->Eulerangle[tCount][7][2] << "\t"
			<< this->Eulerangle[tCount][8][0] << "\t" << this->Eulerangle[tCount][8][1] << "\t" << this->Eulerangle[tCount][8][2] << "\t"
			<< this->Eulerangle[tCount][9][0] << "\t" << this->Eulerangle[tCount][9][1] << "\t" << this->Eulerangle[tCount][9][2] << "\n";


	}


	avatarDataFile.close();

}

void GenData::data_save()
{
	
	
	GenData Databuf;
	Databuf = gst::endptSmoothchange(this);
	Databuf = gst::generateSmoothchange(&Databuf);
	
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/Trajec Smt Edit.txt");
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Databuf.maxframe << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < Databuf.maxframe; tCount++)
	{

		avatarDataFile
			<< Databuf.QuatData[tCount].JointsQuat[0].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[1].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[2].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[3].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[4].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[5].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[6].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[7].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[8].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[9].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[2] << "\n";


	}


	avatarDataFile.close();

}

void GenData::data_sample()
{



	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "Boxing motion/sample_data%d.txt",this->motion_idx);
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << (int)this->maxframe/3 << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < this->maxframe; tCount++)
	{
		if(tCount %3==0)
		{
		avatarDataFile
			<< this->QuatData[tCount].JointsQuat[0].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[0].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[0].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[0].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[1].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[1].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[1].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[1].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[2].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[2].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[2].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[2].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[3].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[3].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[3].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[3].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[4].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[4].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[4].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[4].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[5].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[5].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[5].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[5].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[6].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[6].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[6].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[6].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[7].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[7].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[7].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[7].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[8].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[8].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[8].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[8].mData[2] << "\t"
			<< this->QuatData[tCount].JointsQuat[9].mData[3] << "\t" << this->QuatData[tCount].JointsQuat[9].mData[0] << "\t" << this->QuatData[tCount].JointsQuat[9].mData[1] << "\t" << this->QuatData[tCount].JointsQuat[9].mData[2] << "\n";

		}
	}


	avatarDataFile.close();

}
void GenData::FindthrePoint()
{
	TVec3 Init, Endpt;
	TVec3 NewVector;
	quaternion firstquat[3];

	quaternion Init_q, End_q;
	vector<double>InitS, EndS;
	quaternion buf[3];
	int middle_idx;
	int Genframe = 50;
	middle_idx=gst::find_middlept(this);

	//int Prev_frame = 1;
	int Last_frame = this->maxframe - 1;

	//// 0 프레임 팔
	//for (int i = 0; i < 3; i++)
	//	firstquat[i] = this->QuatData[0].JointsQuat[i].Inverse();

	///*cout << "Ges QUAt before" << this->QuatData[Prev_frame].JointsQuat[2].mData[0] << ", " << this->QuatData[Prev_frame].JointsQuat[2].mData[1] <<
	//	", " << this->QuatData[Prev_frame].JointsQuat[2].mData[2] << ", " << this->QuatData[Prev_frame].JointsQuat[2].mData[3] << endl;*/

	//	// 0+1 프레임
	//for (int i = 0; i < 3; i++)
	//	buf[i] = this->QuatData[Prev_frame].JointsQuat[i].mutiplication(firstquat[i]);

	///*cout << "Ges QUAt hie before" << buf[2].mData[0] << ", " << buf[2].mData[1] <<
	//	", " << buf[2].mData[2] << ", " << buf[2].mData[3] << endl;*/



	//Init_q = buf[1].Inverse().mutiplication(buf[2]);

	////cout << "Ges QUAt" << Init_q.mData[0] << ", " << Init_q.mData[1] << ", " << Init_q.mData[2] << ", " << Init_q.mData[3] << endl;
	//// last 프레임
	//for (int i = 0; i < 3; i++)
	//	buf[i] = this->QuatData[Last_frame].JointsQuat[i].mutiplication(firstquat[i]);

	GenData* GenDatacontrol;
	GenDatacontrol=(new GenData());
	GenData Databuf;
	GenDatacontrol->maxframe = 4;
	GenDatacontrol->QuatData[0] = this->QuatData[0];
	GenDatacontrol->QuatData[1] = this->QuatData[1];
	GenDatacontrol->QuatData[2] = this->QuatData[middle_idx];
	GenDatacontrol->QuatData[3] = this->QuatData[Last_frame];
	//GenDatacontrol->QuatData[2].JointsQuat[2] = calQuat.mutiplication(this->QuatData[Last_frame].JointsQuat[2]);

	gst::addMidFrame(2, GenDatacontrol);
	gst::addMidFrame(GenDatacontrol->maxframe-1, GenDatacontrol);
	gst::EditMidFrame(2, GenDatacontrol);
	//Databuf = gst::generateIntermediateFrames(Genframe, GenDatacontrol);
	Databuf = *GenDatacontrol;
	
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/Extract 3Point_Ad_mid.txt");
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Databuf.maxframe << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < Databuf.maxframe; tCount++)
	{

		avatarDataFile
			<< Databuf.QuatData[tCount].JointsQuat[0].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[1].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[2].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[3].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[4].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[5].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[6].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[7].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[8].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[9].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[2] << "\n";


	}


	avatarDataFile.close();

}

void GenData::FindInitEpt()
{
	TVec3 Init, Endpt;
	TVec3 NewVector;
	quaternion firstquat[3];
	
	quaternion Init_q,End_q;
	vector<double>InitS,EndS;
	quaternion buf[3];
	int Prev_frame = 1;
	int Last_frame = this->maxframe - 1;

	// 0 프레임 팔
	for(int i =0;i<3;i++)
	firstquat[i] = this->QuatData[0].JointsQuat[i].Inverse();
	
	/*cout << "Ges QUAt before" << this->QuatData[Prev_frame].JointsQuat[2].mData[0] << ", " << this->QuatData[Prev_frame].JointsQuat[2].mData[1] <<
		", " << this->QuatData[Prev_frame].JointsQuat[2].mData[2] << ", " << this->QuatData[Prev_frame].JointsQuat[2].mData[3] << endl;*/

	// 0+1 프레임
	for (int i = 0; i < 3; i++)
		buf[i] = this->QuatData[Prev_frame].JointsQuat[i].mutiplication(firstquat[i]);
	
	/*cout << "Ges QUAt hie before" << buf[2].mData[0] << ", " << buf[2].mData[1] <<
		", " << buf[2].mData[2] << ", " << buf[2].mData[3] << endl;*/


	
	Init_q = buf[1].Inverse().mutiplication(buf[2]);

	//cout << "Ges QUAt" << Init_q.mData[0] << ", " << Init_q.mData[1] << ", " << Init_q.mData[2] << ", " << Init_q.mData[3] << endl;
	// last 프레임
	for (int i = 0; i < 3; i++)
		buf[i] = this->QuatData[Last_frame].JointsQuat[i].mutiplication(firstquat[i]);

	
	End_q = buf[1].Inverse().mutiplication(buf[2]);

	/*hierarchy = this->QuatData[0].JointsQuat[1].mutiplication(this->QuatData[0].JointsQuat[0]);
	firstquat = this->QuatData[0].JointsQuat[2].mutiplication(hierarchy);

	
	Init_q = this->QuatData[0 + 1].JointsQuat[2].mutiplication(hierarchy);

	hierarchy = this->QuatData[this->maxframe - 1].JointsQuat[1].mutiplication(this->QuatData[this->maxframe - 1].JointsQuat[0]);
	End_q = this->QuatData[this->maxframe - 1].JointsQuat[2].mutiplication(hierarchy);*/


	Init = gst::quaternion_to_trajectory(Init_q);
	Endpt = gst::quaternion_to_trajectory(End_q);

	
	InitS = gst::vec_Theta(Init);
	EndS = gst::vec_Theta(Endpt);

	vector<double> edit_angle;
	edit_angle.push_back(EndS[0] - 40);
	edit_angle.push_back(EndS[1] - 20);

	double vertical_ang = 10;
	double hori_ang = 20;

	quaternion longituteQuat, lattitudeQuat, combinedQuat;
	longituteQuat.mData[3] = cos(vertical_ang * PI / 180);
	longituteQuat.mData[0] = 1 * (sin(vertical_ang * PI / 180));
	longituteQuat.mData[1] = 0 * (sin(vertical_ang * PI / 180));
	longituteQuat.mData[2] = 0 * (sin(vertical_ang * PI / 180));
	//// computing quaternion for vertical swing
	//longituteQuat.mData[3] = cos(longitude * PI / 180);
	//longituteQuat.mData[0] = 1 * (sin(longitude * PI / 180));
	//longituteQuat.mData[1] = 0 * (sin(longitude * PI / 180));
	//longituteQuat.mData[2] = 0 * (sin(longitude * PI / 180));

	lattitudeQuat.mData[3] = cos(hori_ang * PI / 180);
	lattitudeQuat.mData[0] = 0 * (sin(hori_ang * PI / 180));
	lattitudeQuat.mData[1] = 0 * (sin(hori_ang * PI / 180));
	lattitudeQuat.mData[2] = 1 * (sin(hori_ang * PI / 180));

	////computing quternion for lateral swing
	//lattitudeQuat.mData[3] = cos(lattitude * PI / 180);
	//lattitudeQuat.mData[0] = 0 * (sin(lattitude * PI / 180));
	//lattitudeQuat.mData[1] = 0 * (sin(lattitude * PI / 180));
	//lattitudeQuat.mData[2] = 1 * (sin(lattitude * PI / 180));

	NewVector = gst::Ang_deg_Vec(edit_angle);

	quaternion calQuat;
	TVec3 cal_axis;
	double cal_angle;
	cal_axis = gst::vector_cross(Endpt, NewVector);
	cal_angle = gst::vector_dot(Endpt, NewVector);

	calQuat = gst::AxisagltoQuat(cal_axis, cal_angle);
	combinedQuat = lattitudeQuat.mutiplication(longituteQuat);
	int Genframe = 50;
	quaternion Rot_Q;
	Rot_Q=End_q.mutiplication(combinedQuat);
	GenData Databuf;
	
	// 참조 궤적 생성
	/*GenData* GenDatacontrol;
	GenDatacontrol=(new GenData());

	GenDatacontrol->maxframe = 3;
	GenDatacontrol->QuatData[0] = this->QuatData[0];
	GenDatacontrol->QuatData[1] = this->QuatData[1];
	GenDatacontrol->QuatData[2] = this->QuatData[Last_frame];
	GenDatacontrol->QuatData[2].JointsQuat[2] = calQuat.mutiplication(this->QuatData[Last_frame].JointsQuat[2]);

	Databuf = gst::generateIntermediateFrames(Genframe, GenDatacontrol);*/
	

	//생성된 궤적 전체 곱
	for(int i=0;i<this->maxframe;i++)
	Databuf.QuatData[i] = this->QuatData[i];

	for (int i = 1; i < this->maxframe; i++)
	Databuf.QuatData[i].JointsQuat[2] = calQuat.mutiplication(this->QuatData[i].JointsQuat[2]);
	
	Databuf.maxframe = this->maxframe;
	
	this->EditData = &Databuf;
	/*cout << "Edit Theta/ pi" << "theta : " << edit_angle[0] << "pi : " << edit_angle[1] << endl;
	vector<double> check_ANG;

	check_ANG= gst::vec_Theta(NewVector);
	cout << "cal point" << Init._x  << ", "<<Init._y << ", " << Init._z << " //  " << Endpt._x << ", "<< Endpt._y << ", " << Endpt._z  << endl;
	cout << "cal Theta/ pi" <<"theta : " << InitS[0] <<"pi : " << InitS[1] << " // Final  Theta  : " << EndS[0] << "pi : " << EndS[1] << endl;
	cout << "New point" << NewVector._x << ", " << NewVector._y << ", " << NewVector._z << endl;
	cout << "New Theta/ pi" << "theta : " << check_ANG[0] << "pi : " << check_ANG[1] << endl;*/
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/Rotate Data index%d.txt", this->motion_idx);
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Databuf.maxframe << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < Databuf.maxframe; tCount++)
	{

		avatarDataFile
			<< Databuf.QuatData[tCount].JointsQuat[0].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[0].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[1].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[1].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[2].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[2].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[3].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[3].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[4].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[4].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[5].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[5].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[6].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[6].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[7].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[7].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[8].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[8].mData[2] << "\t"
			<< Databuf.QuatData[tCount].JointsQuat[9].mData[3] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[0] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[1] << "\t" << Databuf.QuatData[tCount].JointsQuat[9].mData[2] << "\n";


	}


	avatarDataFile.close();
}
void GenData::GenEditSlerp()
{
	//int GenNum = 50;
	float Slerp_ration = 0.5;

	int motion_idx = 2;

	int big_maxfrm, sm_maxfrm;
	int big_idx, sm_idx;

	int maxFRm = this->EditData->maxframe;

	if (this->maxframe > this->EditData->maxframe)
	{
		big_maxfrm = this->maxframe;
		big_idx = 0;
		sm_maxfrm = this->EditData->maxframe;
		sm_idx = 1;
	}
	else
	{
		big_maxfrm = this->EditData->maxframe;
		big_idx = 1;
		sm_maxfrm = this->maxframe;
		sm_idx = 0;
	}
	vector<JointQuat> Databuf(500);
	vector<int> close_idx;
	
	//close_idx = gst::idxSort(this->EditData,this);
	//maxFRm = close_idx.size();
	////sm_maxfrm = 52;
	//cout <<"idx check  : " <<close_idx.size() <<"   // "<< sm_maxfrm<<"// " << big_maxfrm << endl;
	//for (int i = 0; i < maxFRm; i++)
	//{
	//	int right, left;

	//	 right= i;
	//	 left = close_idx[i];
	//	/*if (i < maxFRm / 3) {
	//		
	//		right = close_idx[i];
	//		left =i ;
	//	}
	//	else
	//	{
	//		 
	//		 right = int(big_maxfrm/ sm_maxfrm * i);
	//		 left =i ;

	//	}*/

	//	Databuf[i].JointsQuat[0] = Databuf[i].JointsQuat[0].SLERP(this->QuatData[left].JointsQuat[0], this->EditData->QuatData[right].JointsQuat[0], Slerp_ration);
	//	Databuf[i].JointsQuat[1] = Databuf[i].JointsQuat[1].SLERP(this->QuatData[left].JointsQuat[1], this->EditData->QuatData[right].JointsQuat[1], Slerp_ration);
	//	Databuf[i].JointsQuat[2] = Databuf[i].JointsQuat[2].SLERP(this->QuatData[left].JointsQuat[2], this->EditData->QuatData[right].JointsQuat[2], Slerp_ration);
	//	Databuf[i].JointsQuat[3] = Databuf[i].JointsQuat[3].SLERP(this->QuatData[left].JointsQuat[3], this->EditData->QuatData[right].JointsQuat[3], Slerp_ration);
	//	Databuf[i].JointsQuat[4] = Databuf[i].JointsQuat[4].SLERP(this->QuatData[left].JointsQuat[4], this->EditData->QuatData[right].JointsQuat[4], Slerp_ration);
	//	Databuf[i].JointsQuat[5] = Databuf[i].JointsQuat[5].SLERP(this->QuatData[left].JointsQuat[5], this->EditData->QuatData[right].JointsQuat[5], Slerp_ration);
	//	Databuf[i].JointsQuat[6] = Databuf[i].JointsQuat[6].SLERP(this->QuatData[left].JointsQuat[6], this->EditData->QuatData[right].JointsQuat[6], Slerp_ration);
	//	Databuf[i].JointsQuat[7] = Databuf[i].JointsQuat[7].SLERP(this->QuatData[left].JointsQuat[7], this->EditData->QuatData[right].JointsQuat[7], Slerp_ration);
	//	Databuf[i].JointsQuat[8] = Databuf[i].JointsQuat[8].SLERP(this->QuatData[left].JointsQuat[8], this->EditData->QuatData[right].JointsQuat[8], Slerp_ration);
	//	Databuf[i].JointsQuat[9] = Databuf[i].JointsQuat[9].SLERP(this->QuatData[left].JointsQuat[9], this->EditData->QuatData[right].JointsQuat[9], Slerp_ration);
	//																														   
	//	//cout << p_frame << endl;
	//}

	maxFRm = sm_maxfrm;
	for (int i=0 ,j = 0; i < sm_maxfrm; i++)
	{
		j = int(big_maxfrm / sm_maxfrm * i);

		int p_frame, n_frame;
		if (sm_idx == 0)
		{
			p_frame = i;
			n_frame = j;
		}
		else
		{
			p_frame = j;
			n_frame = i;

		}


		Databuf[i].JointsQuat[0] = Databuf[i].JointsQuat[0].SLERP(this->QuatData[p_frame].JointsQuat[0], this->EditData->QuatData[n_frame].JointsQuat[0], Slerp_ration);
		Databuf[i].JointsQuat[1] = Databuf[i].JointsQuat[1].SLERP(this->QuatData[p_frame].JointsQuat[1], this->EditData->QuatData[n_frame].JointsQuat[1], Slerp_ration);
		Databuf[i].JointsQuat[2] = Databuf[i].JointsQuat[2].SLERP(this->QuatData[p_frame].JointsQuat[2], this->EditData->QuatData[n_frame].JointsQuat[2], Slerp_ration);
		Databuf[i].JointsQuat[3] = Databuf[i].JointsQuat[3].SLERP(this->QuatData[p_frame].JointsQuat[3], this->EditData->QuatData[n_frame].JointsQuat[3], Slerp_ration);
		Databuf[i].JointsQuat[4] = Databuf[i].JointsQuat[4].SLERP(this->QuatData[p_frame].JointsQuat[4], this->EditData->QuatData[n_frame].JointsQuat[4], Slerp_ration);
		Databuf[i].JointsQuat[5] = Databuf[i].JointsQuat[5].SLERP(this->QuatData[p_frame].JointsQuat[5], this->EditData->QuatData[n_frame].JointsQuat[5], Slerp_ration);
		Databuf[i].JointsQuat[6] = Databuf[i].JointsQuat[6].SLERP(this->QuatData[p_frame].JointsQuat[6], this->EditData->QuatData[n_frame].JointsQuat[6], Slerp_ration);
		Databuf[i].JointsQuat[7] = Databuf[i].JointsQuat[7].SLERP(this->QuatData[p_frame].JointsQuat[7], this->EditData->QuatData[n_frame].JointsQuat[7], Slerp_ration);
		Databuf[i].JointsQuat[8] = Databuf[i].JointsQuat[8].SLERP(this->QuatData[p_frame].JointsQuat[8], this->EditData->QuatData[n_frame].JointsQuat[8], Slerp_ration);
		Databuf[i].JointsQuat[9] = Databuf[i].JointsQuat[9].SLERP(this->QuatData[p_frame].JointsQuat[9], this->EditData->QuatData[n_frame].JointsQuat[9], Slerp_ration);

		//cout << p_frame << endl;
	}
	// 캘리브레이션 저장
	Databuf[0].JointsQuat[0] = this->QuatData[0].JointsQuat[0];
	Databuf[0].JointsQuat[1] = this->QuatData[0].JointsQuat[1];
	Databuf[0].JointsQuat[2] = this->QuatData[0].JointsQuat[2];
	Databuf[0].JointsQuat[3] = this->QuatData[0].JointsQuat[3];
	Databuf[0].JointsQuat[4] = this->QuatData[0].JointsQuat[4];
	Databuf[0].JointsQuat[5] = this->QuatData[0].JointsQuat[5];
	Databuf[0].JointsQuat[6] = this->QuatData[0].JointsQuat[6];
	Databuf[0].JointsQuat[7] = this->QuatData[0].JointsQuat[7];
	Databuf[0].JointsQuat[8] = this->QuatData[0].JointsQuat[8];
	Databuf[0].JointsQuat[9] = this->QuatData[0].JointsQuat[9];

	// check nan value
	for (int joint = 0; joint < 10; joint++)
	{

		for (int i = 0; i < maxFRm; i++)
		{

			if (isnan(Databuf[i].JointsQuat[joint].mData[3]) != 0)
			{
				Databuf[i].JointsQuat[joint] = Databuf[i - 1].JointsQuat[joint];

			}
		}
	}

	// save new data set
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;

	sprintf_s(fileName, "GenData/Gen-Edit-SLERPData-%dmotion-ratio%f.txt", motion_idx, Slerp_ration);
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	//avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << maxFrame << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];

	cout << "pp" << maxFRm << endl;
	for (int tCount = 0; tCount < maxFRm; tCount++)
	{

		avatarDataFile
			<< Databuf[tCount].JointsQuat[0].mData[3] << "\t" << Databuf[tCount].JointsQuat[0].mData[0] << "\t" << Databuf[tCount].JointsQuat[0].mData[1] << "\t" << Databuf[tCount].JointsQuat[0].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[1].mData[3] << "\t" << Databuf[tCount].JointsQuat[1].mData[0] << "\t" << Databuf[tCount].JointsQuat[1].mData[1] << "\t" << Databuf[tCount].JointsQuat[1].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[2].mData[3] << "\t" << Databuf[tCount].JointsQuat[2].mData[0] << "\t" << Databuf[tCount].JointsQuat[2].mData[1] << "\t" << Databuf[tCount].JointsQuat[2].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[3].mData[3] << "\t" << Databuf[tCount].JointsQuat[3].mData[0] << "\t" << Databuf[tCount].JointsQuat[3].mData[1] << "\t" << Databuf[tCount].JointsQuat[3].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[4].mData[3] << "\t" << Databuf[tCount].JointsQuat[4].mData[0] << "\t" << Databuf[tCount].JointsQuat[4].mData[1] << "\t" << Databuf[tCount].JointsQuat[4].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[5].mData[3] << "\t" << Databuf[tCount].JointsQuat[5].mData[0] << "\t" << Databuf[tCount].JointsQuat[5].mData[1] << "\t" << Databuf[tCount].JointsQuat[5].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[6].mData[3] << "\t" << Databuf[tCount].JointsQuat[6].mData[0] << "\t" << Databuf[tCount].JointsQuat[6].mData[1] << "\t" << Databuf[tCount].JointsQuat[6].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[7].mData[3] << "\t" << Databuf[tCount].JointsQuat[7].mData[0] << "\t" << Databuf[tCount].JointsQuat[7].mData[1] << "\t" << Databuf[tCount].JointsQuat[7].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[8].mData[3] << "\t" << Databuf[tCount].JointsQuat[8].mData[0] << "\t" << Databuf[tCount].JointsQuat[8].mData[1] << "\t" << Databuf[tCount].JointsQuat[8].mData[2] << "\t"
			<< Databuf[tCount].JointsQuat[9].mData[3] << "\t" << Databuf[tCount].JointsQuat[9].mData[0] << "\t" << Databuf[tCount].JointsQuat[9].mData[1] << "\t" << Databuf[tCount].JointsQuat[9].mData[2] << "\n";
	}
	avatarDataFile.close();
	Databuf.clear();
}

void GenData::ExtractSeed()
{
	int Keyframe = 25;
	int maxFrame = this->maxframe;
	vector<JointQuat> Databuf(Keyframe);
	double gap = maxFrame / Keyframe;

	for (int i = 0 ,j=0; i < Keyframe; i++) {

		j = int(round(i*gap));
		Databuf[i].JointsQuat[0] = this->QuatData[j].JointsQuat[0];
		Databuf[i].JointsQuat[1] = this->QuatData[j].JointsQuat[1];
		Databuf[i].JointsQuat[2] = this->QuatData[j].JointsQuat[2];
		Databuf[i].JointsQuat[3] = this->QuatData[j].JointsQuat[3];
		Databuf[i].JointsQuat[4] = this->QuatData[j].JointsQuat[4];
		Databuf[i].JointsQuat[5] = this->QuatData[j].JointsQuat[5];
		Databuf[i].JointsQuat[6] = this->QuatData[j].JointsQuat[6];
		Databuf[i].JointsQuat[7] = this->QuatData[j].JointsQuat[7];
		Databuf[i].JointsQuat[8] = this->QuatData[j].JointsQuat[8];
		Databuf[i].JointsQuat[9] = this->QuatData[j].JointsQuat[9];		
	}

		// save new data set
		ofstream avatarDataFile;
		time_t curr_time;
		curr_time = time(NULL);
		tm *tm_local = localtime(&curr_time);

		//ofstream avatarDataFile;
		char fileName[1024];
		cout << "Generate Data" << endl;

		sprintf_s(fileName, "GenData/SeedData index%d.txt",this->motion_idx);
		avatarDataFile.open(fileName);
		//ofstream avatarDataFile;

		//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
		//avatarDataFile.open("CaptureData/writedata7slot.txt");

		avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Keyframe << "\n";

		//Quatnode.nodelist[0]->writeQuat[0][0];
		for (int tCount = 0; tCount < Keyframe; tCount++)
		{

			avatarDataFile
				<< Databuf[tCount].JointsQuat[0].mData[3] << "\t" << Databuf[tCount].JointsQuat[0].mData[0] << "\t" << Databuf[tCount].JointsQuat[0].mData[1] << "\t" << Databuf[tCount].JointsQuat[0].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[1].mData[3] << "\t" << Databuf[tCount].JointsQuat[1].mData[0] << "\t" << Databuf[tCount].JointsQuat[1].mData[1] << "\t" << Databuf[tCount].JointsQuat[1].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[2].mData[3] << "\t" << Databuf[tCount].JointsQuat[2].mData[0] << "\t" << Databuf[tCount].JointsQuat[2].mData[1] << "\t" << Databuf[tCount].JointsQuat[2].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[3].mData[3] << "\t" << Databuf[tCount].JointsQuat[3].mData[0] << "\t" << Databuf[tCount].JointsQuat[3].mData[1] << "\t" << Databuf[tCount].JointsQuat[3].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[4].mData[3] << "\t" << Databuf[tCount].JointsQuat[4].mData[0] << "\t" << Databuf[tCount].JointsQuat[4].mData[1] << "\t" << Databuf[tCount].JointsQuat[4].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[5].mData[3] << "\t" << Databuf[tCount].JointsQuat[5].mData[0] << "\t" << Databuf[tCount].JointsQuat[5].mData[1] << "\t" << Databuf[tCount].JointsQuat[5].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[6].mData[3] << "\t" << Databuf[tCount].JointsQuat[6].mData[0] << "\t" << Databuf[tCount].JointsQuat[6].mData[1] << "\t" << Databuf[tCount].JointsQuat[6].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[7].mData[3] << "\t" << Databuf[tCount].JointsQuat[7].mData[0] << "\t" << Databuf[tCount].JointsQuat[7].mData[1] << "\t" << Databuf[tCount].JointsQuat[7].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[8].mData[3] << "\t" << Databuf[tCount].JointsQuat[8].mData[0] << "\t" << Databuf[tCount].JointsQuat[8].mData[1] << "\t" << Databuf[tCount].JointsQuat[8].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[9].mData[3] << "\t" << Databuf[tCount].JointsQuat[9].mData[0] << "\t" << Databuf[tCount].JointsQuat[9].mData[1] << "\t" << Databuf[tCount].JointsQuat[9].mData[2] << "\n";
		
		}


		avatarDataFile.close();
		Databuf.clear();
}

void JointPosition::loadreferencedatafromfile()
{
	int tCount = 0;
	string fileName;
	std::ifstream _filestream;
	std::string _line;
	int _option;
	std::string _dummy;
	for(int i = 0; i < this - > loadingfilenum; i++) {
		int lineCount = 0;
		int count = 0;
		fileName = ".\\GestureData\\reference\\loadfile" + to_string(this - > loadingfilecount + 1) + ".txt";
		_filestream.open(fileName);
		cout << "check" << fileName << endl;
		while(std::getline(_filestream, _line)) {
			std::stringstream _linestream;
			_linestream << _line;
			//cout << "gesture working" << endl;
			if(count == 0) {
				_linestream >> this - > posename[this - > loadingfilecount];
				count++;
				cout << "check" << _line << endl;
				continue;
			}
			if(count == 1) {
				_linestream >> _line >> this - > referFramenum[this - > loadingfilecount];
				count++;
				cout << "check" << this - > referFramenum[this - > loadingfilecount] << endl;
				continue;
			}
			_linestream >> 
                                this - > loadData[lineCount].Jointspos[0][0] >> 
                                this - > loadData[lineCount].Jointspos[0][1] >> 
                                this - > loadData[lineCount].Jointspos[0][2] >> 
                                this - > loadData[lineCount].Jointspos[1][0] >> 
                                this - > loadData[lineCount].Jointspos[1][1] >> 
                                this - > loadData[lineCount].Jointspos[1][2] >> 
                                this - > loadData[lineCount].Jointspos[2][0] >> 
                                this - > loadData[lineCount].Jointspos[2][1] >> 
                                this - > loadData[lineCount].Jointspos[2][2] >> 
                                this - > loadData[lineCount].Jointspos[3][0] >> 
                                this - > loadData[lineCount].Jointspos[3][1] >> 
                                this - > loadData[lineCount].Jointspos[3][2] >> 
                                this - > loadData[lineCount].Jointspos[4][0] >> 
                                this - > loadData[lineCount].Jointspos[4][1] >> 
                                this - > loadData[lineCount].Jointspos[4][2] >> 
                                this - > loadData[lineCount].Jointspos[5][0] >> 
                                this - > loadData[lineCount].Jointspos[5][1] >> 
                                this - > loadData[lineCount].Jointspos[5][2] >> 
                                this - > loadData[lineCount].Jointspos[6][0] >> 
                                this - > loadData[lineCount].Jointspos[6][1] >> 
                                this - > loadData[lineCount].Jointspos[6][2] >> 
                                this - > loadData[lineCount].Jointspos[7][0] >> 
                                this - > loadData[lineCount].Jointspos[7][1] >> 
                                this - > loadData[lineCount].Jointspos[7][2] >> 
                                this - > loadData[lineCount].Jointspos[8][0] >> 
                                this - > loadData[lineCount].Jointspos[8][1] >> 
                                this - > loadData[lineCount].Jointspos[8][2] >> 
                                this - > loadData[lineCount].Jointspos[9][0] >> 
                                this - > loadData[lineCount].Jointspos[9][1] >> 
                                this - > loadData[lineCount].Jointspos[9][2] >> 
                                this - > loadData[lineCount].Jointspos[10][0] >> 
                                this - > loadData[lineCount].Jointspos[10][1] >> 
                                this - > loadData[lineCount].Jointspos[10][2] >> 
                                this - > loadData[lineCount].Jointspos[11][0] >> 
                                this - > loadData[lineCount].Jointspos[11][1] >> 
                                this - > loadData[lineCount].Jointspos[11][2] >> 
                                this - > loadData[lineCount].Jointspos[12][0] >> 
                                this - > loadData[lineCount].Jointspos[12][1] >> 
                                this - > loadData[lineCount].Jointspos[12][2] >> 
                                this - > loadData[lineCount].Jointspos[13][0] >> 
                                this - > loadData[lineCount].Jointspos[13][1] >> 
                                this - > loadData[lineCount].Jointspos[13][2] >> 
                                this - > loadData[lineCount].Jointspos[14][0] >> 
                                this - > loadData[lineCount].Jointspos[14][1] >> 
                                this - > loadData[lineCount].Jointspos[14][2] >> 
                                this - > loadData[lineCount].Jointspos[15][0] >> 
                                this - > loadData[lineCount].Jointspos[15][1] >> 
                                this - > loadData[lineCount].Jointspos[15][2];
			lineCount++;
		}
		_filestream.close();
		calloadavg(this - > referFramenum[i], this - > referencepose[i]);
		memset(this - > loadData, NULL, sizeof(this - > loadData));
		this - > loadingfilecount++;
	}
}

void GenData::genQuaternion() {

	int GenNum = 50;
	int maxFrame = this->maxframe;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dis(0, 999);
	vector<JointQuat> TotalDatabuf(3000);
	int Totalcount = 0;
	for(int i=0;i<GenNum;i++){



	srand((unsigned int)time(0));
	quaternion RotateQ, RotateQ_lower;
	int randFrame = dis(gen) % (10)+1;
	int randDirection = dis(gen) % 3;
	double randAngle = (double)dis(gen)/ 999;
	
	RotateQ=gst::AxisagltoQuat(randDirection, randAngle);

	int randFrame1 = dis(gen) % (10) + 1;
	int randDirection1 = dis(gen) % 3;
	double randAngle1 = (double)dis(gen) / 999;
	//quaternion RotateQ, RotateQ_lower;
	RotateQ_lower = gst::AxisagltoQuat(randDirection, randAngle);

	cout << "check  RandFrame : " << randFrame << "  Rand Direction : " << randDirection << "rand angle : " << randAngle << endl;

	vector<JointQuat> Databuf(1000);
	
	// edit quat
	bool Once = true;
	for (int j = 0; j < maxFrame; j++) {
		if (j>=randFrame) {
			if((j!=1&&j!=2)&&Once){
				quaternion sub1,sub2, sub3, sub4;
				sub1 = gst::AxisagltoQuat(randDirection, randAngle/3);
				sub2 = gst::AxisagltoQuat(randDirection, randAngle/3*2);

				sub3 = gst::AxisagltoQuat(randDirection1, randAngle1 / 3);
				sub4 = gst::AxisagltoQuat(randDirection1, randAngle1 / 3 * 2);


				Databuf[j-2].JointsQuat[2] = sub1.mutiplication(Databuf[j-2].JointsQuat[2]);
				Databuf[j-2].JointsQuat[3] = sub1.mutiplication(Databuf[j-2].JointsQuat[3]);

				Databuf[j - 1].JointsQuat[2] = sub3.mutiplication(sub2.mutiplication(Databuf[j - 1].JointsQuat[2]));
				Databuf[j - 1].JointsQuat[3] = sub4.mutiplication(sub2.mutiplication(Databuf[j - 1].JointsQuat[3]));

				
				Once = false;
			}
			Databuf[j].JointsQuat[2] = RotateQ.mutiplication(this->QuatData[j].JointsQuat[2]);
			Databuf[j].JointsQuat[3] = RotateQ_lower.mutiplication(RotateQ.mutiplication(this->QuatData[j].JointsQuat[3]));
			//cout << "index check " << j << endl;
			for (int k = 0; k < 4; k++) {
				Databuf[j].JointsQuat[0].mData[k] = this->QuatData[j].JointsQuat[0].mData[k];
				Databuf[j].JointsQuat[1].mData[k] = this->QuatData[j].JointsQuat[1].mData[k];				
				Databuf[j].JointsQuat[4].mData[k] = this->QuatData[j].JointsQuat[4].mData[k];
				Databuf[j].JointsQuat[5].mData[k] = this->QuatData[j].JointsQuat[5].mData[k];
				Databuf[j].JointsQuat[6].mData[k] = this->QuatData[j].JointsQuat[6].mData[k];
				Databuf[j].JointsQuat[7].mData[k] = this->QuatData[j].JointsQuat[7].mData[k];
				Databuf[j].JointsQuat[8].mData[k] = this->QuatData[j].JointsQuat[8].mData[k];
				Databuf[j].JointsQuat[9].mData[k] = this->QuatData[j].JointsQuat[9].mData[k];
				}					   
		}
		else {
				for (int k = 0; k < 4; k++) {
					Databuf[j].JointsQuat[0].mData[k] = this->QuatData[j].JointsQuat[0].mData[k];
					Databuf[j].JointsQuat[1].mData[k] = this->QuatData[j].JointsQuat[1].mData[k];
					Databuf[j].JointsQuat[2].mData[k] = this->QuatData[j].JointsQuat[2].mData[k];
					Databuf[j].JointsQuat[3].mData[k] = this->QuatData[j].JointsQuat[3].mData[k];
					Databuf[j].JointsQuat[4].mData[k] = this->QuatData[j].JointsQuat[4].mData[k];
					Databuf[j].JointsQuat[5].mData[k] = this->QuatData[j].JointsQuat[5].mData[k];
					Databuf[j].JointsQuat[6].mData[k] = this->QuatData[j].JointsQuat[6].mData[k];
					Databuf[j].JointsQuat[7].mData[k] = this->QuatData[j].JointsQuat[7].mData[k];
					Databuf[j].JointsQuat[8].mData[k] = this->QuatData[j].JointsQuat[8].mData[k];
					Databuf[j].JointsQuat[9].mData[k] = this->QuatData[j].JointsQuat[9].mData[k];
					}
		}

	}

	// save new data set
	ofstream avatarDataFile;
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Data" << endl;
	
	sprintf_s(fileName, "GenData/GenerateData-%d.txt", i );
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	//avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << maxFrame << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < maxFrame; tCount++)
	{
		
			avatarDataFile
				<< Databuf[tCount].JointsQuat[0].mData[3] << "\t" << Databuf[tCount].JointsQuat[0].mData[0] << "\t" << Databuf[tCount].JointsQuat[0].mData[1] << "\t" << Databuf[tCount].JointsQuat[0].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[1].mData[3] << "\t" << Databuf[tCount].JointsQuat[1].mData[0] << "\t" << Databuf[tCount].JointsQuat[1].mData[1] << "\t" << Databuf[tCount].JointsQuat[1].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[2].mData[3] << "\t" << Databuf[tCount].JointsQuat[2].mData[0] << "\t" << Databuf[tCount].JointsQuat[2].mData[1] << "\t" << Databuf[tCount].JointsQuat[2].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[3].mData[3] << "\t" << Databuf[tCount].JointsQuat[3].mData[0] << "\t" << Databuf[tCount].JointsQuat[3].mData[1] << "\t" << Databuf[tCount].JointsQuat[3].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[4].mData[3] << "\t" << Databuf[tCount].JointsQuat[4].mData[0] << "\t" << Databuf[tCount].JointsQuat[4].mData[1] << "\t" << Databuf[tCount].JointsQuat[4].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[5].mData[3] << "\t" << Databuf[tCount].JointsQuat[5].mData[0] << "\t" << Databuf[tCount].JointsQuat[5].mData[1] << "\t" << Databuf[tCount].JointsQuat[5].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[6].mData[3] << "\t" << Databuf[tCount].JointsQuat[6].mData[0] << "\t" << Databuf[tCount].JointsQuat[6].mData[1] << "\t" << Databuf[tCount].JointsQuat[6].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[7].mData[3] << "\t" << Databuf[tCount].JointsQuat[7].mData[0] << "\t" << Databuf[tCount].JointsQuat[7].mData[1] << "\t" << Databuf[tCount].JointsQuat[7].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[8].mData[3] << "\t" << Databuf[tCount].JointsQuat[8].mData[0] << "\t" << Databuf[tCount].JointsQuat[8].mData[1] << "\t" << Databuf[tCount].JointsQuat[8].mData[2] << "\t"
				<< Databuf[tCount].JointsQuat[9].mData[3] << "\t" << Databuf[tCount].JointsQuat[9].mData[0] << "\t" << Databuf[tCount].JointsQuat[9].mData[1] << "\t" << Databuf[tCount].JointsQuat[9].mData[2] << "\n";

			if (Totalcount < 3000) {
				TotalDatabuf[Totalcount].JointsQuat[0] = Databuf[tCount].JointsQuat[0];
				TotalDatabuf[Totalcount].JointsQuat[1] = Databuf[tCount].JointsQuat[1];
				TotalDatabuf[Totalcount].JointsQuat[2] = Databuf[tCount].JointsQuat[2];
				TotalDatabuf[Totalcount].JointsQuat[3] = Databuf[tCount].JointsQuat[3];
				TotalDatabuf[Totalcount].JointsQuat[4] = Databuf[tCount].JointsQuat[4];
				TotalDatabuf[Totalcount].JointsQuat[5] = Databuf[tCount].JointsQuat[5];
				TotalDatabuf[Totalcount].JointsQuat[6] = Databuf[tCount].JointsQuat[6];
				TotalDatabuf[Totalcount].JointsQuat[7] = Databuf[tCount].JointsQuat[7];
				TotalDatabuf[Totalcount].JointsQuat[8] = Databuf[tCount].JointsQuat[8];
				TotalDatabuf[Totalcount].JointsQuat[9] = Databuf[tCount].JointsQuat[9];

				//avatarDataFile << "\n";			
				Totalcount++;
			}
	}
	avatarDataFile.close();
	Databuf.clear();

	}

	ofstream avatarDataFile;

	//ofstream avatarDataFile;
	char fileName[1024];
	cout << "Generate Total Data" << endl;

	sprintf_s(fileName, "GenData/GenerateTotalData1.txt");
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Totalcount/2 << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = 0; tCount < Totalcount/2; tCount++)
	{

		avatarDataFile
			<< TotalDatabuf[tCount].JointsQuat[0].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[0].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[0].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[0].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[1].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[1].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[1].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[1].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[2].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[2].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[2].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[2].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[3].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[3].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[3].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[3].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[4].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[4].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[4].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[4].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[5].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[5].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[5].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[5].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[6].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[6].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[6].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[6].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[7].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[7].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[7].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[7].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[8].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[8].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[8].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[8].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[9].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[9].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[9].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[9].mData[2] << "\n";


		//avatarDataFile << "\n";			
		
	}

	avatarDataFile.close();

	sprintf_s(fileName, "GenData/GenerateTotalData2.txt");
	avatarDataFile.open(fileName);
	//ofstream avatarDataFile;

	//sprintf_s(fileName, "CData\\AvatarData-00%d-%d%d%d.txt", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	//avatarDataFile.open("CaptureData/writedata7slot.txt");

	avatarDataFile << "FULLBODY\t" << 1 << "\n" << "Frames:" << "\t" << Totalcount/2 << "\n";

	//Quatnode.nodelist[0]->writeQuat[0][0];
	for (int tCount = Totalcount / 2; tCount < Totalcount; tCount++)
	{

		avatarDataFile
			<< TotalDatabuf[tCount].JointsQuat[0].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[0].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[0].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[0].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[1].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[1].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[1].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[1].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[2].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[2].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[2].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[2].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[3].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[3].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[3].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[3].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[4].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[4].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[4].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[4].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[5].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[5].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[5].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[5].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[6].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[6].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[6].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[6].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[7].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[7].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[7].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[7].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[8].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[8].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[8].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[8].mData[2] << "\t"
			<< TotalDatabuf[tCount].JointsQuat[9].mData[3] << "\t" << TotalDatabuf[tCount].JointsQuat[9].mData[0] << "\t" << TotalDatabuf[tCount].JointsQuat[9].mData[1] << "\t" << TotalDatabuf[tCount].JointsQuat[9].mData[2] << "\n";

		//avatarDataFile << "\n";			

	}

	avatarDataFile.close();
}
