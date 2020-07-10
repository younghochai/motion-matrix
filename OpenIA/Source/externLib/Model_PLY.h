#pragma once
#include <windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <gl/GL.h>
#include <gl/GLU.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <freeglut.h>

class Model_PLY
{
public:
	int Load(char *filename);
	void Draw();
	float* calculateNormal(float *coord1, float *coord2, float *coord3);
	Model_PLY();

	float* Faces_Triangles;
	float* Faces_Quads;
	float* Vertex_Buffer;
	float* Normals;

	int TotalConnectedTriangles;
	int TotalConnectedQuads;
	int TotalConnectedPoints;
	int TotalFaces;


};