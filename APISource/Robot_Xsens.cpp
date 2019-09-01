// Robot_Xsens.cpp : Defines the entry point for the console application.
//

#include"XsensConnection.h"

#include <math.h>
#include <stdio.h>
#include <glut.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <freeglut.h>
#include <ctime>
#include "tgaload.h" //Texture mapping
#include "Comparision.h"

using namespace std;


#define STEP 0.1
#define TORSO_HEIGHT 6.0
#define TORSO_RADIUS 2.0
#define UPPER_ARM_HEIGHT 3.0
#define UPPER_ARM_RADIUS  0.5
#define LOWER_ARM_HEIGHT 2.5
#define LOWER_ARM_RADIUS  0.5
#define UPPER_LEG_HEIGHT 4.0
#define UPPER_LEG_RADIUS  0.6
#define LOWER_LEG_HEIGHT 3.5
#define LOWER_LEG_RADIUS  0.6
#define ELBOW_RADIUS 0.6
#define KNEE_RADIUS 0.7
#define HAND_RADIUS 0.6
#define FOOT_RADIUS 0.7
#define SHOULDER_RADIUS 0.8
#define HEAD_HEIGHT 1.5
#define HEAD_RADIUS 1.0
#define NECK_HEIGHT 1.0
#define NECK_RADIUS 0.4

#define NULL 0
#define PI 3.14159265359
#define SAMPLESIZE 100.0f
struct TVec3 {
	float _x;
	float _y;
	float _z;
};

int option = -1;
int animation = -3; //teleytaio animation
int done = 0; //an oloklirwthike to teleytaio animation
double rotate_ = 0;
double horizontal = 0;
double zoominout = 1.2;
double xr = 0, yr = 0, zr = 0;

void head();
void torso();
void left_upper_arm();
void right_upper_arm();
void left_upper_leg();
void right_upper_leg();

typedef float point[3];

XsensConnection connectXS;

//Threads Synchronization
CRITICAL_SECTION m_cs;
//--------------------------------------------------
typedef struct treenode
{
	GLfloat m[16];
	void(*f)();
	struct treenode *sibling;
	struct treenode *child;
}treenode;

typedef treenode* t_ptr;

static GLfloat theta[11] = { 0.0,0.0,0.0,180.0,0.0,180.0,0.0,180.0,0.0,180.0,0.0 }; /* initial joint angles */

static GLint angle = 2;

GLUquadricObj *t, *h, *lua, *lla, *rua, *rla, *lll, *rll, *rul, *lul;
GLUquadricObj *relb, *lelb, *rknee, *lknee, *nk, *lhand, *rhand, *lfoot, *rfoot, *rsh, *lsh;

//double size = 1.0;

treenode torso_node, head_node, lua_node, rua_node, lll_node, rll_node,
lla_node, rla_node, rul_node, lul_node,
relb_node, lelb_node, rknee_node, lknee_node, nk_node, lhand_node, rhand_node, lfoot_node, rfoot_node,
rsh_node, lsh_node;

//Output to file
ofstream L_rawqfile, U_rawqfile, P_rawfile ,LatLongfile;
ofstream L_qfile1, L_rfile2, L_sfqfile, U_sfqfile, Calibfile, RPY_lowpass;

bool fileClose = false;
float** uqdata;
float** lqdata;

float** uqdataDB;
float** lqdataDB;

bool bReadFile = false;
int isize = 0;
int dsize = 0;
bool startAnim = false;
double prevDegree = 0;

quaternion QuatData_U, QuatData_L, QuatData_C;
quaternion firstInvQuat_U, firstInvQuat_L, firstInvQuat_C;
quaternion qPA, qPrevPA, qPrevInvsPA, firstPlvCalib;
quaternion uqPA,uqPrevPA, uqPrevInvsPA;
quaternion currentQuat, nextQuat, firstQuat;
quaternion QuatFirstVect1, QuatFirstVect2, QuatAxis;
float prev_angle = 0.0;

quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);

quaternion Calib_att, Calib_tpos, Calib_cal(1.00, 0.00, 0.00, 0.00);
quaternion Calib_xe(0.71, 0.71, 0.00, 0.00);
quaternion Calib_ye(0.71, 0.00, 0.71, 0.00);
bool startCalib = false;

TVector3 V0, V1;
TVector3 avgRPY;
int indexP = 0;
int centerindexP = 1;
int averageIndexP = 0;
int Counterindex = 0;
bool First_calibrate = true;  // Reset Quaternion
bool AttAxisCalib = false;
bool TposAxisCalib = false;
bool isCalib = false;
bool isFirst = true;
bool isRQFirst = true;
int width = 1800;
int height = 900;

TVector3 tempVec;

float xrot = 0.0f;
float yrot = 0.0f;

float xdiff = 0.0f;
float ydiff = 0.0f;
float zval = 3.5f;

float pointTranslateX = 0.0f;
float pointTranslateY = 0.0f;
float pointTranslateZ = zval;

float cameraX, cameraY;
int lastMouseX, lastMouseY;

bool mouseDown = false;

float PA_data[20014][4];
float uPA_data[20014][4];
float drawGrid[20014][4];


float uDB_data[20014][4];
float lDB_data[20014][4];
int dbCount = 0;
int cIndexArray[200];
int CenterIndex = 0;
std::clock_t start;
double duration;
float avgAngle = 0;

int fileCount = 0;
char fileName[1024];
char LfileName[1024];
char UfileName[1024];

char charstring[1024];

bool isMatched = false;
float diff = 10;
float diff1 = 10;
float diff2 = 10;
float diff3 = 10;

float threshold = 0.15f;
float color[4] = { 0, 0, 0, 1 };
float mcolor[4] = { 1, 0, 0, 1 };

//Texturemapping

GLuint texture_id[2];
GLUquadricObj *sphere;
GLfloat LightAmbient[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat LightDiffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
GLfloat LightPosition[] = { 5.0f, 25.0f, 15.0f, 1.0f };
GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };

const int   TEXT_WIDTH = 8;
const int   TEXT_HEIGHT = 24;
void *font = GLUT_BITMAP_TIMES_ROMAN_24;
///////////////////////////////////////////////////////////////////////////////
// write 2d text using GLUT
// The projection matrix must be set to orthogonal before call this function.
///////////////////////////////////////////////////////////////////////////////
void drawString(const char *str, int x, int y, float color[4], void *font)
{
	glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
	glDisable(GL_LIGHTING);     // need to disable lighting for proper text color
	glDisable(GL_TEXTURE_2D);
	glDepthFunc(GL_ALWAYS);

	glColor4fv(color);          // set text color
	glRasterPos2i(x, y);        // place text position

								// loop all characters in the string
	while (*str)
	{
		glutBitmapCharacter(font, *str);
		++str;
	}

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glDepthFunc(GL_LEQUAL);
	glPopAttrib();
}

///////////////////////////////////////////////////////////////////////////////
// draw a string in 3D space
///////////////////////////////////////////////////////////////////////////////
void drawString3D(const char *str, float pos[3], float color[4], void *font)
{
	glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
	glDisable(GL_LIGHTING);     // need to disable lighting for proper text color
	glDisable(GL_TEXTURE_2D);

	glColor4fv(color);          // set text color
	glRasterPos3fv(pos);        // place text position

								// loop all characters in the string
	while (*str)
	{
		glutBitmapCharacter(font, *str);
		++str;
	}

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glPopAttrib();
}

float myRoundFunction(float value)
{
	float roundval = trunc(value * 1000.0) / 1000.0;

	return roundval;
}

float dot(TVec3 a, TVec3 b)  //calculates dot product of a and b
{
	return a._x * b._x + a._y * b._y + a._z * b._z;
}

float dist(TVec3 a, TVec3 b)
{
	return sqrt((a._x - b._x)*(a._x - b._x) + (a._y - b._y)*(a._y - b._y) + (a._z - b._z)*(a._z - b._z));
}

float mag(TVec3 a)  //calculates magnitude of a
{
	return std::sqrt(a._x * a._x + a._y * a._y + a._z * a._z);
}

void quaternionToEulerAngles(quaternion q, TVector3& RPY)
{
	 //roll (x-axis rotation)
	double sinr_cosp = 2.0 * (q.mData[3] * q.mData[0] + q.mData[1] * q.mData[2]);
	double cosr_cosp = 1.0 - 2.0 * (q.mData[0] * q.mData[0] + q.mData[1] * q.mData[1]);
	RPY._x = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2.0 * (q.mData[3] * q.mData[1] - q.mData[2] * q.mData[0]);
	if (fabs(sinp) >= 1)
		RPY._y = (double)copysign(PI / 2, sinp); // use 90 degrees if out of range
	else
		RPY._y = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2.0 * (q.mData[3] * q.mData[2] + q.mData[0] * q.mData[1]);
	double cosy_cosp = 1.0 - 2.0 * (q.mData[1] * q.mData[1] + q.mData[2] * q.mData[2]);
	RPY._z = atan2(siny_cosp, cosy_cosp);

	//---------------------------------XYZ----------------------------
	//// roll (x-axis rotation)
	//double sinr_cosp = -2.0* (q.mData[1]* q.mData[2] - q.mData[3] * q.mData[0]);
	//double cosr_cosp = (q.mData[3]* q.mData[3] - q.mData[0]* q.mData[0] - q.mData[1]* q.mData[1] + q.mData[2]* q.mData[2]);
	//RPY._x = atan2(sinr_cosp, cosr_cosp);

	//// pitch (y-axis rotation)
	//double sinp = 2.0* (q.mData[0] *q.mData[2] + q.mData[3] * q.mData[1]);
	//if (fabs(sinp) >= 1)
	//	RPY._y = (double)copysign(PI / 2, sinp); // use 90 degrees if out of range
	//else
	//	RPY._y = asin(sinp);

	//// yaw (z-axis rotation)
	//double siny_cosp = -2.0* (q.mData[0]* q.mData[1] - q.mData[3] * q.mData[2]);
	//double cosy_cosp = (q.mData[3]* q.mData[3] + q.mData[0]* q.mData[0] - q.mData[1]* q.mData[1] - q.mData[2]* q.mData[2]);
	//RPY._z = atan2(siny_cosp, cosy_cosp);

	//---------------------------------Robotics----------------------------
	//// roll (x-axis rotation)
	//double sinr_cosp = 2.0 * (q.mData[1] * q.mData[2] - q.mData[3] * q.mData[0]);
	//double cosr_cosp = 2.0 * (q.mData[3] * q.mData[3]) - 1.0 + 2 * (q.mData[2] * q.mData[2]);
	//RPY._x = atan2(sinr_cosp, cosr_cosp);

	//// pitch (y-axis rotation)
	//double sinp = 2.0 * (q.mData[0] * q.mData[2] + q.mData[3] * q.mData[1]);
	//double siny = sqrt(1-(2.0 * (q.mData[0] * q.mData[2]) + 2*(q.mData[3] * q.mData[1])));
	////if (fabs(sinp) >= 1)
	////	RPY._y = (double)copysign(PI / 2, sinp); // use 90 degrees if out of range
	////else
	//	RPY._y = -atan(sinp/siny);

	//// yaw (z-axis rotation)
	//double siny_cosp = 2.0 * (q.mData[0] * q.mData[1] - q.mData[3] * q.mData[2]);
	//double cosy_cosp = 2.0 * (q.mData[3] * q.mData[3]) -1.0 + 2*  (q.mData[0] * q.mData[0]);
	//RPY._z = atan2(siny_cosp, cosy_cosp);
}

void drawCoordinate()
{
	//glDisable(GL_LIGHTING);
	glLineWidth(3.0);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0); // x-axis red
	glVertex3i(0, 0, 0);
	glVertex3i(2.5, 0, 0);
	glColor3f(0, 1, 0); // y-axis green
	glVertex3i(0, 0, 0);
	glVertex3i(0, 2.5, 0);
	glColor3f(0, 0, 1); // z-axis blue
	glVertex3i(0, 0, 0);
	glVertex3i(0, 0, 2.5);
	glEnd();
	//glEnable(GL_LIGHTING);	
	glColor3f(0.8, 0.4, 0.2);
}
void drawcenterCoordinate()
{
	GLUquadricObj *quadric = gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);

	glPushMatrix();
	glRotatef(90, 0, 1, 0);
	glColor3f(1, 0, 0); // x-axis red
	gluCylinder(quadric, 0.005, 0.005, 1.2, 10, 10);
	glTranslatef(0, 0, 1.2);
	glRotatef(0, 0, 0, 1);
	glutSolidCone(0.05, 0.08, 10, 10);
	glPopMatrix();

	glPushMatrix();
	glRotatef(-90, 1, 0, 0);
	glColor3f(0, 1, 0); // y-axis green
	gluCylinder(quadric, 0.005, 0.005, 1.2, 10, 10);
	glTranslatef(0, 0, 1.2);
	glRotatef(0, 1, 0, 0);
	glutSolidCone(0.05, 0.08, 10, 10);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0, 0, 1); // z-axis blue
	glRotatef(-90, 0, 0, 1);
	gluCylinder(quadric, 0.005, 0.005, 1.2, 10, 10);
	glTranslatef(0, 0, 1.2);
	glutSolidCone(0.05, 0.08, 10, 10);
	glPopMatrix();
}
void drawCoordinate(float axis)
{
	//glDisable(GL_LIGHTING);
	glLineWidth(3.0);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0); // x-axis red
	glVertex3f(-axis, 0, 0);
	glVertex3f(axis, 0, 0);
	glColor3f(0, 1, 0); // y-axis green
	glVertex3f(0, -axis, 0);
	glVertex3f(0, axis, 0);
	glColor3f(0, 0, 1); // z-axis blue
	glVertex3f(0, 0, -axis);
	glVertex3f(0, 0, axis);
	glEnd();
	//glEnable(GL_LIGHTING);
}

float GetRotationAngleAndAxis(float qWXYZ[4], float axis[3])
{
	float w = qWXYZ[3];
	float x = qWXYZ[0];
	float y = qWXYZ[1];
	float z = qWXYZ[2];
	float f = sqrt(x*x + y*y + z*z);
	if (f != 0.0)
	{
		axis[0] = x / f;
		axis[1] = y / f;
		axis[2] = z / f;
	}
	else
	{
		w = 1.0;
		axis[0] = 0.0;
		axis[1] = 0.0;
		axis[2] = 0.0;
	}

	// atan2() provides a more accurate angle result than acos()
	return 2.0*atan2(f, w);
}

float ver[8][3] =
{
	{ -10.0,-8.0,10.0 },
	{ -10.0,-8.5,10.0 },
	{ 10.0,-8.5,10.0 },
	{ 10.0,-8.0,10.0 },
	{ -10.0,-8.0,-10.0 },
	{ -10.0,-8.5,-10.0 },
	{ 10.0,-8.5,-10.0 },
	{ 10.0,-8.0,-10.0 },
};

float hver[8][3] =
{
	{ -0.5, 0.0,  1.0 },
	{ -0.5, 0.0,  1.0 },
	{ 0.5, 0.0,  1.0 },
	{ 0.5, 0.0,  1.0 },
	{ -0.5, 0.0, -1.0 },
	{ -0.5, 0.0, -1.0 },
	{ 0.5, 0.0, -1.0 },
	{ 0.5, 0.0, -1.0 },
};

void quad(int a, int b, int c, int d)
{
	glBegin(GL_QUADS);
	glVertex3fv(ver[a]);
	glVertex3fv(ver[b]);
	glVertex3fv(ver[c]);
	glVertex3fv(ver[d]);
	glEnd();
}

void hquad(int a, int b, int c, int d)
{
	glBegin(GL_QUADS);
	glVertex3fv(hver[a]);
	glVertex3fv(hver[b]);
	glVertex3fv(hver[c]);
	glVertex3fv(hver[d]);
	glEnd();
}

void DrawHand()
{
	//glDisable(GL_LIGHTING);
	glColor3f(0.9, 0.2, 0.2);
	hquad(0, 3, 2, 1);

	hquad(2, 3, 7, 6);

	hquad(0, 4, 7, 3);

	hquad(1, 2, 6, 5);

	hquad(4, 5, 6, 7);

	hquad(0, 1, 5, 4);
	//glEnable(GL_LIGHTING);
	glColor3f(0.8, 0.4, 0.2);
}
void DrawGrid()
{
	//glDisable(GL_LIGHTING);
	glColor3f(0.9, 0.9, 0.9);
	quad(0, 3, 2, 1);
	quad(2, 3, 7, 6);
	quad(0, 4, 7, 3);
	quad(1, 2, 6, 5);
	quad(4, 5, 6, 7);
	quad(0, 1, 5, 4);
	/*glBegin(GL_QUADS);
	glVertex3f(-10.1, -8.1, 10);
	glVertex3f(10.1, -8.1, 10);
	glVertex3f(10.5, -8.51, -10);
	glVertex3f(-10.5, -8.51, -10);
	glEnd();*/
	//glEnable(GL_LIGHTING);
}

void drawElipsoid(int SLICES, int STACKS, float SCALE_X, float SCALE_Y, float SCALE_Z) {
	glEnable(GL_NORMALIZE);

	//top of cylinder
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	for (int i = 0; i <= SLICES; ++i) {
		float x = SCALE_X * sin(i * 2 * PI / SLICES);
		float y = SCALE_Y * cos(i * 2 * PI / SLICES);
		glVertex3f(x, y, 0.0f);
	}
	glEnd();

	//main part of cylinder
	for (int j = 0; j < STACKS; ++j) {
		glBegin(GL_TRIANGLE_STRIP);
		for (int i = 0; i <= SLICES; ++i) {
			float x = SCALE_X * sin(i * 2 * PI / SLICES);
			float y = SCALE_Y * cos(i * 2 * PI / SLICES);
			float z = j * SCALE_Z / STACKS;
			glNormal3f(x, y, 0.0f);
			glVertex3f(x, y, z);
			z = (j + 1) * SCALE_Z / STACKS;
			glVertex3f(x, y, z);
		}
		glEnd();
	}

	//bottom of cylinder
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, SCALE_Z);
	for (int i = 0; i <= SLICES; ++i) {
		float x = SCALE_X * sin(i * 2 * PI / SLICES);
		float y = SCALE_Y * cos(i * 2 * PI / SLICES);
		glVertex3f(x, y, SCALE_Z);
	}
	glEnd();
}

void traverse(treenode* root)
{
	if (root == NULL) return;
	glPushMatrix();
	glMultMatrixf(root->m);
	root->f();
	if (root->child != NULL) traverse(root->child);
	glPopMatrix();
	if (root->sibling != NULL) traverse(root->sibling);
}

void torso()
{
	glPushMatrix();

	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glRotatef(180.0, 0.0, 0.0, 1.0);
	gluCylinder(t, TORSO_RADIUS / 1.2, TORSO_RADIUS, TORSO_HEIGHT, 3, 30);
	//glutSolidCube(TORSO_RADIUS);
	//drawElipsoid(10,10,1.0,1.0,1.0);
	glPopMatrix();
}

void head()
{
	glPushMatrix();

	glTranslatef(0.0, HEAD_HEIGHT, 0.0);
	glScalef(HEAD_RADIUS, HEAD_HEIGHT, HEAD_RADIUS);
	gluSphere(h, HEAD_RADIUS, 15, 15);
	//drawCoordinate();
	//glasses
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glTranslatef(0.0f, -0.4f, -0.15f);
	gluCylinder(h, 0.9*HEAD_RADIUS, 0.9*HEAD_RADIUS, HEAD_HEIGHT / 5, 10, 10);

	glPopMatrix();
}

void neck()
{
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	gluCylinder(nk, NECK_RADIUS, NECK_RADIUS, NECK_HEIGHT, 10, 10);
	glPopMatrix();
}

void rightShoulder()
{
	glPushMatrix();
	gluSphere(relb, SHOULDER_RADIUS, 15, 15);
	glPopMatrix();
}

void leftShoulder()
{
	glPushMatrix();
	gluSphere(lelb, SHOULDER_RADIUS, 15, 15);
	glPopMatrix();
}

void rightElbow()
{
	glPushMatrix();
	gluSphere(relb, ELBOW_RADIUS, 15, 15);
	glPopMatrix();
}

void leftElbow()
{
	glPushMatrix();
	gluSphere(lelb, ELBOW_RADIUS, 15, 15);
	glPopMatrix();
}

void rightKnee()
{
	glPushMatrix();
	gluSphere(rknee, KNEE_RADIUS, 15, 15);
	glPopMatrix();
}

void leftKnee()
{
	glPushMatrix();
	gluSphere(lknee, KNEE_RADIUS, 15, 15);
	glPopMatrix();
}

void leftFoot()
{
	glPushMatrix();
	gluSphere(lknee, FOOT_RADIUS, 15, 15);
	glPopMatrix();
}

void rightFoot()
{
	glPushMatrix();
	gluSphere(lknee, FOOT_RADIUS, 15, 15);
	glPopMatrix();
}

void rightHand()
{
	glPushMatrix();
	//gluSphere(lknee, HAND_RADIUS, 15, 15);
	glutSolidCube(1);
	glPopMatrix();
}

void leftHand()
{
	glPushMatrix();
	//gluSphere(lknee, HAND_RADIUS, 15, 15);
	glutSolidCube(1);
	/*glRotatef(-90.0, 1.0, 0.0, 0.0);
	glColor3f(1.0,0,0);
	glutSolidCone(1, 1, 4, 30);*/
	/*glColor3f(0.0, 0, 1);
	glTranslatef(0.0f, -0.4f, -0.15f);
	glutSolidCone(1, 1, 2, 30);*/
	//DrawHand();
	glPopMatrix();
}

void left_upper_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(lua, UPPER_ARM_RADIUS, UPPER_ARM_RADIUS - 0.1, UPPER_ARM_HEIGHT, 10, 10);
	glPopMatrix();
	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void left_lower_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(lla, LOWER_ARM_RADIUS - 0.1, LOWER_ARM_RADIUS - 0.15, LOWER_ARM_HEIGHT, 10, 10);
	glPopMatrix();
	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void right_upper_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(rua, UPPER_ARM_RADIUS, UPPER_ARM_RADIUS - 0.1, UPPER_ARM_HEIGHT, 10, 10);
	glPopMatrix();
	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void right_lower_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(rla, LOWER_ARM_RADIUS - 0.1, LOWER_ARM_RADIUS - 0.15, LOWER_ARM_HEIGHT, 10, 10);
	glPopMatrix();
	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void left_upper_leg()
{
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	gluCylinder(lul, UPPER_LEG_RADIUS, UPPER_LEG_RADIUS - 0.1, UPPER_LEG_HEIGHT, 10, 10);
	glPopMatrix();
}

void left_lower_leg()
{
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	gluCylinder(lll, LOWER_LEG_RADIUS - 0.1, LOWER_LEG_RADIUS - 0.2, LOWER_LEG_HEIGHT, 10, 10);
	glPopMatrix();
}

void right_upper_leg()
{
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	gluCylinder(rul, UPPER_LEG_RADIUS, UPPER_LEG_RADIUS - 0.1, UPPER_LEG_HEIGHT, 10, 10);
	glPopMatrix();
}

void right_lower_leg()
{
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	gluCylinder(rll, LOWER_LEG_RADIUS - 0.1, LOWER_LEG_RADIUS - 0.2, LOWER_LEG_HEIGHT, 10, 10);
	glPopMatrix();
}

void drawText(char*string, int x, int y)
{
	char *c;
	glPushMatrix();
	glTranslatef(x, y, 0);
	glScalef(0.1, -0.1, 1);
	for (c = string; *c != '\0'; c++)
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
	}
	glPopMatrix();
}

void renderCylinder(float x1, float y1, float z1, float x2, float y2, float z2, float radius, int subdivisions, GLUquadricObj *quadric)
{
	float vx = x2 - x1;
	float vy = y2 - y1;
	float vz = z2 - z1;
	float v = sqrt(vx*vx + vy*vy + vz*vz);
	float ax;

	if (fabs(vz) < 1.0e-3) {
		ax = 57.2957795*acos(vx / v); // rotation angle in x-y plane
		if (vy <= 0.0)
			ax = -ax;
	}
	else {
		ax = 57.2957795*acos(vz / v); // rotation angle
		if (vz <= 0.0)
			ax = -ax;
	}

	float rx = -vy*vz;
	float ry = vx*vz;

	glPushMatrix();
	//draw the cylinder body
	glTranslatef(x1, y1, z1);
	if (fabs(vz) < 1.0e-3) {
		glRotated(90.0, 0, 1, 0.0); // Rotate & align with x axis
		glRotated(ax, -1.0, 0.0, 0.0); // Rotate to point 2 in x-y plane
	}
	else {
		glRotated(ax, rx, ry, 0.0); // Rotate about rotation vector
	}
	gluQuadricOrientation(quadric, GLU_OUTSIDE);
	gluCylinder(quadric, radius, radius, v, subdivisions, 1);

	////draw the first cap
	//gluQuadricOrientation(quadric, GLU_INSIDE);
	//gluDisk(quadric, 0.0, radius, subdivisions, 1);
	//glTranslatef(0, 0, v);

	////draw the second cap
	//gluQuadricOrientation(quadric, GLU_OUTSIDE);
	//gluDisk(quadric, 0.0, radius, subdivisions, 1);
	glPopMatrix();
}

void IntializeRobotLight()
{
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_ambient[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess = { 100.0 };
	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat light_diffuse[] = { 0.4, 0.4, 0.4, 1.0 };
	//GLfloat light_diffuse[] = { 1, 0.87, 0.75, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { -10.0, 0.0, 10.0, 0.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

	glEnable(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	//glClearColor(1.0, 1.0, 1.0, 1.0);
	//glClearColor(0, 0, 0, 0);

	glColor3f(1.0, 0.0, 0.0);
}
void
initLightAndMaterial(void)
{
	static float ambient[] =
	{ 0.1, 0.1, 0.1, 1.0 };
	static float diffuse[] =
	{ 0.5, 1.0, 1.0, 1.0 };
	static float position[] =
	{ 90.0, 90.0, 150.0, 0.0 };

	static float front_mat_shininess[] =
	{ 60.0 };
	static float front_mat_specular[] =
	{ 0.2, 0.2, 0.2, 1.0 };
	static float front_mat_diffuse[] =
	{ 0.5, 0.5, 0.28, 1.0 };
	static float back_mat_shininess[] =
	{ 60.0 };
	static float back_mat_specular[] =
	{ 0.5, 0.5, 0.2, 1.0 };
	static float back_mat_diffuse[] =
	{ 1.0, 0.9, 0.2, 1.0 };

	static float lmodel_ambient[] =
	{ 1.0, 1.0, 1.0, 1.0 };

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glMaterialfv(GL_FRONT, GL_SHININESS, front_mat_shininess);
	glMaterialfv(GL_FRONT, GL_SPECULAR, front_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, front_mat_diffuse);
	glMaterialfv(GL_BACK, GL_SHININESS, back_mat_shininess);
	glMaterialfv(GL_BACK, GL_SPECULAR, back_mat_specular);
	glMaterialfv(GL_BACK, GL_DIFFUSE, back_mat_diffuse);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	//glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_SMOOTH);
}

void InitializeLight()
{
	GLfloat mat_ambient_0[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat mat_diffuse_0[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_specular_0[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_position_0[] = { -8.5f, -8.5f, 10.8f, 1.0f };
	//GLfloat mat_position_0[] = { 0.5f, 0.5f, 0.8f, 0.0f};	
	GLfloat mat_shininess[] = { 128.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, mat_ambient_0);
	//glLightdv(GL_LIGHT0, GL_DIFFUSE, mat_diffuse_0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, mat_specular_0);
	glLightfv(GL_LIGHT0, GL_POSITION, mat_position_0);

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient_0);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular_0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse_0);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	GLfloat mat_ambient_1[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat mat_diffuse_1[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_specular_1[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_position_1[] = { 9.5f, 9.5f, 10.0f, 1.0f };
	GLfloat mat_spotdir_1[] = { 15.0f, 10.0f, 10.0f };

	glLightfv(GL_LIGHT1, GL_SPECULAR, mat_specular_1);
	glLightfv(GL_LIGHT1, GL_POSITION, mat_position_1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, mat_diffuse_1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, mat_ambient_1);

	/*glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.5f);
	glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.5f);
	glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.2f);*/

	glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 120.0f);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, mat_spotdir_1);
	glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 2.0f);

	GLfloat mat_ambient_2[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat mat_diffuse_2[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_specular_2[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	//GLfloat mat_position_2[] = { 0.9f, -0.5f, 0.6f, 1.0f};	
	//GLfloat mat_position_2[] = { -0.5f, 0.5f, 0.6f, 1.0f};	
	GLfloat mat_position_2[] = { -8.5f, 9.0f, 8.6f, 1.0f };

	glLightfv(GL_LIGHT2, GL_SPECULAR, mat_specular_2);
	glLightfv(GL_LIGHT2, GL_POSITION, mat_position_2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, mat_diffuse_2);
	glLightfv(GL_LIGHT2, GL_AMBIENT, mat_ambient_2);


	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	::glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
	//::glClearColor(0.0f, 0.0f, 0.0f, 0.0f);	
	::glShadeModel(GL_SMOOTH);
	//::glShadeModel(GL_FLAT);	

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	glFrontFace(GL_CCW);

}

void drawQuads(float x, float y, float z)
{
	double theta, phi;

	theta = acos(z) * 180 / PI;
	phi = atan2(y, x) * 180 / PI;
	//cout << theta  << "," << phi  << endl;
	theta = round(theta / 8) * 8 + 2;
	phi = round(phi / 6) * 6;

	//cout << theta  << "," << phi  << endl;

	theta = theta * PI / 180;
	phi = phi * PI / 180;

	x = 1.01*sin(theta)*cos(phi); y = 1.01*sin(theta)*sin(phi); z = 1.01*cos(theta);
	glColor4f(1.0, 0.6, 0.6, 0.5);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_SRC_ALPHA);
	glBegin(GL_QUADS);                      // Draw A Quad
											// Top right
	glVertex3f(x, y, z);

	// Top left
	phi = phi + (6 * PI / 180);
	x = 1.01*sin(theta)*cos(phi); y = 1.01*sin(theta)*sin(phi); z = 1.01*cos(theta);
	glVertex3f(x, y, z);

	// Bottom Left
	theta = theta - (8 * PI / 180);
	x = 1.01*sin(theta)*cos(phi); y = 1.01*sin(theta)*sin(phi); z = 1.01*cos(theta);
	glVertex3f(x, y, z);


	// Bottom Right
	phi = phi - (6 * PI / 180);
	x = 1.01*sin(theta)*cos(phi); y = 1.01*sin(theta)*sin(phi); z = 1.01*cos(theta);
	glVertex3f(x, y, z);
	glEnd();

	glDisable(GL_BLEND);
}


void renderCylinder_convenient(float x1, float y1, float z1, float x2, float y2, float z2, float radius, int subdivisions)
{
	//the same quadric can be re-used for drawing many cylinders
	GLUquadricObj *quadric = gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);
	renderCylinder(x1, y1, z1, x2, y2, z2, radius, subdivisions, quadric);
	gluDeleteQuadric(quadric);
}

void showInfo(/*std::stringstream &ss, int tWidth, int tHeight*/)
{
	
	
	// backup current model-view matrix
	glPushMatrix();                     // save current modelview matrix
	glLoadIdentity();                   // reset modelview matrix

										// set to 2D orthogonal projection
	glMatrixMode(GL_PROJECTION);        // switch to projection matrix
	glPushMatrix();                     // save current projection matrix
	glLoadIdentity();                   // reset projection matrix
	gluOrtho2D(0, width/2, 0, height);  // set to orthogonal projection
	
	//Vector3 v1 = sphereVector;                          // first vector on sphere
	//Vector3 v2 = trackball.getVector(mouseX, mouseY);   // second vector on sphere
	//float angle = RAD2DEG * acosf(v1.dot(v2) / (v1.length() * v2.length()));

	// for print infos

	//std::stringstream ss;
	//ss << "Standard : (" << connectXS.ax[0] << ", " << connectXS.ax[1] << ", " << connectXS.ax[2] << ", " << connectXS.ax[3] << ")";
	
	
	/////////////////////
	isMatched = false;
	if (diff < threshold && diff == diff1)
	{
		std::stringstream ss;
		ss << "Standard Curl: (" << diff1 << "[Match])";
		drawString(ss.str().c_str(), width / 4 + 150, height / 2 - TEXT_HEIGHT, mcolor, font);
		//ss.str("");
		//printf("Standard Curl:%f (Match)\n", diff1);
		isMatched = true;
	}
	else
	{
		std::stringstream ss;
		ss << "Standard Curl: (" << diff1 << "[No-Match])";
		drawString(ss.str().c_str(), width / 4 + 150, height / 2 - TEXT_HEIGHT, color, font);
		//printf("Standard Curl:%f (No-Match)\n", diff1);
	}



	//if (!isMatched)
	{


		if (diff < threshold && diff == diff2)
		{
			std::stringstream ss;
			ss << "Close Curl: (" << diff2 << "[Match])";
			drawString(ss.str().c_str(), width / 4 + 150, height / 2 - (2 * TEXT_HEIGHT), mcolor, font);
				
			isMatched = true;
		}
		else
		{
			std::stringstream ss;
			ss << "Close Curl: (" << diff2 << "[No-Match])";
			drawString(ss.str().c_str(), width / 4 + 150, height / 2 - (2 * TEXT_HEIGHT), color, font);
			
		}
	}

	//if (!isMatched)
	{


		if (diff < threshold && diff == diff3)
		{
			std::stringstream ss;
			ss << "Wide Curl: (" << diff3 << "[Match])";
			drawString(ss.str().c_str(), width / 4 + 150, height / 2 - (3 * TEXT_HEIGHT), mcolor, font);
			
			isMatched = true;
		}
		else
		{
			std::stringstream ss;
			ss << "Wide Curl: (" << diff3 << "[No-Match])";
			drawString(ss.str().c_str(), width / 4 + 150, height / 2 - (3 * TEXT_HEIGHT), color, font);
			
		}
	}


	if (!isMatched)
	{
		std::stringstream ss;
		ss << "No match found!!";
		drawString(ss.str().c_str(), width / 4 + 150, height / 2 - (4 * TEXT_HEIGHT), mcolor, font);
		
	}
	
	///////////////////
	std::stringstream ss;
	ss << "Press SPACE mode.";
	drawString(ss.str().c_str(), 2, 2, color, font);
	ss.str("");

	// unset floating format
	ss << std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);

	// restore projection matrix
	glPopMatrix();                   // restore to previous projection matrix

									 // restore modelview matrix
	glMatrixMode(GL_MODELVIEW);      // switch to modelview matrix
	glPopMatrix();                   // restore to previous modelview matrix
}

void Robotdisplay(void)
{
	glViewport(0, 0, width / 2, height);
	glScissor(0, 0, width / 2, height);
	//::glClearColor(0.8f, 0.8f, 0.8f, 0.0f);
	/*glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHT1);
	glDisable(GL_LIGHT2);*/
	//IntializeRobotLight();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-15, 15, -15, 15, -15, 15);
	glRotatef(rotate_, xr, yr, zr);
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//glScalef(zoominout, zoominout, zoominout);
	DrawGrid();
	glColor3f(0.8, 0.4, 0.2);
	traverse(&torso_node);
	
	glPushMatrix();
	glTranslatef(0.0, 1.0, 2.0);
	glRotatef(-180, 0, 1, 0);
	glRotatef(-180, 1, 0, 0);
	drawCoordinate();
	glPopMatrix();

	showInfo();
	/*glPushMatrix();
	glTranslatef(5.5, -5.0, 0.0);
	drawCoordinate();
	float P_axis[3] = { 0.0,0.0,0.0 };
	float P_angle = (qP.GetRotationAngleAndAxis(P_axis))* 180.0 / 3.14159;
	glColor3f(0.5, 0.5, 0.0);
	glRotatef(P_angle, P_axis[0], P_axis[1], P_axis[2]);
	GLUquadricObj *Pri_axis = gluNewQuadric();
	gluQuadricOrientation(Pri_axis, GLU_OUTSIDE);
	gluCylinder(Pri_axis, 0.2, 0.2, 3.0, 10, 1);
	gluDeleteQuadric(Pri_axis);
	glPopMatrix();*/

	//glutSwapBuffers();
}
void PrincipalAxis(void)
{
	//glDisable(GL_SMOOTH);
	//glDisable(GL_LIGHTING);
	//glDisable(GL_LIGHT0);
	//glDisable(GL_DEPTH_TEST);
	//glDisable(GL_LIGHTING);

	InitializeLight();
	//::glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
	//initLightAndMaterial();
	glViewport(width / 2, 0, width / 2, height);
	glScissor(width / 2, 0, width / 2, height);
	/*glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);*/
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-2, 2, -2, 2, -1, 10);

	glMatrixMode(GL_MODELVIEW);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glLoadIdentity();
	gluLookAt(
		pointTranslateX, pointTranslateY, pointTranslateZ,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f
	);

	glScalef(zval - 2.5, zval - 2.5, zval - 2.5);
	//glPushMatrix();

	//glDisable(GL_CULL_FACE);
	glDisable(GL_COLOR_MATERIAL);
	//glDisable(GL_SPOT_CUTOFF);
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	glColor3f(1.0, 1.0, 1.0);
	glBindTexture(GL_TEXTURE_2D, texture_id[0]);
	glRotatef(-180, 0, 1, 0);
	glRotatef(-90, 1, 0, 0);
	gluSphere(sphere, 1.0, 50, 50);
	//glutSolidSphere(1.0, 50, 50);
	//glutWireSphere(1.0, 20, 20);
	//glPopMatrix();
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);

	
	drawcenterCoordinate();
	float r = 1, g = 0, b = 0;
	/*if (!fileClose && indexP > 0) {*/
	int j = 0;
	for (int i = 1; i < indexP; i++)
	{
		if (PA_data[i][0] == 0)
			continue;

		glColor3f(r, g, b);

		if (PA_data[i][0] == 9) {
			r = 0;
			g = 1;
			b = 0;

			if (j == 1)
			{
				b = 1;
				g = 0;
			}
			j++;
			glColor3f(r, g, b);
			//cout << "R:" << r << "\tG:" << g << "\tB:" << b << endl;
			continue;
		}
		float fnorm = sqrt(PA_data[i][1] * PA_data[i][1] + PA_data[i][2] * PA_data[i][2] + PA_data[i][3] * PA_data[i][3]);
		glPushMatrix();
		glTranslatef(1.011*PA_data[i][1] / fnorm, 1.011*PA_data[i][2] / fnorm, 1.011*PA_data[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		/*glRotatef(PA_data[i][0],PA_data[i][1], PA_data[i][2], PA_data[i][3]);
		drawCoordinate(0.1);*/
		glPopMatrix();

		glColor3f(r, g + 0.64, b);

		fnorm = sqrt(uPA_data[i][1] * uPA_data[i][1] + uPA_data[i][2] * uPA_data[i][2] + uPA_data[i][3] * uPA_data[i][3]);
		glPushMatrix();
		glTranslatef(1.011*uPA_data[i][1] / fnorm, 1.011*uPA_data[i][2] / fnorm, 1.011*uPA_data[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		/*glRotatef(PA_data[i][0],PA_data[i][1], PA_data[i][2], PA_data[i][3]);
		drawCoordinate(0.1);*/
		glPopMatrix();
		// draw quads
		/*glPushMatrix();
		drawQuads(PA_data[i][1] / fnorm, PA_data[i][2] / fnorm, PA_data[i][3] / fnorm);
		glPopMatrix();*/
		//draw quads end

	}



	for (int i = 0; i < dbCount; i++)
	{
		/*if (PA_data[i][0] == 0)
			continue;*/

		glColor3f(1, 0, 1);

		
		float fnorm = sqrt(uDB_data[i][1] * uDB_data[i][1] + uDB_data[i][2] * uDB_data[i][2] + uDB_data[i][3] * uDB_data[i][3]);
		glPushMatrix();
		glTranslatef(1.011*uDB_data[i][1] / fnorm, 1.011*uDB_data[i][2] / fnorm, 1.011*uDB_data[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		/*glRotatef(PA_data[i][0],PA_data[i][1], PA_data[i][2], PA_data[i][3]);
		drawCoordinate(0.1);*/
		glPopMatrix();

		glColor3f(1, 0.4, 0);

		fnorm = sqrt(lDB_data[i][1] * lDB_data[i][1] + lDB_data[i][2] * lDB_data[i][2] + lDB_data[i][3] * lDB_data[i][3]);
		glPushMatrix();
		glTranslatef(1.011*lDB_data[i][1] / fnorm, 1.011*lDB_data[i][2] / fnorm, 1.011*lDB_data[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		/*glRotatef(PA_data[i][0],PA_data[i][1], PA_data[i][2], PA_data[i][3]);
		drawCoordinate(0.1);*/
		glPopMatrix();
		// draw quads
		if (!fileClose) 
		{
			glColor3f(0, 0, 0);
			glPushMatrix();
			//drawQuads(drawGrid[0][1], drawGrid[0][2], drawGrid[0][3]);
			glTranslatef(drawGrid[0][1], drawGrid[0][2], drawGrid[0][3]);
			glutSolidSphere(0.025, 30, 30);
			glPopMatrix();
		}
		
		//draw quads end

	}


	//for (int i = 0; i < indexP; i++)
	//{
	//	
	//	glColor3f(1.0, 0.0, 0.0);
	//	double ytk_out_Norm = sqrt(y_tk_out[i][0] * y_tk_out[i][0] + y_tk_out[i][1] * y_tk_out[i][1] + y_tk_out[i][2] * y_tk_out[i][2]);
	//	glPushMatrix();
	//	glTranslatef(y_tk_out[i][0]/ ytk_out_Norm, y_tk_out[i][1]/ ytk_out_Norm, y_tk_out[i][2]/ ytk_out_Norm);
	//	glutSolidSphere(0.005, 30, 30);
	//	/*glRotatef(PA_data[i][0],PA_data[i][1], PA_data[i][2], PA_data[i][3]);
	//	drawCoordinate(0.1);*/
	//	glPopMatrix();

	//}

	/*for (int i = 0; i < averageIndexP; i++)
	{
	if (PA_data[i][0] == 0)
	continue;
	float avg_x = AVG_PA_data[i][1];
	float avg_y = AVG_PA_data[i][2];
	float avg_z = AVG_PA_data[i][3];

	glPushMatrix();
	glTranslatef((avg_x) / sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z),
	avg_y / sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z)
	, avg_z / sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z));
	glColor3f(1.0, 0.0, 0.0);
	glutSolidSphere(0.02, 20, 20);
	glPopMatrix();
	}*/



	//for (int i = 0; i < indexP - 1; i++)
	//{
	//	
	//	glPushMatrix();
	//	//glColor3f(0.4, 0.1, 0.98);
	//	glColor3f(0.9, 0.0, 0.9);
	//	renderCylinder_convenient(y_tk_out[i][0], y_tk_out[i][1], y_tk_out[i][2], y_tk_out[i + 1][0], y_tk_out[i + 1][1], y_tk_out[i + 1][2], 0.01, 20);
	//	glPopMatrix();
	//}
	glPopMatrix();
	/*glDisable(GL_LIGHTING);
	glEnable(GL_LIGHTING);
	glEnable(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);*/
}

void Display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_POLYGON_SMOOTH);

	/*char pcResult[250] = 'Hello';
	sprintf(charstring, "PC : %d ", pcResult);
	drawText(charstring, 10, 80);*/

	Robotdisplay();

	//glDisable(GL_COLOR_MATERIAL);
	PrincipalAxis();
	//glEnable(GL_COLOR_MATERIAL);
	/*fourSphere();
	TextDispaly();*/

	glFlush();
	glutSwapBuffers();
}

void inverseKinematics()
{
	//switch (animation)
	//{
	//case 0: //an to teleytaio animation einai to 0 antistrepse to
	//	if (theta[5] < 180.0)
	//	{
	//		theta[5] += STEP;
	//		theta[3] += STEP;
	//		theta[1] -= 0.2*STEP;
	//	}
	//	else animation = option; //an exei antistrafei tote eimaste stin arxiki thesi kai to neo animation einai to option

	//	glPushMatrix();

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[5], 1.0, 0.0, 0.0);

	//	glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[3], 1.0, 0.0, 0.0);

	//	glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0.0, TORSO_HEIGHT - 0.25*NECK_HEIGHT, 0.0);
	//	glRotatef(theta[1], 1.0, 0.0, 0.0);

	//	glGetFloatv(GL_MODELVIEW_MATRIX, nk_node.m);

	//	glPopMatrix();
	//	break;
	//case 1:
	//	if (theta[9] < 180.0)
	//	{
	//		theta[9] += STEP;
	//		theta[10] -= STEP;
	//	}
	//	else animation = option;

	//	glPushMatrix();

	//	glLoadIdentity();
	//	glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	//	glRotatef(theta[9], 1.0, 0.0, 0.0);

	//	glGetFloatv(GL_MODELVIEW_MATRIX, rul_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0.0, UPPER_LEG_HEIGHT, 0.0);

	//	glGetFloatv(GL_MODELVIEW_MATRIX, rknee_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
	//	glRotatef(theta[10], 1.0, 0.0, 0.0);

	//	glGetFloatv(GL_MODELVIEW_MATRIX, rll_node.m);

	//	glPopMatrix();
	//	break;
	//	/*case 6: case -1:
	//	if (theta[9] < 180.0)
	//	{
	//	theta[9] += STEP;
	//	theta[10] -= STEP;
	//	theta[7] += STEP;
	//	theta[8] -= STEP;
	//	theta[5] -= 1.3*STEP;
	//	theta[6] += STEP;
	//	theta[3] += STEP;
	//	theta[4] -= STEP;
	//	horizontal -= 0.03*STEP;
	//	}
	//	else animation = option;

	//	glPushMatrix();

	//	glLoadIdentity();
	//	glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	//	glRotatef(theta[9], 1.0, 0.0, -1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rul_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
	//	glRotatef(theta[10], 1.0, 0.0, -1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rll_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, horizontal, 0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, torso_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[5], -1.0, 0.0, -1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[6], 1.0, 1.0, 1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rla_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[3], 1.0, 0.0, -1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[4], -1.0, -1.0, 1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

	//	glPopMatrix();
	//	break;*/
	//case 7: case -2:
	//	if (theta[9] < 180.0)
	//	{
	//		theta[9] += STEP;
	//		theta[10] -= STEP;
	//		theta[7] += STEP;
	//		theta[8] -= STEP;
	//		theta[5] -= STEP;
	//		theta[6] += STEP;
	//		theta[3] += STEP;
	//		theta[4] -= STEP;
	//		horizontal -= 0.03*STEP;
	//	}
	//	else animation = option;

	//	glPushMatrix();

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[5], 0.0, 0.0, 1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[6], 0.0, 0.0, 1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rla_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[3], 0.0, 0.0, 1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, 0, 0.0);
	//	glRotatef(theta[4], 0.0, 0.0, 1.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

	//	glLoadIdentity();
	//	glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	//	glRotatef(theta[9], 1.0, 0.0, 0.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rul_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
	//	glRotatef(theta[10], 1.0, 0.0, 0.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, rll_node.m);

	//	glLoadIdentity();
	//	glTranslatef(-TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	//	glRotatef(theta[7], 1.0, 0.0, 0.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, lul_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
	//	glRotatef(theta[8], 1.0, 0.0, 0.0);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, lll_node.m);

	//	glLoadIdentity();
	//	glTranslatef(0, -horizontal, -horizontal);
	//	glGetFloatv(GL_MODELVIEW_MATRIX, torso_node.m);

	//	glPopMatrix();
	//	break;
	//case 6:

	//{glPushMatrix();
	//float mdlv[16];
	//glGetFloatv(GL_MODELVIEW_MATRIX, mdlv);
	//vtkQuaternion<float> q = vtkQuaternion<float>(connectXS.ax[3], connectXS.ax[0], connectXS.ax[1], connectXS.ax[2]);

	//float qw = sqrt(1 + mdlv[0] + mdlv[5] + mdlv[10]);
	//float qx = (mdlv[9] - mdlv[6]) / (4 * qw);
	//float qy = (mdlv[2] - mdlv[8]) / (4 * qw);
	//float qz = (mdlv[4] - mdlv[1]) / (4 * qw);
	//vtkQuaternion<float> qA = vtkQuaternion<float>(qw, qx, qy, qz);

	//q = qA.Inverse() * q;
	//q.Normalize();

	//float axis[3];
	//float angle = q.GetRotationAngleAndAxis(axis);

	//glLoadIdentity();
	////glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_ARM_HEIGHT, 0.0);
	//glTranslatef(0.0, 0.0, 0.0);
	////glRotatef(90, 1.0, 0.0, 0.0);
	//glRotatef(angle * 180.0 / 3.14159, axis[2], axis[0], -axis[1]);
	//glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

	//glLoadIdentity();
	//glTranslatef(0.0, UPPER_ARM_HEIGHT, 0.0);

	//glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);



	//glGetFloatv(GL_MODELVIEW_MATRIX, mdlv);
	//q = vtkQuaternion<float>(connectXS.ax2[3], connectXS.ax2[0], connectXS.ax2[1], connectXS.ax2[2]);

	//qw = sqrt(1 + mdlv[0] + mdlv[5] + mdlv[10]);
	//qx = (mdlv[9] - mdlv[6]) / (4 * qw);
	//qy = (mdlv[2] - mdlv[8]) / (4 * qw);
	//qz = (mdlv[4] - mdlv[1]) / (4 * qw);
	//qA = vtkQuaternion<float>(qw, qx, qy, qz);

	//q = qA.Inverse() * q;
	//q.Normalize();

	//axis[3];
	//angle = q.GetRotationAngleAndAxis(axis);

	//glLoadIdentity();
	//glTranslatef(0.0, ELBOW_RADIUS / 2, 0.0);
	//glRotatef(theta[10], 1.0, 0.0, 0.0);
	//glRotatef(angle * 180.0 / 3.14159, axis[2], axis[0], axis[1]);
	//glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

	//glPopMatrix(); }
	//break;
	//default: animation = option;
	//}
}

void matchDBTrajectory(char * Ufile, char * Lfile)
{	
	diff = 0;
	diff1 = Comparision::getDiffBtwTrajectory("Load\\Standard\\UFormFile.csv", "Load\\Standard\\LFormFile.csv", Ufile, Lfile);
	diff2 = Comparision::getDiffBtwTrajectory("Load\\Close\\UFormFile.csv", "Load\\Close\\LFormFile.csv", Ufile, Lfile);
	diff3 = Comparision::getDiffBtwTrajectory("Load\\Wide\\UFormFile.csv", "Load\\Wide\\LFormFile.csv", Ufile, Lfile);
	
	if (diff1 < diff2)
	{
		diff = diff1;
	}
	else if (diff2 < diff3)
	{
		diff = diff2;
	}
	else 
		diff = diff3;
		
	if (diff3 < diff1)
	{
		diff = diff3;
	}
	
	//if (diff < 0.1 && diff==diff1)
	//{
	//	std::stringstream ss;
	//	ss << "Standard Curl: (" << diff1 << "[Match])";
	//	showInfo(ss, width / 4 + 150, height / 2 + 250);
	//	printf("Standard Curl:%f (Match)\n", diff1);
	//	isMatched = true;
	//}
	//else
	//{
	//	std::stringstream ss;
	//	ss << "Standard Curl: (" << diff1 << "[No-Match])";
	//	showInfo(ss, width / 4 + 150, height / 2 + 250);
	//	printf("Standard Curl:%f (No-Match)\n", diff1);
	//}

	//

	////if (!isMatched)
	//{
	//	

	//	if (diff < 0.1 && diff == diff2)
	//	{
	//		std::stringstream ss;
	//		ss << "Close Curl: (" << diff2 << "[Match])";
	//		showInfo(ss, width / 4 + 150, height / 2 + 250-(2*TEXT_HEIGHT));
	//		printf("Close Curl:%f (Match)\n", diff2);
	//		isMatched = true;
	//	}
	//	else
	//	{
	//		std::stringstream ss;
	//		ss << "Close Curl: (" << diff2 << "[No-Match])";
	//		showInfo(ss, width / 4 + 150, height / 2 + 250 - (2 * TEXT_HEIGHT));
	//		printf("Close Curl:%f (No-Match)\n", diff2);
	//	}
	//}
	//
	////if (!isMatched)
	//{
	//	

	//	if (diff < 0.1 && diff == diff3)
	//	{
	//		std::stringstream ss;
	//		ss << "Wide Curl: (" << diff3 << "[Match])";
	//		showInfo(ss, width / 4 + 150, height / 2 + 250 - (3 * TEXT_HEIGHT));
	//		printf("Wide Curl:%f (Match)\n", diff3);
	//		isMatched = true;
	//	}
	//	else
	//	{
	//		std::stringstream ss;
	//		ss << "Wide Curl: (" << diff3 << "[No-Match])";
	//		showInfo(ss, width / 4 + 150, height / 2 + 250 - (3 * TEXT_HEIGHT));
	//		printf("Wide Curl:%f (not-Match)\n", diff3);
	//	}
	//}
	//

	//if (!isMatched)
	//{
	//	std::stringstream ss;
	//	ss << "No match found!!";
	//	showInfo(ss, width / 4 + 150, height / 2 + 250 - (4 * TEXT_HEIGHT));
	//	printf("No match found");
	//}
}


void idle()
{
	switch (option)
	{

	case 0:
	{
		bool stop_restart = connectXS.stop_and_restart_everything;
		if (stop_restart) {
			std::cout << "Thread closed restart program again........!" << std::endl;
		}
		else {
			//std::cout << "Connect xSens........!" << std::endl;
			connectXS.isRunning = true;
			connectXS.bxMTdisconnect = false;
		}
	}
	break;

	case 1:
	{
		connectXS.waitForConnections = false;
		bool DataAvailable = connectXS.newDataAvailable;

		if (DataAvailable && startAnim)
		{
			//............Right Arm.............//

			glPushMatrix();

			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

			//std::cout << "Angle:" << qw << "X:" << qx << "Y:" << qy << "Z:" << qz << std::endl;

			float qwxyz[4] = { connectXS.ax[0], connectXS.ax[1], connectXS.ax[2], connectXS.ax[3] };

			QuatData_U = quaternion(connectXS.ax[0], connectXS.ax[1], connectXS.ax[2], connectXS.ax[3]);
			QuatData_L = quaternion(connectXS.ax2[0], connectXS.ax2[1], connectXS.ax2[2], connectXS.ax2[3]);
			QuatData_C = quaternion(connectXS.r_ax3[0], connectXS.r_ax3[1], connectXS.r_ax3[2], connectXS.r_ax3[3]);

			

			//firstInvQuat_C = QuatData_C.Inverse();
			//std::cout << "Angle:" << firstInvQuat_L.mData[3] << "X:" << firstInvQuat_L.mData[0] << "Y:" << firstInvQuat_L.mData[1] << "Z:" << firstInvQuat_L.mData[2] << std::endl;
			double q0 , q1, q2, q3;
			double angle_rad ;
			double angle_deg ;
			double x;
			double y;
			double z;
			double fnorm;

			if (First_calibrate  /*&& fileClose*/) {

				std::cout << "AttentionPose:" << std::endl;

				firstInvQuat_U = QuatData_U.Inverse();
				std::cout << "Upper-Arm Quat:\t" << firstInvQuat_U.mData[3] << "\t" << firstInvQuat_U.mData[0] << "\t" << firstInvQuat_U.mData[1] << "\t" << firstInvQuat_U.mData[2] << std::endl;

				firstInvQuat_L = QuatData_L;
				std::cout << "Lower-Arm Quat:\t" << firstInvQuat_L.mData[3] << "\t" << firstInvQuat_L.mData[0] << "\t" << firstInvQuat_L.mData[1] << "\t" << firstInvQuat_L.mData[2] << std::endl;
				firstInvQuat_L = QuatData_L.Inverse();

				firstPlvCalib = QuatData_C.Inverse();
				std::cout << "Pelvis Quat:\t" << QuatData_C.mData[3] << "\t" << QuatData_C.mData[0] << "\t" << QuatData_C.mData[1] << "\t" << QuatData_C.mData[2] << std::endl;

				time_t curr_time;
				curr_time = time(NULL);
				tm *tm_local = localtime(&curr_time);

				/*sprintf_s(fileName, "CData\\CalibFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
				
				Calibfile.open(fileName);
				Calibfile << QuatData_U.mData[3] << "," << QuatData_U.mData[0] << "," << QuatData_U.mData[1] << "," << QuatData_U.mData[2] << "," << "UpperArm" << "\n";
				Calibfile << QuatData_L.mData[3] << "," << QuatData_L.mData[0] << "," << QuatData_L.mData[1] << "," << QuatData_L.mData[2] << "," << "LowerArm" << "\n";
				Calibfile << QuatData_C.mData[3] << "," << QuatData_C.mData[0] << "," << QuatData_C.mData[1] << "," << QuatData_C.mData[2] << "," << "Pelvis" << "\n";
				Calibfile.close();*/
				First_calibrate = false;

							/*time_t curr_time;
				curr_time = time(NULL);
				tm *tm_local = localtime(&curr_time);*/

				/*sprintf_s(fileName, "CData\\BodyFixPoint-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
				Calibfile.open(fileName);
				Calibfile << QuatData_C.mData[3] << "," << QuatData_C.mData[0] << "," << QuatData_C.mData[1] << "," << QuatData_C.mData[2] << "," << "ForeArm" << "\n";
				Calibfile.close();*/
			}

			firstInvQuat_C = QuatData_C.mutiplication(firstPlvCalib);
			//std::cout << "W:\t" << firstInvQuat_C.mData[3] << "X:\t" << firstInvQuat_C.mData[0] << "Y:\t" << firstInvQuat_C.mData[1] << "Z:\t" << firstInvQuat_C.mData[2] << std::endl;

			quaternion reset_U = firstInvQuat_C.Inverse().mutiplication(QuatData_U.mutiplication(firstInvQuat_U));
			reset_U.normalize();
			quaternion usf_q = reset_U;

			float q0u = reset_U.mData[3];
			float q1u = reset_U.mData[0];
			float q2u = reset_U.mData[2];
			float q3u = -reset_U.mData[1];


			float angle_radu = acos(q0u) * 2;
			float angle_degu = angle_radu * 180 / PI;

			float xu = q1u / sin(angle_radu / 2);
			float yu = q2u / sin(angle_radu / 2);
			float zu = q3u / sin(angle_radu / 2);
			float fnormu = sqrt(xu*xu + yu*yu + zu*zu);
			//glRotatef(lla_angle, lla_axis[0], lla_axis[1], lla_axis[2]);
			glRotatef(angle_degu, xu / fnormu, yu / fnormu, zu / fnormu);

			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);
			////std::cout << "Angle-RU:" << lua_angle << "X:" << lua_axis[0] << "Y:" << lua_axis[2] << "Z:" << -lua_axis[1] << std::endl;
			//
			////Right elbow
			glLoadIdentity();
			glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);

			//Right lower arm
			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

			quaternion reset_L = QuatData_U.mutiplication(firstInvQuat_U).Inverse().mutiplication(QuatData_L.mutiplication(firstInvQuat_L));
			//quaternion reset_L = QuatData_C.Inverse().mutiplication(QuatData_L.mutiplication(firstInvQuat_L));
			//quaternion reset_L = (firstInvQuat_L.mutiplication( QuatData_L))/*.mutiplication(firstInvQuat_C)*/;

			quaternion lsf_q = reset_L; //(myRoundFunction(reset_L.mData[0]), myRoundFunction(reset_L.mData[1]), myRoundFunction(reset_L.mData[2]), myRoundFunction(reset_L.mData[3]));
			lsf_q.normalize();
			//sf_q = sf_q.mutiplication(quaternion(-0.138931378,-0.085475045, -0.38201938, 0.909644591));

			//std::cout << " sf_q before:%.15" << sf_q.mData[3] << "\tX:" << sf_q.mData[0] << "\tY:" << sf_q.mData[1] << "\tZ:" << sf_q.mData[2] << std::endl;

			//glRotatef(theta[10], 1.0, 0.0, 0.0);
			//glRotatef(angle, -axis[2], axis[0], -axis[1]);
			
			/*lla_axis[0] = lua_axis[0] - lla_axis[0];
			lla_axis[1] = lua_axis[1] - lla_axis[1];
			lla_axis[2] = lua_axis[2] - lla_axis[2];*/
				

			q0 = lsf_q.mData[3];
			q1 = lsf_q.mData[0];
			q2 = lsf_q.mData[2];
			q3 = -lsf_q.mData[1];


			angle_rad = acos(q0) * 2;
			//angle_rad -= angle_radu;
			angle_deg = angle_rad * 180 / PI;
			
			//angle_deg -= angle_degu;

			x = q1 / sin(angle_rad / 2);
			y = q2 / sin(angle_rad / 2);
			z = q3 / sin(angle_rad / 2);
			
			fnorm = sqrt(x*x + y*y + z*z);

			
			glLoadIdentity();
			glTranslatef(0.0, 0.0, 0.0);
			glRotatef(angle_deg, x / fnorm, y / fnorm, z / fnorm);
			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);
			glPopMatrix();


			/////////////////////////
			
			
			//quaternion tempQuat1 = BodyQuat.mutiplication(lsf_q);//Case-1

			//quaternion tempQuat1 = BodyQuat.mutiplication(usf_q.mutiplication(lsf_q));//Case-3

			quaternion tempQuat2 = BodyQuat.mutiplication(usf_q);

			quaternion tempQuat1 = tempQuat2.mutiplication(lsf_q);//Case-2 usf_q

			/*if (isRQFirst)
			{
				QuatFirstVect1 = tempQuat1;
				QuatFirstVect2 = tempQuat2;
				isRQFirst = false;
				L_sfqfile << lsf_q.mData[3] << "," << lsf_q.mData[0] << "," << lsf_q.mData[1] << "," << lsf_q.mData[2] << "\n";
				U_sfqfile << usf_q.mData[3] << "," << usf_q.mData[0] << "," << usf_q.mData[1] << "," << usf_q.mData[2] << "\n";

				break;
			}*/
			
			quaternion lTransfBodyQuat = tempQuat1;
			quaternion uTransfBodyQuat = tempQuat2;
			quaternion vQuat(tempVec._x, tempVec._y, tempVec._z, 0);
			
			lTransfBodyQuat = lTransfBodyQuat.mutiplication(vQuat.mutiplication(lTransfBodyQuat.Inverse()));
			uTransfBodyQuat = uTransfBodyQuat.mutiplication(vQuat.mutiplication(uTransfBodyQuat.Inverse()));

			///////////////////////////////////////////////////////
			if (fileClose)
			{
				uqPrevInvsPA = uqPrevPA;
				uqPrevPA = usf_q.Inverse();

				qPrevInvsPA = qPrevPA;
				qPrevPA = lsf_q.Inverse();
				if (!isFirst)
				{
					qPA = lsf_q.mutiplication(qPrevInvsPA);
					uqPA = usf_q.mutiplication(uqPrevInvsPA);
				}
				else
				{
					isFirst = false;
					break;
				}

				if (qPA.mData[3] >= 0.99999 && uqPA.mData[3] >= 0.99999)
				{
					//printf("qP > 0.9999\n");
					//printf("Skip both\n");
					break;
				}

				if (qPA.mData[3] >= 0.99999)
				{
					lsf_q = qPrevInvsPA.Inverse();
					//printf("Skip lower arm\n");
				}

				if (uqPA.mData[3] >= 0.99999)
				{
					usf_q = uqPrevInvsPA.Inverse();
					//printf("Skip upper arm\n");
				}

				/*U_rawqfile << connectXS.ax[3] << "," << connectXS.ax[0] << "," << connectXS.ax[1] << "," << connectXS.ax[2] << "\n";
				L_rawqfile << connectXS.ax2[3] << "," << connectXS.ax2[0] << "," << connectXS.ax2[1] << "," << connectXS.ax2[2] << "\n";
				P_rawfile << connectXS.r_ax3[3] << "," << connectXS.r_ax3[0] << "," << connectXS.r_ax3[1] << "," << connectXS.r_ax3[2] << "\n";*/
				L_sfqfile << lsf_q.mData[3] << "," << lsf_q.mData[0] << "," << lsf_q.mData[1] << "," << lsf_q.mData[2] << "\n";
				U_sfqfile << usf_q.mData[3] << "," << usf_q.mData[0] << "," << usf_q.mData[1] << "," << usf_q.mData[2] << "\n";
					
				
				indexP++;
				PA_data[indexP][0] = 1;// TransfBodyQuat.mData[3];
				PA_data[indexP][1] = lTransfBodyQuat.mData[0];
				PA_data[indexP][2] = lTransfBodyQuat.mData[1];
				PA_data[indexP][3] = lTransfBodyQuat.mData[2];

				uPA_data[indexP][0] = 1;// TransfBodyQuat.mData[3];
				uPA_data[indexP][1] = uTransfBodyQuat.mData[0];
				uPA_data[indexP][2] = uTransfBodyQuat.mData[1];
				uPA_data[indexP][3] = uTransfBodyQuat.mData[2];

				break;
							

				//printf("current quat: %.5f\t%.5f\t%.5f\t%.5f\n", currentQuat.mData[3], currentQuat.mData[0], currentQuat.mData[1], currentQuat.mData[2]);
				//-------------------------------------- //

				//std::cout << "printf: " << duration << '\n';
				//printf(" AxisAngle: %f \t %f \t %.15f\n", dot(a, b) / (mag(a)*mag(b)), Axis_Angle * 180 / PI, q0);
			}
			else
			{
				drawGrid[0][0] = 1;// TransfBodyQuat.mData[3];
				drawGrid[0][1] = lTransfBodyQuat.mData[0];
				drawGrid[0][2] = lTransfBodyQuat.mData[1];
				drawGrid[0][3] = lTransfBodyQuat.mData[2];

			}

		}
	}
	break;

	case 2:
		startAnim = !startAnim;
		for (int i = 0; i < (int)connectXS.mtwDevices.size(); ++i)
		{
			std::cout << "\n reset:" << connectXS.mtwDevices[i]->resetOrientation(XRM_Alignment) << std::endl;
		}
		First_calibrate = true;
		
		option = 1;
		break;


	case 7:
	{
		connectXS.bxMTdisconnect = true;
		if (connectXS.closeMtW_Succes || !connectXS.isRunning)	exit(0);
	}

	break;

	case 8:
	{
		if (bReadFile)
		{
			//............Right Arm.............//
			if (isize >= dsize)
			{
				bReadFile = false;
				isize = 0;
				L_rfile2.close();
				RPY_lowpass.close();

				matchDBTrajectory("Load\\UFormFile.csv", "Load\\LFormFile.csv");
				break;
			}
			///////////////
			quaternion qutObj;
			glPushMatrix();
			glLoadIdentity();
			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

			quaternion reset_U(uqdata[isize][0], uqdata[isize][1], uqdata[isize][2], uqdata[isize][3]);
			float q0u = reset_U.mData[3];
			float q1u = reset_U.mData[0];
			float q2u = reset_U.mData[2];
			float q3u = -reset_U.mData[1];


			float angle_radu = acos(q0u) * 2;
			float angle_degu = angle_radu * 180 / PI;

			float xu = q1u / sin(angle_radu / 2);
			float yu = q2u / sin(angle_radu / 2);
			float zu = q3u / sin(angle_radu / 2);
			float fnormu = sqrt(xu*xu + yu*yu + zu*zu);
			//glRotatef(lla_angle, lla_axis[0], lla_axis[1], lla_axis[2]);
			glRotatef(angle_degu, xu / fnormu, yu / fnormu, zu / fnormu);

			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);
			////std::cout << "Angle-RU:" << lua_angle << "X:" << lua_axis[0] << "Y:" << lua_axis[2] << "Z:" << -lua_axis[1] << std::endl;
			
			glLoadIdentity();
			glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);


			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);
			quaternion reset_L(lqdata[isize][0], lqdata[isize][1], lqdata[isize][2], lqdata[isize][3]);
			quaternion firstQuat_Plv(uqdata[isize][0], uqdata[isize][1], uqdata[isize][2], uqdata[isize][3]);

			firstInvQuat_C = firstQuat_Plv.Inverse();

			quaternion sf_q = reset_L;
			sf_q.normalize();
					

			float q0 = lqdata[isize][3];
			float q1 = lqdata[isize][0];
			float q2 = lqdata[isize][2];
			float q3 = -lqdata[isize][1];

			
			if (q0 == 9)
			{
				indexP++;
				PA_data[indexP][0] = 9;
				PA_data[indexP][1] = 0;
				PA_data[indexP][2] = 0;
				PA_data[indexP][3] = 0;
				isize++;
				glPopMatrix();
				isRQFirst = true;
				break;
			}

			float angle_rad = acos(q0) * 2;
			float angle_deg = angle_rad * 180 / PI;
			float x = q1 / sin(angle_rad / 2);
			float y = q2 / sin(angle_rad / 2);
			float z = q3 / sin(angle_rad / 2);

			float fnorm = sqrt(x*x + y*y + z*z);
			
			
			glLoadIdentity();
			glTranslatef(0.0, 0.0, 0.0);
			glRotatef(angle_deg, x / fnorm, y / fnorm, z / fnorm);
			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);
			glPopMatrix();
			//break;

			/*qPrevInvsPA = qPrevPA;
			qPrevPA = sf_q.Inverse();
			if (!isFirst)
				qPA = sf_q.mutiplication(qPrevInvsPA);
			else
			{
				isFirst = false;
				isize++;
				break;
			}

			if (qPA.mData[3] >= 0.99999 || isinf(x) || isnan(x)) {

				isize++;
				break;
			}*/

			////////////////LowPassFilter//////////////
			//float h_delta = 1 / 60;
			//float Tf_constant = h_delta * 5;

			//float alpha = 0.2;//h_delta / (Tf_constant + h_delta);


			//quaternion BodyQuat( -0.0130125, 0.702762, -0.711143, 0.0152293);
			quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);
			//quaternion tempQuat = BodyQuat.mutiplication(sf_q);

			quaternion tempQuat2 = BodyQuat.mutiplication(reset_U);

			quaternion tempQuat1 = tempQuat2.mutiplication(reset_L);//Case-2 usf_q
			//quaternion tempQuat1 = BodyQuat.mutiplication(reset_L);
			
			TVector3 TransfBodyQuat1 = tempQuat1.quternionMatrices(tempQuat1, tempVec);
			TVector3 TransfBodyQuat2 = tempQuat2.quternionMatrices(tempQuat2, tempVec);
			
			///////////////////////////////////
			indexP++;

			
			
			PA_data[indexP][0] = 1.0;
			PA_data[indexP][1] = TransfBodyQuat1._x;
			PA_data[indexP][2] = TransfBodyQuat1._y;
			PA_data[indexP][3] = TransfBodyQuat1._z;

			uPA_data[indexP][0] = 1.0;
			uPA_data[indexP][1] = TransfBodyQuat2._x;
			uPA_data[indexP][2] = TransfBodyQuat2._y;
			uPA_data[indexP][3] = TransfBodyQuat2._z;
			
			//PA_data[indexP][0] = 1;// TransfBodyQuat.mData[3];
			//PA_data[indexP][1] = TransfBodyQuat.mData[0];
			//PA_data[indexP][2] = TransfBodyQuat.mData[1];
			//PA_data[indexP][3] = TransfBodyQuat.mData[2];

			isize++;
			break;


			/*if (Counterindex < windowSize)
			{
			if (Counterindex == 0)
			{
			V0._x =  RPY._x;
			V0._y =  RPY._y;
			V0._z =  RPY._z;

			isize++;
			Counterindex++;
			break;
			}


			V1._x = RPY._x - V0._x;
			V1._y = RPY._y - V0._y;
			V1._z = RPY._z - V0._z;

			avgRPY._x +=  V1._x;
			avgRPY._y +=  V1._y;
			avgRPY._z +=  V1._z;

			V0._x =RPY._x;
			V0._y =RPY._y;
			V0._z =RPY._z;

			isize++;
			Counterindex++;
			break;
			}

			Counterindex = 0;
			cout << "avg: \t" << (avgRPY._x) <<"\t" << (avgRPY._y) << "\t" << (avgRPY._z) << endl;*/

			/*if (abs(avgRPY._x)/ (windowSize - 1) < 0.017 && abs(avgRPY._y)/ (windowSize - 1) < 0.017 && abs(avgRPY._z)/ (windowSize - 1) < 0.017)
			{
			isize++;

			avgRPY._x = 0;
			avgRPY._y = 0;
			avgRPY._z = 0;
			break;
			}

			avgRPY._x = 0;
			avgRPY._y = 0;
			avgRPY._z = 0;*/


			////////////

			//qPrevInvsPA = qPrevPA;
			//qPrevPA = sf_q.Inverse();
			//if (!isFirst)
			//	qPA = sf_q.mutiplication(qPrevInvsPA);
			//else
			//{
			//	isFirst = false;
			//	break;
			//}

			//q0 = qPA.mData[3];
			//q1 = qPA.mData[0];
			//q2 = qPA.mData[1];
			//q3 = qPA.mData[2];
			////printf("%.5f\t%.5f\t%.5f\t%.5f\n", Test2Axis.mData[3], Test2Axis.mData[0], Test2Axis.mData[1], Test2Axis.mData[2]);



			//angle_rad = acos(q0) * 2;
			//angle_deg = angle_rad * 180 / PI;
			//x = q1 / sin(angle_rad / 2);
			//y = q2 / sin(angle_rad / 2);
			//z = q3 / sin(angle_rad / 2);

			//fnorm = sqrt(x*x + y*y + z*z);
			///////////////////////////////////////////

			quaternion raw_q;
			raw_q.mData[0] = lqdata[isize][0];// x / fnorm;
			raw_q.mData[1] = lqdata[isize][1];// y / fnorm;
			raw_q.mData[2] = lqdata[isize][2];// z / fnorm;
			raw_q.mData[3] = lqdata[isize][3];// 0;

			if (!isFirst)
			{
				/*qPA = sf_q.mutiplication(QuatFirstVect.mutiplication(sf_q.Inverse()));
				qPA = qPA.normalize();*/
				qPA = raw_q.mutiplication(QuatFirstVect1.mutiplication(raw_q.Inverse()));
				//qPA = qPA.normalize();
			}
			else
			{
				QuatFirstVect1.mData[0] = lqdata[isize][0];// x / fnorm;
				QuatFirstVect1.mData[1] = lqdata[isize][1];// y / fnorm;
				QuatFirstVect1.mData[2] = lqdata[isize][2];// z / fnorm;

				QuatFirstVect1.mData[3] = lqdata[isize][3];
				printf("V0: %.5f\t%.5f\t%.5f\n", QuatFirstVect1.mData[0], QuatFirstVect1.mData[1], QuatFirstVect1.mData[2]);

				V1 = qutObj.quternionMatrices(QuatFirstVect1, V0);

				isFirst = false;
				break;
			}

			indexP++;
			PA_data[indexP][0] = sf_q.mData[3];

			float axis_norm = sqrt(qPA.mData[0] * qPA.mData[0] + qPA.mData[1] * qPA.mData[1] + qPA.mData[2] * qPA.mData[2]);

			QuatFirstVect1.mData[0] = PA_data[indexP][1] = qPA.mData[0] / axis_norm;
			QuatFirstVect1.mData[1] = PA_data[indexP][2] = qPA.mData[1] / axis_norm;
			QuatFirstVect1.mData[2] = PA_data[indexP][3] = qPA.mData[2] / axis_norm;
			printf("%d \tV1: %.5f\t%.5f\t%.5f\n", indexP, qPA.mData[0], qPA.mData[1], qPA.mData[2]);
			/*indexP++;
			PA_data[indexP][0] = angle_rad;
			PA_data[indexP][1] = x / fnorm;
			PA_data[indexP][2] = y / fnorm;
			PA_data[indexP][3] = z / fnorm;*/
			/*q0 = sf_q.mData[3];
			q1 = sf_q.mData[0];
			q2 = sf_q.mData[2];
			q3 = -sf_q.mData[1];

			fnorm = sqrt(q1*q1 + q2*q2 + q3*q3);
			indexP++;
			PA_data[indexP][0] = q0;
			PA_data[indexP][1] = q1 / fnorm;
			PA_data[indexP][2] = q2 / fnorm;
			PA_data[indexP][3] = q3 / fnorm;*/
			glPopMatrix();
			//printf("AngleAxis: %.5f\t%.5f\t%.5f\t%.5f\n", angle_deg, x / fnorm, y / fnorm, z / fnorm);
			isize++;
			break;
			/*qPrevInvsPA = qPrevPA;
			qPrevPA = reset_L.Inverse();
			if (indexP != 0)
			qPA = reset_L.mutiplication(qPrevInvsPA);

			q0 = qPA.mData[3];
			q1 = qPA.mData[0];
			q2 = qPA.mData[2];
			q3 = -qPA.mData[1];

			angle_rad = acos(q0) * 2;
			angle_deg = angle_rad;
			x = q1 / sin(angle_rad / 2);
			y = q2 / sin(angle_rad / 2);
			z = q3 / sin(angle_rad / 2);

			fnorm = sqrt(x*x + y*y + z*z);

			indexP++;
			PA_data[indexP][0] = angle_deg;
			PA_data[indexP][1] = x/ fnorm;
			PA_data[indexP][2] = y/ fnorm;
			PA_data[indexP][3] = z/ fnorm;*/
			//std::cout << "i:" << indexP << "	" << "Angle:" << angle_deg << "	X:" << x << "	Y:" << y << "	Z:" << z << "\n";

			/////--------------20190712
			qPrevInvsPA = qPrevPA;
			qPrevPA = sf_q.Inverse();
			if (!isFirst)
				qPA = sf_q.mutiplication(qPrevInvsPA);
			else
			{
				currentQuat.mData[3] = sf_q.mData[3];
				currentQuat.mData[0] = sf_q.mData[0];
				currentQuat.mData[1] = sf_q.mData[2];
				currentQuat.mData[2] = -sf_q.mData[1];
				isFirst = false;
				cIndexArray[0] = centerindexP;
				break;
			}

			//quaternion Test2Axis = currentQuat.mutiplication(sf_q.Inverse());//Test-2
			quaternion Test2Axis = firstInvQuat_C.mutiplication(sf_q);//Test-3
																	  //printf("Eqn-2: %f, %f, %f, %f\n", Test2Axis.mData[0], (Test2Axis.mData[1]), Test2Axis.mData[2], Test2Axis.mData[3]);
																	  //		 Test2Axis = sf_q.Inverse().mutiplication(firstInvQuat_C.Inverse());//Test-3
																	  //printf("Eqn-1: %f, %f, %f, %f\n", Test2Axis.mData[0], (Test2Axis.mData[1]), Test2Axis.mData[2], Test2Axis.mData[3]);


			q0 = Test2Axis.mData[3];
			q1 = Test2Axis.mData[0];
			q2 = Test2Axis.mData[2];
			q3 = -Test2Axis.mData[1];
			//printf("%.5f\t%.5f\t%.5f\t%.5f\n", Test2Axis.mData[3], Test2Axis.mData[0], Test2Axis.mData[1], Test2Axis.mData[2]);


			angle_rad = acos(q0) * 2;
			angle_deg = angle_rad * 180 / PI;
			x = q1 / sin(angle_rad / 2);
			y = q2 / sin(angle_rad / 2);
			z = q3 / sin(angle_rad / 2);

			fnorm = sqrt(x*x + y*y + z*z);

			indexP++;
			PA_data[indexP][0] = angle_rad;
			PA_data[indexP][1] = x / fnorm;
			PA_data[indexP][2] = y / fnorm;
			PA_data[indexP][3] = z / fnorm;
			//printf("skip out: %f\t", (2 * acos((reset_L.mData[0])*(xCalib.mData[0])+ (reset_L.mData[1])*(xCalib.mData[1]) + (reset_L.mData[2])*(xCalib.mData[2]) + (reset_L.mData[3])*(xCalib.mData[3]))) * 180 / PI);

			//---------------------

			duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

			//std::cout << "printf: " << duration << '\n';
			//printf("Time:  %f \t Angle: %f \n", duration, PA_angle );

			glPopMatrix();

			isize++;
		}
	}
	break;

	case 9:
		bool DataAvailable = connectXS.newDataAvailable;
		if (DataAvailable && startCalib)
		{
			if (AttAxisCalib)
				Calib_att = quaternion(connectXS.ax2[0], connectXS.ax2[1], connectXS.ax2[2], connectXS.ax2[3]);

			if (TposAxisCalib)
				Calib_tpos = quaternion(connectXS.ax2[0], connectXS.ax2[1], connectXS.ax2[2], connectXS.ax2[3]);
		}
		break;
	}

	glutPostRedisplay();
}

void menu(int id)
{
	option = id;
	done = 0;
	if (id == 3) { rotate_ = 90; xr = 0; yr = 1; zr = 0; }
	if (id == 4) { rotate_ = 0;  xr = 0; yr = 1; zr = 0; }
	if (id == 5) { rotate_ = 30; xr = 0; yr = 1; zr = 0; }
	if (id == 6) { rotate_ = 90; xr = 1; yr = 0; zr = 0; }
}

void myReshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (w <= h)
		glOrtho(-10.0, 10.0, -10.0 * (GLfloat)h / (GLfloat)w,
			10.0 * (GLfloat)h / (GLfloat)w, -10.0, 10.0);
	else
		glOrtho(-10.0 * (GLfloat)w / (GLfloat)h,
			10.0 * (GLfloat)w / (GLfloat)h, 0.0, 10.0, -10.0, 10.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void computeIntialpoint()
{
	tempVec._x = 0;
	tempVec._y = 0;
	tempVec._z = 0;

	float q0 = BodyQuat.mData[3];
	float q1 = BodyQuat.mData[0];
	float q2 = BodyQuat.mData[1];
	float q3 = BodyQuat.mData[2];

	float angle_rad = acos(q0) * 2;
	float angle_deg = angle_rad * 180 / PI;
	float x = q1 / sin(angle_rad / 2);
	float y = q2 / sin(angle_rad / 2);
	float z = q3 / sin(angle_rad / 2);

	float fnorm = sqrt(x*x + y*y + z*z);

	tempVec._x = x / fnorm;
	tempVec._y = y / fnorm;
	tempVec._z = z / fnorm;
}

void myinit()
{
	computeIntialpoint();


	//IntializeRobotLight();
	///////////////////////Texture mapping///////////////////////////
	image_t   temp_image;

	//glClearColor(1.0, 1.0, 1.0, 0.0);
	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(2, texture_id);

	glBindTexture(GL_TEXTURE_2D, texture_id[0]);
	glEnable(GL_BLEND);							// Enable Blending       (disable alpha testing)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	tgaLoad("..//worldmgrs.tga", &temp_image, TGA_FREE | TGA_LOW_QUALITY);

	//glEnable(GL_CULL_FACE);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	sphere = gluNewQuadric();
	gluQuadricDrawStyle(sphere, GLU_FILL);
	gluQuadricNormals(sphere, GLU_SMOOTH);
	gluQuadricOrientation(sphere, GLU_OUTSIDE);
	gluQuadricTexture(sphere, GL_TRUE);

	/////// allocate quadrics with filled drawing style /////////

	h = gluNewQuadric();
	gluQuadricDrawStyle(h, GLU_FILL);
	t = gluNewQuadric();
	gluQuadricDrawStyle(t, GLU_FILL);
	lua = gluNewQuadric();
	gluQuadricDrawStyle(lua, GLU_FILL);
	lelb = gluNewQuadric();
	gluQuadricDrawStyle(lelb, GLU_FILL);
	lla = gluNewQuadric();
	gluQuadricDrawStyle(lla, GLU_FILL);
	rua = gluNewQuadric();
	gluQuadricDrawStyle(rua, GLU_FILL);
	rla = gluNewQuadric();
	gluQuadricDrawStyle(rla, GLU_FILL);
	lul = gluNewQuadric();
	gluQuadricDrawStyle(lul, GLU_FILL);
	lll = gluNewQuadric();
	gluQuadricDrawStyle(lll, GLU_FILL);
	rul = gluNewQuadric();
	gluQuadricDrawStyle(rul, GLU_FILL);
	rll = gluNewQuadric();
	gluQuadricDrawStyle(rll, GLU_FILL);
	rknee = gluNewQuadric();
	gluQuadricDrawStyle(rknee, GLU_FILL);
	lknee = gluNewQuadric();
	gluQuadricDrawStyle(lknee, GLU_FILL);
	relb = gluNewQuadric();
	gluQuadricDrawStyle(relb, GLU_FILL);
	nk = gluNewQuadric();
	gluQuadricDrawStyle(nk, GLU_FILL);
	rhand = gluNewQuadric();
	gluQuadricDrawStyle(rhand, GLU_FILL);
	lhand = gluNewQuadric();
	gluQuadricDrawStyle(lhand, GLU_FILL);
	lfoot = gluNewQuadric();
	gluQuadricDrawStyle(lfoot, GLU_FILL);
	rfoot = gluNewQuadric();
	gluQuadricDrawStyle(rfoot, GLU_FILL);
	rsh = gluNewQuadric();
	gluQuadricDrawStyle(rsh, GLU_FILL);
	lsh = gluNewQuadric();
	gluQuadricDrawStyle(lsh, GLU_FILL);

	/* Set up tree */
	//C1-Torso
	glLoadIdentity();
	glRotatef(theta[0], 0.0, 1.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, torso_node.m);
	torso_node.f = torso;
	torso_node.sibling = NULL;
	torso_node.child = &nk_node;
	//C2-Neck
	glLoadIdentity();
	glTranslatef(0.0, TORSO_HEIGHT - 0.25*NECK_HEIGHT, 0.0);
	glRotatef(theta[1], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, nk_node.m);
	nk_node.f = neck;
	nk_node.sibling = &lsh_node;
	nk_node.child = &head_node;
	//C3-
	glLoadIdentity();
	glTranslatef(-(TORSO_RADIUS + UPPER_ARM_RADIUS), 0.9*TORSO_HEIGHT, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lsh_node.m);
	lsh_node.f = leftShoulder;
	lsh_node.sibling = &rsh_node;
	lsh_node.child = &lua_node;
	//C4
	glLoadIdentity();
	glTranslatef(TORSO_RADIUS + UPPER_ARM_RADIUS, 0.9*TORSO_HEIGHT, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rsh_node.m);
	rsh_node.f = rightShoulder;
	rsh_node.sibling = &lul_node;
	rsh_node.child = &rua_node;

	glLoadIdentity();
	glTranslatef(0.0, 0.75*NECK_HEIGHT, 0.0);
	glRotatef(theta[2], 0.0, 1.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, head_node.m);
	head_node.f = head;
	head_node.sibling = NULL;
	head_node.child = NULL;

	glLoadIdentity();
	glTranslatef(0.0, 0.0, 0.0);
	glRotatef(theta[3], 0.0, 1.0, 0.0); //glRotatef(theta[3], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);
	lua_node.f = left_upper_arm;
	lua_node.sibling = NULL;
	lua_node.child = &lelb_node;

	glLoadIdentity();
	glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0); //glTranslatef(0.0, UPPER_ARM_HEIGHT, 0.0);
	glRotatef(theta[4], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);
	lelb_node.f = leftElbow;
	lelb_node.sibling = NULL;
	lelb_node.child = &lla_node;

	glLoadIdentity();
	glTranslatef(0, 0, 0.0);
	glRotatef(theta[5], 0.0, 1.0, 0.0);//glRotatef(theta[5], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);
	rua_node.f = right_upper_arm;
	rua_node.sibling = NULL;
	rua_node.child = &relb_node;

	glLoadIdentity();
	glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0);//glTranslatef(0.0, UPPER_ARM_HEIGHT, 0.0);
	glRotatef(theta[4], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, relb_node.m);
	relb_node.f = rightElbow;
	relb_node.sibling = NULL;
	relb_node.child = &rla_node;

	glLoadIdentity();
	glTranslatef(-TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	glRotatef(theta[7], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lul_node.m);
	lul_node.f = left_upper_leg;
	lul_node.sibling = &rul_node;
	lul_node.child = &lknee_node;

	glLoadIdentity();
	glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	glRotatef(theta[9], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rul_node.m);
	rul_node.f = right_upper_leg;
	rul_node.sibling = NULL;
	rul_node.child = &rknee_node;

	glLoadIdentity();
	glTranslatef(0.0, -ELBOW_RADIUS / 2, 0.0); //glTranslatef(0.0, ELBOW_RADIUS / 2, 0.0);
	glRotatef(theta[4], 0.0, 1.0, 0.0);		   //glRotatef(theta[4], 1.0, 1.0, 0.0);
											   //glRotatef(30, -1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);
	lla_node.f = left_lower_arm;
	lla_node.sibling = NULL;
	lla_node.child = &lhand_node;

	glLoadIdentity();
	glTranslatef(0.0, -ELBOW_RADIUS / 2, 0.0);//glTranslatef(0.0, ELBOW_RADIUS / 2, 0.0);
	glRotatef(theta[4], 0.0, 1.0, 0.0);		 //glRotatef(theta[4], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rla_node.m);
	rla_node.f = right_lower_arm;
	rla_node.sibling = NULL;
	rla_node.child = &rhand_node;

	glLoadIdentity();
	glTranslatef(0.0, -LOWER_ARM_HEIGHT, 0.0);//glTranslatef(0.0, LOWER_ARM_HEIGHT, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rhand_node.m);
	rhand_node.f = rightHand;
	rhand_node.sibling = NULL;
	rhand_node.child = NULL;

	glLoadIdentity();
	glTranslatef(0.0, -LOWER_ARM_HEIGHT, 0.0);//glTranslatef(0.0, LOWER_ARM_HEIGHT, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lhand_node.m);
	lhand_node.f = leftHand;
	lhand_node.sibling = NULL;
	lhand_node.child = NULL;

	glLoadIdentity();
	glTranslatef(0.0, UPPER_LEG_HEIGHT, 0.0);
	glRotatef(theta[10], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lknee_node.m);
	lknee_node.f = leftKnee;
	lknee_node.sibling = NULL;
	lknee_node.child = &lll_node;

	glLoadIdentity();
	glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
	glRotatef(theta[8], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lll_node.m);
	lll_node.f = left_lower_leg;
	lll_node.sibling = NULL;
	lll_node.child = &lfoot_node;

	glLoadIdentity();
	glTranslatef(0.0, LOWER_LEG_HEIGHT, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lfoot_node.m);
	lfoot_node.f = leftFoot;
	lfoot_node.sibling = NULL;
	lfoot_node.child = NULL;

	glLoadIdentity();
	glTranslatef(0.0, UPPER_LEG_HEIGHT, 0.0);
	glRotatef(theta[10], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rknee_node.m);
	rknee_node.f = rightKnee;
	rknee_node.sibling = NULL;
	rknee_node.child = &rll_node;

	glLoadIdentity();
	glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
	glRotatef(theta[10], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rll_node.m);
	rll_node.f = right_lower_leg;
	rll_node.sibling = NULL;
	rll_node.child = &rfoot_node;

	glLoadIdentity();
	glTranslatef(0.0, LOWER_LEG_HEIGHT, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, rfoot_node.m);
	rfoot_node.f = rightFoot;
	rfoot_node.sibling = NULL;
	rfoot_node.child = NULL;

	glLoadIdentity();
}


void mouseWheel(int button, int dir, int x, int y)
{
	if (dir > 0)
	{
		zval = zval - 0.1;
	}
	else
	{
		zval = zval + 0.1;
	}
	pointTranslateX = zval*(cos(xrot*PI / 180)*  sin(yrot*PI / 180));
	pointTranslateY = zval * (sin(xrot*PI / 180));
	pointTranslateZ = zval*(cos(xrot*PI / 180) * cos(yrot*PI / 180));
	glutPostRedisplay();
}

void mouseEvent(int button, int state, int x, int y)
{
	//if (button == 3) // It's a wheel event
	//{
	//	zoominout = zoominout+0.1;
	//}

	//if (button == 4) // It's a wheel event
	//{
	//	zoominout = zoominout-0.1;
	//}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		mouseDown = true;

		xdiff = (yrot + x);
		ydiff = -y + xrot;
		//printf("xrot = %f\tyrot = %f\n", xdiff, ydiff);
	}
	else
		mouseDown = false;
}
void mouseMotion(int x, int y)
{
	if (mouseDown)
	{
		yrot = -(x + xdiff);
		xrot = y + ydiff;
		if (xrot > 89) xrot = 89.0f;
		if (xrot < -89) xrot = -89.0f;
		
		pointTranslateX = zval*(cos(xrot*PI / 180) * sin(yrot*PI / 180));
		pointTranslateY = zval*(sin(xrot*PI / 180));
		pointTranslateZ = zval*(cos(xrot*PI / 180) * cos(yrot*PI / 180));

		//if (pointTranslateY > 3.49)
		//{
		//	pointTranslateX = -pointTranslateX;
		//	//pointTranslateY = -pointTranslateY;
		//	pointTranslateZ = -pointTranslateZ;
		//}
		//printf("pointTranslateX = %f\tpointTranslateY = %f\tpointTranslateZ = %f\n", pointTranslateX, pointTranslateY, pointTranslateZ);
		glutPostRedisplay();
	}
}




void keyBoardEvent(unsigned char key, int x, int y)
{
	//printf("key_code =%d  \n", key);

	if (key == 115)//key "s"
	{
		if (connectXS.stop_and_restart_everything) {
			std::cout << "Thread closed restart program again........!" << std::endl;
		}
		else {
			std::cout << "Connect xSens........!" << std::endl;
			connectXS.isRunning = true;
			connectXS.bxMTdisconnect = false;

		}
	}

	if (key == 'x')
	{
		AttAxisCalib = true;
	}


	if (key == 'y')
	{
		TposAxisCalib = true;
	}

	if (key == 100)//key "d"
	{
		//connectXS.waitForConnections = false;
	}

	if (key == 99)//key "c"
	{
		//connectXS.bxMTdisconnect = true;
	}

	if (key == 114)//key "r"
	{
		std::cout << "Disconnect and Close xSense..........!" << std::endl;
		connectXS.stop_and_restart_everything = true;
	}

	if (key == 'a')
	{
		startAnim = !startAnim;
		for (int i = 0; i < (int)connectXS.mtwDevices.size(); ++i)
		{
			std::cout << "\n reset:" << connectXS.mtwDevices[i]->resetOrientation(XRM_Alignment) << std::endl;
		}
		First_calibrate = true;

	}

	if (key == 'v')
	{
		startAnim = !startAnim;
		First_calibrate = true;
		isRQFirst = true;
		std::cout << "\n Ready for recording...!" << std::endl;
	}

	if (key == 'u')
	{
		fileClose = !fileClose;
		if (fileClose)
		{
			std::cout << "\n Recording....!" << std::endl;

			time_t curr_time;
			curr_time = time(NULL);
			tm *tm_local = localtime(&curr_time);
			/*char dirName[1024];
			sprintf_s(dirName,"%d-%d%d%d", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			string dir = {dirName};
			CreateDirectory(dir.c_str, NULL);*/
			//fileClose = true;
			
			/*sprintf_s(fileName, "CData\\LRawFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			L_rawqfile.open(fileName);
			sprintf_s(fileName, "CData\\URawFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			U_rawqfile.open(fileName);
			sprintf_s(fileName, "CData\\PRawFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			P_rawfile.open(fileName);*/
			
			/*sprintf_s(fileName, "LFormFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			L_qfile1.open(fileName);*/
			/*sprintf_s(fileName, "CData\\AxisAngleFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			L_rfile2.open(fileName);*/
			sprintf_s(LfileName, "CData\\LFormFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			L_sfqfile.open(LfileName);

			sprintf_s(UfileName, "CData\\UFormFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			U_sfqfile.open(UfileName);
			/*sprintf_s(fileName, "CData\\TransBodyFile-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
			LatLongfile.open(fileName);*/
			//printf("File Open\n");

			fileCount++;
			indexP = 0;
			memset(PA_data, 0, 80056 * (sizeof(float)));
			memset(uPA_data, 0, 80056 * (sizeof(float)));
			//First_calibrate = true;
			memset(cIndexArray, 0, 200 * (sizeof(int)));
			CenterIndex = 0;
			start = std::clock();
			prev_angle = 0.0;
			isFirst = true;
			//isRQFirst = true;
			averageIndexP = 0;
		}
		else
		{

			//fileClose = false;
			/*L_rawqfile.close();
			U_rawqfile.close();
			P_rawfile.close();*/
			L_sfqfile.close();
			U_sfqfile.close();

			//LatLongfile.close();
			//printf("File Close\n");
			/*AVG_PA_data[0][1] = AVG_PA_data[0][1] / indexP;
			AVG_PA_data[0][2] = AVG_PA_data[0][2] / indexP;
			AVG_PA_data[0][3] = AVG_PA_data[0][3] / indexP;*/
			//printf("Recording stop..!\t Angle: %f \n", uPA_data[0][0] * 180 / PI);
			/*duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

			std::cout << "Total Time: " << duration << '\n';*/

			matchDBTrajectory(UfileName, LfileName);
			
		}
	}

	if (key == 49) //Key-1
	{
		std::ifstream infile;
		infile.open("./Load/LFormFile.csv");

		infile.unsetf(std::ios_base::skipws);

		unsigned line_count = std::count(
			std::istream_iterator<char>(infile),
			std::istream_iterator<char>(),
			'\n');
		//std::cout << "Lines: " << line_count << "\n";

		const int icount = (const int)line_count;
		dsize = icount;
		uqdata = new float*[icount];
		lqdata = new float*[icount];
		for (int i = 0; i < icount; ++i)
		{
			uqdata[i] = new float[4];
			lqdata[i] = new float[4];
		}

		int i = 0;
		//float a, b, c, d;
		//float a1, b1, c1, d1;


		std::ifstream infileu;
		infileu.open("./Load/LFormFile.csv");

		if (!infileu) { cout << "Cannot open input file.\n"; }
		else { cout << "C4 data\n"; }
		int j = 0;
		for (std::string line; getline(infileu, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				lqdata[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					lqdata[j][i + 1] = stod(line);
				}
				i++;
			}
			j++;

			/*if (!(iss >> a >>",	" >> b >> ",	" >> c >> ",	" >> d)) { break; }

			uqdata[i][3] = a;
			uqdata[i][0] = b;
			uqdata[i][1] = c;
			uqdata[i][2] = d;
			i++;			*/
		}
		
		
		/*time_t curr_time;
		curr_time = time(NULL);
		tm *tm_local = localtime(&curr_time);
		sprintf_s(fileName, "CData\\RPY-File-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
		L_rfile2.open(fileName);
		sprintf_s(fileName, "CData\\RPY-Lowpass-File-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
		RPY_lowpass.open(fileName);*/
		std::ifstream infilep;
		infilep.open("./Load/UFormFile.csv");

		if (!infilep) { cout << "Cannot open input file.\n"; }
		else { cout << "Plv data\n"; }
		j = 0;
		for (std::string line; getline(infilep, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				uqdata[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					uqdata[j][i + 1] = stod(line);
				}
				i++;

			}
			j++;

		}
		//std::ifstream infileL;

		//infileL.open("./Data060619/0/LFormFile-000-155258.csv");

		//if (!infileL) {
		//	cout << "Cannot open input file.\n";

		//}
		//else
		//{
		//	cout << "C5 data\n";
		//}

		// j = 0;
		//for (std::string line; getline(infileL, line); )
		//{
		//	std::istringstream iss(line);

		//	if (!(iss >> a >> b >> c >> d)) { break; } // error

		//	lqdata[j][3] = a;
		//	lqdata[j][0] = b;
		//	lqdata[j][1] = c;
		//	lqdata[j][2] = d;
		//	j++;
		//}
			

		infile.close();
		infileu.close();
		//infileL.close();

		bReadFile = true;
		isRQFirst = true;
		indexP = 0;
		Counterindex = 0;
		memset(PA_data, 0, 8056 * (sizeof(int)));
		memset(uPA_data, 0, 8 * (sizeof(int)));
		//First_calibrate = true;

		start = std::clock();
	}

	if (key == '7')
	{
		memset(uDB_data, 0, 80056 * (sizeof(float)));
		memset(lDB_data, 0, 80056 * (sizeof(float)));

		std::ifstream infile;
		infile.open("Load\\Standard\\UFormFile.csv");

		infile.unsetf(std::ios_base::skipws);

		unsigned line_count = std::count(
			std::istream_iterator<char>(infile),
			std::istream_iterator<char>(),
			'\n');
		//std::cout << "Lines: " << line_count << "\n";

		const int icount = (const int)line_count;
		dbCount = icount;
		uqdataDB = new float*[icount];
		lqdataDB = new float*[icount];
		for (int i = 0; i < icount; ++i)
		{
			uqdataDB[i] = new float[4];
			lqdataDB[i] = new float[4];
		}

		int i = 0;
		//float a, b, c, d;
		//float a1, b1, c1, d1;


		std::ifstream infileu;
		infileu.open("Load\\Standard\\UFormFile.csv");

		if (!infileu) { cout << "Cannot open input file.\n"; }
		else { cout << "Standard-Upperarm\n"; }
		int j = 0;
		for (std::string line; getline(infileu, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				uqdataDB[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					uqdataDB[j][i + 1] = stod(line);
				}
				i++;
			}
			j++;
		}

		std::ifstream infilep;
		infilep.open("Load\\Standard\\LFormFile.csv");

		if (!infilep) { cout << "Cannot open input file.\n"; }
		else { cout << "Standard-lowerarm\n"; }
		j = 0;
		for (std::string line; getline(infilep, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				lqdataDB[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					lqdataDB[j][i + 1] = stod(line);
				}
				i++;

			}
			j++;

		}

		for (int i = 0; i<icount; i++)
		{
			quaternion sfq_ua(uqdataDB[i][0], uqdataDB[i][1], uqdataDB[i][2], uqdataDB[i][3]);
			quaternion sfq_la(lqdataDB[i][0], lqdataDB[i][1], lqdataDB[i][2], lqdataDB[i][3]);
			quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);

			quaternion tempQuat2 = BodyQuat.mutiplication(sfq_ua);

			quaternion tempQuat1 = tempQuat2.mutiplication(sfq_la);

			TVector3 TransfBodyQuat1 = tempQuat1.quternionMatrices(tempQuat1, tempVec);
			TVector3 TransfBodyQuat2 = tempQuat2.quternionMatrices(tempQuat2, tempVec);

			lDB_data[i][0] = 1.0;
			lDB_data[i][1] = TransfBodyQuat1._x;
			lDB_data[i][2] = TransfBodyQuat1._y;
			lDB_data[i][3] = TransfBodyQuat1._z;

			uDB_data[i][0] = 1.0;
			uDB_data[i][1] = TransfBodyQuat2._x;
			uDB_data[i][2] = TransfBodyQuat2._y;
			uDB_data[i][3] = TransfBodyQuat2._z;
		}
	}

	if (key == '8')
	{
		memset(uDB_data, 0, 80056 * (sizeof(float)));
		memset(lDB_data, 0, 80056 * (sizeof(float)));

		std::ifstream infile;
		infile.open("Load\\Close\\UFormFile.csv");

		infile.unsetf(std::ios_base::skipws);

		unsigned line_count = std::count(
			std::istream_iterator<char>(infile),
			std::istream_iterator<char>(),
			'\n');
		//std::cout << "Lines: " << line_count << "\n";

		const int icount = (const int)line_count;
		dbCount = icount;
		uqdataDB = new float*[icount];
		lqdataDB = new float*[icount];
		for (int i = 0; i < icount; ++i)
		{
			uqdataDB[i] = new float[4];
			lqdataDB[i] = new float[4];
		}

		int i = 0;
		//float a, b, c, d;
		//float a1, b1, c1, d1;


		std::ifstream infileu;
		infileu.open("Load\\Close\\UFormFile.csv");

		if (!infileu) { cout << "Cannot open input file.\n"; }
		else { cout << "Close-UpperArm\n"; }
		int j = 0;
		for (std::string line; getline(infileu, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				uqdataDB[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					uqdataDB[j][i + 1] = stod(line);
				}
				i++;
			}
			j++;
		}

		std::ifstream infilep;
		infilep.open("Load\\Close\\LFormFile.csv");

		if (!infilep) { cout << "Cannot open input file.\n"; }
		else { cout << "Close-Lowerarm\n"; }
		j = 0;
		for (std::string line; getline(infilep, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				lqdataDB[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					lqdataDB[j][i + 1] = stod(line);
				}
				i++;

			}
			j++;

		}

		for (int i = 0; i<icount; i++)
		{
			quaternion sfq_ua(uqdataDB[i][0], uqdataDB[i][1], uqdataDB[i][2], uqdataDB[i][3]);
			quaternion sfq_la(lqdataDB[i][0], lqdataDB[i][1], lqdataDB[i][2], lqdataDB[i][3]);
			quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);

			quaternion tempQuat2 = BodyQuat.mutiplication(sfq_ua);

			quaternion tempQuat1 = tempQuat2.mutiplication(sfq_la);

			TVector3 TransfBodyQuat1 = tempQuat1.quternionMatrices(tempQuat1, tempVec);
			TVector3 TransfBodyQuat2 = tempQuat2.quternionMatrices(tempQuat2, tempVec);

			lDB_data[i][0] = 1.0;
			lDB_data[i][1] = TransfBodyQuat1._x;
			lDB_data[i][2] = TransfBodyQuat1._y;
			lDB_data[i][3] = TransfBodyQuat1._z;

			uDB_data[i][0] = 1.0;
			uDB_data[i][1] = TransfBodyQuat2._x;
			uDB_data[i][2] = TransfBodyQuat2._y;
			uDB_data[i][3] = TransfBodyQuat2._z;
		}
	}

	if (key == '9')
	{
		memset(uDB_data, 0, 80056 * (sizeof(float)));
		memset(lDB_data, 0, 80056 * (sizeof(float)));

		std::ifstream infile;
		infile.open("Load\\Wide\\UFormFile.csv");

		infile.unsetf(std::ios_base::skipws);

		unsigned line_count = std::count(
			std::istream_iterator<char>(infile),
			std::istream_iterator<char>(),
			'\n');
		//std::cout << "Lines: " << line_count << "\n";

		const int icount = (const int)line_count;
		dbCount = icount;
		uqdataDB = new float*[icount];
		lqdataDB = new float*[icount];
		for (int i = 0; i < icount; ++i)
		{
			uqdataDB[i] = new float[4];
			lqdataDB[i] = new float[4];
		}

		int i = 0;
		//float a, b, c, d;
		//float a1, b1, c1, d1;


		std::ifstream infileu;
		infileu.open("Load\\Wide\\UFormFile.csv");

		if (!infileu) { cout << "Cannot open input file.\n"; }
		else { cout << "Wide-Upperarm\n"; }
		int j = 0;
		for (std::string line; getline(infileu, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				uqdataDB[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					uqdataDB[j][i + 1] = stod(line);
				}
				i++;
			}
			j++;
		}

		std::ifstream infilep;
		infilep.open("Load\\Wide\\LFormFile.csv");

		if (!infilep) { cout << "Cannot open input file.\n"; }
		else { cout << "Wide-Lowerarm\n"; }
		j = 0;
		for (std::string line; getline(infilep, line); )
		{
			std::istringstream iss(line);

			std::string delimiter = ",";
			size_t pos = 0;
			std::string token;
			int i = 3;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				//std::cout << token << std::endl;
				line.erase(0, pos + delimiter.length());

				lqdataDB[j][i] = stod(token);
				if (i == 3) { i = -1; }
				if (i == 1) {
					lqdataDB[j][i + 1] = stod(line);
				}
				i++;

			}
			j++;

		}

		for(int i=0; i<icount; i++)
		{
			quaternion sfq_ua(uqdataDB[i][0], uqdataDB[i][1], uqdataDB[i][2], uqdataDB[i][3]);
			quaternion sfq_la(lqdataDB[i][0], lqdataDB[i][1], lqdataDB[i][2], lqdataDB[i][3]);
			quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);
		
			quaternion tempQuat2 = BodyQuat.mutiplication(sfq_ua);

			quaternion tempQuat1 = tempQuat2.mutiplication(sfq_la);

			TVector3 TransfBodyQuat1 = tempQuat1.quternionMatrices(tempQuat1, tempVec);
			TVector3 TransfBodyQuat2 = tempQuat2.quternionMatrices(tempQuat2, tempVec);

			lDB_data[i][0] = 1.0;
			lDB_data[i][1] = TransfBodyQuat1._x;
			lDB_data[i][2] = TransfBodyQuat1._y;
			lDB_data[i][3] = TransfBodyQuat1._z;

			uDB_data[i][0] = 1.0;
			uDB_data[i][1] = TransfBodyQuat2._x;
			uDB_data[i][2] = TransfBodyQuat2._y;
			uDB_data[i][3] = TransfBodyQuat2._z;
		}
	}
	

	if (key == 'q') { exit(0); }
}

int targc;
char** targv;

DWORD WINAPI RoboticArm(LPVOID lpParam)
{
	glutInit(&targc, targv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("Robot");

	myinit();
	glutReshapeFunc(myReshape);
	glutIdleFunc(idle);
	glutDisplayFunc(Display);
	glutKeyboardFunc(keyBoardEvent);
	glutMouseFunc(mouseEvent);
	glutMotionFunc(mouseMotion);
	glutMouseWheelFunc(mouseWheel);
	glutCreateMenu(menu);
	glutAddMenuEntry(" Start Xsens ", 0);
	glutAddMenuEntry(" Data Capture", 1);
	glutAddMenuEntry(" Read file ", 8);
	glutAddMenuEntry(" Calibration ", 9);
	glutAddMenuEntry(" Reset ", 2);
	glutAddMenuEntry(" Side View ", 3);
	glutAddMenuEntry(" Front View ", 4);
	glutAddMenuEntry(" 3/4 View ", 5);
	glutAddMenuEntry(" Top View ", 6);
	glutAddMenuEntry(" Close ", 7);

	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glutMainLoop();
	return 0;
}

DWORD WINAPI XSensDataReader(LPVOID lpParam)
{

	bool stop_restart = connectXS.stop_and_restart_everything;

	while (!stop_restart) {
		bool MTdisconnect = connectXS.bxMTdisconnect;
		if (!MTdisconnect) {
			connectXS.xmtConnect();
		}
		else {
			Sleep(60);
		}
	}

	return 0;
}

// Main Thread Goes here
int
main()
{
	//targc = argc; targv = argv;

	// Data of Thread 1
	int Data_Of_Thread_1 = 1;
	// Data of Thread 2
	int Data_Of_Thread_2 = 2;
	// Data of Thread 3
	//int Data_Of_Thread_3 = 3;
	// Data of Thread 4
	//int Data_Of_Thread_4 = 4;

	// variable to hold handle of Thread 1
	HANDLE Handle_Of_Thread_1 = 0;
	// variable to hold handle of Thread 2 
	HANDLE Handle_Of_Thread_2 = 0;
	// variable to hold handle of Thread 3 
	//HANDLE Handle_Of_Thread_3 = 0;

	HANDLE Array_Of_Thread_Handles[2];

	//Initilize the critical section
	InitializeCriticalSection(&m_cs);

	// Create thread 1.
	Handle_Of_Thread_1 = CreateThread(NULL, 0,
		XSensDataReader, &Data_Of_Thread_1, 0, NULL);
	if (Handle_Of_Thread_1 == NULL)
		ExitProcess(Data_Of_Thread_1);

	// Create thread 2.
	Handle_Of_Thread_2 = CreateThread(NULL, 0,
		RoboticArm, &Data_Of_Thread_2, 0, NULL);
	if (Handle_Of_Thread_2 == NULL)
		ExitProcess(Data_Of_Thread_2);

	// Create thread 3.
	/*Handle_Of_Thread_3 = CreateThread(NULL, 0,
	PrincipalAxisSphere, &Data_Of_Thread_3, 0, NULL);
	if (Handle_Of_Thread_3 == NULL)
	ExitProcess(Data_Of_Thread_3);
	*/

	// Store Thread handles in Array of Thread
	// Handles as per the requirement
	// of WaitForMultipleObjects() 
	Array_Of_Thread_Handles[0] = Handle_Of_Thread_1;
	Array_Of_Thread_Handles[1] = Handle_Of_Thread_2;
	//Array_Of_Thread_Handles[2] = Handle_Of_Thread_3;

	// Wait until all threads have terminated.
	WaitForMultipleObjects(2, Array_Of_Thread_Handles, TRUE, INFINITE);

	// Delete critical Section
	DeleteCriticalSection(&m_cs);

	// Close all thread handles upon completion.
	CloseHandle(Handle_Of_Thread_1);
	CloseHandle(Handle_Of_Thread_2);
	//CloseHandle(Handle_Of_Thread_3);

}