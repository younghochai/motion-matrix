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


struct Avatar {
	quaternion b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;
	quaternion prv_b0, prv_b1, prv_b2, prv_b3, prv_b4,
		prv_b5, prv_b6, prv_b7, prv_b8, prv_b9;
	ofstream fb0, fb1, fb2, fb3, fb4, fb5, fb6, fb7, fb8, fb9;
};
quaternion qInit = { 0,0,0,1 };
struct Avatar avatar = { qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit };

int option = -1, subOption = -1;
double rotate_ = 0;

double xr = 0, yr = 0, zr = 0;

void head();
void torso();
void left_upper_arm();
void right_upper_arm();
void left_upper_leg();
void right_upper_leg();

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
//0 , 1 , 2 ,   3 , 4 ,  5  , 6 ,  7  , 8 ,  9  ,10
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
ofstream outDataL, outDataU;

bool fileClose = false;
float** uqdata;
float** lqdata;

float** uqdataDB;
float** lqdataDB;

bool bReadFile = false;
bool bReadDBFile = false;
int isize = 0;
int dsize = 0;
int idbsize = 0;
bool startAnim = false;

quaternion QuatData_RightUpperArm, QuatData_RightLowerArm, QuatData_Pelvis, QuatData_LeftUpperArm, QuatData_LeftLowerArm;
quaternion QuatData_RightUpperLeg, QuatData_RightLowerLeg, QuatData_head, QuatData_LeftUpperLeg, QuatData_LeftLowerLeg;
quaternion firstInvQuat_RightUpperArm, firstInvQuat_RightLowerArm, firstInvQuat_C;
quaternion firstInvQuat_RightUpperLeg, firstInvQuat_RightLowerLeg, firstInvQuat_head;
quaternion firstInvQuat_LeftUpperArm, firstInvQuat_LeftLowerArm;
quaternion firstInvQuat_LeftUpperLeg, firstInvQuat_LeftLowerLeg;
quaternion qPA, qPrevInvsPA, firstPlvCalib, firstHeadCalib;

struct CurveProperty curveProperty;

quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);

int trajCount = 0;
int LindexP = 0;
int indexDB = 0;
bool firstCalib = true;  // Reset Quaternion

bool isFirst = true;
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


float traj_b0[20014][4];
float traj_b1[20014][4];
float traj_b2[20014][4];
float traj_b3[20014][4];
float traj_b4[20014][4];
float traj_b5[20014][4];
float traj_b6[20014][4];
float traj_b7[20014][4];
float traj_b8[20014][4];
float traj_b9[20014][4];


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

char charstring[1024];

bool isMatched = false;
float diff = 10;
float diff1 = 10;
float diff2 = 10;
float diff3 = 10;
int stdPercent, closePercent, widePercent;

float threshold = 0.15f;
float color[4] = { 0, 0, 0, 1 };
float mcolor[4] = { 1, 0, 0, 1 };

//Texturemapping
GLuint texture_id[2];
GLUquadricObj *sphere;

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

void quad(int a, int b, int c, int d)
{
	glBegin(GL_QUADS);
	glVertex3fv(ver[a]);
	glVertex3fv(ver[b]);
	glVertex3fv(ver[c]);
	glVertex3fv(ver[d]);
	glEnd();
}

void DrawGrid()
{
	glColor3f(0.9, 0.9, 0.9);
	quad(0, 3, 2, 1);
	quad(2, 3, 7, 6);
	quad(0, 4, 7, 3);
	quad(1, 2, 6, 5);
	quad(4, 5, 6, 7);
	quad(0, 1, 5, 4);
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

void bodyJoint_circle(float r, int width)
{
	float x, y;
	int w = width;
	float radius = r;
	glLineWidth(w);
	glBegin(GL_LINES);

	x = (float)radius * cos(359 * PI / 180.0f);
	y = (float)radius * sin(359 * PI / 180.0f);
	for (int j = 0; j < 360; j++)
	{
		glVertex2f(x, y);
		x = (float)radius * cos(j * PI / 180.0f);
		y = (float)radius * sin(j * PI / 180.0f);
		glVertex2f(x, y);
	}
	glEnd();
}

void torso()
{
	// Belly
	glColor3f(0.68, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0.0);
	glRotatef(90, 1, 0, 0);
	glutSolidCone(1.0, 0.75, 6, 6);
	glPopMatrix();

	//Belly Center joint Circle
	glColor3f(0, 0, 0);
	glPushMatrix();
	glTranslatef(0, 0.25, 0);
	glRotatef(180, 1, 0, 0);
	bodyJoint_circle(0.25, 2);
	glPopMatrix();

	//Belly Left joint Circle 
	glColor3f(0, 0, 0);
	glPushMatrix();
	glTranslatef(-0.75, 0, 0);
	glRotatef(180, 1, 0, 0);
	bodyJoint_circle(0.25, 2);
	glPopMatrix();


	//Belly Right joint Circle 
	glColor3f(0, 0, 0);
	glPushMatrix();
	glTranslatef(0.75, 0, 0);
	glRotatef(180, 1, 0, 0);
	bodyJoint_circle(0.25, 2);
	glPopMatrix();

	//Upper Body
	glColor3f(0.68, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 5.75, 0.0);
	glRotatef(90, 1, 0, 0);
	glutSolidCone(0.8, 5.5, 6, 6);
	glPopMatrix();


}

void head()
{
	//Head
	glColor3f(0.68, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(-0.1, 0.25, 0);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(0.55, 2.05, 6, 6);
	glPopMatrix();


}

void neck()
{
	//Neck 
	glColor3f(0.68, 0.14, 0.21);
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.0, 0.0);
	glutSolidCone(0.55, 1.05, 6, 6);
	glPopMatrix();

	//Neck Joint Circle
	glPushMatrix();
	glRotatef(180, 0, 1, 0);
	glTranslatef(0.0, 1, 0);
	bodyJoint_circle(0.2, 2);
	glPopMatrix();

}

void rightShoulder()
{
	//Right Shoulder
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(-1.85, 0.10, 0);
	glRotatef(90, 0.12, 1, 0);
	glutSolidCone(0.35, 2.25, 6, 6);
	glPopMatrix();


	//Right Shoulder joint Circle
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glTranslatef(0.0, 0.2, 0);
	bodyJoint_circle(0.5, 2);
	glPopMatrix();


}



void leftShoulder()
{
	//Left Shoulder
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(1.85, 0.125, 0);
	glRotatef(-90, -0.1, 1, 0.0);
	glutSolidCone(0.35, 2.25, 6, 6);
	glPopMatrix();


	//Left Shoulder Joint Circle
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glTranslatef(0.0, 0.2, 0);
	bodyJoint_circle(0.5, 2);
	glPopMatrix();
}

void rightElbow()
{
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(180, 1, 0, 0);
	bodyJoint_circle(0.5, 2);
	glPopMatrix();
}

void leftElbow()
{
	//Left Elbow
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(180, 1, 0, 0);
	bodyJoint_circle(0.5, 2);
	glPopMatrix();
}

void rightKnee()
{
	//Right Knee
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glRotatef(180, 0, 1, 0);
	glTranslatef(-0.15, 0.8, 0);
	bodyJoint_circle(0.5, 2);
	glPopMatrix();


}

void leftKnee()
{
	//Left Knee
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glRotatef(180, 0, 1, 0);
	glTranslatef(0.0, 0.8, 0);
	bodyJoint_circle(0.5, 2);
	glPopMatrix();
}

void leftFoot()
{
	//Left Foot
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.8, 0);
	glRotatef(90, 1, 0, 0);
	glutSolidCone(0.75, 0.75, 6, 6);
	glPopMatrix();

	//Left Foot Joint Circle
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glTranslatef(0.0, 0.8, 0);
	glRotatef(180, 0, 1, 0);
	glTranslatef(0.0, -0.65, 0);
	bodyJoint_circle(0.2, 2);
	glPopMatrix();

}

void rightFoot()
{
	//Right Foot
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.15, 0.8, 0);
	glRotatef(90, 1, 0, 0);
	glutSolidCone(0.75, 0.75, 6, 6);
	glPopMatrix();

	//Right Foot Joint Circle
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glTranslatef(0.15, 0.15, 0);
	glRotatef(180, 0, 1, 0);
	bodyJoint_circle(0.2, 2);
	glPopMatrix();
}

void rightHand()
{
	//Right Hand
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(0.55, 0.75, 5, 5);
	glPopMatrix();

	//Right Hand Joint Circle
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glRotatef(180, 0, 1, 0);
	glTranslatef(0.0, 0.65, 0);
	bodyJoint_circle(0.2, 2);
	glPopMatrix();
}

void leftHand()
{
	//Left Hand
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(0.55, 0.75, 5, 5);
	glPopMatrix();

	//Left Hand Joint Circle
	glColor3f(0.0, 0.0, 0.0);
	glPushMatrix();
	glRotatef(180, 0, 1, 0);
	glTranslatef(0.0, 0.65, 0);
	bodyJoint_circle(0.2, 2);
	glPopMatrix();
}

void left_upper_arm()
{
	//Left Upper Arm Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_ARM_RADIUS - 0.1, 0.5, 10, 10);
	glPopMatrix();

	//Left Upper Arm
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_ARM_RADIUS - 0.1, UPPER_ARM_HEIGHT, 10, 10);
	glPopMatrix();

	//Co-ordinate
	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void left_lower_arm()
{

	//Left Lower Arm Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_ARM_RADIUS - 0.15, 0.5, 10, 10);
	glPopMatrix();

	//Left Lower Arm
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_ARM_RADIUS - 0.15, LOWER_ARM_HEIGHT, 10, 10);
	glPopMatrix();


	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void right_upper_arm()
{
	//right upper Arm Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_ARM_RADIUS - 0.1, 0.5, 10, 10);
	glPopMatrix();

	//right upper Arm
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_ARM_RADIUS - 0.1, UPPER_ARM_HEIGHT, 10, 10);
	glPopMatrix();

	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void right_lower_arm()
{

	//Right Lower Arm Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_ARM_RADIUS - 0.15, 0.5, 10, 10);
	glPopMatrix();

	//Right Lower Arm Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_ARM_RADIUS - 0.15, LOWER_ARM_HEIGHT, 10, 10);
	glPopMatrix();

	glPushMatrix();
	glRotatef(180.0, 0.0, 1.0, 0.0);//-90
	glRotatef(180.0, 1.0, 0.0, 0.0);
	drawCoordinate();
	glPopMatrix();
}

void left_upper_leg()
{
	//Left Upper Leg Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 1.0, 0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_LEG_RADIUS - 0.1, UPPER_LEG_HEIGHT, 10, 10);
	glPopMatrix();

	//Left Upper Leg
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 1.0, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_LEG_RADIUS - 0.1, 0.5, 10, 10);
	glPopMatrix();


}

void left_lower_leg()
{
	//Left lower Leg Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.8, 0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_LEG_RADIUS - 0.2, LOWER_LEG_HEIGHT, 10, 10);
	glPopMatrix();


	//Left lower Leg
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.0, 0.8, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_LEG_RADIUS - 0.2, 0.5, 10, 10);
	glPopMatrix();
}





void right_upper_leg()
{
	//right upper Leg Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.15, 1.0, 0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_LEG_RADIUS - 0.1, UPPER_LEG_HEIGHT, 10, 10);
	glPopMatrix();

	//right upper Leg
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.15, 1.0, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(UPPER_LEG_RADIUS - 0.1, 0.5, 10, 10);
	glPopMatrix();

}

void right_lower_leg()
{
	//right lower  Leg Cap
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.15, 0.8, 0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_LEG_RADIUS - 0.2, LOWER_LEG_HEIGHT, 10, 10);
	glPopMatrix();

	//right lower  Leg 
	glColor3f(0.98, 0.14, 0.21);
	glPushMatrix();
	glTranslatef(0.15, 0.8, 0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	glutSolidCone(LOWER_LEG_RADIUS - 0.2, 0.5, 10, 10);
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

void InitializeLight()
{
	GLfloat mat_ambient_0[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat mat_diffuse_0[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_specular_0[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_position_0[] = { 0.0f, 0.0f, -4.0f, 0.0f };
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
	GLfloat mat_position_1[] = { -8.5f, -8.5f, 3.0f, 1.0f };
	GLfloat mat_spotdir_1[] = { -10.0f, 0.0f, 0.0f };

	glLightfv(GL_LIGHT1, GL_SPECULAR, mat_specular_1);
	glLightfv(GL_LIGHT1, GL_POSITION, mat_position_1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, mat_diffuse_1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, mat_ambient_1);

	/*glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.5f);
	glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.5f);
	glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.2f);*/

	glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 50.0f);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, mat_spotdir_1);
	glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 2.0f);

	GLfloat mat_ambient_2[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat mat_diffuse_2[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_specular_2[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	//GLfloat mat_position_2[] = { 0.9f, -0.5f, 0.6f, 1.0f};	
	//GLfloat mat_position_2[] = { -0.5f, 0.5f, 0.6f, 1.0f};	
	GLfloat mat_position_2[] = { -8.5f, 9.0f, 3.0f, 1.0f };

	glLightfv(GL_LIGHT2, GL_SPECULAR, mat_specular_2);
	glLightfv(GL_LIGHT2, GL_POSITION, mat_position_2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, mat_diffuse_2);
	glLightfv(GL_LIGHT2, GL_AMBIENT, mat_ambient_2);


	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glEnable(GL_LIGHT1);
	//glEnable(GL_LIGHT2);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	::glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	//::glClearColor(0.0f, 0.0f, 0.0f, 0.0f);	
	::glShadeModel(GL_SMOOTH);
	//::glShadeModel(GL_FLAT);	

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	//glFrontFace(GL_CCW);

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
	gluOrtho2D(0, width / 2, 0, height);  // set to orthogonal projection

	//Vector3 v1 = sphereVector;                          // first vector on sphere
	//Vector3 v2 = trackball.getVector(mouseX, mouseY);   // second vector on sphere
	//float angle = RAD2DEG * acosf(v1.dot(v2) / (v1.length() * v2.length()));

	// for print infos



	/////////////////////
	isMatched = false;
	float normalLowerArmCurveLength = 0.0, normalUpperArmCurveLength = 0.0, normalSpeed, percentage = 0, deviation = 0;
	normalSpeed = (101.0 / 60.0)*1000.0;
	std::stringstream closeness;
	std::stringstream closeness1;
	std::stringstream closeness2;
	std::stringstream closeness3;
	std::stringstream closeness4;
	//if (diff < threshold && diff == diff1)
	if (stdPercent >= 90)
	{
		std::stringstream ss;
		ss << "Falls within the range of STANDARD CURL";
		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - TEXT_HEIGHT, mcolor, font);
		//ss.str("");
		//printf("Standard Curl:%f (Match)\n", diff1);
		isMatched = true;

		normalLowerArmCurveLength = ((1.34 + 0.3) * 180 / PI);
		normalUpperArmCurveLength = ((0.16 + 0.3) * 180 / PI);

		percentage = stdPercent;
		deviation = diff1 * 180 / PI;
	}
	//else
	//{
	//	std::stringstream ss;
	//	ss << "Not within the range of STANDARD CURL";
	//	drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - TEXT_HEIGHT, color, font);
	//	//printf("Standard Curl:%f (No-Match)\n", diff1);
	//}



	//if (!isMatched)
	{


		//if (diff < threshold && diff == diff2)
		if (closePercent >= 90)
		{
			std::stringstream ss;
			ss << "Falls within the range of CLOSE CURL";
			drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (2 * TEXT_HEIGHT), mcolor, font);

			isMatched = true;

			normalLowerArmCurveLength = ((0.88 + 0.3) * 180 / PI);
			normalUpperArmCurveLength = ((0.14 + 0.3) * 180 / PI);

			percentage = closePercent;
			deviation = diff2 * 180 / PI;
		}
		/*else
		{
			std::stringstream ss;
			ss << "Not within the range of CLOSE CURL";
			drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (2 * TEXT_HEIGHT), color, font);

		}*/
	}

	//if (!isMatched)
	{


		//if (diff < threshold && diff == diff3)
		if (widePercent >= 90)
		{
			std::stringstream ss;
			ss << "Falls within the range of WIDE CURL";
			drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (3 * TEXT_HEIGHT), mcolor, font);

			isMatched = true;

			normalLowerArmCurveLength = ((1.19 + 0.3) * 180 / PI);
			normalUpperArmCurveLength = ((0.08 + 0.3) * 180 / PI);

			percentage = widePercent;
			deviation = diff3 * 180 / PI;
		}
		/*else
		{
			std::stringstream ss;
			ss << "Not within the range of WIDE CURL";
			drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (3 * TEXT_HEIGHT), color, font);

		}*/
	}
	closeness.str("");
	closeness1.str("");
	closeness2.str("");
	closeness3.str("");
	closeness4.str("");

	if (stdPercent >= 90)
	{
		if (diff2 < 0.42)
			closeness.str("Suggestion: Slightly closed, try to open your arm");
		else
			closeness.str("Suggestion: Slightly wider, try to close your arm");
	}

	if (closePercent >= 90)
	{
		if (diff1 < 0.39)
			closeness.str("Suggestion: Slightly wider, try to close your arm");
		else
			closeness.str("Suggestion: Slightly closed, try to open your arm");
	}


	if (widePercent >= 90)
	{
		if (diff1 < 0.99)
			closeness.str("Suggestion: Slightly closed, try to open your arm");
		else
			closeness.str("Suggestion: Slightly wider, try to close your arm");
	}


	if (curveProperty.speed < normalSpeed)
		closeness1.str(" # Slightly slowdown");
	else
		closeness1.str(" # Slightly speedup");


	if (curveProperty.upperArmLength > normalUpperArmCurveLength)
		closeness2.str(" # More then normal upper arm movement observed");

	if (curveProperty.LowerArmLength > normalLowerArmCurveLength)
		closeness3.str(" # More then normal lower arm movement observed");


	if (curveProperty.initialOrientationDeviation > (0.15 * 180 / PI))
		closeness4.str(" # Initial orientation missmatch");

	closeness << closeness.str() << closeness1.str() << closeness2.str() << closeness3.str() << closeness4.str();
	if (!isMatched)
	{
		std::stringstream ss;
		ss << "No match found!!";
		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (2 * TEXT_HEIGHT), mcolor, font);

	}
	else
	{
		std::stringstream ss;

		ss << "Curve-Diagnosis: ";
		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (6 * TEXT_HEIGHT), color, font);
		ss.str("");
		ss << setprecision(3) << "Speed: " << curveProperty.speed << "/ms (" << normalSpeed << "/ms)";
		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (7 * TEXT_HEIGHT), color, font);
		ss.str("");

		ss << setprecision(2) << "UpperArm Degree of Curvature : " << curveProperty.upperArmLength << " deg (< " << normalUpperArmCurveLength << "deg)";
		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (8 * TEXT_HEIGHT), color, font);
		ss.str("");
		ss << setprecision(3) << "LowerArm Degree of Curvature : " << curveProperty.LowerArmLength << "deg (< " << normalLowerArmCurveLength << "deg)";

		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (9 * TEXT_HEIGHT), color, font);
		ss.str("");
		ss << percentage << "% of trajectory is within the range (> 90%)";
		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (10 * TEXT_HEIGHT), color, font);
		ss.str("");

		ss << setprecision(3) << "Average angle of deviation: " << (deviation) << "deg ";

		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (12 * TEXT_HEIGHT), color, font);
		ss.str("");
		ss << setprecision(3) << "Initial angle of deviation: " << curveProperty.initialOrientationDeviation << " (<" << (0.15 * 180 / PI) << "deg)";
		drawString(ss.str().c_str(), width / 4 + 150, height / 1.5 - (11 * TEXT_HEIGHT), color, font);
		drawString(closeness.str().c_str(), 10, 50, color, font);
	}

	///////////////////
	std::stringstream ss;
	//ss << "Press SPACE mode.";
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

}

void drawTrajectorySphere(int index, float(&traj_b)[20014][4], float r, float g, float b)
{

	glColor3f(r, g, b);
	float fnorm = sqrt(traj_b[index][1] * traj_b[index][1] + traj_b[index][2] * traj_b[index][2] + traj_b[index][3] * traj_b[index][3]);
	glPushMatrix();
	glTranslatef(1.011*traj_b[index][1] / fnorm, 1.011*traj_b[index][2] / fnorm, 1.011*traj_b[index][3] / fnorm);
	glutSolidSphere(0.009, 30, 30);
	glPopMatrix();
}

void drawTrajectory(void)
{
	InitializeLight();

	glViewport(width / 2, 0, width / 2, height);
	glScissor(width / 2, 0, width / 2, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-2, 2, -2, 2, -1, 10);

	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();
	glLoadIdentity();
	gluLookAt(
		pointTranslateX, pointTranslateY, pointTranslateZ,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f
	);

	glScalef(zval - 2.5, zval - 2.5, zval - 2.5);


	glLineWidth(2.0);
	glColor3f(1.0, 1.0, 1.0);
	glBindTexture(GL_TEXTURE_2D, texture_id[0]);
	glRotatef(-180, 0, 1, 0);
	glRotatef(-90, 1, 0, 0);
	gluSphere(sphere, 1.0, 50, 50);

	drawcenterCoordinate();
	float r = 1, g = 0, b = 0;
	/*if (!fileClose && trajCount > 0) {*/
	int j = 0;
	for (int i = 0; i < trajCount; i++)
	{
		/*if (traj_b3[i][0] == 0)
			continue;*/

			//glColor3f(r, g, b);

			/*if (traj_b3[i][0] == 9) {
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

				continue;
			}*/

		drawTrajectorySphere(i, traj_b0, 0.0, 0.0, 0.0); //black
		drawTrajectorySphere(i, traj_b1, 1.0, 0.0, 0.0); //red
		drawTrajectorySphere(i, traj_b2, 1.0, 0.5, 0.0); //orange
		drawTrajectorySphere(i, traj_b3, 1.0, 1.0, 0.0); //Yellow
		drawTrajectorySphere(i, traj_b4, 0.0, 1.0, 0.0); //Bright green
		drawTrajectorySphere(i, traj_b5, 0.0, 1.0, 1.0); //Cyan
		drawTrajectorySphere(i, traj_b6, 0.0, 0.0, 1.0); //Blue
		drawTrajectorySphere(i, traj_b7, 0.5, 0.0, 1.0); //Voilet
		drawTrajectorySphere(i, traj_b8, 1.0, 0.0, 1.0); //Magenta
		drawTrajectorySphere(i, traj_b9, 0.4, 0.5, 0.8); //Indigo

		/*float fnorm = sqrt(traj_b3[i][1] * traj_b3[i][1] + traj_b3[i][2] * traj_b3[i][2] + traj_b3[i][3] * traj_b3[i][3]);
		glPushMatrix();
		glTranslatef(1.011*traj_b3[i][1] / fnorm, 1.011*traj_b3[i][2] / fnorm, 1.011*traj_b3[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		glPopMatrix();

		glColor3f(r, g + 0.64, b);
		fnorm = sqrt(traj_b2[i][1] * traj_b2[i][1] + traj_b2[i][2] * traj_b2[i][2] + traj_b2[i][3] * traj_b2[i][3]);
		glPushMatrix();
		glTranslatef(1.011*traj_b2[i][1] / fnorm, 1.011*traj_b2[i][2] / fnorm, 1.011*traj_b2[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		glPopMatrix();

		glColor3f(0.2, 0.2, 0.8);
		fnorm = sqrt(traj_b5[i][1] * traj_b5[i][1] + traj_b5[i][2] * traj_b5[i][2] + traj_b5[i][3] * traj_b5[i][3]);
		glPushMatrix();
		glTranslatef(1.011*traj_b5[i][1] / fnorm, 1.011*traj_b5[i][2] / fnorm, 1.011*traj_b5[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		glPopMatrix();

		glColor3f(0.4, 0.6, 0.4);
		fnorm = sqrt(traj_b4[i][1] * traj_b4[i][1] + traj_b4[i][2] * traj_b4[i][2] + traj_b4[i][3] * traj_b4[i][3]);
		glPushMatrix();
		glTranslatef(1.011*traj_b4[i][1] / fnorm, 1.011*traj_b4[i][2] / fnorm, 1.011*traj_b4[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);
		glPopMatrix();*/
	}

	for (int i = 0; i < dbCount; i++)
	{
		glColor3f(1, 0, 1);

		float fnorm = sqrt(uDB_data[i][1] * uDB_data[i][1] + uDB_data[i][2] * uDB_data[i][2] + uDB_data[i][3] * uDB_data[i][3]);
		glPushMatrix();
		glTranslatef(1.011*uDB_data[i][1] / fnorm, 1.011*uDB_data[i][2] / fnorm, 1.011*uDB_data[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);

		glPopMatrix();

		glPushMatrix();
		glColor3f(1, 0.4, 0);

		fnorm = sqrt(lDB_data[i][1] * lDB_data[i][1] + lDB_data[i][2] * lDB_data[i][2] + lDB_data[i][3] * lDB_data[i][3]);
		glTranslatef(1.011*lDB_data[i][1] / fnorm, 1.011*lDB_data[i][2] / fnorm, 1.011*lDB_data[i][3] / fnorm);
		glutSolidSphere(0.009, 30, 30);

		glPopMatrix();
	}
	for (int i = 0; i < dbCount; i++)
	{

		glDisable(GL_LIGHTING);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_CULL_FACE);

		glColor4f(0.13, 0.54, 0.13, 0.3);
		glPushMatrix();
		float fnorm = sqrt(lDB_data[i][1] * lDB_data[i][1] + lDB_data[i][2] * lDB_data[i][2] + lDB_data[i][3] * lDB_data[i][3]);
		if (i > 1 && i <= 49)
			renderCylinder_convenient(lDB_data[i - 1][1] / fnorm, lDB_data[i - 1][2] / fnorm, lDB_data[i - 1][3] / fnorm, lDB_data[i][1] / fnorm, lDB_data[i][2] / fnorm, lDB_data[i][3] / fnorm, 0.15, 30);

		if (i == 1 || i == 50)
		{
			glTranslatef(1.011*lDB_data[i][1] / fnorm, 1.011*lDB_data[i][2] / fnorm, 1.011*lDB_data[i][3] / fnorm);
			glutSolidSphere(0.15, 30, 30);
		}
		glPopMatrix();

		glDisable(GL_BLEND);
		glDisable(GL_CULL_FACE);
		glEnable(GL_LIGHTING);

	}
	glPopMatrix();

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
	drawTrajectory();
	//glEnable(GL_COLOR_MATERIAL);
	/*fourSphere();
	TextDispaly();*/

	glFlush();
	glutSwapBuffers();
}

void matchDBTrajectory(char * Ufile, char * Lfile)
{
	diff = 0;

	diff1 = Comparision::getDiffBtwTrajectory("Load\\Standard\\UFormFile.csv", "Load\\Standard\\LFormFile.csv", Ufile, Lfile, stdPercent, curveProperty);
	if (stdPercent < 90)
	{
		Comparision::resetDiagnosis();
		diff2 = Comparision::getDiffBtwTrajectory("Load\\Close\\UFormFile.csv", "Load\\Close\\LFormFile.csv", Ufile, Lfile, closePercent, curveProperty);
	}
	if (stdPercent < 90 && closePercent < 90)
	{
		Comparision::resetDiagnosis();
		diff3 = Comparision::getDiffBtwTrajectory("Load\\Wide\\UFormFile.csv", "Load\\Wide\\LFormFile.csv", Ufile, Lfile, widePercent, curveProperty);
	}
	cout << diff1 << "," << diff2 << "," << diff3 << endl;

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
}

void openFile()
{
	time_t curr_time;
	curr_time = time(NULL);
	tm *tm_local = localtime(&curr_time);

	sprintf_s(fileName, "CData\\fb0-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb0.open(fileName);
	sprintf_s(fileName, "CData\\fb1-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb1.open(fileName);
	sprintf_s(fileName, "CData\\fb2-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb2.open(fileName);
	sprintf_s(fileName, "CData\\fb3-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb3.open(fileName);
	sprintf_s(fileName, "CData\\fb4-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb4.open(fileName);
	sprintf_s(fileName, "CData\\fb5-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb5.open(fileName);
	sprintf_s(fileName, "CData\\fb6-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb6.open(fileName);
	sprintf_s(fileName, "CData\\fb7-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb7.open(fileName);
	sprintf_s(fileName, "CData\\fb8-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb8.open(fileName);
	sprintf_s(fileName, "CData\\fb9-00%d-%d%d%d.csv", fileCount, tm_local->tm_hour, tm_local->tm_min, tm_local->tm_sec);
	avatar.fb9.open(fileName);

	fileCount++;
}

void closeFile()
{
	avatar.fb0.close();
	avatar.fb1.close();
	avatar.fb2.close();
	avatar.fb3.close();
	avatar.fb4.close();
	avatar.fb5.close();
	avatar.fb6.close();
	avatar.fb7.close();
	avatar.fb8.close();
	avatar.fb9.close();
}

void writeData()
{
	avatar.fb0 << avatar.b0.mData[3] << "," << avatar.b0.mData[0] << "," << avatar.b0.mData[1] << "," << avatar.b0.mData[2] << "\n";
	avatar.fb1 << avatar.b1.mData[3] << "," << avatar.b1.mData[0] << "," << avatar.b1.mData[1] << "," << avatar.b1.mData[2] << "\n";
	avatar.fb2 << avatar.b2.mData[3] << "," << avatar.b2.mData[0] << "," << avatar.b2.mData[1] << "," << avatar.b2.mData[2] << "\n";
	avatar.fb3 << avatar.b3.mData[3] << "," << avatar.b3.mData[0] << "," << avatar.b3.mData[1] << "," << avatar.b3.mData[2] << "\n";
	avatar.fb4 << avatar.b4.mData[3] << "," << avatar.b4.mData[0] << "," << avatar.b4.mData[1] << "," << avatar.b4.mData[2] << "\n";
	avatar.fb5 << avatar.b5.mData[3] << "," << avatar.b5.mData[0] << "," << avatar.b5.mData[1] << "," << avatar.b5.mData[2] << "\n";
	avatar.fb6 << avatar.b6.mData[3] << "," << avatar.b6.mData[0] << "," << avatar.b6.mData[1] << "," << avatar.b6.mData[2] << "\n";
	avatar.fb7 << avatar.b7.mData[3] << "," << avatar.b7.mData[0] << "," << avatar.b7.mData[1] << "," << avatar.b7.mData[2] << "\n";
	avatar.fb8 << avatar.b8.mData[3] << "," << avatar.b8.mData[0] << "," << avatar.b8.mData[1] << "," << avatar.b8.mData[2] << "\n";
	avatar.fb9 << avatar.b9.mData[3] << "," << avatar.b9.mData[0] << "," << avatar.b9.mData[1] << "," << avatar.b9.mData[2] << "\n";

}

void rotateBone(quaternion q, treenode &joint, float tX, float tY, float tZ)
{
	double q0, q1, q2, q3;
	double angle_rad, angle_deg;
	double x, y, z, fnorm;

	q0 = q.mData[3];
	q1 = q.mData[0];
	q2 = q.mData[2];
	q3 = -q.mData[1];


	angle_rad = acos(q0) * 2;
	angle_deg = angle_rad * 180 / PI;

	x = q1 / sin(angle_rad / 2);
	y = q2 / sin(angle_rad / 2);
	z = q3 / sin(angle_rad / 2);
	fnorm = sqrt(x*x + y*y + z*z);


	glLoadIdentity();
	glTranslatef(tX, tY, tZ);
	//glGetFloatv(GL_MODELVIEW_MATRIX, joint.m);
	glRotatef(angle_deg, x / fnorm, y / fnorm, z / fnorm);
	glGetFloatv(GL_MODELVIEW_MATRIX, joint.m);
}

void rotateJoint(treenode &joint, float tX, float tY, float tZ)
{
	glLoadIdentity();
	glTranslatef(tX, tY, tZ);
	glGetFloatv(GL_MODELVIEW_MATRIX, joint.m);

}

void rotateBody(struct Avatar &avatar)
{
	glPushMatrix();
	rotateBone(avatar.b0, torso_node, 0.0, 0.0, 0.0);
	glPopMatrix();
	glPushMatrix();//Head
	rotateJoint(nk_node, 0.0, TORSO_HEIGHT - 0.25*NECK_HEIGHT, 0.0);
	rotateBone(avatar.b1, head_node, 0.0, 0.75*NECK_HEIGHT, 0.0);
	glPopMatrix();

	glPushMatrix();//Right Arm
	//rotateBone(avatar.b0, torso_node, 0.0, 0.0, 0.0);
	rotateJoint(lsh_node, -(TORSO_RADIUS + UPPER_ARM_RADIUS), 0.9*TORSO_HEIGHT, 0.0);
	rotateBone(avatar.b2, lua_node, 0.0, 0.0, 0.0);
	rotateJoint(lelb_node, 0.0, -UPPER_ARM_HEIGHT, 0.0);
	rotateBone(avatar.b3, lla_node, 0.0, -ELBOW_RADIUS / 2, 0.0);
	rotateJoint(lhand_node, 0.0, -LOWER_ARM_HEIGHT, 0.0);
	glPopMatrix();

	glPushMatrix();//Left Arm
		//rotateBone(avatar.b0, torso_node, 0.0, 0.0, 0.0);
	rotateJoint(rsh_node, TORSO_RADIUS + UPPER_ARM_RADIUS, 0.9*TORSO_HEIGHT, 0.0);
	rotateBone(avatar.b4, rua_node, 0.0, 0.0, 0.0);
	rotateJoint(relb_node, 0.0, -UPPER_ARM_HEIGHT, 0.0);
	rotateBone(avatar.b5, rla_node, 0.0, -ELBOW_RADIUS / 2, 0.0);
	rotateJoint(rhand_node, 0.0, -LOWER_ARM_HEIGHT, 0.0);
	glPopMatrix();

	glPushMatrix();//Right Leg
		//rotateBone(avatar.b0, torso_node, 0.0, 0.0, 0.0);
	rotateBone(avatar.b6.mutiplication(quaternion(1, 0, 0, 0)), lul_node, -TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	rotateJoint(lknee_node, 0.0, UPPER_LEG_HEIGHT, 0.0);
	rotateBone(avatar.b7, lll_node, 0.0, KNEE_RADIUS / 2, 0.0);
	rotateJoint(lfoot_node, 0.0, LOWER_LEG_HEIGHT, 0.0);
	glPopMatrix();

	glPushMatrix();//Left Leg
		//rotateBone(avatar.b0, torso_node, 0.0, 0.0, 0.0);
	rotateBone(avatar.b8.mutiplication(quaternion(1, 0, 0, 0)), rul_node, TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	rotateJoint(rknee_node, 0.0, UPPER_LEG_HEIGHT, 0.0);
	rotateBone(avatar.b9, rll_node, 0.0, KNEE_RADIUS / 2, 0.0);
	rotateJoint(rfoot_node, 0.0, LOWER_LEG_HEIGHT, 0.0);
	glPopMatrix();
	//glPopMatrix();
}

bool isNotSameQuat(quaternion currentQuat, quaternion previousQuat)
{
	if (!isFirst)
	{
		qPA = currentQuat.mutiplication(previousQuat.Inverse());
	}
	else
	{
		isFirst = false;
		return false;
	}

	if (qPA.mData[3] >= 0.99999)
	{
		return false;
	}

	//if (qPA.mData[3] >= 0.99999)
	//{
	//	child = qPrevInvsPA.Inverse();
	//	//printf("Skip lower arm\n");
	//}

	//if (uqPA.mData[3] >= 0.99999)
	//{
	//	parent = uqPrevInvsPA.Inverse();
	//	//printf("Skip upper arm\n");
	//}
	return true;
}

void calaculateTrajectory(quaternion parent, quaternion child, TVec3 &parentVec, TVec3 &childVec)
{
	TVec3 trajVec;
	quaternion tempQuat2 = BodyQuat.mutiplication(parent);

	quaternion tempQuat1 = tempQuat2.mutiplication(child);//Case-2 usf_q


	quaternion lTransfBodyQuat = tempQuat1;
	quaternion uTransfBodyQuat = tempQuat2;
	quaternion vQuat(tempVec._x, tempVec._y, tempVec._z, 0);

	lTransfBodyQuat = lTransfBodyQuat.mutiplication(vQuat.mutiplication(lTransfBodyQuat.Inverse()));
	uTransfBodyQuat = uTransfBodyQuat.mutiplication(vQuat.mutiplication(uTransfBodyQuat.Inverse()));


	parentVec = { (float)uTransfBodyQuat.mData[0], (float)uTransfBodyQuat.mData[1], (float)uTransfBodyQuat.mData[2] };
	childVec = { (float)lTransfBodyQuat.mData[0], (float)lTransfBodyQuat.mData[1], (float)lTransfBodyQuat.mData[2] };
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
			switch (subOption)
			{
			case 1:
			{
				QuatData_Pelvis = connectXS.xsIMU.b0;

				QuatData_head = connectXS.xsIMU.b1;

				QuatData_RightUpperArm = connectXS.xsIMU.b2;
				QuatData_RightLowerArm = connectXS.xsIMU.b3;

				QuatData_LeftUpperArm = connectXS.xsIMU.b4;
				QuatData_LeftLowerArm = connectXS.xsIMU.b5;

				QuatData_RightUpperLeg = connectXS.xsIMU.b6;
				QuatData_RightLowerLeg = connectXS.xsIMU.b7;

				QuatData_LeftUpperLeg = connectXS.xsIMU.b8;
				QuatData_LeftLowerLeg = connectXS.xsIMU.b9;


				if (firstCalib  /*&& fileClose*/)
				{

					std::cout << "AttentionPose:" << std::endl;

					firstPlvCalib = QuatData_Pelvis.Inverse();
					std::cout << "Pelvis Quat:\t\t" << QuatData_Pelvis.mData[3] << "\t" << QuatData_Pelvis.mData[0] << "\t" << QuatData_Pelvis.mData[1] << "\t" << QuatData_Pelvis.mData[2] << std::endl;

					firstHeadCalib = QuatData_head.Inverse();
					std::cout << "Head Quat:\t\t" << QuatData_head.mData[3] << "\t" << QuatData_head.mData[0] << "\t" << QuatData_head.mData[1] << "\t" << QuatData_head.mData[2] << std::endl;

					firstInvQuat_RightUpperArm = QuatData_RightUpperArm.Inverse();
					std::cout << "Right Upper-Arm Quat:\t" << QuatData_RightUpperArm.mData[3] << "\t" << QuatData_RightUpperArm.mData[0] << "\t" << QuatData_RightUpperArm.mData[1] << "\t" << QuatData_RightUpperArm.mData[2] << std::endl;

					firstInvQuat_RightLowerArm = QuatData_RightLowerArm.Inverse();
					std::cout << "Right Lower-Arm Quat:\t" << QuatData_RightLowerArm.mData[3] << "\t" << QuatData_RightLowerArm.mData[0] << "\t" << QuatData_RightLowerArm.mData[1] << "\t" << QuatData_RightLowerArm.mData[2] << std::endl;

					firstInvQuat_LeftUpperArm = QuatData_LeftUpperArm.Inverse();
					std::cout << "Left Upper-Arm Quat:\t" << QuatData_LeftUpperArm.mData[3] << "\t" << QuatData_LeftUpperArm.mData[0] << "\t" << QuatData_LeftUpperArm.mData[1] << "\t" << QuatData_LeftUpperArm.mData[2] << std::endl;

					firstInvQuat_LeftLowerArm = QuatData_LeftLowerArm.Inverse();
					std::cout << "Left Lower-Arm Quat:\t" << QuatData_LeftLowerArm.mData[3] << "\t" << QuatData_LeftLowerArm.mData[0] << "\t" << QuatData_LeftLowerArm.mData[1] << "\t" << QuatData_LeftLowerArm.mData[2] << std::endl;

					firstInvQuat_RightUpperLeg = QuatData_RightUpperLeg.Inverse();
					std::cout << "Right Upper-Leg Quat:\t" << QuatData_RightUpperLeg.mData[3] << "\t" << QuatData_RightUpperLeg.mData[0] << "\t" << QuatData_RightUpperLeg.mData[1] << "\t" << QuatData_RightUpperLeg.mData[2] << std::endl;

					firstInvQuat_RightLowerLeg = QuatData_RightLowerLeg.Inverse();
					std::cout << "Right Lower-Leg Quat:\t" << QuatData_RightLowerLeg.mData[3] << "\t" << QuatData_RightLowerLeg.mData[0] << "\t" << QuatData_RightLowerLeg.mData[1] << "\t" << QuatData_RightLowerLeg.mData[2] << std::endl;

					firstInvQuat_LeftUpperLeg = QuatData_LeftUpperLeg.Inverse();
					std::cout << "Left Upper-Leg Quat:\t" << QuatData_LeftUpperLeg.mData[3] << "\t" << QuatData_LeftUpperLeg.mData[0] << "\t" << QuatData_LeftUpperLeg.mData[1] << "\t" << QuatData_LeftUpperLeg.mData[2] << std::endl;

					firstInvQuat_LeftLowerLeg = QuatData_LeftLowerLeg.Inverse();
					std::cout << "Left Lower-Leg Quat:\t" << QuatData_LeftLowerLeg.mData[3] << "\t" << QuatData_LeftLowerLeg.mData[0] << "\t" << QuatData_LeftLowerLeg.mData[1] << "\t" << firstInvQuat_LeftLowerArm.mData[2] << std::endl;

					firstCalib = false;
				}

				firstInvQuat_C = QuatData_Pelvis.mutiplication(firstPlvCalib);

				quaternion reset_head = firstInvQuat_C.Inverse().mutiplication(QuatData_head.mutiplication(firstHeadCalib));
				reset_head.normalize();

				quaternion reset_RUA = firstInvQuat_C.Inverse().mutiplication(QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm));
				reset_RUA.normalize();

				quaternion reset_RLA = QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm).Inverse().mutiplication(QuatData_RightLowerArm.mutiplication(firstInvQuat_RightLowerArm));
				reset_RLA.normalize();

				quaternion reset_RUL = firstInvQuat_C.Inverse().mutiplication(QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg));
				reset_RUL.normalize();

				quaternion reset_RLL = QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg).Inverse().mutiplication(QuatData_RightLowerLeg.mutiplication(firstInvQuat_RightLowerLeg));
				reset_RLL.normalize();

				quaternion reset_LUA = firstInvQuat_C.Inverse().mutiplication(QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm));
				reset_LUA.normalize();

				quaternion reset_LLA = QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm).Inverse().mutiplication(QuatData_LeftLowerArm.mutiplication(firstInvQuat_LeftLowerArm));
				reset_LLA.normalize();

				quaternion reset_LUL = firstInvQuat_C.Inverse().mutiplication(QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg));
				reset_LUL.normalize();

				quaternion reset_LLL = QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg).Inverse().mutiplication(QuatData_LeftLowerLeg.mutiplication(firstInvQuat_LeftLowerLeg));


				avatar.b0 = firstInvQuat_C;
				avatar.b1 = reset_head;
				avatar.b2 = reset_RUA;
				avatar.b3 = reset_RLA;
				avatar.b4 = reset_LUA;
				avatar.b5 = reset_LLA;
				avatar.b6 = reset_RUL;
				avatar.b7 = reset_RLL;
				avatar.b8 = reset_LUL;
				avatar.b9 = reset_LLL;

				rotateBody(avatar);

				bool isMotion = false;
				if (fileClose)
				{
					if (isNotSameQuat(avatar.b2, avatar.prv_b2) || isNotSameQuat(avatar.b3, avatar.prv_b3) ||
						isNotSameQuat(avatar.b4, avatar.prv_b4) || isNotSameQuat(avatar.b5, avatar.prv_b5) ||
						isNotSameQuat(avatar.b6, avatar.prv_b6) || isNotSameQuat(avatar.b7, avatar.prv_b7) ||
						isNotSameQuat(avatar.b8, avatar.prv_b8) || isNotSameQuat(avatar.b9, avatar.prv_b9) ||
						isNotSameQuat(avatar.b0, avatar.prv_b0) || isNotSameQuat(avatar.b1, avatar.prv_b1))
					{
						writeData();
					}

					if (isNotSameQuat(avatar.b0, avatar.prv_b0))
					{
						isMotion = true;
						TVec3 b0Vec, b1Vec;

						quaternion vQuat(tempVec._x, tempVec._y, tempVec._z, 0);

						quaternion uTransfBodyQuat = avatar.b0.mutiplication(vQuat.mutiplication(avatar.b0.Inverse()));

						traj_b0[trajCount][0] = 1;
						traj_b0[trajCount][1] = uTransfBodyQuat.mData[0];
						traj_b0[trajCount][2] = uTransfBodyQuat.mData[1];
						traj_b0[trajCount][3] = uTransfBodyQuat.mData[2];
					}

					if (isNotSameQuat(avatar.b1, avatar.prv_b1))
					{
						isMotion = true;
						TVec3 b0Vec, b1Vec;
						calaculateTrajectory(avatar.b1, avatar.b1, b0Vec, b1Vec);


						traj_b1[trajCount][0] = 1;
						traj_b1[trajCount][1] = b0Vec._x;
						traj_b1[trajCount][2] = b0Vec._y;
						traj_b1[trajCount][3] = b0Vec._z;

					}

					if (isNotSameQuat(avatar.b2, avatar.prv_b2) || isNotSameQuat(avatar.b3, avatar.prv_b3))
					{
						isMotion = true;

						TVec3 b2Vec, b3Vec;
						calaculateTrajectory(avatar.b2, avatar.b3, b2Vec, b3Vec);

						traj_b2[trajCount][0] = 1;
						traj_b2[trajCount][1] = b2Vec._x;
						traj_b2[trajCount][2] = b2Vec._y;
						traj_b2[trajCount][3] = b2Vec._z;

						traj_b3[trajCount][0] = 1;
						traj_b3[trajCount][1] = b3Vec._x;
						traj_b3[trajCount][2] = b3Vec._y;
						traj_b3[trajCount][3] = b3Vec._z;

					}

					if (isNotSameQuat(avatar.b4, avatar.prv_b4) || isNotSameQuat(avatar.b5, avatar.prv_b5))
					{
						isMotion = true;

						TVec3 b4Vec, b5Vec;
						calaculateTrajectory(avatar.b4, avatar.b5, b4Vec, b5Vec);


						traj_b4[trajCount][0] = 1;
						traj_b4[trajCount][1] = b4Vec._x;
						traj_b4[trajCount][2] = b4Vec._y;
						traj_b4[trajCount][3] = b4Vec._z;

						traj_b5[trajCount][0] = 1;
						traj_b5[trajCount][1] = b5Vec._x;
						traj_b5[trajCount][2] = b5Vec._y;
						traj_b5[trajCount][3] = b5Vec._z;

					}

					if (isNotSameQuat(avatar.b6, avatar.prv_b6) || isNotSameQuat(avatar.b7, avatar.prv_b7))
					{
						isMotion = true;
						TVec3 b6Vec, b7Vec;
						calaculateTrajectory(avatar.b6, avatar.b7, b6Vec, b7Vec);

						traj_b6[trajCount][0] = 1;
						traj_b6[trajCount][1] = b6Vec._x;
						traj_b6[trajCount][2] = b6Vec._y;
						traj_b6[trajCount][3] = b6Vec._z;

						traj_b7[trajCount][0] = 1;
						traj_b7[trajCount][1] = b7Vec._x;
						traj_b7[trajCount][2] = b7Vec._y;
						traj_b7[trajCount][3] = b7Vec._z;

					}

					if (isNotSameQuat(avatar.b8, avatar.prv_b8) || isNotSameQuat(avatar.b9, avatar.prv_b9))
					{
						isMotion = true;
						TVec3 b8Vec, b9Vec;
						calaculateTrajectory(avatar.b8, avatar.b9, b8Vec, b9Vec);

						traj_b8[trajCount][0] = 1;
						traj_b8[trajCount][1] = b8Vec._x;
						traj_b8[trajCount][2] = b8Vec._y;
						traj_b8[trajCount][3] = b8Vec._z;

						traj_b9[trajCount][0] = 1;
						traj_b9[trajCount][1] = b9Vec._x;
						traj_b9[trajCount][2] = b9Vec._y;
						traj_b9[trajCount][3] = b9Vec._z;

					}


					if (!isMotion)
					{
						break;
					}

					avatar.prv_b0 = avatar.b0;
					avatar.prv_b1 = avatar.b1;

					avatar.prv_b2 = avatar.b2;
					avatar.prv_b3 = avatar.b3;
					avatar.prv_b4 = avatar.b4;
					avatar.prv_b5 = avatar.b5;

					avatar.prv_b6 = avatar.b6;
					avatar.prv_b7 = avatar.b7;
					avatar.prv_b8 = avatar.b8;
					avatar.prv_b9 = avatar.b9;

					trajCount++;
				}
			}
			break;

			case 2:
			{
				QuatData_Pelvis = connectXS.xsIMU.b0;

				QuatData_RightUpperArm = connectXS.xsIMU.b1;
				QuatData_RightLowerArm = connectXS.xsIMU.b2;

				QuatData_LeftUpperArm = connectXS.xsIMU.b3;
				QuatData_LeftLowerArm = connectXS.xsIMU.b4;

				if (firstCalib  /*&& fileClose*/)
				{

					std::cout << "AttentionPose:" << std::endl;

					firstPlvCalib = QuatData_Pelvis.Inverse();
					std::cout << "Pelvis Quat:\t\t" << QuatData_Pelvis.mData[3] << "\t" << QuatData_Pelvis.mData[0] << "\t" << QuatData_Pelvis.mData[1] << "\t" << QuatData_Pelvis.mData[2] << std::endl;

					firstInvQuat_RightUpperArm = QuatData_RightUpperArm.Inverse();
					std::cout << "Right Upper-Arm Quat:\t" << firstInvQuat_RightUpperArm.mData[3] << "\t" << firstInvQuat_RightUpperArm.mData[0] << "\t" << firstInvQuat_RightUpperArm.mData[1] << "\t" << firstInvQuat_RightUpperArm.mData[2] << std::endl;

					firstInvQuat_RightLowerArm = QuatData_RightLowerArm.Inverse();
					std::cout << "Right Lower-Arm Quat:\t" << firstInvQuat_RightLowerArm.mData[3] << "\t" << firstInvQuat_RightLowerArm.mData[0] << "\t" << firstInvQuat_RightLowerArm.mData[1] << "\t" << firstInvQuat_RightLowerArm.mData[2] << std::endl;

					firstInvQuat_LeftUpperArm = QuatData_LeftUpperArm.Inverse();
					std::cout << "Left Upper-Arm Quat:\t" << firstInvQuat_LeftUpperArm.mData[3] << "\t" << firstInvQuat_LeftUpperArm.mData[0] << "\t" << firstInvQuat_LeftUpperArm.mData[1] << "\t" << firstInvQuat_LeftUpperArm.mData[2] << std::endl;

					firstInvQuat_LeftLowerArm = QuatData_LeftLowerArm.Inverse();
					std::cout << "Left Lower-Arm Quat:\t" << firstInvQuat_LeftLowerArm.mData[3] << "\t" << firstInvQuat_LeftLowerArm.mData[0] << "\t" << firstInvQuat_LeftLowerArm.mData[1] << "\t" << firstInvQuat_LeftLowerArm.mData[2] << std::endl;


					firstCalib = false;
				}

				firstInvQuat_C = QuatData_Pelvis.mutiplication(firstPlvCalib);

				quaternion reset_RUA = firstInvQuat_C.Inverse().mutiplication(QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm));
				reset_RUA.normalize();

				quaternion reset_RLA = QuatData_RightUpperArm.mutiplication(firstInvQuat_RightUpperArm).Inverse().mutiplication(QuatData_RightLowerArm.mutiplication(firstInvQuat_RightLowerArm));
				reset_RLA.normalize();



				quaternion reset_LUA = firstInvQuat_C.Inverse().mutiplication(QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm));
				reset_LUA.normalize();

				quaternion reset_LLA = QuatData_LeftUpperArm.mutiplication(firstInvQuat_LeftUpperArm).Inverse().mutiplication(QuatData_LeftLowerArm.mutiplication(firstInvQuat_LeftLowerArm));
				reset_LLA.normalize();

				avatar.b0 = firstInvQuat_C;
				avatar.b2 = reset_RUA;
				avatar.b3 = reset_RLA;
				avatar.b4 = reset_LUA;
				avatar.b5 = reset_LLA;

				rotateBody(avatar);

				bool isMotion = false;
				if (fileClose)
				{

					if (isNotSameQuat(avatar.b2, avatar.prv_b2) || isNotSameQuat(avatar.b3, avatar.prv_b3) ||
						isNotSameQuat(avatar.b4, avatar.prv_b4) || isNotSameQuat(avatar.b5, avatar.prv_b5) ||
						isNotSameQuat(avatar.b0, avatar.prv_b0))
					{
						writeData();
					}

					if (isNotSameQuat(avatar.b0, avatar.prv_b0))
					{
						isMotion = true;
						TVec3 b0Vec, b1Vec;

						quaternion vQuat(tempVec._x, tempVec._y, tempVec._z, 0);

						quaternion uTransfBodyQuat = avatar.b0.mutiplication(vQuat.mutiplication(avatar.b0.Inverse()));

						traj_b0[trajCount][0] = 1;
						traj_b0[trajCount][1] = uTransfBodyQuat.mData[0];
						traj_b0[trajCount][2] = uTransfBodyQuat.mData[1];
						traj_b0[trajCount][3] = uTransfBodyQuat.mData[2];
					}



					if (isNotSameQuat(avatar.b2, avatar.prv_b2) || isNotSameQuat(avatar.b3, avatar.prv_b3))
					{
						isMotion = true;

						TVec3 b2Vec, b3Vec;
						calaculateTrajectory(avatar.b2, avatar.b3, b2Vec, b3Vec);

						traj_b2[trajCount][0] = 1;
						traj_b2[trajCount][1] = b2Vec._x;
						traj_b2[trajCount][2] = b2Vec._y;
						traj_b2[trajCount][3] = b2Vec._z;

						traj_b3[trajCount][0] = 1;
						traj_b3[trajCount][1] = b3Vec._x;
						traj_b3[trajCount][2] = b3Vec._y;
						traj_b3[trajCount][3] = b3Vec._z;
					}

					if (isNotSameQuat(avatar.b4, avatar.prv_b4) || isNotSameQuat(avatar.b5, avatar.prv_b5))
					{
						isMotion = true;

						TVec3 b4Vec, b5Vec;
						calaculateTrajectory(avatar.b4, avatar.b5, b4Vec, b5Vec);


						traj_b4[trajCount][0] = 1;
						traj_b4[trajCount][1] = b4Vec._x;
						traj_b4[trajCount][2] = b4Vec._y;
						traj_b4[trajCount][3] = b4Vec._z;

						traj_b5[trajCount][0] = 1;
						traj_b5[trajCount][1] = b5Vec._x;
						traj_b5[trajCount][2] = b5Vec._y;
						traj_b5[trajCount][3] = b5Vec._z;
					}


					if (!isMotion)
					{
						break;
					}

					avatar.prv_b0 = avatar.b0;

					avatar.prv_b2 = avatar.b2;
					avatar.prv_b3 = avatar.b3;
					avatar.prv_b4 = avatar.b4;
					avatar.prv_b5 = avatar.b5;


					trajCount++;

				}

			}
			break;

			case 3:
			{
				QuatData_Pelvis = connectXS.xsIMU.b0;

				QuatData_RightUpperLeg = connectXS.xsIMU.b1;
				QuatData_RightLowerLeg = connectXS.xsIMU.b2;

				QuatData_LeftUpperLeg = connectXS.xsIMU.b3;
				QuatData_LeftLowerLeg = connectXS.xsIMU.b4;

				if (firstCalib  /*&& fileClose*/)
				{

					std::cout << "AttentionPose:" << std::endl;

					firstPlvCalib = QuatData_Pelvis.Inverse();
					std::cout << "Pelvis Quat:\t" << QuatData_Pelvis.mData[3] << "\t" << QuatData_Pelvis.mData[0] << "\t" << QuatData_Pelvis.mData[1] << "\t" << QuatData_Pelvis.mData[2] << std::endl;

					firstInvQuat_RightUpperLeg = QuatData_RightUpperLeg.Inverse();
					std::cout << "Right Upper-Leg Quat:\t" << QuatData_RightUpperLeg.mData[3] << "\t" << QuatData_RightUpperLeg.mData[0] << "\t" << QuatData_RightUpperLeg.mData[1] << "\t" << QuatData_RightUpperLeg.mData[2] << std::endl;

					firstInvQuat_RightLowerLeg = QuatData_RightLowerLeg.Inverse();
					std::cout << "Right Lower-Leg Quat:\t" << QuatData_RightLowerLeg.mData[3] << "\t" << QuatData_RightLowerLeg.mData[0] << "\t" << QuatData_RightLowerLeg.mData[1] << "\t" << QuatData_RightLowerLeg.mData[2] << std::endl;

					firstInvQuat_LeftUpperLeg = QuatData_LeftUpperLeg.Inverse();
					std::cout << "Left Upper-Arm Quat:\t" << QuatData_LeftUpperLeg.mData[3] << "\t" << QuatData_LeftUpperLeg.mData[0] << "\t" << QuatData_LeftUpperLeg.mData[1] << "\t" << QuatData_LeftUpperLeg.mData[2] << std::endl;

					firstInvQuat_LeftLowerLeg = QuatData_LeftLowerLeg.Inverse();
					std::cout << "Left Lower-Arm Quat:\t" << QuatData_LeftLowerLeg.mData[3] << "\t" << QuatData_LeftLowerLeg.mData[0] << "\t" << QuatData_LeftLowerLeg.mData[1] << "\t" << firstInvQuat_LeftLowerArm.mData[2] << std::endl;

					firstCalib = false;
				}

				firstInvQuat_C = QuatData_Pelvis.mutiplication(firstPlvCalib);

				quaternion reset_RUL = firstInvQuat_C.Inverse().mutiplication(QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg));
				reset_RUL.normalize();
				//quaternion usf_q = reset_RUL;
				quaternion reset_RLL = QuatData_RightUpperLeg.mutiplication(firstInvQuat_RightUpperLeg).Inverse().mutiplication(QuatData_RightLowerLeg.mutiplication(firstInvQuat_RightLowerLeg));
				reset_RLL.normalize();
				//quaternion lsf_q = reset_RLL;


				quaternion reset_LUL = firstInvQuat_C.Inverse().mutiplication(QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg));
				reset_LUL.normalize();
				//usf_q = reset_LUL;
				quaternion reset_LLL = QuatData_LeftUpperLeg.mutiplication(firstInvQuat_LeftUpperLeg).Inverse().mutiplication(QuatData_LeftLowerLeg.mutiplication(firstInvQuat_LeftLowerLeg));
				reset_LLL.normalize();

				avatar.b0 = firstInvQuat_C;
				avatar.b6 = reset_RUL;
				avatar.b7 = reset_RLL;
				avatar.b8 = reset_LUL;
				avatar.b9 = reset_LLL;

				rotateBody(avatar);

				bool isMotion = false;
				if (fileClose)
				{

					if (isNotSameQuat(avatar.b6, avatar.prv_b6) || isNotSameQuat(avatar.b7, avatar.prv_b7) ||
						isNotSameQuat(avatar.b8, avatar.prv_b8) || isNotSameQuat(avatar.b9, avatar.prv_b9) ||
						isNotSameQuat(avatar.b0, avatar.prv_b0))
					{
						writeData();
					}

					if (isNotSameQuat(avatar.b0, avatar.prv_b0))
					{
						isMotion = true;
						TVec3 b0Vec, b1Vec;

						quaternion vQuat(tempVec._x, tempVec._y, tempVec._z, 0);

						quaternion uTransfBodyQuat = avatar.b0.mutiplication(vQuat.mutiplication(avatar.b0.Inverse()));

						traj_b0[trajCount][0] = 1;
						traj_b0[trajCount][1] = uTransfBodyQuat.mData[0];
						traj_b0[trajCount][2] = uTransfBodyQuat.mData[1];
						traj_b0[trajCount][3] = uTransfBodyQuat.mData[2];
					}

					if (isNotSameQuat(avatar.b6, avatar.prv_b6) || isNotSameQuat(avatar.b7, avatar.prv_b7))
					{
						isMotion = true;
						TVec3 b6Vec, b7Vec;
						calaculateTrajectory(avatar.b6, avatar.b7, b6Vec, b7Vec);

						traj_b6[trajCount][0] = 1;
						traj_b6[trajCount][1] = b6Vec._x;
						traj_b6[trajCount][2] = b6Vec._y;
						traj_b6[trajCount][3] = b6Vec._z;

						traj_b7[trajCount][0] = 1;
						traj_b7[trajCount][1] = b7Vec._x;
						traj_b7[trajCount][2] = b7Vec._y;
						traj_b7[trajCount][3] = b7Vec._z;

					}

					if (isNotSameQuat(avatar.b8, avatar.prv_b8) || isNotSameQuat(avatar.b9, avatar.prv_b9))
					{
						isMotion = true;
						TVec3 b8Vec, b9Vec;
						calaculateTrajectory(avatar.b8, avatar.b9, b8Vec, b9Vec);

						traj_b8[trajCount][0] = 1;
						traj_b8[trajCount][1] = b8Vec._x;
						traj_b8[trajCount][2] = b8Vec._y;
						traj_b8[trajCount][3] = b8Vec._z;

						traj_b9[trajCount][0] = 1;
						traj_b9[trajCount][1] = b9Vec._x;
						traj_b9[trajCount][2] = b9Vec._y;
						traj_b9[trajCount][3] = b9Vec._z;

					}


					if (!isMotion)
					{
						break;
					}

					avatar.prv_b0 = avatar.b0;

					avatar.prv_b6 = avatar.b6;
					avatar.prv_b7 = avatar.b7;
					avatar.prv_b8 = avatar.b8;
					avatar.prv_b9 = avatar.b9;

					trajCount++;
				}

			}
			break;

			default:
				break;
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
		firstCalib = true;

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
				outDataL.close();
				outDataU.close();
				matchDBTrajectory("Load\\UFormFile.csv", "Load\\LFormFile.csv");
				break;
			}

			///////////////
			quaternion qutObj;
			glPushMatrix();
			glLoadIdentity();
			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

			quaternion reset_U(uqdata[isize][0], uqdata[isize][1], uqdata[isize][2], uqdata[isize][3]);
			quaternion reset_L(lqdata[isize][0], lqdata[isize][1], lqdata[isize][2], lqdata[isize][3]);
			float q0 = reset_U.mData[3];
			float q1 = reset_U.mData[0];
			float q2 = reset_U.mData[2];
			float q3 = -reset_U.mData[1];

			float angle_rad = acos(q0) * 2;
			float angle_deg = angle_rad * 180 / PI;

			float x = q1 / sin(angle_rad / 2);
			float y = q2 / sin(angle_rad / 2);
			float z = q3 / sin(angle_rad / 2);
			float fnorm = sqrt(x*x + y*y + z*z);

			glRotatef(angle_deg, x / fnorm, y / fnorm, z / fnorm);

			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

			glLoadIdentity();
			glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);

			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

			q0 = lqdata[isize][3];
			q1 = lqdata[isize][0];
			q2 = lqdata[isize][2];
			q3 = -lqdata[isize][1];



			if (q0 == 9)
			{
				trajCount++;
				traj_b3[trajCount][0] = 9;
				traj_b3[trajCount][1] = 0;
				traj_b3[trajCount][2] = 0;
				traj_b3[trajCount][3] = 0;
				isize++;
				glPopMatrix();

				break;
			}

			angle_rad = acos(q0) * 2;
			angle_deg = angle_rad * 180 / PI;
			x = q1 / sin(angle_rad / 2);
			y = q2 / sin(angle_rad / 2);
			z = q3 / sin(angle_rad / 2);

			fnorm = sqrt(x*x + y*y + z*z);

			glLoadIdentity();
			glTranslatef(0.0, 0.0, 0.0);
			glRotatef(angle_deg, x / fnorm, y / fnorm, z / fnorm);
			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);
			glPopMatrix();

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

			quaternion tempQuat1 = BodyQuat.mutiplication(reset_U);
			quaternion tempQuat2 = tempQuat1.mutiplication(reset_L);

			TVector3 TransfBodyQuat1 = tempQuat2.quternionMatrices(tempQuat2, tempVec);
			TVector3 TransfBodyQuat2 = tempQuat1.quternionMatrices(tempQuat1, tempVec);

			///////////////////////////////////
			trajCount++;

			if (trajCount == 1)
			{
				outDataL.open("pointData\\StandarduserDataL.txt");
				outDataU.open("pointData\\StandarduserDataU.txt");
			}
			traj_b3[trajCount][0] = 1.0;
			traj_b3[trajCount][1] = TransfBodyQuat1._x;
			traj_b3[trajCount][2] = TransfBodyQuat1._y;
			traj_b3[trajCount][3] = TransfBodyQuat1._z;

			outDataL << traj_b3[trajCount][1] << "\t" << traj_b3[trajCount][2] << "\t" << traj_b3[trajCount][3] << endl;

			traj_b2[trajCount][0] = 1.0;
			traj_b2[trajCount][1] = TransfBodyQuat2._x;
			traj_b2[trajCount][2] = TransfBodyQuat2._y;
			traj_b2[trajCount][3] = TransfBodyQuat2._z;

			outDataU << traj_b2[trajCount][1] << "\t" << traj_b2[trajCount][2] << "\t" << traj_b2[trajCount][3] << endl;
			//traj_b3[trajCount][0] = 1;// TransfBodyQuat.mData[3];
			//traj_b3[trajCount][1] = TransfBodyQuat.mData[0];
			//traj_b3[trajCount][2] = TransfBodyQuat.mData[1];
			//traj_b3[trajCount][3] = TransfBodyQuat.mData[2];

			isize++;
			break;
		}
	}
	break;

	case 9:
	{

	}
	break;

	case 10:
	{
		if (bReadDBFile)
		{
			//............Right Arm.............//
			if (idbsize >= dbCount)
			{
				bReadDBFile = false;
				idbsize = 0;
				outDataL.close();
				outDataU.close();
				break;
			}

			///////////////
			quaternion qutObj;
			glPushMatrix();
			glLoadIdentity();
			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

			quaternion reset_U(uqdataDB[idbsize][0], uqdataDB[idbsize][1], uqdataDB[idbsize][2], uqdataDB[idbsize][3]);
			quaternion reset_L(lqdataDB[idbsize][0], lqdataDB[idbsize][1], lqdataDB[idbsize][2], lqdataDB[idbsize][3]);

			float q0 = reset_U.mData[3];
			float q1 = reset_U.mData[0];
			float q2 = reset_U.mData[2];
			float q3 = -reset_U.mData[1];


			float angle_rad = acos(q0) * 2;
			float angle_deg = angle_rad * 180 / PI;

			float x = q1 / sin(angle_rad / 2);
			float y = q2 / sin(angle_rad / 2);
			float z = q3 / sin(angle_rad / 2);
			float fnorm = sqrt(x*x + y*y + z*z);

			glRotatef(angle_deg, x / fnorm, y / fnorm, z / fnorm);

			glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

			glLoadIdentity();
			glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0);
			glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);

			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

			q0 = reset_L.mData[3];
			q1 = reset_L.mData[0];
			q2 = reset_L.mData[2];
			q3 = -reset_L.mData[1];

			angle_rad = acos(q0) * 2;
			angle_deg = angle_rad * 180 / PI;
			x = q1 / sin(angle_rad / 2);
			y = q2 / sin(angle_rad / 2);
			z = q3 / sin(angle_rad / 2);

			fnorm = sqrt(x*x + y*y + z*z);

			glLoadIdentity();
			glTranslatef(0.0, 0.0, 0.0);
			glRotatef(angle_deg, x / fnorm, y / fnorm, z / fnorm);
			glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);
			glPopMatrix();

			quaternion tempQuat1 = BodyQuat.mutiplication(reset_U);
			quaternion tempQuat2 = tempQuat1.mutiplication(reset_L);

			TVector3 TransfBodyQuat1 = tempQuat2.quternionMatrices(tempQuat2, tempVec);
			TVector3 TransfBodyQuat2 = tempQuat1.quternionMatrices(tempQuat1, tempVec);

			///////////////////////////////////

			indexDB++;
			if (indexDB == 1) {
				outDataL.open("pointData\\outDataDBL.txt");
				outDataU.open("pointData\\outDataDBU.txt");
			}
			lDB_data[indexDB][0] = 1.0;
			lDB_data[indexDB][1] = TransfBodyQuat1._x;
			lDB_data[indexDB][2] = TransfBodyQuat1._y;
			lDB_data[indexDB][3] = TransfBodyQuat1._z;

			outDataL << lDB_data[indexDB][1] << "\t" << lDB_data[indexDB][2] << "\t" << lDB_data[indexDB][3] << endl;

			uDB_data[indexDB][0] = 1.0;
			uDB_data[indexDB][1] = TransfBodyQuat2._x;
			uDB_data[indexDB][2] = TransfBodyQuat2._y;
			uDB_data[indexDB][3] = TransfBodyQuat2._z;

			outDataU << uDB_data[indexDB][1] << "\t" << uDB_data[indexDB][2] << "\t" << uDB_data[indexDB][3] << endl;

			idbsize++;
			break;
		}
	}
	break;
	}
	glutPostRedisplay();
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
	glTranslatef(-TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);//glTranslatef(-TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	glRotatef(theta[7], 1.0, 0.0, 0.0);//glRotatef(theta[7], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, lul_node.m);
	lul_node.f = left_upper_leg;
	lul_node.sibling = &rul_node;
	lul_node.child = &lknee_node;

	glLoadIdentity();
	glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);//glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
	glRotatef(theta[9], 1.0, 0.0, 0.0);//glRotatef(theta[9], 1.0, 0.0, 0.0);
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

	///////////////////////Texture mapping///////////////////////////
	image_t   temp_image;

	//glClearColor(1.0, 1.0, 1.0, 0.0);
	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(2, texture_id);

	glBindTexture(GL_TEXTURE_2D, texture_id[0]);
	//glEnable(GL_BLEND);							// Enable Blending       (disable alpha testing)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	tgaLoad("worldmgrs.tga", &temp_image, TGA_FREE | TGA_LOW_QUALITY);

	//glEnable(GL_CULL_FACE);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	sphere = gluNewQuadric();
	gluQuadricDrawStyle(sphere, GLU_FILL);
	gluQuadricNormals(sphere, GLU_SMOOTH);
	gluQuadricOrientation(sphere, GLU_INSIDE);
	gluQuadricTexture(sphere, GL_TRUE);

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
		firstCalib = true;

	}

	if (key == 'v')
	{
		startAnim = !startAnim;
		firstCalib = true;

		connectXS.xsIMU = { qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit };
		avatar = { qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit,qInit };
		std::cout << "\n Ready for recording...!" << std::endl;
	}

	if (key == 'u')
	{
		fileClose = !fileClose;
		if (fileClose)
		{
			std::cout << "\n Recording....!" << std::endl;

			openFile();

			trajCount = 0;
			LindexP = 0;
			memset(traj_b0, 0, 80056 * (sizeof(float)));
			memset(traj_b1, 0, 80056 * (sizeof(float)));
			memset(traj_b2, 0, 80056 * (sizeof(float)));
			memset(traj_b3, 0, 80056 * (sizeof(float)));
			memset(traj_b4, 0, 80056 * (sizeof(float)));
			memset(traj_b5, 0, 80056 * (sizeof(float)));
			memset(traj_b6, 0, 80056 * (sizeof(float)));
			memset(traj_b7, 0, 80056 * (sizeof(float)));
			memset(traj_b8, 0, 80056 * (sizeof(float)));
			memset(traj_b9, 0, 80056 * (sizeof(float)));

			memset(cIndexArray, 0, 200 * (sizeof(int)));
			CenterIndex = 0;
			start = std::clock();
			isFirst = true;

		}
		else
		{
			closeFile();
			//matchDBTrajectory(UfileName, LfileName);

		}

		Comparision::resetDiagnosis();
	}

	if (key == 49) //Key-1
	{
		std::ifstream infile;
		infile.open("./Load/4LeftLFormFile.csv");

		infile.unsetf(std::ios_base::skipws);

		unsigned line_count = std::count(
			std::istream_iterator<char>(infile),
			std::istream_iterator<char>(),
			'\n');

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

		std::ifstream infileu;
		infileu.open("./Load/4LeftLFormFile.csv");

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
		}

		std::ifstream infilep;
		infilep.open("./Load/5LeftUFormFile.csv");

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

		infile.close();
		infileu.close();

		bReadFile = true;
		trajCount = 0;
		memset(traj_b3, 0, 8056 * (sizeof(int)));
		memset(traj_b2, 0, 8 * (sizeof(int)));
		start = std::clock();
		Comparision::resetDiagnosis();
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

		bReadDBFile = true;
		indexDB = 0;

		memset(uDB_data, 0, 8056 * (sizeof(int)));
		memset(lDB_data, 0, 8 * (sizeof(int)));
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
		bReadDBFile = true;
		indexDB = 0;

		memset(uDB_data, 0, 8056 * (sizeof(int)));
		memset(lDB_data, 0, 8 * (sizeof(int)));

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
		bReadDBFile = true;
		indexDB = 0;

		memset(uDB_data, 0, 8056 * (sizeof(int)));
		memset(lDB_data, 0, 8 * (sizeof(int)));
	}


	if (key == 'q') { exit(0); }
}

void menu(int id)
{
	option = id;

	if (id == 3) { rotate_ = 90; xr = 0; yr = 1; zr = 0; }
	if (id == 4) { rotate_ = 0;  xr = 0; yr = 1; zr = 0; }
	if (id == 5) { rotate_ = 30; xr = 0; yr = 1; zr = 0; }
	if (id == 6) { rotate_ = 90; xr = 1; yr = 0; zr = 0; }
}

void dataCapture(int id)
{
	option = 1;
	subOption = id;
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

	int dataCaptureSub = glutCreateMenu(dataCapture);
	glutAddMenuEntry("Full Body", 1);
	glutAddMenuEntry("Upper Body", 2);
	glutAddMenuEntry("Lower Body", 3);

	glutCreateMenu(menu);
	glutAddMenuEntry(" Start Xsens ", 0);
	glutAddSubMenu(" Data Capture", dataCaptureSub);

	glutAddMenuEntry(" Read file ", 8);
	glutAddMenuEntry(" Read DB file ", 10);
	//glutAddMenuEntry(" Calibration ", 9);
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
