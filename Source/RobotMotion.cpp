// RobotMotion.cpp : OpenGL application.
/*
Comment Author: Bharatesh C
This is a Source for the OpenGL robot 3D model
*/
#include"XsensConnection.h"

#include <math.h>
#include <stdio.h>
#include <GL/glut.h>
#include <stdlib.h>


#include <vtkQuaternion.h>
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

int option = -1;
int animation = -3; 
int done = 0; 
double rotate_ = 0;
double horizontal = 0;

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

typedef struct treenode
{
	GLfloat m[16];
	void(*f)();
	struct treenode *sibling;
	struct treenode *child;
}treenode;

typedef treenode* t_ptr;

static GLfloat theta[11] = { 0.0,0.0,0.0,180.0,0.0,180.0,0.0,
180.0,0.0,180.0,0.0 }; /* initial joint angles */

static GLint angle = 2;

GLUquadricObj *t, *h, *lua, *lla, *rua, *rla, *lll, *rll, *rul, *lul;
GLUquadricObj *relb, *lelb, *rknee, *lknee, *nk, *lhand, *rhand, *lfoot, *rfoot, *rsh, *lsh;


treenode torso_node, head_node, lua_node, rua_node, lll_node, rll_node,
lla_node, rla_node, rul_node, lul_node,
relb_node, lelb_node, rknee_node, lknee_node, nk_node, lhand_node, rhand_node, lfoot_node, rfoot_node,
rsh_node, lsh_node;

void drawCoordinate()
{
	/*glDisable(GL_LIGHTING);
	glLineWidth(3.0);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0); // x-axis red
	glVertex3i(-2.5, 0, 0);
	glVertex3i(2.5, 0, 0);
	glColor3f(0, 1, 0); // y-axis green
	glVertex3i(0, -2.5, 0);
	glVertex3i(0, 2.5, 0);
	glColor3f(0, 0, 1); // z-axis blue
	glVertex3i(0, 0, -2.5);
	glVertex3i(0, 0, 2.5);
	glEnd();
	glEnable(GL_LIGHTING);*/
}

void DrawGrid()
{
	glDisable(GL_LIGHTING);
	glColor3f(.3, .3, .3);
	glBegin(GL_QUADS);
	glVertex3f(-10, -8.1, 10);
	glVertex3f(10, -8.1, 10);
	glVertex3f(10, -8.51, -10);
	glVertex3f(-10, -8.51, -10);
	glEnd();

	/*glBegin(GL_LINES);
	for (int i = 0; i <= 10; i++) {
	if (i == 0) { glColor3f(.6, .3, .3); }
	else { glColor3f(.25, .25, .25); };
	glVertex3f(i, 0, 0);
	glVertex3f(i, 0, 10);
	if (i == 0) { glColor3f(.3, .3, .6); }
	else { glColor3f(.25, .25, .25); };
	glVertex3f(0, 0, i);
	glVertex3f(10, 0, i);
	};
	glEnd();*/
	glEnable(GL_LIGHTING);
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
	gluCylinder(t, TORSO_RADIUS / 1.2, TORSO_RADIUS, TORSO_HEIGHT, 10, 10);
	glPopMatrix();
}

void head()
{
	glPushMatrix();

	glTranslatef(0.0, HEAD_HEIGHT, 0.0);
	glScalef(HEAD_RADIUS, HEAD_HEIGHT, HEAD_RADIUS);
	gluSphere(h, HEAD_RADIUS, 10, 10);
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
	gluSphere(relb, SHOULDER_RADIUS, 10, 10);
	glPopMatrix();
}

void leftShoulder()
{
	glPushMatrix();
	gluSphere(lelb, SHOULDER_RADIUS, 10, 10);
	glPopMatrix();
}

void rightElbow()
{
	glPushMatrix();
	gluSphere(relb, ELBOW_RADIUS, 10, 10);
	glPopMatrix();
}

void leftElbow()
{
	glPushMatrix();
	gluSphere(lelb, ELBOW_RADIUS, 10, 10);
	glPopMatrix();
}

void rightKnee()
{
	glPushMatrix();
	gluSphere(rknee, KNEE_RADIUS, 10, 10);
	glPopMatrix();
}

void leftKnee()
{
	glPushMatrix();
	gluSphere(lknee, KNEE_RADIUS, 10, 10);
	glPopMatrix();
}

void leftFoot()
{
	glPushMatrix();
	gluSphere(lknee, FOOT_RADIUS, 10, 10);
	glPopMatrix();
}

void rightFoot()
{
	glPushMatrix();
	gluSphere(lknee, FOOT_RADIUS, 10, 10);
	glPopMatrix();
}

void rightHand()
{
	glPushMatrix();
	gluSphere(lknee, HAND_RADIUS, 10, 10);
	glPopMatrix();
}

void leftHand()
{
	glPushMatrix();
	gluSphere(lknee, HAND_RADIUS, 10, 10);
	glPopMatrix();
}

void left_upper_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(lua, UPPER_ARM_RADIUS, UPPER_ARM_RADIUS - 0.1, UPPER_ARM_HEIGHT, 10, 10);
	drawCoordinate();
	glPopMatrix();
}

void left_lower_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(lla, LOWER_ARM_RADIUS - 0.1, LOWER_ARM_RADIUS - 0.15, LOWER_ARM_HEIGHT, 10, 10);
	drawCoordinate();
	glPopMatrix();
}

void right_upper_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(rua, UPPER_ARM_RADIUS, UPPER_ARM_RADIUS - 0.1, UPPER_ARM_HEIGHT, 10, 10);
	drawCoordinate();
	glPopMatrix();
}

void right_lower_arm()
{
	glPushMatrix();
	glRotatef(90.0, 1.0, 0.0, 0.0);//-90
	gluCylinder(rla, LOWER_ARM_RADIUS - 0.1, LOWER_ARM_RADIUS - 0.15, LOWER_ARM_HEIGHT, 10, 10);
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

void display(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-15, 15, -15, 15, -15, 15);
	glRotatef(rotate_, 0, 1, 0);
	glMatrixMode(GL_MODELVIEW);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glColor3f(0.4, 0.4, 0.4);
	DrawGrid();
	traverse(&torso_node);

	glutSwapBuffers();
}

void inverseKinematics()
{
	switch (animation)
	{
	case 0: //an to teleytaio animation einai to 0 antistrepse to
		if (theta[5] < 180.0)
		{
			theta[5] += STEP;
			theta[3] += STEP;
			theta[1] -= 0.2*STEP;
		}
		else animation = option; //an exei antistrafei tote eimaste stin arxiki thesi kai to neo animation einai to option

		glPushMatrix();

		glLoadIdentity();
		glTranslatef(0, 0, 0.0);
		glRotatef(theta[5], 1.0, 0.0, 0.0);

		glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);

		glLoadIdentity();
		glTranslatef(0, 0, 0.0);
		glRotatef(theta[3], 1.0, 0.0, 0.0);

		glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

		glLoadIdentity();
		glTranslatef(0.0, TORSO_HEIGHT - 0.25*NECK_HEIGHT, 0.0);
		glRotatef(theta[1], 1.0, 0.0, 0.0);

		glGetFloatv(GL_MODELVIEW_MATRIX, nk_node.m);

		glPopMatrix();
		break;
	case 1:
		if (theta[9] < 180.0)
		{
			theta[9] += STEP;
			theta[10] -= STEP;
		}
		else animation = option;

		glPushMatrix();

		glLoadIdentity();
		glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
		glRotatef(theta[9], 1.0, 0.0, 0.0);

		glGetFloatv(GL_MODELVIEW_MATRIX, rul_node.m);

		glLoadIdentity();
		glTranslatef(0.0, UPPER_LEG_HEIGHT, 0.0);

		glGetFloatv(GL_MODELVIEW_MATRIX, rknee_node.m);

		glLoadIdentity();
		glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
		glRotatef(theta[10], 1.0, 0.0, 0.0);

		glGetFloatv(GL_MODELVIEW_MATRIX, rll_node.m);

		glPopMatrix();
		break;
		
	case 7: case -2:
		if (theta[9] < 180.0)
		{
			theta[9] += STEP;
			theta[10] -= STEP;
			theta[7] += STEP;
			theta[8] -= STEP;
			theta[5] -= STEP;
			theta[6] += STEP;
			theta[3] += STEP;
			theta[4] -= STEP;
			horizontal -= 0.03*STEP;
		}
		else animation = option;

		glPushMatrix();

		glLoadIdentity();
		glTranslatef(0, 0, 0.0);
		glRotatef(theta[5], 0.0, 0.0, 1.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);

		glLoadIdentity();
		glTranslatef(0, 0, 0.0);
		glRotatef(theta[6], 0.0, 0.0, 1.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, rla_node.m);

		glLoadIdentity();
		glTranslatef(0, 0, 0.0);
		glRotatef(theta[3], 0.0, 0.0, 1.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

		glLoadIdentity();
		glTranslatef(0, 0, 0.0);
		glRotatef(theta[4], 0.0, 0.0, 1.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

		glLoadIdentity();
		glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
		glRotatef(theta[9], 1.0, 0.0, 0.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, rul_node.m);

		glLoadIdentity();
		glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
		glRotatef(theta[10], 1.0, 0.0, 0.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, rll_node.m);

		glLoadIdentity();
		glTranslatef(-TORSO_RADIUS / 2, 0.1*UPPER_LEG_HEIGHT, 0.0);
		glRotatef(theta[7], 1.0, 0.0, 0.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, lul_node.m);

		glLoadIdentity();
		glTranslatef(0.0, KNEE_RADIUS / 2, 0.0);
		glRotatef(theta[8], 1.0, 0.0, 0.0);
		glGetFloatv(GL_MODELVIEW_MATRIX, lll_node.m);

		glLoadIdentity();
		glTranslatef(0, -horizontal, -horizontal);
		glGetFloatv(GL_MODELVIEW_MATRIX, torso_node.m);

		glPopMatrix();
		break;
	case 6:

	{glPushMatrix();
	float mdlv[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, mdlv);
	vtkQuaternion<float> q = vtkQuaternion<float>(connectXS.ax[3], connectXS.ax[0], connectXS.ax[1], connectXS.ax[2]);

	float qw = sqrt(1 + mdlv[0] + mdlv[5] + mdlv[10]);
	float qx = (mdlv[9] - mdlv[6]) / (4 * qw);
	float qy = (mdlv[2] - mdlv[8]) / (4 * qw);
	float qz = (mdlv[4] - mdlv[1]) / (4 * qw);
	vtkQuaternion<float> qA = vtkQuaternion<float>(qw, qx, qy, qz);

	q = qA.Inverse() * q;
	q.Normalize();

	float axis[3];
	float angle = q.GetRotationAngleAndAxis(axis);

	glLoadIdentity();
	//glTranslatef(TORSO_RADIUS / 2, 0.1*UPPER_ARM_HEIGHT, 0.0);
	glTranslatef(0.0, 0.0, 0.0);
	//glRotatef(90, 1.0, 0.0, 0.0);
	glRotatef(angle * 180.0 / 3.14159, axis[2], axis[0], -axis[1]);
	glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);

	glLoadIdentity();
	glTranslatef(0.0, UPPER_ARM_HEIGHT, 0.0);

	glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);



	glGetFloatv(GL_MODELVIEW_MATRIX, mdlv);
	q = vtkQuaternion<float>(connectXS.ax2[3], connectXS.ax2[0], connectXS.ax2[1], connectXS.ax2[2]);

	qw = sqrt(1 + mdlv[0] + mdlv[5] + mdlv[10]);
	qx = (mdlv[9] - mdlv[6]) / (4 * qw);
	qy = (mdlv[2] - mdlv[8]) / (4 * qw);
	qz = (mdlv[4] - mdlv[1]) / (4 * qw);
	qA = vtkQuaternion<float>(qw, qx, qy, qz);

	q = qA.Inverse() * q;
	q.Normalize();

	axis[3];
	angle = q.GetRotationAngleAndAxis(axis);

	glLoadIdentity();
	glTranslatef(0.0, ELBOW_RADIUS / 2, 0.0);
	glRotatef(theta[10], 1.0, 0.0, 0.0);
	glRotatef(angle * 180.0 / 3.14159, axis[2], axis[0], axis[1]);
	glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

	glPopMatrix(); }
	break;
	default: animation = option;
	}
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

				if (DataAvailable)
				{
					//............Right Arm.............//

					glPushMatrix();
										

					glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);
					float qw = sqrt(1 + lua_node.m[0] + lua_node.m[5] + lua_node.m[10]) / 2.0f;
					float qx = (lua_node.m[6] - lua_node.m[9]) / (4.0f * qw);
					float qy = (lua_node.m[8] - lua_node.m[2]) / (4.0f * qw);
					float qz = (lua_node.m[1] - lua_node.m[4]) / (4.0f * qw);
					vtkQuaternion<float> qA = vtkQuaternion<float>(qw, qx, qy, qz);
					//std::cout << "Angle:" << qw << "X:" << qx << "Y:" << qy << "Z:" << qz << std::endl;

					float qwxyz[4] = { connectXS.ax[0], connectXS.ax[1], connectXS.ax[2], connectXS.ax[3] };
					vtkQuaternion<float> q = vtkQuaternion<float>(qwxyz[3], -qwxyz[2], -qwxyz[1], -qwxyz[0]);
					//std::cout << "Angle:" << ax[3] << "X:" << -ax[2] << "Y:" << -ax[1] << "Z:" << -ax[0] << std::endl;
					q = qA.Inverse() * q;
					q.Normalize();

					float lua_axis[3];
					float lua_angle = ((q.GetRotationAngleAndAxis(lua_axis))* 180.0f / 3.14159f);

					glLoadIdentity();
					glTranslatef(0.0, 0.0, 0.0);
					glRotatef(lua_angle, lua_axis[0], lua_axis[2], -lua_axis[1]);
					glGetFloatv(GL_MODELVIEW_MATRIX, lua_node.m);
					std::cout << "Angle-RU:" << lua_angle << "X:" << lua_axis[0] << "Y:" << lua_axis[2] << "Z:" << lua_axis[1] << std::endl;
					
					//Right elbow
					glLoadIdentity();
					glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0);
					glGetFloatv(GL_MODELVIEW_MATRIX, lelb_node.m);
					
					//Right lower arm
					glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

					qw = sqrt(1 + lla_node.m[0] + lla_node.m[5] + lla_node.m[10]) / 2.0f;
					qx = (lla_node.m[6] - lla_node.m[9]) / (4 * qw);
					qy = (lla_node.m[8] - lla_node.m[2]) / (4 * qw);
					qz = (lla_node.m[1] - lla_node.m[4]) / (4 * qw);

					qA = vtkQuaternion<float>(qw, qx, qy, qz);

					q = vtkQuaternion<float>(connectXS.ax2[3], -connectXS.ax2[2], -connectXS.ax2[1], -connectXS.ax2[0]);
					q = qA.Inverse() * q;
					q.Normalize();

					float lla_axis[3] = {0.0,0.0,0.0};
					float lla_angle = (q.GetRotationAngleAndAxis(lla_axis))* 180.0 / 3.14159;

					glLoadIdentity();
					glTranslatef(0.0, 0.0, 0.0);
										
					lla_angle -= lua_angle;

					glRotatef(lla_angle , lla_axis[0], lla_axis[2], -lla_axis[1]);
					std::cout << "Angle-RL:" << lla_angle << "X:" << lla_axis[0] << "Y:" << lla_axis[2] << "Z:" << lla_axis[1] << std::endl;
					glGetFloatv(GL_MODELVIEW_MATRIX, lla_node.m);

					
					glPopMatrix();

					//............Left Arm.............//
					glPushMatrix();

					
					glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);
					qw = sqrt(1 + rua_node.m[0] + rua_node.m[5] + rua_node.m[10]) / 2.0f;
					qx = (rua_node.m[6] - rua_node.m[9]) / (4.0f * qw);
					qy = (rua_node.m[8] - rua_node.m[2]) / (4.0f * qw);
					qz = (rua_node.m[1] - rua_node.m[4]) / (4.0f * qw);
					qA = vtkQuaternion<float>(qw, qx, qy, qz);
					//std::cout << "Angle:" << qw << "X:" << qx << "Y:" << qy << "Z:" << qz << std::endl;

					q = vtkQuaternion<float>(connectXS.r_ax[3], connectXS.r_ax[2], connectXS.r_ax[1], connectXS.r_ax[0]);
					//std::cout << "Angle:" << ax[3] << "X:" << -ax[2] << "Y:" << -ax[1] << "Z:" << -ax[0] << std::endl;
					q = qA.Inverse() * q;
					q.Normalize();

					float rua_axis[3];
					float rua_angle = ((q.GetRotationAngleAndAxis(rua_axis))* 180.0f / 3.14159f);

					glLoadIdentity();
					glTranslatef(0.0, 0.0, 0.0);
					glRotatef(rua_angle, rua_axis[0], -rua_axis[2], rua_axis[1]);
					glGetFloatv(GL_MODELVIEW_MATRIX, rua_node.m);
					std::cout << "Angle-LU:" << rua_angle << "X:" << rua_axis[0] << "Y:" << rua_axis[2] << "Z:" << rua_axis[1] << std::endl;
					
					//Left elbow
					glLoadIdentity();
					glTranslatef(0.0, -UPPER_ARM_HEIGHT, 0.0);
					glGetFloatv(GL_MODELVIEW_MATRIX, relb_node.m);
					
					//Left lower arm
					glGetFloatv(GL_MODELVIEW_MATRIX, rla_node.m);

					qw = sqrt(1 + rla_node.m[0] + rla_node.m[5] + rla_node.m[10]) / 2.0f;
					qx = (rla_node.m[6] - rla_node.m[9]) / (4 * qw);
					qy = (rla_node.m[8] - rla_node.m[2]) / (4 * qw);
					qz = (rla_node.m[1] - rla_node.m[4]) / (4 * qw);

					qA = vtkQuaternion<float>(qw, qx, qy, qz);

					q = vtkQuaternion<float>(connectXS.r_ax2[3], connectXS.r_ax2[2], connectXS.r_ax2[1], connectXS.r_ax2[0]);
					q = qA.Inverse() * q;
					q.Normalize();

					float rla_axis[3] = { 0.0,0.0,0.0 };
					float rla_angle = (q.GetRotationAngleAndAxis(rla_axis))* 180.0 / 3.14159;

					glLoadIdentity();
					glTranslatef(0.0, 0.0, 0.0);
					
					rla_angle -= rua_angle;

					glRotatef(rla_angle , rla_axis[0], -rla_axis[2], rla_axis[1]);
					std::cout << "Angle-LL:" << rla_angle << "X:" << rla_axis[0] << "Y:" << rla_axis[2] << "Z:" << rla_axis[1] << std::endl;
					glGetFloatv(GL_MODELVIEW_MATRIX, rla_node.m);


					glPopMatrix();
				}
			}
		break;
	case 2:
		inverseKinematics();
		break;

	case 7:
		if (animation != option) inverseKinematics();
		else
		{
			if (theta[9] > 130.0)
			{
				theta[9] -= STEP;
				theta[10] += STEP;
				theta[7] -= STEP;
				theta[8] += STEP;
				theta[5] += STEP;
				theta[6] -= STEP;
				theta[3] -= STEP;
				theta[4] += STEP;
				horizontal += 0.03*STEP;
			}
			
						
		}
		break;

	case 6:

			{
				connectXS.bxMTdisconnect = true;
				if (connectXS.closeMtW_Succes || !connectXS.isRunning)	exit(0);
			}
		
		break;

	}
	glutPostRedisplay();
}

void menu(int id)
{
	option = id;
	done = 0;
	if (id == 3) rotate_ = -90;
	if (id == 4) rotate_ = 0;
	if (id == 5) rotate_ = -30;
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

void myinit()
{
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_ambient[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess = { 100.0 };
	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	//GLfloat light_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat light_diffuse[] = { 1, 0.87, 0.75, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 10.0, 10.0, 10.0, 0.0 };

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
	glClearColor(0, 0, 0, 0);


	glColor3f(1.0, 0.0, 0.0);



	/* allocate quadrics with filled drawing style */

	h = gluNewQuadric();
	gluQuadricDrawStyle(h, GLU_FILL);
	t = gluNewQuadric();
	gluQuadricDrawStyle(t, GLU_FILL);
	lua = gluNewQuadric();
	gluQuadricDrawStyle(lua, GLU_FILL);
	///////////////////////////////
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
	//C1
	glLoadIdentity();
	glRotatef(theta[0], 0.0, 1.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, torso_node.m);
	torso_node.f = torso;
	torso_node.sibling = NULL;
	torso_node.child = &nk_node;
	//C2
	glLoadIdentity();
	glTranslatef(0.0, TORSO_HEIGHT - 0.25*NECK_HEIGHT, 0.0);
	glRotatef(theta[1], 1.0, 0.0, 0.0);
	glGetFloatv(GL_MODELVIEW_MATRIX, nk_node.m);
	nk_node.f = neck;
	nk_node.sibling = &lsh_node;
	nk_node.child = &head_node;
	//C3
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

void keyBoardEvent(unsigned char key, int x, int y)
{
	printf("key_code =%d  \n", key);

	if (key == 115)
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

	if (key == 100)
	{
		connectXS.waitForConnections = false;
	}

	if (key == 99)
	{
		connectXS.bxMTdisconnect = true;
	}

	if (key == 114)
	{
		std::cout << "Disconnect and Close xSense..........!" << std::endl;
		connectXS.stop_and_restart_everything = true;
	}

	if (key == 120) { exit(0); }
}

int targc;
char** targv;

DWORD WINAPI RoboticArm(LPVOID lpParam)
{
	glutInit(&targc, targv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(600, 600);
	glutCreateWindow("Robot");

	myinit();
	glutReshapeFunc(myReshape);
	glutIdleFunc(idle);
	glutDisplayFunc(display);
	glutKeyboardFunc(keyBoardEvent);

	glutCreateMenu(menu);
	glutAddMenuEntry(" Start Xsens ", 0);
	glutAddMenuEntry(" Data Capture", 1);
	//glutAddMenuEntry(" Dance ", 7);
	//glutAddMenuEntry(" Gangnam Style Dance ", 6);
	glutAddMenuEntry(" Reset ", 2);
	glutAddMenuEntry(" Left View ", 3);
	glutAddMenuEntry(" Front View ", 4);
	glutAddMenuEntry(" 3/4 View ", 5);
	glutAddMenuEntry(" Close ", 6);

	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glutMainLoop();
	return 0;
}

DWORD WINAPI XSensDataReader(LPVOID lpParam) {

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
main(int argc, char** argv)
{
	targc = argc; targv = argv;
	// Data of Thread 1
	int Data_Of_Thread_1 = 1;
	// Data of Thread 2
	int Data_Of_Thread_2 = 2;
	// Data of Thread 3
	int Data_Of_Thread_3 = 3;
	// Data of Thread 4
	//int Data_Of_Thread_4 = 4;

	// variable to hold handle of Thread 1
	HANDLE Handle_Of_Thread_1 = 0;
	// variable to hold handle of Thread 2 
	HANDLE Handle_Of_Thread_2 = 0;


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



	// Store Thread handles in Array of Thread
	// Handles as per the requirement
	// of WaitForMultipleObjects() 
	Array_Of_Thread_Handles[0] = Handle_Of_Thread_1;
	Array_Of_Thread_Handles[1] = Handle_Of_Thread_2;


	// Wait until all threads have terminated.
	WaitForMultipleObjects(2, Array_Of_Thread_Handles, TRUE, INFINITE);

	// Delete critical Section
	DeleteCriticalSection(&m_cs);

	// Close all thread handles upon completion.
	CloseHandle(Handle_Of_Thread_1);
	CloseHandle(Handle_Of_Thread_2);

}
