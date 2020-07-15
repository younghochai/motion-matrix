#include "iaMotionSphere.h"
#include <iostream>
#include <iomanip>
#include "iaVitruvianAvatar.h"
#include "Model_PLY.h"

using namespace std;
MotionSphere ms;
SphereUtility expertSU;
Model_PLY rightHandPLY, leftHandPLY, rightFootPLY, leftFootPLY, kneePLY, elbowPLY;
struct PixelColor {
	GLfloat r;
	GLfloat g;
	GLfloat b;
	GLfloat a;
};

struct StencilHash
{
	int stencilIndex;
	int hashIndex;

};

float xrot = 0.0f;
float yrot = 0.0f;

float xdiff = 0.0f;
float ydiff = 0.0f;
float zval = 1.5f;
float zval1 = 1.5f;
float zval2 = 1.5f;
float zval3 = 1.5f;
float zval4 = 1.5f;
PixelColor pixelColor, pointColor;

float pointTranslateX = 0.0f;
float pointTranslateY = 0.0f;
float pointTranslateZ = zval;

float pointTranslateX1 = 0.0f;
float pointTranslateY1 = 0.0f;
float pointTranslateZ1 = zval1;

float pointTranslateX2 = 0.0f;
float pointTranslateY2 = 0.0f;
float pointTranslateZ2 = zval2;

float pointTranslateX3 = 0.0f;
float pointTranslateY3 = 0.0f;
float pointTranslateZ3 = zval3;

float pointTranslateX4 = 0.0f;
float pointTranslateY4 = 0.0f;
float pointTranslateZ4 = zval4;

float cameraX, cameraY;
int lastMouseX, lastMouseY;

bool mouseDown = false;

GLuint texture_id[2];
GLUquadricObj *sphere;
int targc;
char** targv;

int trajCount = 0;
int expTrajCount = 0;
int LindexP = 0;
int indexDB = 0;
bool bReadFile = false;
bool bReadDBFile = false;
bool toggleOption = true;
bool toggleEdit = false;
int isize = 0;
int dsize = 0;
int idbsize = 0;
bool startAnim = false;
quaternion qi = { 0,0,0,1 };
Avatar avatar = { qi,qi,qi,qi,qi,qi,qi,qi,qi,qi };
//SphereUtility su;

double xr = 0, yr = 0, zr = 0;
GLuint stencilIndex;
int printIndex[10] = { 0,0,0,0,0,0,0,0,0,0 };
int arrowIndex = 0;
int expertPrintIndex = 0;
int sphereID = 0;

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

float exptraj_b0[20014][4];
float exptraj_b1[20014][4];
float exptraj_b2[20014][4];
float exptraj_b3[20014][4];
float exptraj_b4[20014][4];
float exptraj_b5[20014][4];
float exptraj_b6[20014][4];
float exptraj_b7[20014][4];
float exptraj_b8[20014][4];
float exptraj_b9[20014][4];

float stencilHash[10][255];

quaternion BodyQuat(1.29947E-16, 0.707106781, -0.707106781, 1.41232E-32);

bool MotionSphere::keyPressed;

bool rotEnable1 = false;
bool rotEnable2 = false;
bool rotEnable3 = false;
bool rotEnable4 = false;

bool enableComparision = false;

void *font = GLUT_BITMAP_TIMES_ROMAN_24;
float textColor[4] = { 1, 0, 0, 1 };
float pos[3];
std::stringstream ss;

///////////////////////////////////////////////////////////////////////////////
// write 2d text using GLUT
// The projection matrix must be set to orthogonal before call this function.
///////////////////////////////////////////////////////////////////////////////
void drawString(const char *str, float x, float y, float color[4], void *font)
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

void sphereColoredCube(float sphere_radius/*, float twist*/)
{
	float r, g, b;
	//getColorByAngle(twist,r,g,b);
	glColor3f(r, g, b);
	glBegin(GL_QUADS);
	//Back Face
	glColor3f(0.753, 0.753, 0.753); //Sliver
	glVertex3f(-sphere_radius, sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, -sphere_radius, -sphere_radius);
	glVertex3f(-sphere_radius, -sphere_radius, -sphere_radius);

	//Right Face
	glColor3f(1, 0.64, 0); //yellow
	glVertex3f(sphere_radius, sphere_radius, sphere_radius);
	glVertex3f(sphere_radius, sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, -sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, -sphere_radius, sphere_radius);

	//Front Face
	glColor3f(0.863, 0.078, 0.235); //Salmon red
	glVertex3f(-sphere_radius, sphere_radius, sphere_radius);
	glVertex3f(sphere_radius, sphere_radius, sphere_radius);
	glVertex3f(sphere_radius, -sphere_radius, sphere_radius);
	glVertex3f(-sphere_radius, -sphere_radius, sphere_radius);

	//Left Face
	glColor3f(0.133, 0.545, 0.133); //ForestGreen
	glVertex3f(-sphere_radius, sphere_radius, -sphere_radius);
	glVertex3f(-sphere_radius, sphere_radius, sphere_radius);
	glVertex3f(-sphere_radius, -sphere_radius, sphere_radius);
	glVertex3f(-sphere_radius, -sphere_radius, -sphere_radius);

	//Top Face
	glColor3f(0.000, 0.000, 0.502); //Navy blue
	glVertex3f(-sphere_radius, sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, sphere_radius, sphere_radius);
	glVertex3f(-sphere_radius, sphere_radius, sphere_radius);

	//Bottom Face
	glColor3f(1, 1, 0); // orange
	glVertex3f(-sphere_radius, -sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, -sphere_radius, -sphere_radius);
	glVertex3f(sphere_radius, -sphere_radius, sphere_radius);
	glVertex3f(-sphere_radius, -sphere_radius, sphere_radius);
	glEnd();
}

void renderColoredCube(float x1, float y1, float z1, float x2, float y2, float z2, float radiusBase, float radiusTop, int subdivisions, GLUquadricObj *quadric, bool isCone)
{
	float vx = x2 - x1;
	float vy = y2 - y1;
	float vz = z2 - z1;
	float v = sqrt(vx*vx + vy * vy + vz * vz);
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

	float rx = -vy * vz;
	float ry = vx * vz;

	glPushMatrix();
	//draw the cylinder body
	//glTranslatef(x1, y1, z1);
	if (fabs(vz) < 1.0e-3) {
		glRotated(90.0, 0, 1, 0.0); // Rotate & align with x axis
		glRotated(ax, -1.0, 0.0, 0.0); // Rotate to point 2 in x-y plane
	}
	else {
		glRotated(ax, rx, ry, 0.0); // Rotate about rotation vector
	}
	gluQuadricOrientation(quadric, GLU_OUTSIDE);
	if (isCone)
		//glutSolidCone(radiusBase, radiusTop, subdivisions, subdivisions);
		//glutSolidCube(radiusBase);
		sphereColoredCube(radiusBase);
	else
		gluCylinder(quadric, radiusBase, radiusTop, v, subdivisions, 1);
	glPopMatrix();
}

void renderColoredCube_convenient(float x1, float y1, float z1, float x2, float y2, float z2, float radiusBase, float radiusTop, int subdivisions, bool isCone)
{
	//the same quadric can be re-used for drawing many cylinders
	GLUquadricObj *quadric = gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);
	renderColoredCube(x1, y1, z1, x2, y2, z2, radiusBase, radiusTop, subdivisions, quadric, isCone);
	gluDeleteQuadric(quadric);
}

void renderCylinder(float x1, float y1, float z1, float x2, float y2, float z2, float radiusBase, float radiusTop, int subdivisions, GLUquadricObj *quadric, bool isCone)
{
	float vx = x2 - x1;
	float vy = y2 - y1;
	float vz = z2 - z1;
	float v = sqrt(vx*vx + vy * vy + vz * vz);
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

	float rx = -vy * vz;
	float ry = vx * vz;

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
	if (isCone)
		glutSolidCone(radiusBase, radiusTop, subdivisions, subdivisions);
	//glutSolidCube(radiusBase);
	//sphereColoredCube(radiusBase);
	else
		gluCylinder(quadric, radiusBase, radiusTop, v, subdivisions, 1);
	glPopMatrix();
}

void renderCylinder_convenient(float x1, float y1, float z1, float x2, float y2, float z2, float radiusBase, float radiusTop, int subdivisions, bool isCone)
{
	//the same quadric can be re-used for drawing many cylinders
	GLUquadricObj *quadric = gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);
	renderCylinder(x1, y1, z1, x2, y2, z2, radiusBase, radiusTop, subdivisions, quadric, isCone);
	gluDeleteQuadric(quadric);
}

void InitializeLight()
{
	GLfloat mat_ambient_0[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat mat_diffuse_0[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	GLfloat mat_specular_0[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_position_0[] = { 0.0f, 0.0f, -4.0f, 0.0f };
	//GLfloat mat_position_0[] = { 0.5f, 0.5f, 0.8f, 0.0f};	
	GLfloat mat_shininess[] = { 128.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, mat_ambient_0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, mat_diffuse_0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, mat_specular_0);
	glLightfv(GL_LIGHT0, GL_POSITION, mat_position_0);

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient_0);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular_0);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse_0);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	GLfloat mat_ambient_1[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat mat_diffuse_1[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_specular_1[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_position_1[] = { -8.5f, 8.5f, 3.0f, 1.0f };
	GLfloat mat_spotdir_1[] = { 10.0f, 0.0f, 5.0f };

	glLightfv(GL_LIGHT1, GL_SPECULAR, mat_specular_1);
	glLightfv(GL_LIGHT1, GL_POSITION, mat_position_1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, mat_diffuse_1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, mat_ambient_1);

	glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.5f);
	glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.5f);
	glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.2f);

	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 120.0f);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, mat_spotdir_1);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 2.0f);

	GLfloat mat_ambient_2[] = { 0.05f, 0.05f, 0.05f, 1.0f };
	GLfloat mat_diffuse_2[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_specular_2[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	//GLfloat mat_position_2[] = { 0.9f, -0.5f, 0.6f, 1.0f};	
	//GLfloat mat_position_2[] = { -0.5f, 0.5f, 0.6f, 1.0f};	
	GLfloat mat_position_2[] = { 0.0f, 0.0f, 3.0f, 1.0f };

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

	::glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	//::glClearColor(0.0f, 0.0f, 0.0f, 0.0f);	
	::glShadeModel(GL_SMOOTH);
	//::glShadeModel(GL_FLAT);	

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	//glFrontFace(GL_CCW);

}

void sphereAxis(float length, bool coneFlag)
{
	GLUquadricObj *quadric = gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);

	glPushMatrix();
	glRotatef(90, 0, 1, 0);
	glColor3f(1, 0, 0); // x-axis red
	gluCylinder(quadric, 0.005, 0.005, length, 10, 10);
	glTranslatef(0, 0, 1.2);
	glRotatef(0, 0, 0, 1);
	if (coneFlag)
		glutSolidCone(0.05, 0.08, 10, 10);
	glPopMatrix();

	glPushMatrix();
	glRotatef(-90, 1, 0, 0);
	glColor3f(0, 1, 0); // y-axis green
	gluCylinder(quadric, 0.005, 0.005, length, 10, 10);
	glTranslatef(0, 0, 1.2);
	glRotatef(0, 1, 0, 0);
	if (coneFlag)
		glutSolidCone(0.05, 0.08, 10, 10);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0, 0, 1); // z-axis blue
	glRotatef(-90, 0, 0, 1);
	gluCylinder(quadric, 0.005, 0.005, length, 10, 10);
	glTranslatef(0, 0, 1.2);
	if (coneFlag)
		glutSolidCone(0.05, 0.08, 10, 10);
	glPopMatrix();
}

float getDistance(float x1, float y1, float z1, float x2, float y2, float z2)
{
	float dist = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
	return dist;
}

void getColorByAngle(double twist, double& r, double& g, double& b)
{
	double incr = 0.25;
	if (twist < 0)
	{
		r = (130 - abs(twist)*incr) / 255;
		g = 1.0;
		b = 0.0;
	}
	if (twist > 0)
	{
		r = 1.0;
		g = (130 - abs(twist)*incr) / 255;
		b = 0.0;
	}

	/*double incr = 0.0166666666666666666667;
	if (twist <= 60)
	{
		r = 1;
		g = abs(twist)*incr;
		b = 0;
	}
	else if (twist > 60 && twist <= 120)
	{
		r = 1 - abs(twist - 60)*incr;
		g = 1;
		b = 0;
	}
	else if (twist > 120 && twist <= 180)
	{
		r = 0;
		g = 1;
		b = abs(twist-120)*incr;
	}
	else if (twist > 180 && twist <= 240)
	{
		r = 0;
		g = 1- abs(twist - 180)*incr;
		b = 1;
	}
	else if (twist > 240 && twist <= 280)
	{
		r = abs(twist - 240)*incr;
		g = 0 ;
		b = 1;
	}
	else if (twist > 280 && twist <= 360)
	{
		r = 1;
		g = 0;
		b = 1- abs(twist - 280)*incr;
	}*/
}



void drawTriangles(float centerX, float centerY, float centerZ, float angle)
{
	// the radius of the bead
	float sr = 0.03;

	//find the twist angle in degrees
	angle = angle * 180 / PI;
	//glColor3f(0, 0, 0);
	//float theta = acos(centerZ);
	//float phi = atan(centerY / centerX);
	float length = sqrt(pow(centerX, 2) + pow(centerY, 2));
	/*TVec3 prevPoint = { sr*cos(phi), sr*sin(phi), 0 };*/
	TVec3 prevPoint = { sr*(centerX / length), sr*(centerY / length),0 };

	//Find the rotation matrix for the up vector
	TVec3 up = { 0,1,0 };
	TVec3 direction = { centerX,centerY,centerZ };
	TVec3 left = ms.su->vecCrossProduct(up, direction);
	ms.su->vecNormalize(left);
	up = ms.su->vecCrossProduct(left, direction);
	ms.su->vecNormalize(up);
	float matrix[] = { left._x, left._y, left._z, 0.0f,     //LEFT
		up._x, up._y, up._z, 0.0f,                       //UP 
		direction._x, direction._y, direction._z, 0.0f,  //FORWARD
		1.011*centerX, 1.011*centerY, 1.011*centerZ, 1.0f };    //TRANSLATION TO WHERE THE OBJECT SHOULD BE PLACED



	if (angle > 0)
	{

		for (int i = 90; i < (90 + (int)angle); i = i + 5)
		{
			//TVec3 point2 = { sr*cos(i*PI / 180),sr*sin(i*PI / 180), 0 };
			TVec3 point2 = { sr*cos(i*PI / 180),0,sr*sin(i*PI / 180) };
			if (i == 90) glColor3f(0, 0, 1);
			else glColor3f(0, 0, 0);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			glTranslatef(1.011*centerX, 1.011*centerY, 1.011*centerZ);
			glBegin(GL_LINES);
			for (float t = 0; t < 1; t = t + 0.1)
			{
				TVec3 newPoint1 = ms.su->vecSLERP(point2, prevPoint, t);
				TVec3 newPoint2 = ms.su->vecSLERP(point2, prevPoint, t + 0.1);
				glVertex3f(newPoint1._x, newPoint1._y, newPoint1._z);
				glVertex3f(newPoint2._x, newPoint2._y, newPoint2._z);

			}
			glEnd();
			//glutSwapBuffers();
			glPopMatrix();
			//glLoadIdentity();
		}
	}
	else if (angle < 0)
	{
		for (int i = 90; i > (90 + (int)angle); i = i - 5)
		{
			//TVec3 point2 = { sr*cos(i*PI / 180),sr*sin(i*PI / 180), 0 };
			TVec3 point2 = { sr*cos(i*PI / 180),0,sr*sin(i*PI / 180) };
			if (i == 90) glColor3f(0, 0, 1);
			else glColor3f(0, 0, 0);
			glPushMatrix();
			glTranslatef(1.011*centerX, 1.011*centerY, 1.011*centerZ);
			glBegin(GL_LINES);
			for (float t = 0; t <= 1; t = t + 0.1)
			{
				TVec3 newPoint1 = ms.su->vecSLERP(point2, prevPoint, t);
				TVec3 newPoint2 = ms.su->vecSLERP(point2, prevPoint, t + 0.1);
				glVertex3f(newPoint1._x, newPoint1._y, newPoint1._z);
				glVertex3f(newPoint2._x, newPoint2._y, newPoint2._z);

			}
			glEnd();
			glPopMatrix();
		}
	}
	glutPostRedisplay();
}
Avatar getFirstInverse(int index)
{
	Avatar result;
	result.b0 = ms.su->avatarData[index].b0.mutiplication(ms.su->avatarData[1].b0.Inverse());
	result.b1 = ms.su->avatarData[index].b1.mutiplication(ms.su->avatarData[1].b1.Inverse());
	result.b2 = ms.su->avatarData[index].b2.mutiplication(ms.su->avatarData[1].b2.Inverse());
	result.b3 = ms.su->avatarData[index].b3.mutiplication(ms.su->avatarData[1].b3.Inverse());
	result.b4 = ms.su->avatarData[index].b4.mutiplication(ms.su->avatarData[1].b4.Inverse());
	result.b5 = ms.su->avatarData[index].b5.mutiplication(ms.su->avatarData[1].b5.Inverse());
	result.b6 = ms.su->avatarData[index].b6.mutiplication(ms.su->avatarData[1].b6.Inverse());
	result.b7 = ms.su->avatarData[index].b7.mutiplication(ms.su->avatarData[1].b7.Inverse());
	result.b8 = ms.su->avatarData[index].b8.mutiplication(ms.su->avatarData[1].b8.Inverse());
	result.b9 = ms.su->avatarData[index].b9.mutiplication(ms.su->avatarData[1].b9.Inverse());

	return result;
}

quaternion getQuatByIndex(int boneID, int index, SphereUtility *su)
{
	Avatar firstInverse = getFirstInverse(index);
	switch (boneID)
	{
		case 0: return firstInverse.b0; /*su->avatarData[index].b0;*/ break;
		case 1: return firstInverse.b0.Inverse().mutiplication(firstInverse.b1);  break;
		case 2: return firstInverse.b1.Inverse().mutiplication(firstInverse.b2);  break;
		case 3: return firstInverse.b2.Inverse().mutiplication(firstInverse.b3);  break;
		case 4: return firstInverse.b1.Inverse().mutiplication(firstInverse.b4);  break;
		case 5: return firstInverse.b4.Inverse().mutiplication(firstInverse.b5);  break;
		case 6: return firstInverse.b0.Inverse().mutiplication(firstInverse.b6);  break;
		case 7: return firstInverse.b6.Inverse().mutiplication(firstInverse.b7);  break;
		case 8: return firstInverse.b0.Inverse().mutiplication(firstInverse.b8);  break;
		case 9: return firstInverse.b8.Inverse().mutiplication(firstInverse.b9);  break;
	}
}

void drawPLYByBoneID(int boneID)
{
	switch (boneID)
	{
	case 2: elbowPLY.Draw(); break;
	case 3: /*glScalef(0.007, 0.0071, 0.0071);*/ rightHandPLY.Draw(); break;

	case 4: elbowPLY.Draw(); break;
	case 5: leftHandPLY.Draw(); break;

	case 6: kneePLY.Draw(); break;
	case 7: rightFootPLY.Draw(); break;

	case 8:	kneePLY.Draw(); break;
	case 9: leftFootPLY.Draw(); break;

	default: break;
	}
}



void sphereDraw(int index, float(&traj_b)[20014][4], float r, float g, float b, int sIndex, int boneID, bool expert)
{
	SphereUtility *currentSU;
	double mr, mg, mb;
	

	if (stencilIndex > 0)
		VitruvianAvatar::vitruvianAvatarUpdate = getFirstInverse(stencilIndex - 1);
	else
		VitruvianAvatar::vitruvianAvatarUpdate = getFirstInverse(trajCount-1);

	if (expert)
	{
		currentSU = &expertSU;
	}

	else
	{
		currentSU = ms.su;
	}

	quaternion q = getQuatByIndex(boneID, index, currentSU);
	float angle = acos(q.mData[3]);
	float axisX = q.mData[0] / sin(angle); float axisY = q.mData[1] / sin(angle); float axisZ = q.mData[2] / sin(angle);
	angle = 2 * angle * 180 / PI;
	float sphere_radius = 1;
	//glDisable(GL_LIGHTING);
	glColor3f(r, g, b);
	float fnorm = sqrt(traj_b[index][1] * traj_b[index][1] + traj_b[index][2] * traj_b[index][2] + traj_b[index][3] * traj_b[index][3]);

	//End of Drawing Line
	float theta, phi, psi;
	float ri;
	//int step = 1;
	TVec3 s;
	float dist;
	float twist;
	if (expert)
	{
		dist = getDistance(traj_b[index][1], traj_b[index][2], traj_b[index][3], traj_b[expertPrintIndex][1], traj_b[expertPrintIndex][2], traj_b[expertPrintIndex][3]);
		twist = currentSU->twistAngles[index][boneID];
	}
	else
	{
		dist = getDistance(traj_b[index][1], traj_b[index][2], traj_b[index][3], traj_b[printIndex[boneID]][1], traj_b[printIndex[boneID]][2], traj_b[printIndex[boneID]][3]);
		twist = currentSU->twistAngles[index][boneID];
	}

	if (index > 0 && toggleOption)
	{
		glLineWidth(3.0f);
		glPushMatrix();
		renderCylinder_convenient(-1.01*traj_b[index - 1][1] / fnorm, -1.01*traj_b[index - 1][2] / fnorm, 1.01*traj_b[index - 1][3] / fnorm,
			-1.01*traj_b[index][1] / fnorm, -1.01*traj_b[index][2] / fnorm, 1.01*traj_b[index][3] / fnorm, 0.01, 0.01, 30, false);
		glPopMatrix();

		
	}

	glPushMatrix();
	theta = asin(traj_b[index][3] / fnorm) * 180 / PI;
	phi = atan2(traj_b[index][1] / fnorm, -traj_b[index][2] / fnorm) * 180 / PI;
	ri = 0.03;
	if (stencilIndex == sIndex && stencilIndex != 0)
	{
		ss.str("");
		ss << setprecision(2) << (twist) * 180 / PI;
		pos[0] = 1.3*cos(theta*PI / 180)*sin(-phi * PI / 180); pos[1] = 1.3*cos(theta*PI / 180)*cos(-phi * PI / 180); pos[2] = 1.3*sin(theta*PI / 180);
		drawString3D(ss.str().c_str(), pos, textColor, font);
		ri = 0.05;
	}
	if (index == 0 || index % 10 == 0 /*|| dist > 0.3*/ || index == currentSU->noOfFrames - 1 || index == trajCount - 1)
	{
		if (index == 0)
		{
			printIndex[boneID] = 0;
			arrowIndex = 0;
		}

		//Find the Vector between the immidiate previous point to identify the axis of rotation.
		TVec3 orderedPair;
		if (expert)
		{
			orderedPair = { traj_b[index][1] - traj_b[expertPrintIndex][1], -traj_b[index][2] + traj_b[expertPrintIndex][2] ,traj_b[index][3] - traj_b[expertPrintIndex][3] };
		}
		else
			orderedPair = { traj_b[index][1] - traj_b[printIndex[boneID]][1], -traj_b[index][2] + traj_b[printIndex[boneID]][2] ,traj_b[index][3] - traj_b[printIndex[boneID]][3] };

		TVec3 orderedPairZ = { 0 - traj_b[index][1], 0 + traj_b[index][2] ,1 - traj_b[index][3] };

		currentSU->vecNormalize(orderedPair);
		currentSU->vecNormalize(orderedPairZ);

		glDisable(GL_CULL_FACE);

		if (index >= 0 && toggleOption && boneID > 1)
		{

			glPushMatrix();
			glTranslatef(-1.07*traj_b[index][1] / fnorm, -1.07*traj_b[index][2] / fnorm, 1.07*traj_b[index][3] / fnorm);
			glRotatef(-angle, axisX, axisY, -axisZ);
			getColorByAngle(twist * 180 / PI, mr, mg, mb);
			glColor3f(mr, mg, mb);
			glStencilFunc(GL_ALWAYS, sIndex, -1);
			drawPLYByBoneID(boneID);
			glPopMatrix();
		}
		if (expert)
		{
			arrowIndex = expertPrintIndex + (index - expertPrintIndex) / 2;
			expertPrintIndex = index;
			glColor3f(1.000, 0.000, 1.000);
		}

		else
		{
			arrowIndex = printIndex[boneID] + (index - printIndex[boneID]) / 2;
			printIndex[boneID] = index;
			glColor3f(0.000, 0.000, 0.000);
		}

		if (arrowIndex > 0 && toggleOption)
			renderCylinder_convenient(-1.01*traj_b[arrowIndex - 1][1] / fnorm, -1.01*traj_b[arrowIndex - 1][2] / fnorm, 1.01*traj_b[arrowIndex - 1][3] / fnorm,
				-1.01*traj_b[arrowIndex][1] / fnorm, -1.01*traj_b[arrowIndex][2] / fnorm, 1.01*traj_b[arrowIndex][3] / fnorm, 0.03, 0.05, 30, true);
	}
	//Draw A line between Points
	if (index >= 0 && !toggleOption || boneID == 0 || boneID == 1)
	{
		glDisable(GL_CULL_FACE);
		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslatef(-1.02*traj_b[index][1] / fnorm, -1.02*traj_b[index][2] / fnorm, 1.02*traj_b[index][3] / fnorm);
		glStencilFunc(GL_ALWAYS, sIndex, -1);
		getColorByAngle(twist * 180 / PI, mr, mg, mb);
		glColor3f(mr, mg, mb);
		glutSolidSphere(ri, 30, 30);
		glPopMatrix();
	}
	glStencilFunc(GL_ALWAYS, -1, -1);
	glPopMatrix();
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
}

/* Draws the trajectory on the sphere
*/
void drawTrajectory(GLint minWidth, GLint minHeight, GLsizei maxWidth, GLsizei maxHeight, bool rotationFlag, float &camX, float &camY, float &camZ, float &zv, int selectSphere)
{
	InitializeLight();
	// ------ Draw Boundry for Show Information ------------- // 
	glEnable(GL_SCISSOR_TEST);
	glViewport(minWidth, minHeight, maxWidth, maxHeight);
	glScissor(minWidth, minHeight, maxWidth, maxHeight);
	//------ Draw Boundry for Show Information ------------- // 
	glEnable(GL_SCISSOR_TEST);
	glClearDepth(1.0);
	glClearColor(0.690, 0.769, 0.871, 0.0);
	//glClearColor(1, 1, 1, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-zv, zv, -zv, zv, -5, 10);

	glMatrixMode(GL_MODELVIEW);
	//glDisable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glClearColor(1.000, 0.753, 0.796, 0.5);
	glPushMatrix();
	glLoadIdentity();
	//if (rotationFlag)
	{
		gluLookAt(
			camX, camY, camZ,
			0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f
		);
	}

	//glDisable(GL_LIGHT1);
	glLineWidth(2.0);
	glColor3f(1.0, 1.0, 1.0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id[0]);
	//glPushMatrix();
	glRotatef(-180, 0, 1, 0);
	glRotatef(-90, 1, 0, 0);
	//glTranslatef(0, -5, 0);
	glCullFace(GL_FRONT);
	gluSphere(sphere, 1.0, 50, 50);

	glDisable(GL_TEXTURE_2D);
	//glRotatef(-90, 1, 0, 0);
	//glEnable(GL_LIGHT1);
	//sphereAxis(1.2, true);
	glDisable(GL_BLEND);

	int j = 0, i = 0;
	while (i < trajCount || j < expTrajCount - 1)
	{
		if (i >= ms.su->noOfFrames - 1)
		{
			i = ms.su->noOfFrames - 1;
		}

		if (j >= expertSU.noOfFrames - 1)
		{
			j = expertSU.noOfFrames - 1;
		}

		glStencilFunc(GL_ALWAYS, trajCount, -1);
		switch (sphereID)
		{
		case 0:

			sphereDraw(i, traj_b0, 0.000, 0.980, 0.604, i + 1, 0, false); //black
			if (enableComparision) {
				sphereDraw(j, exptraj_b0, 0.0, 0.0, 0.0, j + 1, 0, true);
			}
			break;

		case 1:		sphereDraw(i, traj_b1, 1.0, 0.0, 0.0, i + 1, 1, false); //red
			if (enableComparision)
				sphereDraw(j, exptraj_b1, 0.0, 0.0, 0.0, j + 1, 1, true);
			break;

		case 2:		//sphereDraw(i, traj_b2, 1.0, 0.5, 0.0, i + 1, 2,false); //orange
			sphereDraw(i, traj_b2, 1.0, 0.0, 0.0, i + 1, 2, false);
			//cout <<"("<<i<<","<<j<<")"<< ":  "<<traj_b2[i][1] << "," << traj_b2[i][2] << "," << traj_b2[i][3] << "->";
			//cout << exptraj_b2[j][1] << "," << exptraj_b2[j][2] << "," << exptraj_b2[j][3] << endl;
			if (enableComparision)
				sphereDraw(j, exptraj_b2, 0.0, 0.0, 0.0, j + 1, 2, true);
			break;

		case 3:		sphereDraw(i, traj_b3, 1.0, 0.0, 0.0/*1.0, 1.0, 0.0*/, i + 1, 3, false); //Yellow
			if (enableComparision)
				sphereDraw(j, exptraj_b3, 0.0, 0.0, 0.0, j + 1, 3, true);
			break;

		case 4:		sphereDraw(i, traj_b4, 1.0, 0.0, 0.0/*0.0, 1.0, 0.0*/, i + 1, 4, false); //Bright green
			if (enableComparision)
				sphereDraw(j, exptraj_b4, 0.0, 0.0, 0.0, j + 1, 4, true);
			break;

		case 5:		sphereDraw(i, traj_b5, 1.0, 0.0, 0.0/*0.0, 1.0, 1.0*/, i + 1, 5, false); //Cyan
			if (enableComparision)
				sphereDraw(j, exptraj_b5, 0.0, 0.0, 0.0, j + 1, 5, true);
			break;

		case 6:		sphereDraw(i, traj_b6, 1.0, 0.0, 0.0/*0.0, 0.0, 1.0*/, i + 1, 6, false); //Blue
			if (enableComparision)
				sphereDraw(j, exptraj_b6, 0.0, 0.0, 0.0, j + 1, 6, true);
			break;

		case 7:		sphereDraw(i, traj_b7, 1.0, 0.0, 0.0/*0.5, 0.0, 1.0*/, i + 1, 7, false); //Voilet
			if (enableComparision)
				sphereDraw(j, exptraj_b7, 0.0, 0.0, 0.0, j + 1, 7, true);
			break;

		case 8:		sphereDraw(i, traj_b8, 1.0, 0.0, 0.0/*1.0, 0.0, 1.0*/, i + 1, 8, false); //Magenta
			if (enableComparision)
				sphereDraw(j, exptraj_b8, 0.0, 0.0, 0.0, j + 1, 8, true);
			break;

		case 9:		sphereDraw(i, traj_b9, 1.0, 0.0, 0.0/*0.4, 0.5, 0.8*/, i + 1, 9, false); //Indigo
			if (enableComparision)
				sphereDraw(j, exptraj_b9, 0.0, 0.0, 0.0, j + 1, 9, true);
			break;

		case 10:	sphereDraw(i, traj_b2, 1.0, 0.5, 0.0, i + 1, 2, false);
			sphereDraw(i, traj_b3, 1.0, 1.0, 0.0, i + 1, 3, false);
			if (enableComparision)
			{
				sphereDraw(j, exptraj_b2, 0.412, 0.412, 0.412, j + 1, 2, true);
				sphereDraw(j, exptraj_b3, 0.439, 0.502, 0.565, j + 1, 3, true);
			}
			break;

		case 11:	sphereDraw(i, traj_b4, 0.0, 1.0, 0.0, i + 1, 4, false);
			sphereDraw(i, traj_b5, 0.0, 1.0, 1.0, i + 1, 5, false);
			if (enableComparision)
			{
				sphereDraw(j, exptraj_b4, 0.412, 0.412, 0.412, j + 1, 4, true);
				sphereDraw(j, exptraj_b5, 0.439, 0.502, 0.565, j + 1, 5, true);
			}
			break;

		case 12:	sphereDraw(i, traj_b6, 0.0, 0.0, 1.0, i + 1, 6, false);
			sphereDraw(i, traj_b7, 0.5, 0.0, 1.0, i + 1, 7, false);
			if (enableComparision)
			{
				sphereDraw(j, exptraj_b6, 0.412, 0.412, 0.412, j + 1, 6, true);
				sphereDraw(j, exptraj_b7, 0.439, 0.502, 0.565, j + 1, 7, true);
			}
			break;

		case 13:	sphereDraw(i, traj_b8, 1.0, 0.0, 1.0, i + 1, 8, false);
			sphereDraw(i, traj_b9, 0.4, 0.5, 0.8, i + 1, 9, false);
			if (enableComparision)
			{
				sphereDraw(j, exptraj_b8, 0.412, 0.412, 0.412, j + 1, 8, true);
				sphereDraw(j, exptraj_b9, 0.439, 0.502, 0.565, j + 1, 9, true);
			}
			break;

		case 14:	if (selectSphere == 1)
		{
			sphereDraw(i, traj_b2, 1.0, 0.5, 0.0, i + 1, 2, false);
			sphereDraw(i, traj_b3, 1.0, 1.0, 0.0, i + 1, 3, false);
			if (enableComparision)
			{
				sphereDraw(j, exptraj_b2, 0.412, 0.412, 0.412, j + 1, 2, true);
				sphereDraw(j, exptraj_b3, 0.439, 0.502, 0.565, j + 1, 3, true);
			}
		}
					if (selectSphere == 2)
					{
						sphereDraw(i, traj_b4, 0.0, 1.0, 0.0, i + 1, 4, false);
						sphereDraw(i, traj_b5, 0.0, 1.0, 1.0, i + 1, 5, false);
						if (enableComparision)
						{
							sphereDraw(j, exptraj_b4, 0.412, 0.412, 0.412, j + 1, 4, true);
							sphereDraw(j, exptraj_b5, 0.439, 0.502, 0.565, j + 1, 5, true);
						}
					}
					break;

		case 15:	if (selectSphere == 1)
		{
			sphereDraw(i, traj_b6, 1.0, 0.5, 0.0, i + 1, 6, false);
			sphereDraw(i, traj_b7, 1.0, 1.0, 0.0, i + 1, 7, false);
			if (enableComparision)
			{
				sphereDraw(j, exptraj_b6, 0.412, 0.412, 0.412, j + 1, 6, true);
				sphereDraw(j, exptraj_b7, 0.439, 0.502, 0.565, j + 1, 7, true);
			}
		}
					if (selectSphere == 2)
					{
						sphereDraw(i, traj_b8, 0.0, 1.0, 0.0, i + 1, 8, false);
						sphereDraw(i, traj_b9, 0.0, 1.0, 1.0, i + 1, 9, false);
						if (enableComparision)
						{
							sphereDraw(j, exptraj_b8, 0.412, 0.412, 0.412, j + 1, 8, true);
							sphereDraw(j, exptraj_b9, 0.439, 0.502, 0.565, j + 1, 9, true);
						}
					}
					break;

		case 16:	if (selectSphere == 1)
		{
			sphereDraw(i, traj_b2, 1.0, 0.5, 0.0, i + 1, 2, false);
			sphereDraw(i, traj_b3, 1.0, 1.0, 0.0, i + 1, 3, false);
			if (enableComparision)
			{
				sphereDraw(j, exptraj_b2, 0.412, 0.412, 0.412, j + 1, 2, true);
				sphereDraw(j, exptraj_b3, 0.439, 0.502, 0.565, j + 1, 3, true);
			}
		}
					if (selectSphere == 2)
					{
						sphereDraw(i, traj_b4, 0.0, 1.0, 0.0, i + 1, 4, false);
						sphereDraw(i, traj_b5, 0.0, 1.0, 1.0, i + 1, 5, false);
						if (enableComparision)
						{
							sphereDraw(j, exptraj_b4, 0.412, 0.412, 0.412, j + 1, 4, true);
							sphereDraw(j, exptraj_b5, 0.439, 0.502, 0.565, j + 1, 5, true);
						}
					}
					if (selectSphere == 3)
					{
						sphereDraw(i, traj_b6, 1.0, 0.5, 0.0, i + 1, 6, false);
						sphereDraw(i, traj_b7, 1.0, 1.0, 0.0, i + 1, 7, false);
						if (enableComparision)
						{
							sphereDraw(j, exptraj_b6, 0.412, 0.412, 0.412, j + 1, 6, true);
							sphereDraw(j, exptraj_b7, 0.439, 0.502, 0.565, j + 1, 7, true);
						}
					}
					if (selectSphere == 4)
					{
						sphereDraw(i, traj_b8, 0.0, 1.0, 0.0, i + 1, 8, false);
						sphereDraw(i, traj_b9, 0.0, 1.0, 1.0, i + 1, 9, false);
						if (enableComparision)
						{
							sphereDraw(j, exptraj_b8, 0.412, 0.412, 0.412, j + 1, 8, true);
							sphereDraw(j, exptraj_b9, 0.439, 0.502, 0.565, j + 1, 9, true);
						}
					}
					break;

		}
		i++;
		j++;
	}
	if (enableComparision && expTrajCount < expertSU.noOfFrames)
		expTrajCount++;
	glPopMatrix();
}

/* Callback: runs in an infinte glut loop
*/
void sphereDisplay(void)
{
	InitializeLight();
	glClearDepth(1.0);

	glClearStencil(0);
	glClearColor(0.9, 0.9, 0.9, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_STENCIL_TEST);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	glStencilFunc(GL_ALWAYS, -1, -1);

	glDisable(GL_TEXTURE_2D);

	glEnable(GL_TEXTURE_2D);
	if (sphereID <= 13)
	{
		drawTrajectory(ms.minWidth, ms.minHeight, ms.maxWidth, ms.maxHeight, true, pointTranslateX, pointTranslateY, pointTranslateZ, zval, 0);
	}
	if (sphereID > 13 && sphereID <= 15)
	{
		drawTrajectory(ms.minWidth, ms.maxHeight / 4, ms.maxWidth / 2, ms.maxHeight / 2, rotEnable1, pointTranslateX1, pointTranslateY1, pointTranslateZ1, zval1, 1);
		drawTrajectory(ms.maxWidth / 2, ms.maxHeight / 4, ms.maxWidth / 2, ms.maxHeight / 2, rotEnable2, pointTranslateX2, pointTranslateY2, pointTranslateZ2, zval2, 2);
	}

	if (sphereID == 16)
	{
		drawTrajectory(ms.minWidth, ms.maxHeight / 2, ms.maxWidth / 2, ms.maxHeight / 2, rotEnable1, pointTranslateX1, pointTranslateY1, pointTranslateZ1, zval1, 1);
		drawTrajectory(ms.maxWidth / 2, ms.maxHeight / 2, ms.maxWidth / 2, ms.maxHeight / 2, rotEnable2, pointTranslateX2, pointTranslateY2, pointTranslateZ2, zval2, 2);
		drawTrajectory(ms.minWidth, ms.minHeight, ms.maxWidth / 2, ms.maxHeight / 2, rotEnable3, pointTranslateX3, pointTranslateY3, pointTranslateZ3, zval3, 3);
		drawTrajectory(ms.maxWidth / 2, ms.minHeight, ms.maxWidth / 2, ms.maxHeight / 2, rotEnable4, pointTranslateX4, pointTranslateY4, pointTranslateZ4, zval4, 4);
	}
	glFlush();
	glutSwapBuffers();
}
/* Callback: The mouse click event is defined based on the number of view ports enabled
*/
void sphereMouseEvent(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		mouseDown = true;
	}
	else
		mouseDown = false;

	if (mouseDown && toggleEdit)
	{
		glReadPixels(x, ms.maxHeight - y - 1, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_INT, &stencilIndex);
		printf("The stensil-index = %d\n", stencilIndex);
		xdiff = (yrot + x);
		ydiff = -y + xrot;
	}

	if (mouseDown && !toggleEdit)
	{
		glReadPixels(x, ms.maxHeight - y - 1, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_INT, &stencilIndex);
		glReadPixels(x, ms.maxHeight - y - 1, 1, 1, GL_RGBA, GL_FLOAT, &pointColor);

		//mouseDown = true;

		xdiff = (yrot + x);
		ydiff = -y + xrot;
		//printf("xrot = %f\tyrot = %f\n", xdiff, ydiff);
		if (sphereID <= 13)
		{
			rotEnable1 = rotEnable2 = rotEnable3 = rotEnable4 = false;
		}
		if (sphereID > 13 && sphereID <= 15)
		{
			if (x < ms.maxWidth / 2)
			{
				rotEnable1 = true;
				rotEnable2 = rotEnable3 = rotEnable4 = false;
			}
			if (x > ms.maxWidth / 2)
			{
				rotEnable2 = true;
				rotEnable1 = rotEnable3 = rotEnable4 = false;
			}
		}
		if (sphereID == 16)
		{
			if (y < ms.maxHeight / 2)
			{
				if (x < ms.maxWidth / 2)
				{
					rotEnable1 = true;
					rotEnable3 = rotEnable2 = rotEnable4 = false;
				}
				if (x > ms.maxWidth / 2)
				{
					rotEnable2 = true;
					rotEnable4 = rotEnable3 = rotEnable1 = false;
				}
			}
			if (y > ms.maxHeight / 2)
			{
				if (x < ms.maxWidth / 2)
				{
					rotEnable3 = true;
					rotEnable2 = rotEnable1 = rotEnable4 = false;
				}
				if (x > ms.maxWidth / 2)
				{
					rotEnable4 = true;
					rotEnable1 = rotEnable3 = rotEnable2 = false;
				}
			}
		}
	}
}

/* Callback: Mouse movement on the sphere on left mouse down enables sphere rotation
*/
void sphereMouseMotion(int x, int y)
{
	if (mouseDown && toggleEdit && stencilIndex > 1)
	{
		yrot = -(x + xdiff);
		xrot = (y + ydiff);
		if (xrot > 89) xrot = 89.0f;
		if (xrot < -89) xrot = -89.0f;

		printf("(xrot, ytor)=(%f, %f)\n", xrot, yrot);
		double X = 1 * (cos(xrot*PI / 180) * sin(yrot*PI / 180));
		double Y = 1 * (sin(xrot*PI / 180));
		double Z = 1 * (cos(xrot*PI / 180) * cos(yrot*PI / 180));

		switch (sphereID)
		{
		case 0:	traj_b0[stencilIndex - 1][1] = X;
			traj_b0[stencilIndex - 1][2] = Y;
			traj_b0[stencilIndex - 1][3] = Z; break;
		case 1:	traj_b1[stencilIndex - 1][1] = X;
			traj_b1[stencilIndex - 1][2] = Y;
			traj_b1[stencilIndex - 1][3] = Z; break;
		case 2:	traj_b2[stencilIndex - 1][1] = X;
			traj_b2[stencilIndex - 1][2] = Y;
			traj_b2[stencilIndex - 1][3] = Z; break;
		case 3: traj_b3[stencilIndex - 1][1] = X;
			traj_b3[stencilIndex - 1][2] = Y;
			traj_b3[stencilIndex - 1][3] = Z; break;
		case 4: traj_b4[stencilIndex - 1][1] = X;
			traj_b4[stencilIndex - 1][2] = Y;
			traj_b4[stencilIndex - 1][3] = Z; break;
		case 5:	traj_b5[stencilIndex - 1][1] = X;
			traj_b5[stencilIndex - 1][2] = Y;
			traj_b5[stencilIndex - 1][3] = Z; break;
		case 6: traj_b6[stencilIndex - 1][1] = X;
			traj_b6[stencilIndex - 1][2] = Y;
			traj_b6[stencilIndex - 1][3] = Z; break;
		case 7: traj_b7[stencilIndex - 1][1] = X;
			traj_b7[stencilIndex - 1][2] = Y;
			traj_b7[stencilIndex - 1][3] = Z; break;
		case 8: traj_b8[stencilIndex - 1][1] = X;
			traj_b8[stencilIndex - 1][2] = Y;
			traj_b8[stencilIndex - 1][3] = Z; break;
		case 9: traj_b9[stencilIndex - 1][1] = X;
			traj_b9[stencilIndex - 1][2] = Y;
			traj_b9[stencilIndex - 1][3] = Z; break;
		default:
			break;
		}



	}
	if (mouseDown && !toggleEdit)
	{
		yrot = -(x + xdiff);
		xrot = y + ydiff;
		if (xrot > 89) xrot = 89.0f;
		if (xrot < -89) xrot = -89.0f;

		pointTranslateX = zval * (cos(xrot*PI / 180) * sin(yrot*PI / 180));
		pointTranslateY = zval * (sin(xrot*PI / 180));
		pointTranslateZ = zval * (cos(xrot*PI / 180) * cos(yrot*PI / 180));
		//printf("Changing values on the sphere (x, y, z) = (%f, %f, %f)\n", pointTranslateX, pointTranslateY, pointTranslateZ);
		if (rotEnable1)
		{
			pointTranslateX1 = pointTranslateX;
			pointTranslateY1 = pointTranslateY;
			pointTranslateZ1 = pointTranslateZ;

		}
		if (rotEnable2)
		{
			pointTranslateX2 = pointTranslateX;
			pointTranslateY2 = pointTranslateY;
			pointTranslateZ2 = pointTranslateZ;

		}
		if (rotEnable3)
		{
			pointTranslateX3 = pointTranslateX;
			pointTranslateY3 = pointTranslateY;
			pointTranslateZ3 = pointTranslateZ;

		}
		if (rotEnable4)
		{
			pointTranslateX4 = pointTranslateX;
			pointTranslateY4 = pointTranslateY;
			pointTranslateZ4 = pointTranslateZ;

		}
		glutPostRedisplay();
	}
}
/* Callback: Reshape function for the openGL window
*/
void sphereReshape(int w, int h)
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

/* Calculate the 3D normalized points (swing) and the twist angle
*/
void calculateTrajectory(quaternion parent, quaternion child, TVec3 &parentVec, TVec3 &childVec, float &tAngleParent, float &tAngleChild)
{
	//TRANFORMATION OF SENSROR FRAME TO BODY FRAME
	//quaternion tempQuat1 = BodyQuat.mutiplication(parent);
	//quaternion tempQuat2 = BodyQuat.mutiplication(child);//Case-2 usf_q

	quaternion tempQuat1 = parent;
	quaternion tempQuat2 = child;//Case-2 usf_q


	quaternion lq = tempQuat2;
	quaternion uq = tempQuat1;
	quaternion vQuat(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

	quaternion lw = lq.mutiplication(vQuat.mutiplication(lq.Inverse()));//QVQ-1
	quaternion uw = uq.mutiplication(vQuat.mutiplication(uq.Inverse()));//QVQ-1


	parentVec = { (float)uw.mData[0], (float)uw.mData[1], (float)uw.mData[2] };
	childVec = { (float)lw.mData[0], (float)lw.mData[1], (float)lw.mData[2] };

	tAngleParent = ms.su->getTwistAngle(parentVec, uq);
	tAngleChild = ms.su->getTwistAngle(childVec, lq);

}
/* Copying the expert trajectory into 2d array
*/
void loadexpTraj()
{
	for (int index = 0; index < expertSU.noOfFrames; index++)
	{
		exptraj_b0[index][0] = 1; exptraj_b0[index][1] = expertSU.vectors[index][0]._x;
		exptraj_b0[index][2] = expertSU.vectors[index][0]._y; exptraj_b0[index][3] = expertSU.vectors[index][0]._z;

		exptraj_b1[index][0] = 1; exptraj_b1[index][1] = expertSU.vectors[index][1]._x;
		exptraj_b1[index][2] = expertSU.vectors[index][1]._y; exptraj_b1[index][3] = expertSU.vectors[index][1]._z;

		exptraj_b2[index][0] = 1; exptraj_b2[index][1] = expertSU.vectors[index][2]._x;
		exptraj_b2[index][2] = expertSU.vectors[index][2]._y; exptraj_b2[index][3] = expertSU.vectors[index][2]._z;

		exptraj_b3[index][0] = 1; exptraj_b3[index][1] = expertSU.vectors[index][3]._x;
		exptraj_b3[index][2] = expertSU.vectors[index][3]._y; exptraj_b3[index][3] = expertSU.vectors[index][3]._z;

		exptraj_b4[index][0] = 1; exptraj_b4[index][1] = expertSU.vectors[index][4]._x;
		exptraj_b4[index][2] = expertSU.vectors[index][4]._y; exptraj_b4[index][3] = expertSU.vectors[index][4]._z;

		exptraj_b5[index][0] = 1; exptraj_b5[index][1] = expertSU.vectors[index][5]._x;
		exptraj_b5[index][2] = expertSU.vectors[index][5]._y; exptraj_b5[index][3] = expertSU.vectors[index][5]._z;

		exptraj_b6[index][0] = 1; exptraj_b6[index][1] = expertSU.vectors[index][6]._x;
		exptraj_b6[index][2] = expertSU.vectors[index][6]._y; exptraj_b6[index][3] = expertSU.vectors[index][6]._z;

		exptraj_b7[index][0] = 1; exptraj_b7[index][1] = expertSU.vectors[index][7]._x;
		exptraj_b7[index][2] = expertSU.vectors[index][7]._y; exptraj_b7[index][3] = expertSU.vectors[index][7]._z;

		exptraj_b8[index][0] = 1; exptraj_b8[index][1] = expertSU.vectors[index][8]._x;
		exptraj_b8[index][2] = expertSU.vectors[index][8]._y; exptraj_b8[index][3] = expertSU.vectors[index][8]._z;

		exptraj_b9[index][0] = 1; exptraj_b9[index][1] = expertSU.vectors[index][9]._x;
		exptraj_b9[index][2] = expertSU.vectors[index][9]._y; exptraj_b9[index][3] = expertSU.vectors[index][9]._z;
	}
}
/* reads data files, resets memory, counters and flags for fresh display. called on press of key '1' or enabled
*/
void startFresh(char* fileName)
{
	ms.su->readAvatarData(fileName);
	VitruvianAvatar::isLoaded = true;
	/*ms.su->fullBodytoXYZ();
	ms.su->vectors[i][2]._x*/

	bReadFile = true;
	trajCount = 0;
	expTrajCount = 0;
	ms.su->subOption = 1;
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

	memset(exptraj_b0, 0, 80056 * (sizeof(float)));
	memset(exptraj_b1, 0, 80056 * (sizeof(float)));
	memset(exptraj_b2, 0, 80056 * (sizeof(float)));
	memset(exptraj_b3, 0, 80056 * (sizeof(float)));
	memset(exptraj_b4, 0, 80056 * (sizeof(float)));
	memset(exptraj_b5, 0, 80056 * (sizeof(float)));
	memset(exptraj_b6, 0, 80056 * (sizeof(float)));
	memset(exptraj_b7, 0, 80056 * (sizeof(float)));
	memset(exptraj_b8, 0, 80056 * (sizeof(float)));
	memset(exptraj_b9, 0, 80056 * (sizeof(float)));
	/*expertSU.readAvatarData("..\\src\\data\\RotationData\\ExpertFormFile.txt");
	expertSU.fullBodytoXYZ();*/
	/*expTrajCount = 0;
	loadexpTraj();*/

	glutPostRedisplay();
	MotionSphere::keyPressed = false;
}


void updateTrajectoryArray(int ind)
{
	Avatar firstInverse = getFirstInverse(ind);

	avatar.b0 = firstInverse.b0;
	avatar.b1 = firstInverse.b0.Inverse().mutiplication(firstInverse.b1);
	avatar.b2 = firstInverse.b1.Inverse().mutiplication(firstInverse.b2);
	avatar.b3 = firstInverse.b2.Inverse().mutiplication(firstInverse.b3);
	avatar.b4 = firstInverse.b1.Inverse().mutiplication(firstInverse.b4);
	avatar.b5 = firstInverse.b4.Inverse().mutiplication(firstInverse.b5);
	avatar.b6 = firstInverse.b0.Inverse().mutiplication(firstInverse.b6);
	avatar.b7 = firstInverse.b6.Inverse().mutiplication(firstInverse.b7);
	avatar.b8 = firstInverse.b0.Inverse().mutiplication(firstInverse.b8);
	avatar.b9 = firstInverse.b8.Inverse().mutiplication(firstInverse.b9);

	TVec3 b0Vec, b1Vec;
	float tAngle0, tAngle1;
	quaternion vQuat(ms.su->startingVector._x, ms.su->startingVector._y, -ms.su->startingVector._z, 0);

	quaternion uTransfBodyQuat = avatar.b0.mutiplication(vQuat.mutiplication(avatar.b0.Inverse()));

	traj_b0[ind][0] = 1;
	traj_b0[ind][1] = uTransfBodyQuat.mData[0];
	traj_b0[ind][2] = uTransfBodyQuat.mData[1];
	traj_b0[ind][3] = uTransfBodyQuat.mData[2];
	ms.su->twistAngles[ind][0] = ms.su->getTwistAngle({ uTransfBodyQuat.mData[0] ,uTransfBodyQuat.mData[1] ,uTransfBodyQuat.mData[2] }, uTransfBodyQuat);

	calculateTrajectory(avatar.b0, avatar.b1, b0Vec, b1Vec, tAngle0, tAngle1);

	ms.su->twistAngles[ind][1] = tAngle1;
	traj_b1[ind][0] = 1;
	traj_b1[ind][1] = b1Vec._x;
	traj_b1[ind][2] = b1Vec._y;
	traj_b1[ind][3] = b1Vec._z;


	TVec3 b2Vec, b3Vec;
	float tAngle2, tAngle3;
	calculateTrajectory(avatar.b2, avatar.b3, b2Vec, b3Vec, tAngle2, tAngle3);
	
	ms.su->twistAngles[ind][2] = tAngle2;
	traj_b2[ind][0] = 1;
	traj_b2[ind][1] = b2Vec._x;
	traj_b2[ind][2] = b2Vec._y;
	traj_b2[ind][3] = b2Vec._z;


	ms.su->twistAngles[ind][3] = tAngle3;
	traj_b3[ind][0] = 1;
	traj_b3[ind][1] = b3Vec._x;
	traj_b3[ind][2] = b3Vec._y;
	traj_b3[ind][3] = b3Vec._z;
	{
		float swingAngle = ms.su->vecDotProduct({ traj_b3[0][1],traj_b3[0][2] ,traj_b3[0][3] },
			{ traj_b3[ind][1],traj_b3[ind][2] ,traj_b3[ind][3] });
		//cout << acos(swingAngle)*180/PI << "," << tAngle3*180/PI << endl;
	}

	
	TVec3 b4Vec, b5Vec;
	float tAngle4, tAngle5;
	calculateTrajectory(avatar.b4, avatar.b5, b4Vec, b5Vec, tAngle4, tAngle5);

	ms.su->twistAngles[ind][4] = tAngle4;
	traj_b4[ind][0] = 1;
	traj_b4[ind][1] = b4Vec._x;
	traj_b4[ind][2] = b4Vec._y;
	traj_b4[ind][3] = b4Vec._z;

	//cout << traj_b4[ind][1] << "," << traj_b4[ind][2] << "," << traj_b4[ind][3] << "," << tAngle4 << endl;

	ms.su->twistAngles[ind][5] = tAngle5;
	traj_b5[ind][0] = 1;
	traj_b5[ind][1] = b5Vec._x;
	traj_b5[ind][2] = b5Vec._y;
	traj_b5[ind][3] = b5Vec._z;

	//cout << traj_b5[ind][1] << "," << traj_b5[ind][2] << "," << traj_b5[ind][3] << "," << tAngle5 << endl;

	TVec3 b6Vec, b7Vec;
	float tAngle6, tAngle7;
	calculateTrajectory(avatar.b6, avatar.b7, b6Vec, b7Vec, tAngle6, tAngle7);

	ms.su->twistAngles[ind][6] = tAngle6;
	traj_b6[ind][0] = 1;
	traj_b6[ind][1] = b6Vec._x;
	traj_b6[ind][2] = b6Vec._y;
	traj_b6[ind][3] = b6Vec._z;

	//cout << traj_b6[ind][1] << "," << traj_b6[ind][2] << "," << traj_b6[ind][3] << "," << tAngle6 << endl;

	ms.su->twistAngles[ind][7] = tAngle7;
	traj_b7[ind][0] = 1;
	traj_b7[ind][1] = b7Vec._x;
	traj_b7[ind][2] = b7Vec._y;
	traj_b7[ind][3] = b7Vec._z;

	//cout << traj_b7[ind][1] << "," << traj_b7[ind][2] << "," << traj_b7[ind][3] << "," << tAngle7 << endl;

	TVec3 b8Vec, b9Vec;
	float tAngle8, tAngle9;
	calculateTrajectory(avatar.b8, avatar.b9, b8Vec, b9Vec, tAngle8, tAngle9);

	ms.su->twistAngles[ind][8] = tAngle8;
	traj_b8[ind][0] = 1;
	traj_b8[ind][1] = b8Vec._x;
	traj_b8[ind][2] = b8Vec._y;
	traj_b8[ind][3] = b8Vec._z;

	//cout << traj_b8[ind][1] << "," << traj_b8[ind][2] << "," << traj_b8[ind][3] << "," << tAngle8 << endl;

	ms.su->twistAngles[ind][9] = tAngle9;
	traj_b9[ind][0] = 1;
	traj_b9[ind][1] = b9Vec._x;
	traj_b9[ind][2] = b9Vec._y;
	traj_b9[ind][3] = b9Vec._z;
}

/*
Defines what needs to be done when the main loop is idel
Its a call back function which is executed at regular intervels.
*/

void sphereIdle()
{
	if (MotionSphere::keyPressed)
	{
		startFresh("..\\src\\data\\RotationData\\FormFile.txt");
	}

	if (bReadFile)
	{
		switch (ms.su->subOption)
		{
		case 1:
		{
			if (trajCount >= ms.su->noOfFrames)
			{
				bReadFile = false;
				isize = 0;

				break;
			}
			updateTrajectoryArray(trajCount);
			trajCount++;
		}
		break;
		case 2:
		{
			if (trajCount >= ms.su->noOfFrames)
			{
				bReadFile = false;
				isize = 0;
				break;
			}

			avatar.b0 = ms.su->avatarData[trajCount].b0;
			avatar.b2 = ms.su->avatarData[trajCount].b2;
			avatar.b3 = ms.su->avatarData[trajCount].b3;
			avatar.b4 = ms.su->avatarData[trajCount].b4;
			avatar.b5 = ms.su->avatarData[trajCount].b5;

			TVec3 b0Vec, b1Vec;
			float tAngle0, tAngle1;
			quaternion vQuat(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

			quaternion uTransfBodyQuat = avatar.b0.mutiplication(vQuat.mutiplication(avatar.b0.Inverse()));

			traj_b0[trajCount][0] = 1;
			traj_b0[trajCount][1] = uTransfBodyQuat.mData[0];
			traj_b0[trajCount][2] = uTransfBodyQuat.mData[1];
			traj_b0[trajCount][3] = uTransfBodyQuat.mData[2];
			ms.su->twistAngles[trajCount][0] = ms.su->getTwistAngle({ uTransfBodyQuat.mData[0] ,uTransfBodyQuat.mData[1] ,uTransfBodyQuat.mData[2] }, uTransfBodyQuat);

			TVec3 b2Vec, b3Vec;
			float tAngle2, tAngle3;
			calculateTrajectory(avatar.b2, avatar.b3, b2Vec, b3Vec, tAngle2, tAngle3);

			ms.su->twistAngles[trajCount][2] = tAngle2;
			traj_b2[trajCount][0] = 1;
			traj_b2[trajCount][1] = b2Vec._x;
			traj_b2[trajCount][2] = b2Vec._y;
			traj_b2[trajCount][3] = b2Vec._z;

			ms.su->twistAngles[trajCount][3] = tAngle3;
			traj_b3[trajCount][0] = 1;
			traj_b3[trajCount][1] = b3Vec._x;
			traj_b3[trajCount][2] = b3Vec._y;
			traj_b3[trajCount][3] = b3Vec._z;

			TVec3 b4Vec, b5Vec;
			float tAngle4, tAngle5;
			calculateTrajectory(avatar.b4, avatar.b5, b4Vec, b5Vec, tAngle4, tAngle5);

			ms.su->twistAngles[trajCount][4] = tAngle4;
			traj_b4[trajCount][0] = 1;
			traj_b4[trajCount][1] = b4Vec._x;
			traj_b4[trajCount][2] = b4Vec._y;
			traj_b4[trajCount][3] = b4Vec._z;

			ms.su->twistAngles[trajCount][5] = tAngle5;
			traj_b5[trajCount][0] = 1;
			traj_b5[trajCount][1] = b5Vec._x;
			traj_b5[trajCount][2] = b5Vec._y;
			traj_b5[trajCount][3] = b5Vec._z;

			trajCount++;

		}

		break;

		case 3:
		{

			if (trajCount >= dsize)
			{
				bReadFile = false;
				isize = 0;
				break;
			}

			avatar.b0 = ms.su->avatarData[trajCount].b0;
			avatar.b6 = ms.su->avatarData[trajCount].b6;
			avatar.b7 = ms.su->avatarData[trajCount].b7;
			avatar.b8 = ms.su->avatarData[trajCount].b8;
			avatar.b9 = ms.su->avatarData[trajCount].b9;

			TVec3 b0Vec, b1Vec;
			float tAngle0, tAngle1;
			quaternion vQuat(ms.su->startingVector._x, ms.su->startingVector._y, ms.su->startingVector._z, 0);

			quaternion uTransfBodyQuat = avatar.b0.mutiplication(vQuat.mutiplication(avatar.b0.Inverse()));

			traj_b0[trajCount][0] = 1;
			traj_b0[trajCount][1] = uTransfBodyQuat.mData[0];
			traj_b0[trajCount][2] = uTransfBodyQuat.mData[1];
			traj_b0[trajCount][3] = uTransfBodyQuat.mData[2];
			ms.su->twistAngles[trajCount][0] = ms.su->getTwistAngle({ uTransfBodyQuat.mData[0] ,uTransfBodyQuat.mData[1] ,uTransfBodyQuat.mData[2] }, uTransfBodyQuat);

			TVec3 b6Vec, b7Vec;
			float tAngle6, tAngle7;
			calculateTrajectory(avatar.b6, avatar.b7, b6Vec, b7Vec, tAngle6, tAngle7);

			ms.su->twistAngles[trajCount][6] = tAngle6;
			traj_b6[trajCount][0] = 1;
			traj_b6[trajCount][1] = b6Vec._x;
			traj_b6[trajCount][2] = b6Vec._y;
			traj_b6[trajCount][3] = b6Vec._z;

			ms.su->twistAngles[trajCount][7] = tAngle7;
			traj_b7[trajCount][0] = 1;
			traj_b7[trajCount][1] = b7Vec._x;
			traj_b7[trajCount][2] = b7Vec._y;
			traj_b7[trajCount][3] = b7Vec._z;



			TVec3 b8Vec, b9Vec;
			float tAngle8, tAngle9;
			calculateTrajectory(avatar.b8, avatar.b9, b8Vec, b9Vec, tAngle8, tAngle9);

			ms.su->twistAngles[trajCount][8] = tAngle8;
			traj_b8[trajCount][0] = 1;
			traj_b8[trajCount][1] = b8Vec._x;
			traj_b8[trajCount][2] = b8Vec._y;
			traj_b8[trajCount][3] = b8Vec._z;

			ms.su->twistAngles[trajCount][9] = tAngle9;
			traj_b9[trajCount][0] = 1;
			traj_b9[trajCount][1] = b9Vec._x;
			traj_b9[trajCount][2] = b9Vec._y;
			traj_b9[trajCount][3] = b9Vec._z;


			trajCount++;

		}
		break;

		default:
			break;
		}
	}
	glutPostRedisplay();
}



void SpecialkeyBoardEvent(int key, int x, int y)
{
	quaternion q;
	if (stencilIndex > 0)
	{
		switch (key)
		{
		case GLUT_KEY_DOWN:	q = quaternion(0.0087155743, 0, 5.33894E-18, 0.999962); break;
		case GLUT_KEY_UP:	q = quaternion(-0.0087155743, 0, 5.33894E-18, 0.999962); break;
		case GLUT_KEY_RIGHT:	q = quaternion(0, 5.33894E-18, 0.0087155743, 0.999962); break;
		case GLUT_KEY_LEFT:	q = quaternion(0, 5.33894E-18, -0.0087155743, 0.999962); break;
		}
		switch (sphereID)
		{
		case 0: ms.su->avatarData[stencilIndex - 1].b0 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b0);	break;
		case 1: ms.su->avatarData[stencilIndex - 1].b1 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b1);	break;
		case 2: ms.su->avatarData[stencilIndex - 1].b2 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b2);	break;
		case 3: ms.su->avatarData[stencilIndex - 1].b3 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b3);	break;
		case 4: ms.su->avatarData[stencilIndex - 1].b4 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b4);	break;
		case 5: ms.su->avatarData[stencilIndex - 1].b5 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b5);	break;
		case 6: ms.su->avatarData[stencilIndex - 1].b6 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b6);	break;
		case 7: ms.su->avatarData[stencilIndex - 1].b7 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b7);	break;
		case 8: ms.su->avatarData[stencilIndex - 1].b8 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b8);	break;
		case 9: ms.su->avatarData[stencilIndex - 1].b9 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b9);	break;
		default:
			break;
		}
		updateTrajectoryArray(stencilIndex - 1);
	}
}

/* Two key board functions are defined
	 key '1' : loads the files from two users in a predefined location with in  startFresh() function
	 key 'c' : enables/disables (toggles) comparision between two users.
*/
void keyBoardEvent(unsigned char key, int x, int y)
{
	if (key == '1') //Key-1
	{
		startFresh("..\\src\\data\\RotationData\\FormFile.txt");
	}

	if (key == '2') //Key-1
	{
		startFresh("NewFormFile.txt");
	}

	if (key == 'c')
	{
		{
			expTrajCount = 0;
			enableComparision = !enableComparision;
		}
		glutPostRedisplay();
	}
	if (key == 's')
	{
		ms.su->writeAvatarData("NewFormFile.txt");
	}

}


/*
Mouse wheel function is used for zooming in and zooming out the sphere.
*/
void mouseWheel(int button, int dir, int x, int y)
{
	if (stencilIndex < 1)
		toggleEdit = false;
	else
		toggleEdit = true;
	if (toggleEdit)
	{
		quaternion q;

		if (dir > 0)
			q = quaternion(0, 0.0087155743, 5.33894E-18, 0.999962);
		else
			q = quaternion(0, -0.0087155743, 5.33894E-18, 0.999962);

		switch (sphereID)
		{
		case 0: ms.su->avatarData[stencilIndex - 1].b0 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b0);	break;
		case 1: ms.su->avatarData[stencilIndex - 1].b1 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b1);	break;
		case 2: ms.su->avatarData[stencilIndex - 1].b2 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b2);	break;
		case 3: ms.su->avatarData[stencilIndex - 1].b3 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b3);	break;
		case 4: ms.su->avatarData[stencilIndex - 1].b4 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b4);	break;
		case 5: ms.su->avatarData[stencilIndex - 1].b5 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b5);	break;
		case 6: ms.su->avatarData[stencilIndex - 1].b6 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b6);	break;
		case 7: ms.su->avatarData[stencilIndex - 1].b7 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b7);	break;
		case 8: ms.su->avatarData[stencilIndex - 1].b8 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b8);	break;
		case 9: ms.su->avatarData[stencilIndex - 1].b9 = q.mutiplication(ms.su->avatarData[stencilIndex - 1].b9);	break;
		default:
			break;
		}
		updateTrajectoryArray(stencilIndex - 1);
	}
	if (rotEnable1 && !toggleEdit)
	{
		if (dir > 0)
		{
			zval1 = zval1 - 0.1;
		}
		else
		{
			zval1 = zval1 + 0.1;
		}
		pointTranslateX1 = zval1 * (cos(xrot*PI / 180)*  sin(yrot*PI / 180));
		pointTranslateY1 = zval1 * (sin(xrot*PI / 180));
		pointTranslateZ1 = zval1 * (cos(xrot*PI / 180) * cos(yrot*PI / 180));
	}
	else if (rotEnable2 && !toggleEdit)
	{
		if (dir > 0)
		{
			zval2 = zval2 - 0.1;
		}
		else
		{
			zval2 = zval2 + 0.1;
		}
		pointTranslateX2 = zval2 * (cos(xrot*PI / 180)*  sin(yrot*PI / 180));
		pointTranslateY2 = zval2 * (sin(xrot*PI / 180));
		pointTranslateZ2 = zval2 * (cos(xrot*PI / 180) * cos(yrot*PI / 180));
	}
	else if (rotEnable3 && !toggleEdit)
	{
		if (dir > 0)
		{
			zval3 = zval3 - 0.1;
		}
		else
		{
			zval3 = zval3 + 0.1;
		}
		pointTranslateX3 = zval3 * (cos(xrot*PI / 180)*  sin(yrot*PI / 180));
		pointTranslateY3 = zval3 * (sin(xrot*PI / 180));
		pointTranslateZ3 = zval3 * (cos(xrot*PI / 180) * cos(yrot*PI / 180));
	}
	else if (rotEnable4 && !toggleEdit)
	{
		if (dir > 0)
		{
			zval4 = zval4 - 0.1;
		}
		else
		{
			zval4 = zval4 + 0.1;
		}
		pointTranslateX4 = zval4 * (cos(xrot*PI / 180)*  sin(yrot*PI / 180));
		pointTranslateY4 = zval4 * (sin(xrot*PI / 180));
		pointTranslateZ4 = zval4 * (cos(xrot*PI / 180) * cos(yrot*PI / 180));
	}
	else
	{
		if (!toggleEdit) {
			if (dir > 0)
			{
				zval = zval - 0.1;
			}
			else
			{
				zval = zval + 0.1;
			}
			pointTranslateX = zval * (cos(xrot*PI / 180)*  sin(yrot*PI / 180));
			pointTranslateY = zval * (sin(xrot*PI / 180));
			pointTranslateZ = zval * (cos(xrot*PI / 180) * cos(yrot*PI / 180));
		}
	}

	glutPostRedisplay();
}

/*
Menu items functionality defined.
*/
void menu(int id)
{

	if (id == 17) // if toogle visualization
	{
		toggleOption = !toggleOption;
	}
	else if (id == 18) // if toogle visualization
	{
		toggleEdit = !toggleEdit;
	}
	else // set sphereID as the menu item
	{
		sphereID = id;
	}

	glutPostRedisplay();
}

/* initialize and read MGRS texture
*/
void sphereInitialize()
{
	///////////////////////Texture mapping///////////////////////////
	image_t   temp_image;

	//glClearColor(1.0, 1.0, 1.0, 0.0);
	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(2, texture_id);

	glBindTexture(GL_TEXTURE_2D, texture_id[0]);
	glEnable(GL_BLEND);							// Enable Blending       (disable alpha testing)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	tgaLoad("..\\src\\data\\mmgrs.tga", &temp_image, TGA_FREE | TGA_LOW_QUALITY);


	//glEnable(GL_CULL_FACE);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	sphere = gluNewQuadric();
	gluQuadricDrawStyle(sphere, GLU_FILL);
	gluQuadricNormals(sphere, GLU_SMOOTH);
	gluQuadricOrientation(sphere, GLU_INSIDE);
	gluQuadricTexture(sphere, GL_TRUE);
}

int MotionSphere::sphereMainLoop(MotionSphere newms, char* windowName)
{
	// Get MotionSphere object from the caller
	ms = newms;
	// Load PLY models 
	rightHandPLY.Load("..\\src\\data\\ms\\Right_Hand.ply");
	leftHandPLY.Load("..\\src\\data\\ms\\RiggedLeftHand.ply");
	leftFootPLY.Load("..\\src\\data\\ms\\Left_foot.ply");
	rightFootPLY.Load("..\\src\\data\\ms\\Right_foot.ply");
	kneePLY.Load("..\\src\\data\\ms\\knee.ply");
	elbowPLY.Load("..\\src\\data\\ms\\elbow2.ply");
	// Establish glut communication with OS
	glutInit(&targc, targv);
	//Initialize Window settings
	glClearStencil(0);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE | GLUT_STENCIL);
	glEnable(GL_STENCIL_TEST);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	glStencilFunc(GL_ALWAYS, -1, -1);
	glutInitWindowSize(ms.maxWidth - ms.minWidth, ms.maxHeight - ms.minHeight);
	glutCreateWindow(windowName);
	//Set interaction callbacks.
	sphereInitialize();
	glutReshapeFunc(sphereReshape);
	glutIdleFunc(sphereIdle);
	glutDisplayFunc(sphereDisplay);
	glutKeyboardFunc(keyBoardEvent);
	glutSpecialFunc(SpecialkeyBoardEvent);
	glutMouseFunc(sphereMouseEvent);
	glutMotionFunc(sphereMouseMotion);
	glutMouseWheelFunc(mouseWheel);
	// create a manu option
	int boneSelect = glutCreateMenu(menu);
	glutAddMenuEntry("Bone-0", 0);
	glutAddMenuEntry("Bone-1", 1);
	glutAddMenuEntry("Bone-2", 2);
	glutAddMenuEntry("Bone-3", 3);
	glutAddMenuEntry("Bone-4", 4);
	glutAddMenuEntry("Bone-5", 5);
	glutAddMenuEntry("Bone-6", 6);
	glutAddMenuEntry("Bone-7", 7);
	glutAddMenuEntry("Bone-8", 8);
	glutAddMenuEntry("Bone-9", 9);
	// create sub menu options
	glutCreateMenu(menu);
	glutAddSubMenu("Select Bone", boneSelect);
	glutAddMenuEntry("Right Arm", 10);
	glutAddMenuEntry("Left Arm", 11);
	glutAddMenuEntry("Right Leg", 12);
	glutAddMenuEntry("Left Leg", 13);
	glutAddMenuEntry("Upper Body", 14);
	glutAddMenuEntry("Lower Body", 15);
	glutAddMenuEntry("Fully Body", 16);
	glutAddMenuEntry("Toggle Vis", 17);
	glutAddMenuEntry("Toggle Edit", 18);
	//attach menu items to the right button of the mouse
	glutAttachMenu(GLUT_RIGHT_BUTTON);
	//call glut infinite loop
	glutMainLoop();
	return 0;
}
