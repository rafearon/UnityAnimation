#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifdef WIN32
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif
#endif
#include "articulated_bodies.h"

using namespace std;

//Specify the target end position by clicking the left button of the mouse
//This sample code assumes the joints are all rotational joints with a single degree of freedom, and the rotation axis is (0,0,1)

int width;
int height;
ArticulatedBodies* articulated_object;
bool ready_to_update=false;
float target_x;
float target_y;
float target_z;
float max_length;
int num_bodies;
float dt=0.0033f;

void initializeArticulatedBodies(){
	num_bodies=3;
	articulated_object = new ArticulatedBodies(num_bodies);

	articulated_object->e0[0] = 0.;articulated_object->e0[1] = 0.;articulated_object->e0[2] = 0.;
	max_length = 0.f;
	for(int i=0; i<num_bodies; i++){
		articulated_object->angles[i] = M_PI/6.;
		articulated_object->lengths[i] = 4;
		max_length += articulated_object->lengths[i];
	}

	articulated_object->updateEndEffector();

	articulated_object->target_e[0] = articulated_object->e1[0];
	articulated_object->target_e[1] = articulated_object->e1[1];
	articulated_object->target_e[2] = articulated_object->e1[2];

	target_x = (float)articulated_object->target_e[0];
	target_y = (float)articulated_object->target_e[1];
	target_z = (float)articulated_object->target_e[2];
}

void updateArticulatedBodies(float dt){
	////rescale target if it is too far from the origin
	float distance = (float)sqrt(pow(target_x-(float)articulated_object->e0[0],2)+pow(target_y-(float)articulated_object->e0[1], 2)+pow(target_z-(float)articulated_object->e0[2], 2));
	if(distance > max_length){
		float rescale = max_length/distance;
		target_x = (float)articulated_object->e0[0] + (target_x-(float)articulated_object->e0[0])*rescale;
		target_y = (float)articulated_object->e0[1] + (target_y-(float)articulated_object->e0[1])*rescale;
		target_z = (float)articulated_object->e0[2] + (target_z-(float)articulated_object->e0[2])*rescale;
	}

	articulated_object->target_e[0] = target_x;
	articulated_object->target_e[1] = target_y;
	articulated_object->target_e[2] = target_z;

	float end_distance = (float)sqrt(pow(target_x-(float)articulated_object->e1[0],2)+pow(target_y-(float)articulated_object->e1[1], 2)+pow(target_z-(float)articulated_object->e1[2], 2));

	//cout<< "end distance "<< end_distance <<" max_length " << max_length << endl;
	if(end_distance > max_length*(float)1e-2) articulated_object->update(dt);
}

void drawCube(double l) {
	// yellow side - FRONT
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  1.0, 0.0 );
 	glVertex3f( 0., -0.4, -0.4);       // P1
	glVertex3f( 0,  0.4, -0.4);       // P2
	glVertex3f(  l,  0.4, -0.4);       // P3
	glVertex3f(  l, -0.4, -0.4);       // P4
 
	glEnd();

	// White side - BACK
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  1.0, 1.0 );
	glVertex3f(  l, -0.4, 0.4 );
	glVertex3f(  l,  0.4, 0.4 );
	glVertex3f( 0,  0.4, 0.4 );
	glVertex3f( 0, -0.4, 0.4 );
	glEnd();
 
	// Purple side - RIGHT
	glBegin(GL_POLYGON);
	glColor3f(  1.0,  0.0,  1.0 );
	glVertex3f( l, -0.4, -0.4 );
	glVertex3f( l,  0.4, -0.4 );
	glVertex3f( l,  0.4,  0.4 );
	glVertex3f( l, -0.4,  0.4 );
	glEnd();
 
	// Green side - LEFT
	glBegin(GL_POLYGON);
	glColor3f(   0.0,  1.0,  0.0 );
	glVertex3f( 0, -0.4,  0.4 );
	glVertex3f( 0,  0.4,  0.4 );
	glVertex3f( 0,  0.4, -0.4 );
	glVertex3f( 0, -0.4, -0.4 );
	glEnd();
 
	// Blue side - TOP
	glBegin(GL_POLYGON);
	glColor3f(   0.0,  0.0,  1.0 );
	glVertex3f(  l,  0.4,  0.4 );
	glVertex3f(  l,  0.4, -0.4 );
	glVertex3f( 0,  0.4, -0.4 );
	glVertex3f( 0,  0.4,  0.4 );
	glEnd();
 
	// Red side - BOTTOM
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  0.0,  0.0 );
	glVertex3f(  l, -0.4, -0.4 );
	glVertex3f(  l, -0.4,  0.4 );
	glVertex3f( 0, -0.4,  0.4 );
	glVertex3f( 0, -0.4, -0.4 );
	glEnd();
}

void myReshape(int w, int h)
{
	width=w;height=h;
	//cout<<"window width "<<width<<" height "<<height<<endl;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity(); /* init projection matrix */
    gluPerspective( 60.0, (GLdouble)w/(GLdouble)h, 0.1, 40.0);
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, -20.0, /* eye at (0,0,20) */
			  0.0, 0.0, 0.0, /* lookat point */
			  0.0, 1.0, 0.0); /* up is in +ive y */
}


// Initializes information for drawing within OpenGL.
void init() {
    GLfloat sun_direction[] = { 0.0, 2.0, -1.0, 1.0 };
    GLfloat sun_intensity[] = { 0.7, 0.7, 0.7, 1.0 };
    GLfloat ambient_intensity[] = { 0.3, 0.3, 0.3, 1.0 };

    glClearColor(1.0, 1.0, 1.0, 0.0);   // Set window color to white.

    glEnable(GL_DEPTH_TEST);            // Draw only closest surfaces

    glEnable(GL_LIGHTING);              // Set up ambient light.
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient_intensity);

    glEnable(GL_LIGHT0);                // Set up sunlight.
    glLightfv(GL_LIGHT0, GL_POSITION, sun_direction);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_intensity);

    glEnable(GL_COLOR_MATERIAL);        // Configure glColor().
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	initializeArticulatedBodies();
}

// Draws the current image.
void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear window.
    glColor3f(1.0, 1.0, 1.0);
    glShadeModel(GL_SMOOTH);

	//draw here
	glPushMatrix();
	for(int i=0;i<num_bodies;i++){
		if(i>0){
			glTranslatef(articulated_object->lengths[i-1],0,0);}
		else{
			glTranslatef(articulated_object->e0[0],articulated_object->e0[1],articulated_object->e0[2]);}
		glRotatef(articulated_object->angles[i]/M_PI*180,0,0,1);
		drawCube(articulated_object->lengths[i]);}
	glPopMatrix();

	ready_to_update=true;

	glEnd();

    glutSwapBuffers();
}

void idle(void)
{
    updateArticulatedBodies(dt);
    glutPostRedisplay();
}

void Get_Target_X_Y(const int x,const int y)
{
	//assume target_z is always near z=0 plane in this sample code for simplicity.
	GLdouble modelview[16], projection[16];
	GLint viewport[4];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	//cout<<"viewport "<<viewport[0]<<" "<<viewport[1]<<" "<<viewport[2]<<" "<<viewport[3]<<" "<<endl;
	GLdouble winX = (double)x;
	GLdouble winY = (double)viewport[3] - (double)y;
	GLdouble winZ;
	//get winZ based on the Z coordinate of the world space origin (0,0,0) in the window coordinate
	GLdouble originX=0, originY=0, originZ=0;
	gluProject(originX,originY,originZ,modelview,projection,viewport,&winX,&winY,&winZ);
	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	//cout<<"mouse x "<<winX<<" y "<<winY<<" z "<<winZ<<endl;
	GLdouble objX, objY, objZ;
	gluUnProject(winX,winY,winZ,modelview,projection,viewport,&objX,&objY,&objZ);
	target_x=(float)(objX);
	target_y=(float)(objY);
	target_z=(float)(objZ);
	cout<<"x: "<<target_x<<", y: "<<target_y<<", z: "<<target_z<<endl;
}

void myMouseClick(int button,int state,int x,int y)
{
	if(button == GLUT_LEFT_BUTTON){
		if(state == GLUT_DOWN){
			Get_Target_X_Y(x,y);
		}
	}
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowPosition(50, 100);    // Set up display window.
    glutInitWindowSize(800, 800);
    glutCreateWindow("Joints");
	glutReshapeFunc(myReshape);
    init();
    glutDisplayFunc(draw);
	glutIdleFunc(idle);
	glutMouseFunc(myMouseClick);
    glutMainLoop();
	delete articulated_object;
    return 0;
}
