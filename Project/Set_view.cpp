
#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0
#include <iomanip>   // I/O manipulators

#include <windows.h> // for keyboard input

// user defined functions
#include "timer.h" // for measuring time
#include "rotation.h" // for computing rotation functions
#include "3D_graphics.h" // for DirectX 3D graphics

// 3D graphics window size in pixels
int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 1024; // increase this to increase the window width
int HEIGHT_MAX = 600; // increase this to increase the window height

// background colour for the scene
float BACK_R = 0.0f; // red colour component (0 to 1)
float BACK_G = 0.0f; // green colour component (0 to 1)
float BACK_B = 0.0f; // blue colour component (0 to 1)

// default min and max viewing distances.
// objects closer than VMIN and farther than VMAX are not drawn (ie cannot be seen).
// note the ratio of VMAX/VMIN should be less than 10,000 for most graphics cards.
double VMIN = 0.25; // units of m (or whatever units you draw your object in)
double VMAX = 1000.0; // units of m

using namespace std;



void draw_3D_graphics()
{
	static double Px=0.0,Py=0.0,Pz=0.0,roll=1.57,pitch=1.57,yaw=1.57;
	static int init = 0; 
	
	static double eye[3 + 1];
	static double lookat[3 + 1];
	static double direction[3 + 1];

	static double eye_g[3 + 1], lookat_g[3 + 1], direction_g[3 + 1];
	static double P[3 + 1] = { Px, Py, Pz };
	static double u[3 + 1];
	static double R[3 + 1][3 + 1];

	static double dtheta; 

	static double dz;
	static int toggle = 1;

	if (KEY('U')) toggle *= -1;

	static double t; 
	static double t0; 

	//static mesh m1("tiger.x");

	if(!init) {

		t0 = high_resolution_time(); 

		init = 1;
	} 

	if (toggle == 1){
		eye[1] = 5.0;
		eye[2] = 1.0;
		eye[3] = 0.0;
		lookat[1] = 0.0;
		lookat[2] = 0.0;
		lookat[3] = 0.0;
		direction[1] = 0.0;
		direction[2] = 1.0;
		direction[3] = 0.0;
		set_view(eye, lookat, direction);
	}
	else{
		eye[1] = 0.0;
		eye[2] = 1.0;
		eye[3] = 5.0;
		lookat[1] = 0.0;
		lookat[2] = 0.0;
		lookat[3] = 0.0;
		direction[1] = 0.0;
		direction[2] = 1.0;
		direction[3] = 0.0;

		euler_to_rotation(yaw, pitch, roll, R);


		multiply_Ax(R, eye, u);

		for (int i = 1; i <= 3; i++) eye_g[i] = P[i] + u[i];


		multiply_Ax(R, lookat, u);

		for (int i = 1; i <= 3; i++) lookat_g[i] = P[i] + u[i];


		multiply_Ax(R, direction, direction_g);

		set_view(eye_g, lookat_g, direction_g);
	}

	draw_XYZ(5.0);  
	t = high_resolution_time() - t0; 

	

	//m1.draw(Px,Py,Pz,yaw,pitch,roll);

	
	dz = 0.003; 
	dtheta = 0.0075;

	if( KEY(VK_UP)) Pz += dz;

	if( KEY(VK_DOWN)) Pz -= dz;

	if (KEY(VK_RIGHT)) roll += dtheta;

	if (KEY(VK_LEFT)) roll -= dtheta;
	
}

