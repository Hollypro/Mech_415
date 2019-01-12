
#include <cmath> 
#include <cstdio>  
#include <cstring>
#include <iostream> 
#include <fstream>   
#include <strstream> 
#include <iomanip> 

#include <Windows.h>
#include "MMsystem.h"

#include <chrono>
#include <ctime>


#include "3D_graphics.h" 
#include "HoverCraft.h"

// 3D graphics window size in pixels
int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 1920;
int HEIGHT_MAX = 1080;

// background colour for the scene
float BACK_R = 0.0f; // red colour component (0 to 1)
float BACK_G = 0.0f; // green colour component (0 to 1)
float BACK_B = 0.5f; // blue colour component (0 to 1)

double VMIN = 0.25;
double VMAX = 1000.0;
const double PI = 4 * atan(1.0);

using namespace std;






void draw_3D_graphics(){

	static double X[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	//static double Xd[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	// I commented this out because it's not used.
	static char file_name[11] = "hoverbus.x";

	

	static Hovercraft H1(X, file_name);

	

	//set_view();

	double eye_point[4], lookat_point[4], up_dir[4]; // 3+1 because the prof's library starts at index 1.
	eye_point[1] = -3.0 + H1.get_xy(6);  // x
	eye_point[2] = 0.0 + H1.get_xy(7); // y
	eye_point[3] = 15.0; // z



	// the position you are looking at in global coord
	lookat_point[1] = 0.0 + H1.get_xy(6); // x
	lookat_point[2] = 0.0 + H1.get_xy(7); // y
	lookat_point[3] = 0.0; // z

	// the direction of the top of your head (note: up is a direction, not a point)
	up_dir[1] = 0.0; // dx
	up_dir[2] = 0.0; // dy 
	up_dir[3] = 1.0;;//dz

	set_view(eye_point, lookat_point, up_dir);
	//draw_XYZ(50.0);
	//draw_XYZ(5.0);
	
	H1.eulers();
	H1.sim_step(0.005); 
	H1.input();
	H1.draw();
	H1.text();
	
	
	
	

}

