
#include <cmath> 
#include <cstdio>  
#include <cstring>
#include <iostream> 
#include <fstream>   
#include <strstream> 
#include <iomanip> 
#include <Windows.h>
#include <chrono>
#include <ctime>

#include "timer.h" 
#include "rotation.h"
#include "3D_graphics.h" 

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

ofstream fout("debug.txt");


class Hovercraft {

	double X[8];
	//index# : 0  1 2  3 4   5 6  7
	//StateV : xb u yb v yaw r xc yc
	
	//Xb and yb are measured with respect to the hovercraft
	//u and v are their derivatives respectively
	//yaw is measured with respect to the yc axis in radians.
	//r is the derivative of yaw.
	//xc and yc are universal x and y.


	double Xd[8];
	
	//Holds the derivative of StatV
	//index# : 0 1  2 3  4 5  6  7
	//StateV : u au v av r ar vx vy

	bool Prev_Key;

	double BStat_Var[6];
	//Bullet State variables
	// index 0 1 2 3     4   5
	// Var:  x y z pitch yaw roll

	double BDStatV[6];
	//Bullet State variables derivatives
	// index 0  1  2  3      4    5
	// Var:  vx vy vz dpitch dyaw droll

	int Velocity;

	double Tar_Pos[2];

	double Fol_Stat_V[3];
	//index# : 0 1 2
	//StateV : x y yaw

	double Fol_DStatV[3];
	//index# : 0 1 2
	//StateV : u v r

	double Fol_Velocity;

public:

	
	double c1, c2, c3, J, Mass, L;
	// initialize constants AKA drag coefficients, Inertia and Mass of Hovercraft, and Length between motors.
	
	bool View = true;

	double U[2]; // 2 inputs aka Fr and Fl
	
	//auto TimeThen;

	mesh *p_mesh;
	mesh *p_env;
	mesh *p_bull;
	mesh *p_tar;
	mesh *p_fol;

	Hovercraft(double *X, char *file_name);//constructor

	~Hovercraft();//destructor

	void draw(); //draw hovercraft

	void sim_step(double dt); //sim step

	void eulers(); //calculate eulers

	void input();// change the inputs to Fl or Fr to 1 or 0;

	double get_xy(int index);

	double get_Bullxy(int index);

};


Hovercraft::Hovercraft(double y[8], char *file_name) {
	//Specified array size 8 because it should never be anything else.

	c1 = 0.05; //Drag coefficients 0.5
	c2 = 0.05;
	c3 = 0.05; // So it doesn't spin forever.
	J = 0.1; // inertia 1
	L = 1; // 1 m between motors
	Mass = 0.1; // mass 1

	Prev_Key = false;

	//auto TimeThen = chrono::high_resolution_clock::now();

	for (int i = 0; i < 8; i++){
		// initialize variables to y[i]
		X[i] = y[i];
	}

	for (int i = 1; i <= 2; i++){
		//itialize inputs to 0
		U[i] = 0;
	}

	BStat_Var[0] = X[6];
	BStat_Var[1] = X[7];
	BStat_Var[2] = 0;
	BStat_Var[3] = X[3];
	BStat_Var[4] = 0;
	BStat_Var[5] = 0;
	BDStatV[0] = cos(BStat_Var[4]) * Velocity;
	BDStatV[1] = sin(BStat_Var[4]) * Velocity;

	Velocity = 10;

	BDStatV[0] = cos(BStat_Var[4])*Velocity;
	BDStatV[1] = sin(BStat_Var[4]) * Velocity;
	BDStatV[2] = 0;
	BDStatV[3] = 0;
	BDStatV[4] = 0;
	BDStatV[5] = 0;

	Fol_Stat_V[0] = 0.0;
	Fol_Stat_V[1] = -0.5;
	Fol_Stat_V[2] = 0.0;

	Fol_DStatV[0] = 0.0;
	Fol_DStatV[1] = 0.0;
	Fol_DStatV[2] = 0.0;

	srand(time(NULL)); //Randomly place the target within the map.
	Tar_Pos[0] = rand() % 70 - 35;
	srand(time(NULL));
	Tar_Pos[1] = rand() % 30 - 15;

	p_mesh = new mesh(file_name);
	p_env = new mesh("track2.x");
	p_bull = new mesh("Bullet.x");
	p_tar = new mesh("Target.x");
	p_fol = new mesh(file_name);
}

Hovercraft::~Hovercraft(){
	p_mesh = nullptr;
	p_env = nullptr;
	p_bull = nullptr;
	p_tar = nullptr;
	p_fol = nullptr;
	delete p_tar;
	delete p_fol;
	delete p_bull;
	delete p_env;
	delete p_mesh;
}




void Hovercraft::draw(){ //draw the hover craft

	p_mesh->Scale = 0.3;
	p_env->Scale = 0.05;
	p_bull->Scale = 0.005;
	p_tar->Scale = 0.25;
	p_fol->Scale = 0.3 * 0.5;
	p_env->draw(0.0, 0.0, 1.5, 0.0, 0.0, 0.0);

	// void draw(double Tx, double Ty, double Tz, double yaw, double pitch, double roll);
	p_mesh->draw(X[6], X[7], 0.0, X[4] + PI, 0.0, PI / 2);
	p_fol->draw(Fol_Stat_V[0], Fol_Stat_V[1], 0.0, Fol_Stat_V[2] + PI, 0.0, PI / 2);
	if (View){
		p_bull->draw(BStat_Var[0], BStat_Var[1], 0.0, BStat_Var[4] + PI / 2, 0.0, PI);
		// void draw(double Tx, double Ty, double Tz, double yaw, double pitch, double roll);
		//max				40			15
		//min				-40			-20
		p_tar->draw(Tar_Pos[0], Tar_Pos[1], 0.0, 0.0, 0.0, 0.0);
	}
	else{
		p_bull->draw(BStat_Var[0], BStat_Var[1], -0.5, BStat_Var[4] + PI / 2, 0.0, PI);
		p_tar->draw(Tar_Pos[0], Tar_Pos[1], -0.5, 0.0, 0.0, 0.0);
	}
	// Hovercraft has fixed height so: Tz=0 always.
	// Pitch and Roll are constants for our application.
	// roll is pi/2 because initial hovercraft wasn't well positioned
}


void Hovercraft::sim_step(double dt){
	
	//sim step for eulers
	for (int i = 0; i < 8; i++){
		X[i] = X[i] + Xd[i] * dt;
	}
	for (int i = 0; i < 6; i++){
		BStat_Var[i] += BDStatV[i] * dt;
	}
	for (int i = 0; i < 3; i++){
		Fol_Stat_V[i] += Fol_DStatV[i] * dt;
	}
}

double Hovercraft::get_xy(int index){
	return X[index];
}

double Hovercraft::get_Bullxy(int index){
	return BStat_Var[index];
}

void Hovercraft::input(){
	{
		//auto Then = std::chrono::duration<double>(TimeThen.time_since_epoch());
		//auto TimeNow = chrono::high_resolution_clock::now();					
		//auto Now = std::chrono::duration<double>(TimeNow.time_since_epoch());	

		/*This whole section is to ensure enough time has passed since switching between third person
		view and fps, to prevent me from puking while debugging.*/

		/*It turns out that VS 2013 is known for being unable to deal with time.
		https://stackoverflow.com/questions/17769172/non-conforming-return-value-for-stdchronodurationoperator-in-microsoft
		https://stackoverflow.com/questions/24586804/c11-chrono-in-visual-studio-2013
		*/


		//double TimeSince = Now.count() - Then.count();			

		if (KEY(VK_UP) && !(Prev_Key)){
			View = !View;
			Prev_Key = true;
			//auto TimeThen = chrono::high_resolution_clock::now();
		}
		else if (!(KEY(VK_UP))){ //So that holding the up key won't flash between the two views.
			Prev_Key = false;
		}

		if (View){}
		else{
			// void set_view(double *eye_point, double *lookat_point, double *up_dir, double fov=3.14159/4);

			double eye_point[4], lookat_point[4], up_dir[4]; // 3+1 because the prof's library starts at index 1.


			eye_point[1] = 0.0 + 1 * get_xy(6);  // x
			eye_point[2] = 0.0 + 1 * get_xy(7); // y
			eye_point[3] = -0.3; // z



			// the position you are looking at in global coord
			lookat_point[1] = 0.0 + 1 * get_xy(6) + 2 * cos(get_xy(4)); // x
			lookat_point[2] = 0.0 + 1 * get_xy(7) + 2 * sin(get_xy(4)); // y
			lookat_point[3] = -1.0; // z

			// the direction of the top of your head (note: up is a direction, not a point)
			up_dir[1] = 0.0;//sin(get_xy(4)); // dx
			up_dir[2] = 0.0;//cos(get_xy(4)); // dy 
			up_dir[3] = 1.0;;//dz

			set_view(eye_point, lookat_point, up_dir); // fpv when up is pressed.
		}

		//click down arrow key to reset position *** Change it if you can 
		if (KEY(VK_DOWN)) {
			for (int i = 0; i < 8; i++) {
				X[i] = { 0.0 };
			}
		}

		// Fr goes to 1 so eulers function works
		if (KEY(VK_RIGHT)){
			U[0] = 1.0;
		}
		else if (KEY(0x0058)){ // Press X to have the motor work in reverse
			U[0] = -1.0;
		}
		else{
			U[0] = 0.0;
		}




		// Fl goes to 1
		if (KEY(VK_LEFT)){
			U[1] = 1.0;
		}
		else if (KEY(0x005A)){ // Press Z to have the motor work in reverse
			U[1] = -1.0;
		}
		else{
			U[1] = 0.0;
		}

		if (KEY(VK_SPACE)){

			Velocity = 10 + sqrt(X[1] * X[1] + X[3] * X[3]);
			//Hovercraft State Variable
			//index# : 0  1 2  3 4   5 6  7
			//StateV : xb u yb v yaw r xc yc

			//Bullet State variables
			// index 0 1 2 3     4   5
			// Var:  x y z pitch yaw roll
			BStat_Var[0] = X[6];
			BStat_Var[1] = X[7];
			BStat_Var[2] = 0;
			BStat_Var[3] = X[3];
			BStat_Var[4] = X[4];
			BStat_Var[5] = 0;
			BDStatV[0] = cos(BStat_Var[4]) * Velocity;
			BDStatV[1] = sin(BStat_Var[4]) * Velocity;
		}


		double dist = sqrt((BStat_Var[0] - Tar_Pos[0]) * (BStat_Var[0] - Tar_Pos[0]) + (BStat_Var[1] - Tar_Pos[1])*  (BStat_Var[1] - Tar_Pos[1]));

		if (dist < 0.5){
			srand(time(NULL));
			Tar_Pos[0] = rand() % 80 - 40;
			srand(time(NULL));
			Tar_Pos[1] = rand() % 30 - 15;
		}

		Fol_Velocity = sqrt((Fol_Stat_V[0] - X[6]) * (Fol_Stat_V[0] - X[6]) + (Fol_Stat_V[1] - X[7])*  (Fol_Stat_V[1] - X[7])) * 10.0;
		BDStatV[0] = Fol_Velocity * (Fol_Stat_V[0] - X[6]);
		BDStatV[1] = Fol_Velocity * (Fol_Stat_V[1] - X[7]);
		BDStatV[2] = Fol_Velocity * (Fol_Stat_V[2] - X[4]);
	}
}


void Hovercraft::eulers(){
	// All of the following equations are either from the hovercraft.pdf
	// or based on the declaration of Xd and X.
	
	Xd[0] = X[1]; 
	// dxb/dt = u
	
	Xd[1] = (U[0] + U[1] - c1*X[1]) / Mass + X[3] * X[5];
	// du/dt=(Fl+Fr - c1U)/M + v*r
	
	Xd[2] = X[3];
	// dyb/dt = v
	
	Xd[3] = (-c2 * X[3]) / Mass - X[1] * X[5];
	// dv/dt =-c2*v/M - u*r
	
	Xd[4] = X[5];
	// dyaw/dt = r;
	
	Xd[5] = (0.5*L*(U[1] - U[0]) - c3*X[3]) / J;
	// rd = (0.5*L*(Fr-Fl)-c3*r)/J
	
	Xd[6] = cos(X[4])*X[1] - sin(X[4])*X[3];
	// dxc/dt = cos(yaw)*u-sin(yaw)*v
	
	Xd[7] = sin(X[4])*X[1] + cos(X[4])*X[3];
	// dyc/dt = sin(yaw)*u + cos(yaw)*v
}

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
	eye_point[3] = 1.0; // z



	// the position you are looking at in global coord
	lookat_point[1] = 0.0 + H1.get_xy(6); // x
	lookat_point[2] = 0.0 + H1.get_xy(7); // y
	lookat_point[3] = 0.0; // z

	// the direction of the top of your head (note: up is a direction, not a point)
	up_dir[1] = 1.0; // dx
	up_dir[2] = 0.0; // dy 
	up_dir[3] = 0.0;;//dz

	set_view(eye_point, lookat_point, up_dir);
	//draw_XYZ(100.0);
	draw_XYZ(5.0);

	H1.eulers();
	H1.sim_step(0.008); 
	H1.input();
	H1.draw();


}

