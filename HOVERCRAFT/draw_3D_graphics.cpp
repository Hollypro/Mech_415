
#include <cmath> 
#include <cstdio>  
#include <cstring>
#include <iostream> 
#include <fstream>   
#include <strstream> 
#include <iomanip> 

#include <windows.h>

#include "timer.h" 
#include "rotation.h"
#include "3D_graphics.h" 

// 3D graphics window size in pixels
int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = 800;
int HEIGHT_MAX = 600;

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


	double X[6 + 1]; // Initialize all variables
	double Xd[6 + 1]; // Initialize all derivatives

public:

	double c, J, Mass; // initialize constants AKA drag coefficients, Inertia and Mass of Hovercraft
	
	

	double U[2 + 1]; // 2 inputs aka Fr and Fl
	

	mesh *p_mesh;

	Hovercraft(double *X, char *file_name);//constructor

	~Hovercraft();//destructor

	void draw(); //draw hovercraft

	void sim_step(double dt); //sim step

	void eulers(); //calculate eulers

	void input();// change the inputs to Fl or Fr to 1 or 0;



};


Hovercraft::Hovercraft(double *X, char *file_name) {

	c = 0.5; //Drag coefficients -.6
	J = 1; // inertia 1
	Mass = 1; // mass 1

	for (int i = 1; i <= 6; i++){ // initialize variables to 0

	this->X[i] = X[i];
	
		
	}

	for (int i = 1; i <= 2; i++){//itialize inputs to 0
		U[i] = 0;

	}

	p_mesh = new mesh(file_name);





}

Hovercraft::~Hovercraft(){
	delete p_mesh;
}





void Hovercraft::draw(){ //draw the hover craft

	p_mesh->Scale = 0.7;

	p_mesh->draw(X[4],X[5],0.0,X[6] + PI,0.0,PI/2); // roll is pi/2 because initial hovercraft wasn't well positioned

}


void Hovercraft::sim_step(double dt){ //sim step for eulers

	for (int i = 1; i <= 6; i++) X[i] = X[i] + Xd[i] * dt;




}

void Hovercraft::input(){
	{
	

		
		if (KEY(VK_UP)) {
			
		}

		//click down arrow key to reset position *** Change it if you can 
		if (KEY(VK_DOWN)) {
			for (int i = 1; i <= 6; i++) X[i] = { 0.0 };
		}

		// Fr goes to 1 so eulers function works
		if (KEY(VK_RIGHT))U[2] = 1.0; else{
			U[2] = 0.0;
		} 
			
			
		

		// Fl goes to 1
		if (KEY(VK_LEFT)) U[1] = 1.0; else{
			U[1] = 0.0;
		}
		

	}
}

void Hovercraft::eulers(){

	Xd[1] = (U[1] + U[2] - c*X[1]) / Mass + X[2] * X[3];//ud=(Fl+Fr - cU)/M + v*r
	Xd[2] = -c*X[2] / Mass - X[1] * X[3];//vd=-c*v/M - u*r
	Xd[3] = (0.5*(U[2] - U[1]) - c*X[3]) / J;//[rd = 0.5(Fr-Fl)-c*r]/J
	Xd[4] = cos(X[6])*X[1] - sin(X[6])*X[2];//xcd = cos(yaw)*u-sin(yaw)*v
	Xd[5] = sin(X[6])*X[1] - cos(X[6])*X[2];//yd = sin(yaw)*u - cos(yaw)*v
	Xd[6] = X[3];//yawd=r

}

void draw_3D_graphics(){

	static double x[6 + 1] = { 0.0 };
	static double xd[6 + 1] = { 0.0 };
	static char file_name[] = "hoverbus.x";

	static Hovercraft H1(x, file_name);

	set_view();
	draw_XYZ(5.0);

	H1.eulers();
	H1.sim_step(0.008); 
	H1.input();
	H1.draw();


}
