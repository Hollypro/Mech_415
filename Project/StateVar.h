//This class defines what our state variables are.

#ifndef _STATEVAR_H_
#define _STATEVAR_H_

#include <cmath>

class StateVar{
private:
	double int StatV[8];
	//Defines our state variables
	//index# : 0  1 2  3 4   5 6  7
	//StateV : xb u yb v yaw r xc yc

	//Xb and yb are measured with respect to the hovercraft
	//u and v are their derivatives respectively
	//yaw is measured with respect to the yc axis in radians.
	//r is the derivative of yaw.
	//xc and yc are universal x and y.

	double int DStatV[5];
	//Holds the derivative of StatV
	//index# : 0 1  2 3  4 5  6  7
	//StateV : u au v av r ar vx vy

	double M, c1, c2, c3, J, L;
public:
	StateVar(){ // initializes our state variables to zero.
		StatVar[0] = 0;
		StatVar[1] = 0;
		StatVar[2] = 0;
		StatVar[3] = 0;
		StatVar[4] = 0;
		StatVar[5] = 0;
		StatVar[6] = 0;
		StatVar[7] = 0;

		DStatV[0] = 0;
		DStatV[1] = 0;
		DStatV[2] = 0;
		DStatV[3] = 0;
		DStatV[4] = 0;
		DStatV[5] = 0;
		DStatV[6] = 0;
		DStatV[7] = 0;

		M = 1;
		c1 = 1;
		c2 = 1;
		c3 = 1;
		J = 1;
		L = 1;
	}

	void Set_StatV(double value, int index){
		StatV[index] = value;
	}

	void Set_Param(double iM, double ic1, double ic2, double ic3, double iJ, double iL){
		M = iM;
		c1 = ic1;
		c2 = ic2;
		c3 = ic3;
		J = iJ;
		L = iL;
	}

	void Derivative(double Fl, double Fr){
		DStatV[0] = StatV[1];
		DStatV[1] = (Fl + Fr - c1*StatV[1] + StatV[3] * StatV[5]) / M;
		DStatV[2] = StatV[3];
		DStatV[3] = (-c2*StatV[3] - StatV[1] * StatV[5]) / M;
		DStatV[4] = StatV[5];
		DStatV[5] = (L / 2 * (Fl - Fr) - c3*StatV[5]) / J;
		DStatV[6] = cos(StatV[4])*StatV[1] - sin(StatV[4])*StatV[3];
		DStatV[7] = sin(StatV[4])*StatV[1] + cos(StatV[4])*StatV[3];
	}
};



#endif