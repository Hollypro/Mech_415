#include <cmath>

class StateVar{
private:
	double StatV[8];

	//Defines our state variables

	//index# : 0  1 2  3 4   5 6  7

	//StateV : xb u yb v yaw r xc yc



	//Xb and yb are measured with respect to the hovercraft

	//u and v are their derivatives respectively

	//yaw is measured with respect to the yc axis in radians.

	//r is the derivative of yaw.

	//xc and yc are universal x and y.



	double DStatV[5];

	//Holds the derivative of StatV

	//index# : 0 1  2 3  4 5  6  7

	//StateV : u au v av r ar vx vy


	bool Input[2];
	//Holds the value for motors right and left.
	//The input is either high or low.
	//index : 0		1
	//motor : right	left

	double M, c1, c2, c3, J, L;
	//These are constants for our system.
	//Mass, damping ratios, moment of inertia, and distance between motors respectively.
public:
	StateVar(){ //Our default constructor.
		// initializes our state variables to zero and all constants to 1.

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

		Input[0] = false;
		
		Input [1] = false;


		M = 1;

		c1 = 1;

		c2 = 1;

		c3 = 1;

		J = 1;

		L = 1;

	}
	
	void Get_StatV(int index){

		return StatV[index]; //returns desired state var.

	}



	void Set_StatV(double value, int index){

		StatV[index] = value; //sets desired state var.

	}



	void Set_Param(double iM, double ic1, double ic2, double ic3, double iJ, double iL){

		M = iM; //sets constants.

		c1 = ic1;

		c2 = ic2;

		c3 = ic3;

		J = iJ;

		L = iL;

	}
};