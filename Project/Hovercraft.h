#ifndef _HOVERCRAFT_H_
#define _HOVERCRAFT_H_

#include "StateVar.h"

class Hovercraft{
private:
	//These are the state variables
	StateVar SV;

	//These are the forces produced by the two motors.
	double Fr, Fl;

public:

	void Set_Pos(double x, double y, double yaw){
		SV.Set_StatV(x, 6);
		SV.Set_StatV(y, 7);
		SV.Set_StatV(yaw, 4);
	}

	void Set_Param(double iM, double ic1, double ic2, double ic3, double iJ, double iL){
		SV.Set_Param(iM, ic1, ic2, ic3, iJ, iL);
	}

	double Get_Pos(){
		return{ SV.Get_StatV(6), SV.Get_StatV(7) };
	}
};

#endif