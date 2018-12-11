#ifndef _BULLET_H_
#define _BULLET_H_

class Bullet {
private:
	double Stat_Var[6];
	//State variables
	// index 0 1 2 3     4   5
	// Var:  x y z pitch yaw roll

	double DStatV[6];
	//State variables derivatives
	// index 0  1  2  3      4    5
	// Var:  vx vy vz dpitch dyaw droll


public:

	mesh *p_bull;
	Bullet(){
		Stat_Var[0] = 0;
		Stat_Var[1] = 0;
		Stat_Var[2] = 0;
		Stat_Var[3] = 0;
		Stat_Var[4] = 0;
		Stat_Var[5] = 0;

		int Velocity = 100;

		DStatV[0] = sin(Stat_Var[4])*Velocity;
		DStatV[1] = cos(Stat_Var[4]) * Velocity;
		DStatV[2] = 0;
		DStatV[3] = 0;
		DStatV[4] = 0;
		DStatV[5] = 0;

		p_bull = new mesh("Bullet.x");
	}
	Bullet(double x, double y, double z, double pitch, double yaw, double roll){
		Stat_Var[0] = x;
		Stat_Var[1] = y;
		Stat_Var[2] = z;
		Stat_Var[3] = pitch;
		Stat_Var[4] = yaw;
		Stat_Var[5] = roll;

		int Velocity = 0.00001;

		DStatV[0] = sin(yaw)*Velocity;
		DStatV[1] = cos(yaw) * Velocity;
		DStatV[2] = 0;
		DStatV[3] = 0;
		DStatV[4] = 0;
		DStatV[5] = 0;

		p_bull = new mesh("Bullet.x");
	}

	void reset(double x, double y, double z, double pitch, double yaw, double roll){
		Stat_Var[0] = x;
		Stat_Var[1] = y;
		Stat_Var[2] = z + 1000;
		Stat_Var[3] = pitch;
		Stat_Var[4] = yaw + 3.14159/2;
		Stat_Var[5] = roll;

		int Velocity = 1;

		DStatV[0] = cos(yaw)*Velocity;
		DStatV[1] = sin(yaw) * Velocity;
		DStatV[2] = 0;
		DStatV[3] = 0;
		DStatV[4] = 0;
		DStatV[5] = 0;
	}

	void Euler(double dt){
		Stat_Var[0] = DStatV[0] * dt;
		Stat_Var[1] = DStatV[1] * dt;
		Stat_Var[2] = DStatV[2] * dt;
		Stat_Var[3] = DStatV[3] * dt;
		Stat_Var[4] = DStatV[4] * dt;
		Stat_Var[5] = DStatV[5] * dt;
	}

	double Get_SV(int index){
		return Stat_Var[index];
	}
};

#endif _BULLET_H_