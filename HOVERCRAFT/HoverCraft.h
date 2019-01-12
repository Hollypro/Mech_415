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

	int Velocity;

	double BStat_Var[6];
	//Bullet State variables
	// index 0 1 2 3     4   5
	// Var:  x y z pitch yaw roll

	double BDStatV[6];
	//Bullet State variables derivatives
	// index 0  1  2  3      4    5
	// Var:  vx vy vz dpitch dyaw droll



	double Tar_Pos[2];

	double Fol_Stat_V[3];
	//index# : 0 1 2
	//StateV : x y yaw

	double Fol_DStatV[3];
	//index# : 0 1 2
	//StateV : u v r

	double Fol_Velocity;

public:

	int count;

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
	mesh *p_arrow;

	Hovercraft(double *X, char *file_name);//constructor

	~Hovercraft();//destructor

	void draw(); //draw hovercraft

	void sim_step(double dt); //sim step

	void eulers(); //calculate eulers

	void input();// change the inputs to Fl or Fr to 1 or 0;

	void text();//adds text and crosshairs
	
	
	double get_xy(int index);

	double get_Bullxy(int index);

	

};
