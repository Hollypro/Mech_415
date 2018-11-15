#include <iostream>
#include <cmath>
#include <fstream>

void calculate_Eulers(double X[], double t, int N, double U[], int M);
void calculate_Xd(double Xd[],double X[], double t, int N, double U[], int M);

using namespace std;

int main() {


	const int N = 6;
	const int M = 2;
	double x[N+1];
	double xd[N+1];
	double u[M+1];
	double t;
	double dt;
	double tf;

	x[1]= 0; //u
	x[2]= 0; //v
	x[3]= 0; //r
	x[4]= 1.0;  //xc
	x[5]= 1.0; //yc
	x[6]= 3.14/6; //yaw

	t=0.0;
	dt=0.01;
	tf=1;

	ofstream fout("test.csv");

	while (t<tf){

		fout << t;

		for(int i = 1;i <= N; i++) fout << " , " << x[i];

		calculate_Eulers(x,t,N,u,M);
		calculate_Xd(xd,x,t,N,u,M);

		for(int i=1;i <= N;i++) x[i]=x[i]+xd[i]*t;

		t=t+dt;

		if(t<tf) fout <<endl;


	}


	return 0;
}

void calculate_Eulers(double X[], double t, int N, double U[], int M){

	double Fr,Fl;

	double u,v,r,xc,yc,yaw;




	u = X[1];//u
	v = X[2];//v
	r = X[3];//r
	xc = X[4];//xc
	yc = X[5];//yc
	yaw = X[6];//yaw

	Fr=1;

	Fl=0;

	U[1]=Fr;
	U[2]=Fl;

}

void calculate_Xd(double Xd[],double X[], double t, int N, double U[], int M){ //taking forces and everything else

	double c, J, Mass; //initialize constants

	c = 0.5; //friction coefficient assume all the same for now cause lazy <3

	J = 1.0;

	Mass = 1.0;

	Xd[1] = (U[1]+ U[2]-c*X[1])/Mass+X[2]*X[3];//ud=(Fl+Fr - cU)/M + v*r
	Xd[2] = -c*X[2]/M - X[1]*X[3];//vd=-c*v/M - u*r
	Xd[3] = (0.5*(U[2]-U[1])-c*X[3])/J ;//[rd = 0.5(Fr-Fl)-c*r]/J
	Xd[4] = cos(X[6])*X[1] - sin(X[6])*X[2];//xcd = cos(yaw)*u-sin(yaw)*v
	Xd[5] = sin(X[6])*X[1] - cos(X[6])*X[2];//yd = sin(yaw)*u - cos(yaw)*v
	Xd[6] = X[3];//yawd=r
}
