#include "traj.h"

using namespace std;
using namespace Eigen;



polymat poly;
// Vel initvel;
Parameters param;

// Makes a polynomial vector for (pos, vel, acc, jerk, snap,......)
Eigen::MatrixXd calcTimepersegmentAvgVel(Eigen::MatrixXd x, Eigen::MatrixXd y, Eigen::MatrixXd z,Eigen::MatrixXd psi, int n, double avgvel)
{
	Eigen::MatrixXd temp1, temp2, temp3;
	temp1.resize(n+1,3) ;
  temp1 << x.adjoint(),y.adjoint(),z.adjoint() ;
	temp2 = temp1.block(1,0,n,3) - temp1.block(0,0,n,3) ; // diff(temp1) ;
	temp3 = temp2.array().square().rowwise().sum().array().sqrt() ; // (sqrt(sum(temp2.^2,2)))'

  Eigen::Matrix<double, 1, Dynamic> T;
	T.resize(1,n);
	T = temp3.adjoint() / avgvel ;

	return T ;
}

Eigen::MatrixXd polynom_p(double Ta, double s, double t)
{
	poly.p.setZero(1, 8);
	for (int i = 0; i < 8; i++)	{
		poly.p(0, i) = pow(((t - s) / Ta), i);
	}
	return poly.p;
}

Eigen::MatrixXd polynom_pd(double Ta, double s, double t)
{

	poly.pd.setZero(1, 8);

	for (int i = 0; i < 8; i++)
	{
		int power;
		power = i - 1;

		if (power < 0)
		{
			power = 0;
		}

		poly.pd(0, i) = i * pow(((t - s) / Ta), power) / (pow(Ta, 1));
	}
	return poly.pd;
}

Eigen::MatrixXd polynom_pd2(double Ta, double s, double t)
{

	poly.pd2.setZero(1, 8);

	for (int i = 0; i < 8; i++)
	{
		int power;
		power = i - 2;

		if (power < 0)
		{
			power = 0;
		}

		poly.pd2(0, i) = i * (i - 1) * pow(((t - s) / Ta), power) / (pow(Ta, 2));
	}

	return poly.pd2;
}

Eigen::MatrixXd polynom_pd3(double Ta, double s, double t)
{

	poly.pd3.setZero(1, 8);

	for (int i = 0; i < 8; i++)
	{
		int power;
		power = i - 3;

		if (power < 0)
		{
			power = 0;
		}
		poly.pd3(0, i) = i * (i - 1) * (i - 2) * pow(((t - s) / Ta), power) / (pow(Ta, 3));
	}
	return poly.pd3;

}

Eigen::MatrixXd polynom_pd4(double Ta, double s, double t)
{

	poly.pd4.setZero(1, 8);

	for (int i = 0; i < 8; i++)
	{
		int power;
		power = i - 4;

		if (power < 0)
		{
			power = 0;
		}
		poly.pd4(0, i) = i * (i - 1) * (i - 2) * (i - 3) * pow(((t - s) / Ta), power) / (pow(Ta, 4));
	}

	return poly.pd4;
}

Eigen::MatrixXd polynom_pd5(double Ta, double s, double t)
{

	poly.pd5.setZero(1, 8);

	for (int i = 0; i < 8; i++)
	{
		int power;
		power = i - 5;

		if (power < 0)
		{
			power = 0;
		}
		poly.pd5(0, i) = i * (i - 1) * (i - 2) * (i - 3) * (i - 4) * pow(((t - s) / Ta), power) / (pow(Ta, 5));
	}
	return poly.pd5;
}

Eigen::MatrixXd polynom_pd6(double Ta, double s, double t)
{

	poly.pd6.setZero(1, 8);

	for (int i = 0; i < 8; i++)
	{
		int power;
		power = i - 6;

		if (power < 0)
		{
			power = 0;
		}
		poly.pd6(0, i) = i * (i - 1) * (i - 2) * (i - 3) * (i - 4) * (i - 5) * pow(((t - s) / Ta), power) / (pow(Ta, 6));
	}
	return poly.pd6;
}

////////////////////////////////////////////////////////////////////////////////////
// Constructing A Matrix

	Eigen::MatrixXd make_A1 (int n, MatrixXd T, MatrixXd S)
	{
int counter = 0 ;
Eigen::Matrix<double, Dynamic, 8> adash;
adash.resize(2*n,8);
Eigen::Matrix<double, Dynamic, Dynamic> adash2, adash4;
adash2.resize(2,adash.cols()); 
for (int i = 0; i < n; i++){
  for (int idx = i; idx < i+2; idx++)
  {
    adash.row(counter) = polynom_p(T(0,i),S(0,i),S(0,idx)) ;
    counter++ ;
  }
  

  if (i == 0)
  {
	adash2.resize(2,adash.cols()) ; 
    adash2 = adash.middleRows(2*i,2) ;
  }
  else
  {
	Eigen::Matrix<double, Dynamic, Dynamic> zer1,zer2;
	adash4.resize(2,adash.rows()) ;
	adash4 = adash.middleRows(2*i,2) ;

  
	//   Rows and columns of adash2 to construct block diag
	int n1 = adash2.rows() ;
	int m1 = adash2.cols() ;
 
	// 	 Rows and columns of adash4 to construct block diag
	int n2 = adash4.rows() ;
	int m2 = adash4.cols() ;

	zer1.setZero(n1,m2) ;
	zer2.setZero(n2,m1) ;
	Eigen::MatrixXd adash1 ;
	adash1.resize(n1+n2,m1+m2) ;

	adash1.setZero(n1+n2,m1+m2) ;

	adash1 << adash2, zer1,		// (Way to calc bldiag)
			  zer2,  adash4 ;
    adash2.resize(n1+n2,m1+m2) ;
	adash2<<adash1 ;
  }
  
}
return adash2 ;
}

Eigen::MatrixXd make_A2 (int n, MatrixXd T, MatrixXd S)
{
  Eigen::Matrix<double, Dynamic, Dynamic> adash3 ;
  
  adash3.setZero(6,8*n) ;

  adash3.topLeftCorner(3,8) << polynom_pd(T(0,0),S(0,0),S(0,0)),
                              polynom_pd2(T(0,0),S(0,0),S(0,0)),
                              polynom_pd3(T(0,0),S(0,0),S(0,0));

  adash3.bottomRightCorner(3,8) << polynom_pd(T(0,n-1),S(0,n-1),S(0,n)),
                                  polynom_pd2(T(0,n-1),S(0,n-1),S(0,n)),
                                  polynom_pd3(T(0,n-1),S(0,n-1),S(0,n));

  return adash3 ; 
}

Eigen::MatrixXd make_A3 (int n, MatrixXd T, MatrixXd S)
{
  Eigen::MatrixXd adash5 ;
  adash5.setZero(6*n-6, 8*n) ;
  Eigen::MatrixXd temp ;
	  	  

  if (n!=1)
  {
    for (int k = 0; k < n-1; k++)
    {
	  temp.setZero(6,16) ;
	  temp << polynom_pd (T(0,k),S(0,k),S(0,k+1)), -polynom_pd(T(0,k+1),S(0,k+1),S(0,k+1)),
			  polynom_pd2(T(0,k),S(0,k),S(0,k+1)), -polynom_pd2(T(0,k+1),S(0,k+1),S(0,k+1)),
			  polynom_pd3(T(0,k),S(0,k),S(0,k+1)), -polynom_pd3(T(0,k+1),S(0,k+1),S(0,k+1)),
			  polynom_pd4(T(0,k),S(0,k),S(0,k+1)), -polynom_pd4(T(0,k+1),S(0,k+1),S(0,k+1)),
			  polynom_pd5(T(0,k),S(0,k),S(0,k+1)), -polynom_pd5(T(0,k+1),S(0,k+1),S(0,k+1)),
			  polynom_pd6(T(0,k),S(0,k),S(0,k+1)), -polynom_pd6(T(0,k+1),S(0,k+1),S(0,k+1));
	  adash5.block(6*k,8*k,6,16) << adash5.block(6*k,8*k,6,16) +  temp ;
    }
    
  }

  return adash5 ;

}

Eigen::MatrixXd make_A_way1 (Eigen::MatrixXd adash2, Eigen::MatrixXd adash3, Eigen::MatrixXd adash5, int n)
{
	Eigen::MatrixXd A ;
	A.resize(8*n,8*n) ;
	A << adash2,
		 adash3,
		 adash5 ;
 return A ;
}

Eigen::MatrixXd make_A_way2 (Eigen::MatrixXd adash2, Eigen::MatrixXd adash3, int n)
{
	Eigen::MatrixXd A ;
	A.resize(8*n,8*n) ;
	A << adash2 ,
		 adash3 ;
 return A ;
}

Eigen::MatrixXd make_B (int n, Eigen::MatrixXd x, Eigen::MatrixXd y,Eigen::MatrixXd z,Eigen::MatrixXd psi, Vel initvel)
{
    Eigen::MatrixXd B_new ;
    B_new.setZero(8*n,4) ;
    int cnt = 1;
    for(int i = 0 ; i < n-1; i++){
        B_new.middleRows(2*i+1,2) << x(0,cnt), y(0,cnt), z(0,cnt), psi(0,cnt) ,
                                x(0,cnt), y(0,cnt), z(0,cnt), psi(0,cnt) ;
        cnt = cnt+1 ;	
    }
    B_new.topRows(1)      << x(0,0), y(0,0), z(0,0), psi(0,0) ;
    B_new.middleRows(2*n-1,1) << x(0,n), y(0,n), z(0,n), psi(0,	n) ;

    B_new.middleRows(2*n,1) << initvel.vx, initvel.vy, initvel.vz, initvel.psid ;

    return B_new ;
}

Eigen::MatrixXd construct_S(Eigen::MatrixXd T, int n, double t0)
{
	double st = t0;
	Eigen::MatrixXd S;
	S.resize(1, n+1);
	S(0,0) = st;

	for (int i = 0; i < n; i++)	{
		S(0,i+1) = st + T(0,i) ;
		st = S(0,i+1);
	}
	return S;
	
}

Eigen::MatrixXd compute_sz(Eigen::MatrixXd S,int var,double gap, int n)
{
	Eigen::MatrixXd sz ;
	// sz.resize(1,n+1) ;
	sz.setZero(1,n+1) ;
	

	for(int ids = 0 ; ids < n; ids++){
		sz(0,ids+1) = sz(0,ids) + floor((S(0,ids+1)-S(0,ids))/gap) ;
		// sz(0,ids+1) =  sz(0,ids) ;//floor((S(0,ids+1)-S(0,ids))/gap) ;
	}

    return sz;
}

void func_gener_v2(Eigen::MatrixXd sz,Eigen::MatrixXd coef,Eigen::MatrixXd S,int idt,int idx,double gap,Eigen::MatrixXd T, Eigen::MatrixXd& t, Eigen::MatrixXd& pos2_p, Eigen::MatrixXd& pos2_pd, Eigen::MatrixXd& pos2_pd2, Eigen::MatrixXd& pos2_pd4)
{
	double gap2 ;

	// if (idx!=0)
	// {
		gap2 = gap ;
	// }
	// else
	// {
	// 	gap2 = 0 ;
	// }

	Eigen::VectorXd timet ;
	double hi = S(idx+1) ;
	double low = S(idx) + gap2 ;
	double step = gap ;
	double sizet = ((hi-low)/step)+1 ;
	timet.setLinSpaced(sizet,low,hi) ;

	int niter = timet.size() ;
	Eigen::MatrixXd temp_p, temp_pd, temp_pd2, temp_pd4;
	
	temp_p.setZero(1,niter) ;
	temp_pd.setZero(1,niter) ;
	temp_pd2.setZero(1,niter) ;
	temp_pd4.setZero(1,niter) ;

	Eigen::Matrix<double, 1, 8> tempa, tempb, tempc, tempd ;
	Eigen::Matrix<double, 8, 1> temp2 ;


	for (int idu = 0; idu < niter; idu++)
	{
		tempa << polynom_p(T(0,idx),S(0,idx),timet(idu)) ;
		tempb << polynom_pd(T(0,idx),S(0,idx),timet(idu)) ;
		tempc << polynom_pd2(T(0,idx),S(0,idx),timet(idu)) ;
		tempd << polynom_pd4(T(0,idx),S(0,idx),timet(idu)) ;
		
		temp2 << coef.block(8*idx,idt,8,1) ;
		temp_p(0,idu)   = tempa*temp2 ;
		temp_pd(0,idu)  = tempb*temp2 ;
		temp_pd2(0,idu) = tempc*temp2 ;
		temp_pd4(0,idu) = tempd*temp2 ;
	}
	
	 pos2_p.block(idt,sz(0,idx),1,sz(0,idx+1)-sz(0,idx)) << temp_p ;
	 pos2_pd.block(idt,sz(0,idx),1,sz(0,idx+1)-sz(0,idx)) << temp_pd ;
	 pos2_pd2.block(idt,sz(0,idx),1,sz(0,idx+1)-sz(0,idx)) << temp_pd2 ;
	 pos2_pd4.block(idt,sz(0,idx),1,sz(0,idx+1)-sz(0,idx)) << temp_pd4 ;

	 t.block(0,sz(0,idx),1,sz(0,idx+1)-sz(0,idx)) << timet.adjoint() ;
	
}

void func_vars_generatv2(Eigen::MatrixXd S, Eigen::MatrixXd T,double t0, Eigen::MatrixXd coef, double gap, int n, int idt,Eigen::MatrixXd sz, Eigen::MatrixXd& t, Eigen::MatrixXd& pos2_p, Eigen::MatrixXd& pos2_pd, Eigen::MatrixXd& pos2_pd2, Eigen::MatrixXd& pos2_pd4)
{
	Eigen::MatrixXd timet ;
	int idx = 0 ;

	for (int idx = 0; idx < n; idx++)
	{
		 func_gener_v2(sz, coef, S, idt, idx, gap, T, t, pos2_p, pos2_pd, pos2_pd2, pos2_pd4) ;
	}
	// return timet ;

}

Eigen::MatrixXd my_atan2(Eigen::MatrixXd y, Eigen::MatrixXd x)
{
	int lnt = x.cols();
	Eigen::MatrixXd res;
	res.resize(1,lnt);
	for (int i = 0; i < lnt; i++) {
		res(0,i) = atan2(y(0,i),x(0,i)) ;
	}
	return res ;
}

void func_inputs4(Eigen::MatrixXd pos2_p,Eigen::MatrixXd pos2_pd,Eigen::MatrixXd pos2_pd2,Eigen::MatrixXd pos2_pd4,Eigen::MatrixXd& theta,Eigen::MatrixXd& phi2,Eigen::MatrixXd& fz1, Parameters param)
{
	Eigen::MatrixXd beta_a, beta_b, beta_c, temp1, temp2, temp3, temp4, temp6, temp7, temp8 , temp9, temp10, temp11, g_arr;

	int colpos2_p = pos2_p.cols() ;

	temp1 = pos2_p.block(3 ,0 ,1, colpos2_p);   // p.psi
	temp2 = pos2_pd2.block(0, 0, 1, colpos2_p); // p.xd2
	temp3 = pos2_pd2.block(1, 0, 1, colpos2_p); // p.yd2

	temp4 = temp1.array().cos();
	temp7 = temp1.array().sin();

	beta_a = -temp4.cwiseProduct(temp2) - temp7.cwiseProduct(temp3);

	g_arr.resize(1,colpos2_p);
	g_arr.fill(param.g);

	temp9 = -pos2_pd2.block(2, 0, 1, colpos2_p); // p.zd2
	beta_b = temp9 + g_arr ;

	beta_c = -temp7.cwiseProduct(temp2) + temp4.cwiseProduct(temp3);

	theta << my_atan2(beta_a, beta_b);

	temp10 = beta_a.array().square() + beta_b.array().square();
	temp11 = temp10.array().sqrt();

	phi2 << my_atan2(beta_c, temp11) ;

	fz1 << temp2, temp3, -beta_b;

}

void mellinger_kumarplanner(Eigen::MatrixXd T, int n,double t0, Eigen::MatrixXd x, Eigen::MatrixXd y, Eigen::MatrixXd z, Eigen::MatrixXd psi, Vel initvel,Eigen::MatrixXd& pos2_p, Eigen::MatrixXd& pos2_pd, Eigen::MatrixXd& pos2_pd2, Eigen::MatrixXd& pos2_pd4, Eigen::MatrixXd& t, Eigen::MatrixXd& theta, Eigen::MatrixXd& phi2, Eigen::MatrixXd& fz1, Parameters param, int freq)
{
	Eigen::MatrixXd S = construct_S(T, n, t0);
	Eigen::MatrixXd adash2 = make_A1(n, T, S);
	Eigen::MatrixXd adash3 = make_A2(n, T, S);
	Eigen::MatrixXd adash5 = make_A3(n, T, S);
	Eigen::MatrixXd A;
	if(n!=1) {
		 A = make_A_way1(adash2, adash3, adash5, n) ;
	}
	else {
		A = make_A_way2(adash2, adash5, n) ;
	}

	Eigen::MatrixXd b = make_B (n, x, y, z, psi, initvel);
	Eigen::MatrixXd coef = A.lu().solve(b) ;

	int var = 4;

	double gap = 1.0 / freq;

	Eigen::MatrixXd sz = compute_sz(S,var,gap,n);

	pos2_p.setZero(var,sz(0,n));
	pos2_pd.setZero(var,sz(0,n));
	pos2_pd2.setZero(var,sz(0,n));
	pos2_pd4.setZero(var,sz(0,n));

	t.setZero(1,sz(0,n));

	Eigen::MatrixXd timet;

	for(int idt = 0; idt< var; idt++) {
		func_vars_generatv2(S,T,t0,coef,gap,n,idt,sz,t, pos2_p, pos2_pd, pos2_pd2, pos2_pd4);		// Generate 
	}

	theta.setZero(1,sz(0,n));
	phi2.setZero(1,sz(0,n));
	fz1.setZero(3,sz(0,n));

	func_inputs4(pos2_p, pos2_pd, pos2_pd2, pos2_pd4, theta, phi2, fz1, param);
}

void traj_calc(double pos[3], double vel[3], double psi0, Eigen::MatrixXd& theta, Eigen::MatrixXd& phi2, Eigen::MatrixXd& fz1)
{
	double t0 = 0;
	int n = 3 ;
	double avgvel = 4;

	Eigen::MatrixXd x, y, z, psi ;
	x.resize(1,n+1);
	y.resize(1,n+1);
	z.resize(1,n+1);
	psi.resize(1,n+1);
	
	x << pos[0], 2.329, 2.199,  -7.308;
	y << pos[1], 27.86, 9.001, -12.136;
	z << pos[2], 2.546, 1.993,   3.229;
	psi << psi0,0,0,0;
	
	// psi.fill(-3.142/2);
	
	Eigen::MatrixXd T = calcTimepersegmentAvgVel(x,y,z,psi,n, avgvel) ;
    Vel initvel;
	initvel.vx   = vel[0];
	initvel.vy   = vel[1];
    initvel.vz   = vel[2];
	initvel.psid = 0;


	int freq = 960;
	param.g = 9.81;
	Eigen::MatrixXd pos2_p, pos2_pd, pos2_pd2, pos2_pd4, t;
	
	
	mellinger_kumarplanner(T, n, t0, x,y,z,psi, initvel, pos2_p, pos2_pd, pos2_pd2, pos2_pd4, t, theta, phi2, fz1, param, freq);
}