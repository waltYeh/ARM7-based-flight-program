#include "../Library/mathlib.h"
extern "C" void vector_test(void);
extern "C" void matrix_test(void);
extern "C" void alt_kalman_filter(float process_Q[3],float measure_R[3],
	float dt,float Resource[3],float x_out[3],
	char accFlag,char baroFlag,char gpsFlag);
static void inv(const float x[9], float y[9]);
using namespace math;
Matrix<3, 3> mat3_inv(const Matrix<3, 3> A);
Matrix<2, 2> mat2_inv(const Matrix<2, 2> A);
void vector_test(void)
{
	float r02=1, r12=2, r22=3;
	float v[3]={1,2,3};
	Vector<3> vec_init1;
	vec_init1(0) = 10;
	vec_init1(1) = 20;
	vec_init1(2) = 30;
	Vector<3> vec_init2(r02, r12, r22);
	Vector<3> vec_init3 = vec_init2 * 2;
	vec_init3.set(v);
	vec_init3 = vec_init1 % vec_init2;
}
void matrix_test(void)
{
	float R_array[3][3]={{1,0,0},{0,1,0},{0,0,1}};
	float r00,r10,r22;
	static Matrix<3, 3> R1;
	Matrix<3, 3> R2(R_array);
	Matrix<3, 3> R3;
	R3.set(R_array);
	Vector<3> vec_init1;
	vec_init1(0) = 10;
	vec_init1(1) = 20;
	vec_init1(2) = 30;
	Vector<3> vec_init2=R2*vec_init1;
	R3.anti_sym(vec_init1);
	r00=R2(0,0);
	r10=R2(1,0);
	r22=R2(2,2);
	r00=r00;
	r10=r10;
	r22=r22;
}
void alt_kalman_filter(float process_Q[3],float measure_R[3],
	float dt,float Resource[3],float x_out[3],
	char accFlag,char baroFlag,char gpsFlag)
{
	//Z_k acc, baro, gps
	//x a,v,z
	static Matrix<3, 3> P_apo;
	static Vector<3> x_apo;
	Matrix<3, 3> Q;	
	Q.zero();	
	for(int i=0;i<3;i++){
		Q(i,i) = process_Q[i];			
	}	
	Matrix<3, 3> A;
	A.identity();
	A(1,0) = dt;
	A(2,1) = dt;
	A(2,0) = dt*dt/2;
	Vector<3> x_apr = A * x_apo;
	Matrix<3, 3> P_apr = A * P_apo * A.transposed() + Q;
	if(accFlag == 1 && baroFlag == 1 && gpsFlag == 1 ){
		Matrix<3, 3> H;
		H.zero();
		H(0,0) = 1;
		H(1,2) = 1;
		H(2,2) = 1;
		Matrix<3, 3> R;
		R.zero();
		for(int i=0;i<3;i++)
			R(i,i) = measure_R[i];
		Matrix<3, 3> Inverted = mat3_inv(H * P_apr * H.transposed() + R);
    	Matrix<3, 3> K = P_apr * H.transposed() * Inverted;
		Vector<3> z_k(Resource);
    	x_apo = x_apr + K * (z_k - H * x_apr);
		Matrix<3, 3> I;
		I.identity();
    	P_apo = (I - K * H) * P_apr;	
	}
	else if(accFlag == 1 && baroFlag == 0 && gpsFlag == 0 ){
		Matrix<1, 3> H;
		H.zero();
		H(0,0) = 1;
		Matrix<1, 1> R;
		R.zero();
		R(0,0) = measure_R[0];
		Matrix<1, 1> Inverted = (H * P_apr * H.transposed() + R);
		Inverted(0,0) = 1/Inverted(0,0);
    	Matrix<3, 1> K = P_apr * H.transposed() * Inverted;
		Vector<1> z_k;
		z_k(0) = Resource[0];
    	x_apo = x_apr + K * (z_k - H * x_apr);
		Matrix<3, 3> I;
		I.identity();
    	P_apo = (I - K * H) * P_apr;
	}
	else if(accFlag == 1 && baroFlag == 1 && gpsFlag == 0 ){
		Matrix<2, 3> H;
		H.zero();
		H(0,0) = 1;
		H(1,2) = 1;
		Matrix<2, 2> R;
		R.zero();
		R(0,0) = measure_R[0];
		R(1,1) = measure_R[1];
		Matrix<2, 2> Inverted = mat2_inv(H * P_apr * H.transposed() + R);
    	Matrix<3, 2> K = P_apr * H.transposed() * Inverted;
		Vector<2> z_k;
		z_k(0) = Resource[0];
		z_k(1) = Resource[1];
    	x_apo = x_apr + K * (z_k - H * x_apr);
		Matrix<3, 3> I;
		I.identity();
    	P_apo = (I - K * H) * P_apr;
	}
	else if(accFlag == 1 && baroFlag == 0 && gpsFlag == 1 ){
		Matrix<2, 3> H;
		H.zero();
		H(0,0) = 1;
		H(1,2) = 1;
		Matrix<2, 2> R;
		R.zero();
		R(0,0) = measure_R[0];
		R(1,1) = measure_R[2];
		Matrix<2, 2> Inverted = mat2_inv(H * P_apr * H.transposed() + R);
    	Matrix<3, 2> K = P_apr * H.transposed() * Inverted;
		Vector<2> z_k;
		z_k(0) = Resource[0];
		z_k(1) = Resource[2];
    	x_apo = x_apr + K * (z_k - H * x_apr);
		Matrix<3, 3> I;
		I.identity();
    	P_apo = (I - K * H) * P_apr;
	}
	else{
		x_apo=x_apr;
        P_apo=P_apr;
	}
	for(int i=0;i<3;i++){
    	x_out[i] = x_apo(i);
	}
}
Matrix<2, 2> mat2_inv(const Matrix<2, 2> A)
{
	float inv_det = 1/(A(0,0) * A(1,1) - A(0,1) * A(1,0));
	Matrix<2, 2> B;
	B(0,0) = A(1,1)*inv_det;
	B(0,1) = -A(0,1)*inv_det;
	B(1,0) = -A(1,0)*inv_det;
	B(1,1) = A(0,0)*inv_det;
	return B;
}
Matrix<3, 3> mat3_inv(const Matrix<3, 3> A)
{
	float x[9],y[9];
	Matrix<3, 3> B;
	x[0] = A(0,0);
	x[1] = A(0,1);
	x[2] = A(0,2);
	x[3] = A(1,0);
	x[4] = A(1,1);
	x[5] = A(1,2);
	x[6] = A(2,0);
	x[7] = A(2,1);
	x[8] = A(2,2);
	inv(x, y);
	B(0,0) = y[0];
	B(0,1) = y[1];
	B(0,2) = y[2];
	B(1,0) = y[3];
	B(1,1) = y[4];
	B(1,2) = y[5];
	B(2,0) = y[6];
	B(2,1) = y[7];
	B(2,2) = y[8];
	return B;
}
static void inv(const float x[9], float y[9])
{
  float b_x[9];
  int p1;
  int p2;
  int p3;
  float absx11;
  float absx21;
  float absx31;
  int itmp;
  for (p1 = 0; p1 < 9; p1++) {
    b_x[p1] = x[p1];
  }

  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = (float)fabs(x[0]);
  absx21 = (float)fabs(x[1]);
  absx31 = (float)fabs(x[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = x[1];
    b_x[1] = x[0];
    b_x[3] = x[4];
    b_x[4] = x[3];
    b_x[6] = x[7];
    b_x[7] = x[6];
  } else {
    if (absx31 > absx11) {
      p1 = 6;
      p3 = 0;
      b_x[0] = x[2];
      b_x[2] = x[0];
      b_x[3] = x[5];
      b_x[5] = x[3];
      b_x[6] = x[8];
      b_x[8] = x[6];
    }
  }

  absx11 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx21 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= absx11 * b_x[3];
  b_x[5] -= absx21 * b_x[3];
  b_x[7] -= absx11 * b_x[6];
  b_x[8] -= absx21 * b_x[6];
  if ((float)fabs(b_x[5]) > (float)fabs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx21;
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }

  absx11 = b_x[5] / b_x[4];
  b_x[5] /= b_x[4];
  b_x[8] -= absx11 * b_x[7];
  absx11 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  y[p1] = ((1.0F - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  y[p1 + 1] = absx21;
  y[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0F - b_x[7] * absx11) / b_x[4];
  y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p2 + 1] = absx21;
  y[p2 + 2] = absx11;
  absx11 = 1.0F / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  y[p3 + 1] = absx21;
  y[p3 + 2] = absx11;
}
