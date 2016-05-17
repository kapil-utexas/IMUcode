


#include "imu.h"
#include "retarget.h"
/*
**Chip description: ITG3205: triple-axis MEMS gyroscope ADXL345: Digital Accelerometer
**HMC5883L: 3-Axis Digital Compass IC
*/
//
#define ACC_FREQUENCY 12.5 //12.5hz
#define ACC_SAMPLE_TIME 0.08 //.08 sec
#define LAMBDA 0.9
#define RADIANS_2_DEGREE 180/3.14159265

short  ITG3205_X,ITG3205_Y,ITG3205_Z,ITG3205_T;
int16_t  ADXL345_X,ADXL345_Y,ADXL345_Z;
int16_t  HMC5883L_X,HMC5883L_Y,HMC5883L_Z;
float ADXL345_X_AXIS,ADXL345_Y_AXIS,ADXL345_Z_AXIS;
float conver_x,conver_y,conver_z;
float HMC5883L_ARC;
float normal_mat[3];
float scal_prod;
float vec_prod[3];
float Ri[3][3];
float Rg[3][3];
float outMat[3][3];
float outMat2[3][3];
float outMat3[3][3];
uint8_t initialize = 0;
float pitch,roll,yaw,temp;

float Ra_g[3][3];
float preRa_g[3][3];
/////////////////////////  ITG3205
//初始化ITG3205  ITG3205 Initialize
void Init_ITG3205(void)
{	
  Write_single_reg(ITG3205_Addr,ITG3205_PWR_M,0x00);  
	Write_single_reg(ITG3205_Addr,ITG3205_SMPL,0x07);  
	Write_single_reg(ITG3205_Addr,ITG3205_DLPF,0x1E); 
	Write_single_reg(ITG3205_Addr,ITG3205_INT_C,0x00);  
//	Write_single_reg(ITG3205_Addr,ITG3205_PWR_M,0x00);  
}
 
//Get ITG3205 X Y Z axis and temperature data
void READ_ITG3205_XYZT(void)
{
	uint8_t  data_l,data_h;
	//ITG3205 slave address
  data_l = Read_single_reg(ITG3205_Addr,ITG3205_WHO);
//	printf("ITG3205_Addr:%d\r\n ",data_l);
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GX_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GX_H);
//	printf("ITG3205_XL:%d ,ITG3205_XH:%d \r\n ",data_l,data_h);
	ITG3205_X = (data_h<<8)|data_l;
	ITG3205_X /= 14.375;
	if(ITG3205_X<0)  ITG3205_X = ITG3205_X;
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GY_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GY_H);
//	printf("ITG3205_YL:%d ,ITG3205_YH:%d \r\n ",data_l,data_h);
	ITG3205_Y = (data_h<<8)|data_l;
	ITG3205_Y /= 14.375;
	if(ITG3205_Y<0)  ITG3205_Y = ITG3205_Y;
	
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GZ_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GZ_H);
//	printf("ITG3205_ZL:%d ,ITG3205_ZH:%d \r\n ",data_l,data_h);
	ITG3205_Z = (data_h<<8)|data_l;
	ITG3205_Z /= 14.375;
	if(ITG3205_Z<0)  ITG3205_Z = ITG3205_Z;
	
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_TMP_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_TMP_H);
//	printf("ITG3205_TEMPL:%d ,ITG3205_TEMPH:%d \r\n ",data_l,data_h);
	ITG3205_T = (data_h<<8)|data_l;
	ITG3205_T =35+((double)ITG3205_T+13200)/280;
	if(ITG3205_T<0)  ITG3205_T = -ITG3205_T;
	temp = sqrt( pow(ITG3205_X,2)+pow(ITG3205_Y,2)+pow(ITG3205_Z,2));

printf("Gx:%d,Gy:%d,Gz:%d, Norm:%f\r\n ",ITG3205_X,ITG3205_Y,ITG3205_Z,temp);
	 	
}
//////////////////////////// end ITG3205

/////////////////////////  ADXL345
//初始化ADXL345  ADXL345 initialize
void Init_ADXL345(void)
{
  Write_single_reg(ADXL345_Addr,ADXL345_DATA_FORMAT,0x0B);  //setting the range, here it is set at +-16g with 0x0B
	Write_single_reg(ADXL345_Addr,ADXL345_BW_RATE,0x08);  //12.5Hz with 0x08,set up the data rate or sampling frequency
	Write_single_reg(ADXL345_Addr,ADXL345_POWER,0x08);  
	Write_single_reg(ADXL345_Addr,ADXL345_INT_EN,0x80);  
	Write_single_reg(ADXL345_Addr,ADXL345_OFSX,0x00);  
	Write_single_reg(ADXL345_Addr,ADXL345_OFSY,0x00);  
	Write_single_reg(ADXL345_Addr,ADXL345_OFSZ,0x05);  
}

//Get ADXL345 data information
void ADXL345_Read_XYZt(void)
{
  uint8_t  data_l,data_h;
	float temp;
	//ADXL345 slave address
  data_l = Read_single_reg(ADXL345_Addr,ADXL345_ID);
//	printf("ADXL345_Addr:%d\r\n ",data_l);
	
	
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_X0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_X1);
//	printf("ADXL345_XL:%d ,ADXL345_XH:%d \r\n ",data_l,data_h);
	//sgnX = 0x8000 & data_h;
	ADXL345_X = ((data_h<<8)|data_l);
	
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Y0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Y1);
//	printf("ADXL345_YL:%d ,ADXL345_YH:%d \r\n ",data_l,data_h);
	//sgnY = 0x8000 & data_h;
	ADXL345_Y = ((data_h<<8)|data_l);
	
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Z0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Z1);
//	printf("ADXL345_ZL:%d ,ADXL345_ZH:%d \r\n ",data_l,data_h);
	//sgnZ = 0x8000 & data_h;
	//ADXL345_Z = 0x0000FFFF & ((data_h<<8)|data_l);
	ADXL345_Z = ((data_h<<8)|data_l);
//	printf("ADXL345_X:%x,ADXL345_Y:%x,ADXL345_Z:%x\r\n ",ADXL345_X,ADXL345_Y,ADXL345_Z);
  
//	conver_x=(float)ADXL345_X*4; //3.9 to 4
		conver_x=(float)ADXL345_X*3.9; //3.9 to 4
		conver_y=(float)ADXL345_Y*3.9; //3.9 to 4
		conver_z=(float)ADXL345_Z*3.9; //3.9 to 4




//	conver_y=ADXL345_Y*4;  //float to int
	//conver_z=ADXL345_Z*4;
  //获取ADXL345 angle  ADXL345 angle
	//printf("#%d$%d$%d$",ADXL345_X,ADXL345_Y,ADXL345_Z);
	


temp = sqrt( pow(conver_x,2)+pow(conver_y,2)+pow(conver_z,2));
	printf("Ax:%f,Ay:%f,Az:%f,Norm:%f\r\n",conver_x,conver_y,conver_z,temp);//Actual Accel VAlues
// ADXL345_X_AXIS=(float)(((atan2(conver_z,conver_x)*180)/3.14159265)+180);    
  //ADXL345_Y_AXIS=(float)(((atan2(conver_z,conver_y)*180)/3.14159265)+180); 
//  printf("ADXL345_X_AXIS:%f,ADXL345_Y_AXIS:%f\r\n ",ADXL345_X_AXIS,ADXL345_Y_AXIS);
}	


//////////////////////////// end ADXL345


/////////////////////////  HMC5883L

//初始化HMC5883L  HMC5883L initialize
void Init_HMC5883L(void)
{
  Write_single_reg(HMC5883L_Addr,HMC5883L_configA,0x14);  
	Write_single_reg(HMC5883L_Addr,HMC5883L_MODE,0x00);  
}

void HMC5883L_Read_XYZt(void)
{
  uint8_t  data_l,data_h;
	
	//获取ITG3205 X轴  X_axle
	
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_XL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_XH);
	//	printf("HMC5883L_XL:%d ,HMC5883L_XH:%d \r\n ",data_l,data_h);
	HMC5883L_X = (data_h<<8)|data_l; 
	
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_YL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_YH);
	//	printf("HMC5883L_YL:%d ,HMC5883L_YH:%d \r\n ",data_l,data_h);
	HMC5883L_Y = (data_h<<8)|data_l;
	
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_ZL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_ZH);
//	printf("HMC5883L_ZL:%d ,HMC5883L_ZL:%d \r\n ",data_l,data_h);
	HMC5883L_Z = (data_h<<8)|data_l;
	//printf("Hx:%d,Hy:%d,Hz:%d\r\n ",HMC5883L_X,HMC5883L_Y,HMC5883L_Z);  
  
	if(HMC5883L_X>0x7fff) HMC5883L_X-=0xffff;	
	if(HMC5883L_Y>0x7fff) HMC5883L_Y-=0xffff;
	if(HMC5883L_Z>0x7fff) HMC5883L_Z-=0xffff;	
	
  HMC5883L_ARC=atan2(HMC5883L_Y,HMC5883L_X)*(180/3.14159265);    //Heading in degrees
  HMC5883L_ARC -= 4.033; // subtracting magnetic declination
	if(HMC5883L_ARC<0)
		HMC5883L_ARC += 360;
		
	if(HMC5883L_ARC > 360)
		HMC5883L_ARC -= 360 ;
	//printf("Heading:%f\r\n ",HMC5883L_ARC);
	//delay_ms();
	
}

//////////////////////////// end HMC5883L

/*Initialize the sensor module 
** imu_ic parameter value:  IMU_ITG3205、IMU_ADXL345、IMU_HMC5883L、IMU_ALL_IC
**  IMU_ITG3205 means ITG3205 initialize,IMU_ADXL345 means ADXL345 initialize
**  IMU_HMC5883L means HMC5883L initialize,IMU_ALL_IC means Three chips are initialized
*/
void IMU_INIT(uint8_t imu_ic)
{
	i2cInit();
	switch(imu_ic){
		case IMU_ITG3205: //ITG3205 initialize
			Init_ITG3205();
			break;
		case IMU_ADXL345: //ADXL345 initialize
			Init_ADXL345();
			break;
		case IMU_HMC5883L: //HMC5883L initialize
			Init_HMC5883L();
			break;
		case IMU_ALL_IC:  //All modules initialize
			Init_HMC5883L();
			Init_ADXL345();
			Init_ITG3205();
			break;
	}
}

//code for rotation matrix
 /*void IMU_EvaluateRotationMatrix(float a, float b, float c){
	 //get alpha, beta and gamma
	 
	 float Ra[3][3];
	 Ra[0][0] = (float)(cos(a)) * (float)(cos(b)) ;
	 Ra[0][1] = (float)cos(a)*(float)sin(b)*(float)sin(c) - (float)sin(a)*(float)cos(c);
	 Ra[0][2] = (float)cos(a)*(float)sin(b)*(float)cos(c) - (float)sin(a)*(float)sin(c);
	 Ra[1][0] = (float)sin(a)*(float)cos(b);
	 Ra[1][1] = (float)sin(a)*(float)sin(b)*(float)sin(c) + (float)cos(a)*(float)cos(c);;
	 Ra[1][2] = (float)sin(a)*(float)sin(b)*(float)cos(c) - (float)cos(a)*(float)sin(c);;
	 Ra[2][0] = (float)(-sin(b));
	 Ra[2][1] = (float)cos(b)*(float)sin(c);
	 Ra[2][2] = (float)cos(b)*(float)cos(c);
 }*/

/*code for evaluating rotation matrix in IMU frame of reference
the gyro component is calculated just as a cross product of the mag and acc data

*/
//float Ri[3][3];
void evaluate_rotation_matrix_imu(void){
	float ax,ay,az,hx,hy,hz,kx,ky,kz,temp_x,temp_y,temp_z, temp_hx,temp_hy,temp_hz;
	int i,j;
	//accelerometer vector
	
	normalize(conver_x/1000,conver_y/1000,conver_z/1000);
	 ax = normal_mat[0];
	 ay = normal_mat[1];
	 az = normal_mat[2];

	//magnetometer vector
	temp_hx = (float)(HMC5883L_X ) / 1000;
	temp_hy = (float)(HMC5883L_Y ) /1000;
	temp_hz = (float)(HMC5883L_Z ) /1000;
	//printf("BEFORE %f, %f, %f\r\n",temp_hx,temp_hy,temp_hz);

	scalar_product(conver_x/1000,conver_y/1000,conver_z/1000,temp_hx,temp_hy,temp_hz);
	//printf("SCALAR PRODUCT %f\r\n",scal_prod);	
	temp_x = scal_prod * conver_x/1000;
	 temp_y = scal_prod * conver_y/1000;
	 temp_z = scal_prod * conver_z/1000;
	//printf("TEMP %f, %f, %f\r\n",temp_x,temp_y,temp_z);
	
	hx = temp_hx - temp_x;
	 hy = temp_hy - temp_y;
	 hz = temp_hz - temp_z;
	//printf("AFTER %f, %f, %f\r\n",hx,hy,hz);
	
	normalize(hx,hy,hz);
	hx = normal_mat[0];
	hy = normal_mat[1];
	hz = normal_mat[2];
	//printf("AFTER NORMALIZE %f, %f, %f\r\n",hx,hy,hz);
	
	// vector perpendicular to both h and a vector
	vector_product(hx,hy,hz,ax,ay,az);
	 kx = vec_prod[0];
	 ky = vec_prod[1];
	 kz = vec_prod[2];
	
	normalize(kx,ky,kz);
	kx=normal_mat[0];
	ky=normal_mat[1];
	kz=normal_mat[2];
	
	Ri[0][0] = hx;
	Ri[1][0] = hy;
	Ri[2][0] = hz;
	
	Ri[0][1] = kx;
	Ri[1][1] = ky;
	Ri[2][1] = kz;
	
	Ri[0][2] = ax;
	Ri[1][2] = ay;
	Ri[2][2] = az;

//calculate RM for gyro or Rg
	Rg[0][0] = 1;
	Rg[1][0] = az*ACC_SAMPLE_TIME;
	Rg[2][0] = -1*ay*ACC_SAMPLE_TIME;
	
	Rg[0][1] = -1*az*ACC_SAMPLE_TIME;
	Rg[1][1] = 1;
	Rg[2][1] = ax*ACC_SAMPLE_TIME;
	
	Rg[0][2] = ay*ACC_SAMPLE_TIME;
	Rg[1][2] = -1*ax*ACC_SAMPLE_TIME;
	Rg[2][2] = 1;
	
	
//		if(initialize ==0 )	
//		{
//			for (i=0;i<3;i++)
//        for(j=0;j<3;j++)
//					preRa_g[i][j] = Ri[i][j] ;
//			initialize = 1;
//		}
//	printf("DCM Gyro Matrix\r\n ");
//	printf("%f, %f, %f\r\n",Rg[0][0],Rg[0][1],Rg[0][2]);
//	printf("%f, %f, %f\r\n",Rg[1][0],Rg[1][1],Rg[1][2]);
//	printf("%f, %f, %f\r\n",Rg[2][0],Rg[2][1],Rg[2][2]);
//	printf("%f, %f, %f\r\n",hx,kx,ax);
//	printf("%f, %f, %f\r\n",hy,ky,ay);
//	printf("%f, %f, %f\r\n",hz,kz,az);

}

void normalize(float vec_x,float vec_y,float vec_z){
	//float normal[3];
	float vec_mag;
	
	vec_mag = sqrt( pow(vec_x,2)+pow(vec_y,2)+pow(vec_z,2));
	//printf("VECMAG:%f\r\n ",vec_mag);
	normal_mat[0] = vec_x/vec_mag;
	normal_mat[1] = vec_y/vec_mag;
	normal_mat[2] = vec_z/vec_mag;
	//printf("Normalize\r\n ");
	//printf("%f, %f, %f, %f\r\n",vec_mag,normal_mat[0],normal_mat[1],normal_mat[2]);

}

void scalar_product(float vec_x1,float vec_y1,float vec_z1,float vec_x2,float vec_y2,float vec_z2){
	scal_prod = vec_x1*vec_x2 + vec_y1*vec_y2 + vec_z1*vec_z2;
}

void vector_product(float vec_x1,float vec_y1,float vec_z1,float vec_x2,float vec_y2,float vec_z2){
	vec_prod[0] = vec_y1*vec_z2 - vec_z1*vec_y2;
	vec_prod[1] = -vec_x1*vec_z2 + vec_z1*vec_x2;
	vec_prod[2] = vec_x1*vec_y2 - vec_y1*vec_x2;
//	printf("%f, %f, %f\r\n",vec_prod[0],vec_prod[1],vec_prod[2]);
}

//Linear Filter
void linear_filter(){
	int i,j;
	float temp[3][3];
	//float in1[3][3] = {{1,1,1},{1,1,1},{1,1,1}};
	//float in2[3][3] = {{1,1,1},{1,1,1},{1,1,1}};
	matrix_multiply(preRa_g,Rg); //answer in outMat2
//	for (i=0;i<3;i++){
//        for(j=0;j<3;j++){
//					temp[i][j] = outMat2[i][j];
//          //printf("%f\r\n",outMat[i][j]);
//        } 
//    }

	scalar_multiply(temp,LAMBDA);//answer is in outMat
	for (i=0;i<3;i++){
        for(j=0;j<3;j++){
					temp[i][j] = outMat[i][j];
          //printf("%f\r\n",outMat[i][j]);
        } 
    }

	scalar_multiply(Ri,1-LAMBDA);
    for (i=0;i<3;i++){
        for(j=0;j<3;j++){
					//preRa_g[i][j] = Ra_g[i][j];
					Ra_g[i][j] = temp[i][j] + outMat[i][j];
					preRa_g[i][j] = Ra_g[i][j];
					
			}
		}
//			printf("DCM after filter\r\n ");
//	printf("%f, %f, %f\r\n",Ra_g[0][0],Ra_g[0][1],Ra_g[0][2]);
//	printf("%f, %f, %f\r\n",Ra_g[1][0],Ra_g[1][1],Ra_g[1][2]);
//	printf("%f, %f, %f\r\n",Ra_g[2][0],Ra_g[2][1],Ra_g[2][2]);
//	printf("%f, %f, %f\r\n",outMat2[0][0],outMat2[0][1],outMat2[0][2]);
//	printf("%f, %f, %f\r\n",outMat2[1][0],outMat2[1][1],outMat2[1][2]);
//	printf("%f, %f, %f\r\n",outMat2[2][0],outMat2[2][1],outMat2[2][2]);

}

//Matrix Multiply
void matrix_multiply(float input1[3][3],float input2[3][3]){
	int i,j,k;
	float sum=0;
//    for (i=0;i<3;i++)
//        for(j=0;j<3;j++)
//					outMat2[i][j] = 0;
    for (i=0;i<3;i++){
        for(j=0;j<3;j++){
                for(k=0;k<3;k++){
                sum = sum + input1[i][k] * input2[k][j];
            } 
				//printf("%f\r\n",sum);
				outMat2[i][j] = sum;
				sum=0;			
        } 
    }
		//			printf("DCM after filter\r\n ");
//	printf("%f, %f, %f\r\n",outMat2[0][0],outMat2[0][1],outMat2[0][2]);
//	printf("%f, %f, %f\r\n",outMat2[1][0],outMat2[1][1],outMat2[1][2]);
//	printf("%f, %f, %f\r\n",outMat2[2][0],outMat2[2][1],outMat2[2][2]);
		

}

void scalar_multiply(float input[3][3] ,float lambda){
	int i,j;
	//float **out=0;
	for (i=0;i<3;i++){
        for(j=0;j<3;j++){
					outMat[i][j] = input[i][j] * lambda;
          //printf("%f\r\n",outMat[i][j]);
        } 
    }
	
}

void calculate_p_r_y(){
	float temp;
	yaw = atan2(Ra_g[1][0],Ra_g[0][0] )*RADIANS_2_DEGREE;
	roll = atan2(Ra_g[2][1],Ra_g[2][2])*RADIANS_2_DEGREE;
	
	temp = sqrt( pow(Ra_g[2][1],2)+pow(Ra_g[2][2],2));
	pitch = atan2(-1*Ra_g[2][0],temp)*RADIANS_2_DEGREE;
	//printf("pitch = %f, roll = %f, yaw = %f\r\n",pitch,roll,yaw);
}





