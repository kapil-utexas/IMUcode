


#include "imu.h"
#include "retarget.h"
/*芯片描述: ITG3205:三轴MEMS陀螺仪 ADXL345:数字加速度计 HMC5883L:数字罗盘
**Chip description: ITG3205: triple-axis MEMS gyroscope ADXL345: Digital Accelerometer
**HMC5883L: 3-Axis Digital Compass IC
*/
//
short  ITG3205_X,ITG3205_Y,ITG3205_Z,ITG3205_T;
int16_t  ADXL345_X,ADXL345_Y,ADXL345_Z;
int16_t  HMC5883L_X,HMC5883L_Y,HMC5883L_Z;
float ADXL345_X_AXIS,ADXL345_Y_AXIS,ADXL345_Z_AXIS;
float conver_x,conver_y,conver_z;
float HMC5883L_ARC;
float normal_mat[3];
/////////////////////////  ITG3205
//初始化ITG3205  ITG3205 Initialize
void Init_ITG3205(void)
{	
  Write_single_reg(ITG3205_Addr,ITG3205_PWR_M,0x00);  //复位设备 上电内地寄存器默认设置
	Write_single_reg(ITG3205_Addr,ITG3205_SMPL,0x07);  //采样速率 8MS
	Write_single_reg(ITG3205_Addr,ITG3205_DLPF,0x1E);  //陀螺全面范围正负2000
	Write_single_reg(ITG3205_Addr,ITG3205_INT_C,0x00);  //中断控制寄存器
//	Write_single_reg(ITG3205_Addr,ITG3205_PWR_M,0x00);   // XYZ正常检查，使用内部晶振
}
 
//获取ITG3205 X Y Z轴及温度数据 Get ITG3205 X Y Z axis and temperature data
void READ_ITG3205_XYZT(void)
{
	uint8_t  data_l,data_h;
	//ITG3205 slave address
  data_l = Read_single_reg(ITG3205_Addr,ITG3205_WHO);
//	printf("ITG3205_Addr:%d\r\n ",data_l);
	//获取ITG3205 X轴   ITG3205 X_axle
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GX_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GX_H);
//	printf("ITG3205_XL:%d ,ITG3205_XH:%d \r\n ",data_l,data_h);
	ITG3205_X = (data_h<<8)|data_l;
	ITG3205_X /= 14.375;
	if(ITG3205_X<0)  ITG3205_X = ITG3205_X;
	//获取ITG3205 Y轴  ITG3205 Y_axle
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GY_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GY_H);
//	printf("ITG3205_YL:%d ,ITG3205_YH:%d \r\n ",data_l,data_h);
	ITG3205_Y = (data_h<<8)|data_l;
	ITG3205_Y /= 14.375;
	if(ITG3205_Y<0)  ITG3205_Y = ITG3205_Y;
	//获取ITG3205 Z轴  ITG3205 Z_axle
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GZ_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GZ_H);
//	printf("ITG3205_ZL:%d ,ITG3205_ZH:%d \r\n ",data_l,data_h);
	ITG3205_Z = (data_h<<8)|data_l;
	ITG3205_Z /= 14.375;
	if(ITG3205_Z<0)  ITG3205_Z = ITG3205_Z;
	//获取ITG3205 温度  ITG3205 temperature
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_TMP_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_TMP_H);
//	printf("ITG3205_TEMPL:%d ,ITG3205_TEMPH:%d \r\n ",data_l,data_h);
	ITG3205_T = (data_h<<8)|data_l;
	ITG3205_T =35+((double)ITG3205_T+13200)/280;
	if(ITG3205_T<0)  ITG3205_T = -ITG3205_T;
	//printf("Gx:%d,Gy:%d,Gz:%d\r\n ",ITG3205_X,ITG3205_Y,ITG3205_Z);
	 	
}
//////////////////////////// end ITG3205

/////////////////////////  ADXL345
//初始化ADXL345  ADXL345 initialize
void Init_ADXL345(void)
{
  Write_single_reg(ADXL345_Addr,ADXL345_DATA_FORMAT,0x0B);  //setting the range, here it is set at +-16g with 0x0B
	Write_single_reg(ADXL345_Addr,ADXL345_BW_RATE,0x08);  //12.5Hz with 0x08,set up the data rate or sampling frequency
	Write_single_reg(ADXL345_Addr,ADXL345_POWER,0x08);  //选择电源模式
	Write_single_reg(ADXL345_Addr,ADXL345_INT_EN,0x80);  //使能DATA_READY中断
	Write_single_reg(ADXL345_Addr,ADXL345_OFSX,0x00);   // X偏移量
	Write_single_reg(ADXL345_Addr,ADXL345_OFSY,0x00);   // Y偏移量
	Write_single_reg(ADXL345_Addr,ADXL345_OFSZ,0x05);   // Z偏移量
}

//获取ADXL345数据信息 Get ADXL345 data information
void ADXL345_Read_XYZt(void)
{
  uint8_t  data_l,data_h;
	//ADXL345 slave address
  data_l = Read_single_reg(ADXL345_Addr,ADXL345_ID);
//	printf("ADXL345_Addr:%d\r\n ",data_l);
	
	//获取ADXL345 X轴 ADXL345 X_axle
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_X0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_X1);
//	printf("ADXL345_XL:%d ,ADXL345_XH:%d \r\n ",data_l,data_h);
	//sgnX = 0x8000 & data_h;
	ADXL345_X = ((data_h<<8)|data_l);
	
	//获取ADXL345 Y轴  ADXL345 Y_axle
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Y0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Y1);
//	printf("ADXL345_YL:%d ,ADXL345_YH:%d \r\n ",data_l,data_h);
	//sgnY = 0x8000 & data_h;
	ADXL345_Y = ((data_h<<8)|data_l);
	
	//获取ADXL345 Z轴  ADXL345 Z_axle
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Z0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Z1);
//	printf("ADXL345_ZL:%d ,ADXL345_ZH:%d \r\n ",data_l,data_h);
	//sgnZ = 0x8000 & data_h;
	//ADXL345_Z = 0x0000FFFF & ((data_h<<8)|data_l);
	ADXL345_Z = ((data_h<<8)|data_l);
//	printf("ADXL345_X:%x,ADXL345_Y:%x,ADXL345_Z:%x\r\n ",ADXL345_X,ADXL345_Y,ADXL345_Z);
  
	//分别是加速度X,Y,Z的原始数据，10位的
//	conver_x=(float)ADXL345_X*4; //3.9 to 4
		conver_x=(float)ADXL345_X*3.9; //3.9 to 4
		conver_y=(float)ADXL345_Y*3.9; //3.9 to 4
		conver_z=(float)ADXL345_Z*3.9; //3.9 to 4




//	conver_y=ADXL345_Y*4;  //float to int
	//conver_z=ADXL345_Z*4;
  //获取ADXL345 angle  ADXL345 angle
	//printf("#%d$%d$%d$",ADXL345_X,ADXL345_Y,ADXL345_Z);
	
	//printf("Ax:%f,Ay:%f,Az:%f\r\n",conver_x,conver_y,conver_z);//Actual Accel VAlues
 // ADXL345_X_AXIS=(float)(((atan2(conver_z,conver_x)*180)/3.14159265)+180);    //X轴角度值
  //ADXL345_Y_AXIS=(float)(((atan2(conver_z,conver_y)*180)/3.14159265)+180);  //Y轴角度值
//  printf("ADXL345_X_AXIS:%f,ADXL345_Y_AXIS:%f\r\n ",ADXL345_X_AXIS,ADXL345_Y_AXIS);
}	


//////////////////////////// end ADXL345


/////////////////////////  HMC5883L

//初始化HMC5883L  HMC5883L initialize
void Init_HMC5883L(void)
{
  Write_single_reg(HMC5883L_Addr,HMC5883L_configA,0x14);  //配置寄存器A
	Write_single_reg(HMC5883L_Addr,HMC5883L_MODE,0x00);  //模式寄存器	
}

//获取HMC5883L XYZ数据
void HMC5883L_Read_XYZt(void)
{
  uint8_t  data_l,data_h;
	
	//获取ITG3205 X轴  X_axle
	
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_XL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_XH);
	//	printf("HMC5883L_XL:%d ,HMC5883L_XH:%d \r\n ",data_l,data_h);
	HMC5883L_X = (data_h<<8)|data_l; 
	
	//获取ITG3205 Y轴  Y_axle
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_YL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_YH);
	//	printf("HMC5883L_YL:%d ,HMC5883L_YH:%d \r\n ",data_l,data_h);
	HMC5883L_Y = (data_h<<8)|data_l;
	
	//获取ITG3205 Z轴  Z_axle
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_ZL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_ZH);
//	printf("HMC5883L_ZL:%d ,HMC5883L_ZL:%d \r\n ",data_l,data_h);
	HMC5883L_Z = (data_h<<8)|data_l;
	//printf("Hx:%d,Hy:%d,Hz:%d\r\n ",HMC5883L_X,HMC5883L_Y,HMC5883L_Z);  
  
	//if(HMC5883L_X>0x7fff) HMC5883L_X-=0xffff;	
	//if(HMC5883L_Y>0x7fff) HMC5883L_Y-=0xffff;	
	//HMC5883L获取的角度值  HMC5883L Angle value
	
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
float Ri[3][3];
void evaluate_rotation_matrix_imu(void){
	
	normalize(conver_x,conver_y,conver_z);
	float gx = HMC5883L_X * conver_x *sin( (float)(atan2(conver_x,HMC5883L_X)) );//atan2(y/x)
	float gy = HMC5883L_Y * conver_y *sin( (float)(atan2(conver_y,HMC5883L_Y)) );
	float gz = HMC5883L_Z * conver_z *sin( (float)(atan2(conver_z,HMC5883L_Z)) );
	
	Ri[0][0] = HMC5883L_X;
	Ri[1][0] = HMC5883L_Y;
	Ri[2][0] = HMC5883L_Z;
	
	Ri[0][1] = gx;
	Ri[1][1] = gy;
	Ri[2][1] = gz;
	
	Ri[0][2] = conver_x;
	Ri[1][2] = conver_y;
	Ri[2][2] = conver_z;
	
	printf("DCM Matrix\r\n ");
	printf("%f, %f, %f\r\n",Ri[0][0],Ri[0][1],Ri[0][2]);
	//delay_ms(50);
	printf("%f, %f, %f\r\n",Ri[1][0],Ri[1][1],Ri[1][2]);
	printf("%f, %f, %f\r\n",Ri[2][0],Ri[2][1],Ri[2][2]);
}

void normalize(float vec_x,float vec_y,float vec_z){
	//float normal[3];
	float vec_mag;
	
	vec_mag = pow( (pow(vec_x,2)+pow(vec_y,2)+pow(vec_z,2)) , 0.5);
	normal_mat[0] = normal_mat[0]/vec_mag;
	normal_mat[1] = normal_mat[1]/vec_mag;
	normal_mat[2] = normal_mat[2]/vec_mag;
		
}


