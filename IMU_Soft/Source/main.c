#include "stm32f4xx.h"
/*-----------board.h-----------------*/
/* 测试程序 主要包含IMU程序 串口 1 2 3 */

#include "led.h"
#include "stdio.h"
#include "usart.h"
#include "imu.h"
#include "sdio_sdcard.h"
#include "exfuns.h" 
#include "GPS_decode.h"

extern short  ITG3205_X,ITG3205_Y,ITG3205_Z,ITG3205_T;
//extern float ADXL345_X_AXIS,ADXL345_Y_AXIS;
extern float conver_x,conver_y,conver_z;
extern float HMC5883L_ARC;
extern float pitch,roll,yaw;
extern nmeaINFO info;


/*******Test_Module_TYPE == Test_LED*******/
//void Module_setup(void)
//{
//   LED_Init();
//} 
//void loop(void)
//{
//	// D5 light on off freq:1000ms
//  setLED(1, 0);
//	delay_ms(1000);
//	setLED(1, 1);
//	delay_ms(1000);
//	// D4 light blink freq:2000ms
//	Blinks_led_Feq(2,2000);
//	// D3 light blink freq:1000ms
//	setLED(3, 0);
//	delay_ms(1000);
//	setLED(3, 1);
//	delay_ms(1000);
//}
/*******Test_Module_TYPE == Test_LED END*******/

///*******Test_Module_TYPE == Test_IMU*******/
void Module_setup(void)
{

	u32 total,free;
	Modules_Power_Control(ALL_Module,1);
   USART_INIT(115200,3,ENABLE);
	//serial port 2 initialize, baud rate 9600 ,enable receive interupt (bluebooth)
	 USART_INIT(9600,2,ENABLE);


	//Initializes the IMU all modules
	 IMU_INIT(IMU_ALL_IC);
	GPS_Config();
	LED_Init();
	
	if(SD_Init()!= SD_OK){
		printf("SD Init failed!\r\n");
	}
	else
		printf("SD Init OK!\r\n");
	
   exfuns_init();							//为fatfs相关变量申请内存				 
   f_mount(fs[0],"0:",1); 					//挂载SD卡 

	if(exf_getfree((u8 *)"0",&total,&free))	//得到SD卡的总容量和剩余容量
	{
		printf("SD Card Fatfs Error!\r\n");
	}	
	else{
		printf("SD Total Size:   %d  MB\r\n",total>>10);
		printf("SD  Free Size:   %d  MB\r\n",free>>10);
	}
	
} 
void loop(void)
{
//	  FILINFO fno;
//	  uint32_t total,free;
//	  static FRESULT res;
//    uint8_t AddSDstr[256];
//	  FIL log_file;	
		char file_name[20],read_buffer[10];
	static uint8_t write_num=2,file_num,i;
	uint8_t AddSDstr[256];
	
    //IMU sensor data using Bluetooth printing
	//Get ITG3205 data
	READ_ITG3205_XYZT();
  //Get ADXL345 data
	ADXL345_Read_XYZt();
  //Get HMC5883L data
	HMC5883L_Read_XYZt();
	evaluate_rotation_matrix_imu();
	linear_filter();
	calculate_p_r_y();
	GPS_Parse_Decode();
	
//	//creat_file 
	if(write_num==2){
		sprintf(file_name,"gps_recv_%d.txt",file_num++);
		//创建文件 creat file
		if(SD_Creat_File(file_name))
			printf("create file success\r\n");
		else{
	    printf("create file fail\r\n");
	    while(1);
     }  
   }
//    //MAX file size is 100K ,file size> max_size create a file again  return write_status ==2	  
 
	 sprintf((char *)AddSDstr,"%d-%d-%d, %d:%d:%d\
		 Ax = %f,Ay = %f,Az = %f,Gx = %d,Gy = %d,Gz = %d, Latitude = %f, Longitude = %f\r\n ",info.utc.year,info.utc.mon,info.utc.day,info.utc.hour,info.utc.min,info.utc.sec,conver_x,conver_y,conver_z,ITG3205_X,ITG3205_Y,ITG3205_Z,info.lat,info.lon);
	 write_num=Write_SD(file_name,AddSDstr,102400);
	
	 Read_SD(file_name, (char*)read_buffer, 10*(i++),10);
		USART_Write_LEN((u8 *)read_buffer,10,3);
		delay_ms(100);

 }
//	// Write_SD("gps_data.txt",AddSDstr,102400000);
//	
//	//get GPS data
//	//	 // The value of "Print_Port" is 3 in the retarget.h (serial 3 printing)
//////	 //Initialize serial port 3 testing
//	 //Modules_Power_Control(Zigbee_Power,1);
//	// USART_INIT(9600,3,ENABLE); 
//	
////	 // The value of "Print_Port" is 2 in the retarget.h (Bluetooth printing)
////	 //Initialize serial port 2 testing(blue)
////	 Modules_Power_Control(IMU_Blue_Power,1);
////	 USART_INIT(9600,2,ENABLE); 	
//	 //GPS power on 
//		 

//	//GPS_Parse_Decode();
//}
///*******Test_Module_TYPE == Test_IMU END *******/

///*******Test_Module_TYPE == Test_Blue********/
/*void Module(void)
{
	 //Open Bluetooth power 
   Modules_Power_Control(IMU_Blue_Power,1);
////	 Modules_Power_Control(Zigbee_Power,1);
////	 USART_INIT(9600,3,ENABLE);
	 //Initializes serial port 2
	 USART_INIT(9600,2,ENABLE);
} 
void loop(void)
{
	 uint8_t AddSDstr[256],length;
//	 //接收蓝牙发送的数据，并发送到蓝牙
	 //Receive data from bluetooth 
	 length = USART_RECV_DATA_LEN(AddSDstr,2);
	 //send a string to bluetooth
	 USART_Write_LEN("recv data",strlen("recv data"),2);
//	 //send data to bluetooth
	 USART_Write_LEN(AddSDstr,length,2);	
}*/
///*******Test_Module_TYPE == Test_IMU END *******/

///******Test_Module_TYPE == Test_GPS******/
void Module_setup_gps(void)
{
//	 // The value of "Print_Port" is 3 in the retarget.h (serial 3 printing)
////	 //Initialize serial port 3 testing
////	 Modules_Power_Control(Zigbee_Power,1);
////	 USART_INIT(9600,3,ENABLE); 
//	
//	 // The value of "Print_Port" is 2 in the retarget.h (Bluetooth printing)
//	 //Initialize serial port 2 testing(blue)
//	 Modules_Power_Control(IMU_Blue_Power,1);
	 USART_INIT(9600,2,ENABLE); 	
//	 //GPS power on 
	 Modules_Power_Control(GPS_Power,1);	 
//	 //GPS config
	GPS_Config();
} 
void loop_gps(void)
{
	 //GPS gets data from print to a Bluetooth
	 GPS_Parse_Decode();
}
/*******Test_Module_TYPE == Test_GPS END *******/


///*********Test_Module_TYPE == Test_SD**************/
void Module_setup_sd(void)
{
	u32 total,free;
	//open bule power for testing
//	Modules_Power_Control(Zigbee_Power,1);
	 USART_INIT(115200,3,ENABLE); 
//	//open SD power 
	Modules_Power_Control(SD_Power,1);
	   Modules_Power_Control(IMU_Blue_Power,1);	
	 Modules_Power_Control(GPS_Power,1);
	 //Initializes serial port 2
	 USART_INIT(9600,2,ENABLE);
	 //Initializes the IMU all modules
	 IMU_INIT(IMU_ALL_IC);
	GPS_Config();
			printf("Before SDCARD init\r\n");

//	//初始化SD卡 SD initialize
	
	if(SD_Init()!= SD_OK){
		printf("SD Init failed!\r\n");
	}
	else
		printf("SD Init OK!\r\n");
	
   exfuns_init();							//为fatfs相关变量申请内存				 
   f_mount(fs[0],"0:",1); 					//挂载SD卡 

	if(exf_getfree((u8 *)"0",&total,&free))	//得到SD卡的总容量和剩余容量
	{
		printf("SD Card Fatfs Error!\r\n");
	}	
	else{
		printf("SD Total Size:   %d  MB\r\n",total>>10);
		printf("SD  Free Size:   %d  MB\r\n",free>>10);
	}
}
void loop_sd(void)
{
	char file_name[20],read_buffer[10];
	static uint8_t write_num=2,file_num,i;
	uint8_t AddSDstr[256];
	
//	//creat_file 
	if(write_num==2){
		sprintf(file_name,"gps_recv_%d.txt",file_num++);
		//创建文件 creat file
		if(SD_Creat_File(file_name))
			printf("creat file success\r\n");
		else{
	    printf("creat file fail\r\n");
	    while(1);
     }  
   }
//    //MAX file size is 100K ,file size> max_size create a file again  return write_status ==2	  
	 GPS_Parse_Decode(); 
	 sprintf((char *)AddSDstr,"\r\nLatitude = %f, Longitude = %f\r\n",info.lat,info.lon);
	 write_num=Write_SD(file_name,AddSDstr,102400);
	
	 Read_SD(file_name, (char*)read_buffer, 10*(i++),10);
		USART_Write_LEN((u8 *)read_buffer,10,3);
		delay_ms(100);
}
///*********Test_Module_TYPE == Test_SD END**************/

///***********Test_Module_TYPE == Test_all_module************/
//void Module_setup(void)
//{
//	 u32 total,free;
//	 //Modules Power Control
//	 Modules_Power_Control(ALL_Module,1);
//	 //串口相关程序 serial port 3 initialize, baud rate 115200 ,enable receive interupt	 
//   USART_INIT(115200,3,ENABLE);
//	//serial port 2 initialize, baud rate 9600 ,enable receive interupt (bluebooth)
//	 USART_INIT(9600,2,ENABLE);
//	 //I2C相关程序  IMU all module initialize
//   IMU_INIT(IMU_ALL_IC);
//	 //GPS module initialize config 
// //  GPS_Config();
//	 //LED initialize
//	 LED_Init();
//	 //初始化SD卡 SD initialize
//	 if(SD_Init()!= SD_OK)
//		 printf("SD Init failed!\r\n");
//	 else{
//		 printf("SD Init OK!\r\n");
//		 
//		//打印SD卡信息  SD card capacity 
//		printf("size:   %u MB ",SDCardInfo.CardCapacity>>20);
//	 }	
//    exfuns_init();							//为fatfs相关变量申请内存				 
//    f_mount(fs[0],"0:",1); 					//挂载SD卡
//		if(exf_getfree("0",&total,&free))	//得到SD卡的总容量和剩余容量
//		{
//			printf("SD Card Fatfs Error!\r\n");
//		}	
//		else{
//			printf("SD Total Size:   %d  MB\r\n",total>>10);
//			printf("SD  Free Size:   %d  MB\r\n",free>>10);
//		}	
//    //SD创建文件		
//	  if(!SD_Creat_File("gps_data.txt"))
//		    //创建文件失败 D3 2S闪烁 creat file failed ,D3 blinking frequency 2S
//        setLED(1, 1);		
//} 

//void loop(void)
//{ 
//	  FILINFO fno;
//	  uint32_t total,free;
//	  static FRESULT res;
//    uint8_t AddSDstr[256];
//	  FIL log_file;	
//	  //GPS get data 
//	//  GPS_Parse_Decode();
//	  //GET IMU data
//	  READ_ITG3205_XYZT();	
//	  ADXL345_Read_XYZt();
//	  HMC5883L_Read_XYZt();
//	  //IMU获取数据转换成字符串 方便写入SD卡
//	  sprintf((char *)AddSDstr,"\r\nITG3205_X=%5d,ITG3205_Y=%5d,ITG3205_Z=%5d, ITG3205_T=%d  \r\n\
//			ADXL345_X_AXIS=%.4f,ADXL345_Y_AXIS=%.4f,HMC5883L_ARC=%.4f\r\n",ITG3205_X,ITG3205_Y,ITG3205_Z,ITG3205_T\
//		 ,ADXL345_X_AXIS,ADXL345_Y_AXIS,HMC5883L_ARC);
//		//串口打印IMU数据
////		printf("%s",AddSDstr);
//		//将IMU获取的数据写到SD卡
//		Write_SD("gps_data.txt",AddSDstr,102400000);
//		  //creat and delete file 
////		SD_Creat_File("123.txt");
////		//delete file 
////		res= f_unlink("123.txt");
////		if(res == FR_OK)
////			printf("delete ok\r\n");
//		Blinks_led_Feq(2,2000);
//		Blinks_led_Feq(3,2000);
//}
/***********Test_Module_TYPE == Test_all_module END************/
//TEST IMU
void Module_setup_imu(void)
{
	 //Open IMU power 
   Modules_Power_Control(IMU_Blue_Power,1);	
	 Modules_Power_Control(GPS_Power,1);
	 //Initializes serial port 2
	 USART_INIT(9600,2,ENABLE);
	 //Initializes the IMU all modules
	 IMU_INIT(IMU_ALL_IC);
	GPS_Config();
	
} 
void loop_imu(void)
{
    //IMU sensor data using Bluetooth printing
	//Get ITG3205 data
	READ_ITG3205_XYZT();
  //Get ADXL345 data
	ADXL345_Read_XYZt();
  //Get HMC5883L data
	HMC5883L_Read_XYZt();
	evaluate_rotation_matrix_imu();
	linear_filter();
	delay_ms(50);
	// sprintf((char *)AddSDstr,"\r\nG_X=%5d,G_Y=%5d,G_Z=%5d, Temp=%d  \r\n\
			A_X=%.4f,A_Y=%.4f,MAG=%.4f\r\n",ITG3205_X,ITG3205_Y,ITG3205_Z,ITG3205_T\
		 ,ADXL345_X_AXIS,ADXL345_Y_AXIS,HMC5883L_ARC);
	// Write_SD("gps_data.txt",AddSDstr,102400000);
	
	//get GPS data
	//	 // The value of "Print_Port" is 3 in the retarget.h (serial 3 printing)
////	 //Initialize serial port 3 testing
	 //Modules_Power_Control(Zigbee_Power,1);
	// USART_INIT(9600,3,ENABLE); 
	
//	 // The value of "Print_Port" is 2 in the retarget.h (Bluetooth printing)
//	 //Initialize serial port 2 testing(blue)
//	 Modules_Power_Control(IMU_Blue_Power,1);
//	 USART_INIT(9600,2,ENABLE); 	
	 //GPS power on 
		 

	//GPS_Parse_Decode();
}
//////////////////
int main(void)
{
	//System initialization
  SystemInit();
	//clock initialization  System CLOCK:168MHZ  
  SysTick_Init(168);
	//Closed all module power
	Modules_Power_Control(ALL_Module,DISABLE);
	//Modules_Power_Control(IMU_Blue_Power,ENABLE);
	 //Modules_Power_Control(GPS_Power,ENABLE);
	//Module_setup();
	Module_setup();
	while(1)	
	{ 
	  loop();
		
	}
}

