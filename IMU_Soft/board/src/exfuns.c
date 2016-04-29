#include "string.h"
#include "exfuns.h"
#include "fattester.h"	
#include "malloc.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途							  
////////////////////////////////////////////////////////////////////////////////// 	
#define FILE_MAX_TYPE_NUM		7	//最多FILE_MAX_TYPE_NUM个大类
#define FILE_MAX_SUBT_NUM		4	//最多FILE_MAX_SUBT_NUM个小类

 //文件类型列表
u8*const FILE_TYPE_TBL[FILE_MAX_TYPE_NUM][FILE_MAX_SUBT_NUM]=
{
{"BIN"},			//BIN文件
{"LRC"},			//LRC文件
{"NES"},			//NES文件
{"TXT","C","H"},	//文本文件
{"WAV","MP3","APE","FLAC"},//支持的音乐文件
{"BMP","JPG","JPEG","GIF"},//图片文件
{"AVI"},			//视频文件
};
///////////////////////////////公共文件区,使用malloc的时候////////////////////////////////////////////
FATFS *fs[_VOLUMES];//逻辑磁盘工作区.	 
FIL *file;	  		//文件1
FIL *ftemp;	  		//文件2.
UINT br,bw;			//读写变量
FILINFO fileinfo;	//文件信息
DIR dir;  			//目录

u8 *fatbuf;			//SD卡数据缓存区
///////////////////////////////////////////////////////////////////////////////////////
//为exfuns申请内存
//返回值:0,成功
//1,失败
u8 exfuns_init(void)
{
	u8 i;
	for(i=0;i<_VOLUMES;i++)
	{
		fs[i]=(FATFS*)mymalloc(SRAMIN,sizeof(FATFS));	//为磁盘i工作区申请内存	
		if(!fs[i])break;
	}
	file=(FIL*)mymalloc(SRAMIN,sizeof(FIL));		//为file申请内存
	ftemp=(FIL*)mymalloc(SRAMIN,sizeof(FIL));		//为ftemp申请内存
	fatbuf=(u8*)mymalloc(SRAMIN,512);				//为fatbuf申请内存
	if(i==_VOLUMES&&file&&ftemp&&fatbuf)return 0;  //申请有一个失败,即失败.
	else return 1;	
}

//将小写字母转为大写字母,如果是数字,则保持不变.
u8 char_upper(u8 c)
{
	if(c<'A')return c;//数字,保持不变.
	if(c>='a')return c-0x20;//变为大写.
	else return c;//大写,保持不变
}	      
//报告文件的类型
//fname:文件名
//返回值:0XFF,表示无法识别的文件类型编号.
//		 其他,高四位表示所属大类,低四位表示所属小类.
u8 f_typetell(u8 *fname)
{
	u8 tbuf[5];
	u8 *attr='\0';//后缀名
	u8 i=0,j;
	while(i<250)
	{
		i++;
		if(*fname=='\0')break;//偏移到了最后了.
		fname++;
	}
	if(i==250)return 0XFF;//错误的字符串.
 	for(i=0;i<5;i++)//得到后缀名
	{
		fname--;
		if(*fname=='.')
		{
			fname++;
			attr=fname;
			break;
		}
  	}
	strcpy((char *)tbuf,(const char*)attr);//copy
 	for(i=0;i<4;i++)tbuf[i]=char_upper(tbuf[i]);//全部变为大写 
	for(i=0;i<FILE_MAX_TYPE_NUM;i++)	//大类对比
	{
		for(j=0;j<FILE_MAX_SUBT_NUM;j++)//子类对比
		{
			if(*FILE_TYPE_TBL[i][j]==0)break;//此组已经没有可对比的成员了.
			if(strcmp((const char *)FILE_TYPE_TBL[i][j],(const char *)tbuf)==0)//找到了
			{
				return (i<<4)|j;
			}
		}
	}
	return 0XFF;//没找到		 			   
}	 

//得到磁盘剩余容量
//drv:磁盘编号("0:"/"1:")
//total:总容量	 （单位KB）
//free:剩余容量	 （单位KB）
//返回值:0,正常.其他,错误代码
u8 exf_getfree(u8 *drv,u32 *total,u32 *free)
{
	FATFS *fs1;
	u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //得到磁盘信息及空闲簇数量
    res =(u32)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
    if(res==0)
	{											   
	    tot_sect=(fs1->n_fatent-2)*fs1->csize;	//得到总扇区数
	    fre_sect=fre_clust*fs1->csize;			//得到空闲扇区数	   
#if _MAX_SS!=512				  				//扇区大小不是512字节,则转换为512字节
		tot_sect*=fs1->ssize/512;
		fre_sect*=fs1->ssize/512;
#endif	  
		*total=tot_sect>>1;	//单位为KB
		*free=fre_sect>>1;	//单位为KB 
 	}
	return res;
}	

//path:表示要写文件的文件名包含路径  str: 要写数据
//path: /* Pointer to the file path */ 
//str: write data
//MAX_file_size:最大文件尺寸大小
uint8_t Write_SD(const char *path, uint8_t* str,uint64_t MAX_file_size)
{
	FRESULT res;
	FIL fp;
	FILINFO fno;
	uint32_t write_data_len;
	UINT  pbw;
	//write data length 
  write_data_len=strlen((const char *)str);
	if(write_data_len==0) return 0;
	//OPEN SD file  打开文件
	if(!f_open(&fp,path,FA_WRITE|FA_READ)){
		//判断文件大小 文件尺寸<MAX_file_size 继续写入 文件尺寸>MAX_file_size 创建文件
		   if(f_size(&fp)<MAX_file_size){
				 //Seek File R/W Pointer 				 				 
				 if(!f_lseek(&fp, f_size(&fp))){
//					 printf("file size:%d\r\n",f_size(&fp));
					 //write file  写数据到文件
				   if(!f_write (&fp,str,write_data_len,&pbw)){
						 //写文件成功
						 f_close(&fp);
					   return 1;
					 }
					 else{
						 f_close(&fp);
						 return 0;
					 }
				 }
				 else{
					 f_close(&fp);
					 //追加文件不成功 
			     return 0;	
				 }					 
		 }
		 else{
			 //创建文件
		   f_close(&fp);
			 return 2;
		 }
	 }
	else{
		f_close(&fp);
		return 0;	
	}		
}

//path:表示要写文件的文件名包含路径  str: 数据缓冲 offset:文件中读取数据的位置  length 读取数据长度
//path: /* Pointer to the file path */ 
//str:  data buffer  offset: File offset  length: read date length
uint8_t Read_SD(const TCHAR *path, void *str, uint32_t offset,uint32_t length)
{
	FRESULT res;
	FIL fp;
	uint32_t len;
	UINT  pbw;
	len=length;
  //open file
	if(!f_open(&fp,path,FA_WRITE|FA_READ)){
		if(f_lseek(&fp, offset)){
		  f_close(&fp);
			 return 0;
		}
		else{
			if(f_read(&fp,str,length,&pbw)){
				f_close(&fp);
				return 0;
			}
			else{
				f_close(&fp); 
				return 1;
			}			
		}
	}
	else return 0;
}

//检查文件是否存在若不存在则创建文件，存在则无需创建退出
uint8_t SD_Creat_File(const char *path)
{
  FILINFO fno;
	FIL fp;
	FRESULT res;
	
	res = f_stat(path,&fno);
	if(res==FR_OK){
		 res=f_close (&fp) ;
		//文件已经存在
		return 1;
	}
	else{
		//创建文件 creat file
		res=f_open(&fp,path, FA_CREATE_ALWAYS) ;
		if(res!=FR_OK){
      res=f_close (&fp) ;			
			return 0;
		}
		else
		{
			res=f_close (&fp) ;
			return 1;
		}
	}
//	 res = f_stat(path,&fno);
//	if(res==FR_OK)
//		printf("have a file \r\n");
//	else{
//	  printf("have no file \r\n");
//		//创建文件
//		res=f_open(&fp,path, FA_CREATE_ALWAYS) ;
//		if(res!=FR_OK)		
//			printf("creat file fail\r\n");
//		else
//		{
//			res=f_close(&fp) ;
//			printf("creat file success\r\n");	
//		}
//	}
}







