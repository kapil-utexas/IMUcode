#include "string.h"
#include "exfuns.h"
#include "fattester.h"	
#include "malloc.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;							  
////////////////////////////////////////////////////////////////////////////////// 	
#define FILE_MAX_TYPE_NUM		7	//���FILE_MAX_TYPE_NUM������
#define FILE_MAX_SUBT_NUM		4	//���FILE_MAX_SUBT_NUM��С��

 //�ļ������б�
u8*const FILE_TYPE_TBL[FILE_MAX_TYPE_NUM][FILE_MAX_SUBT_NUM]=
{
{"BIN"},			//BIN�ļ�
{"LRC"},			//LRC�ļ�
{"NES"},			//NES�ļ�
{"TXT","C","H"},	//�ı��ļ�
{"WAV","MP3","APE","FLAC"},//֧�ֵ������ļ�
{"BMP","JPG","JPEG","GIF"},//ͼƬ�ļ�
{"AVI"},			//��Ƶ�ļ�
};
///////////////////////////////�����ļ���,ʹ��malloc��ʱ��////////////////////////////////////////////
FATFS *fs[_VOLUMES];//�߼����̹�����.	 
FIL *file;	  		//�ļ�1
FIL *ftemp;	  		//�ļ�2.
UINT br,bw;			//��д����
FILINFO fileinfo;	//�ļ���Ϣ
DIR dir;  			//Ŀ¼

u8 *fatbuf;			//SD�����ݻ�����
///////////////////////////////////////////////////////////////////////////////////////
//Ϊexfuns�����ڴ�
//����ֵ:0,�ɹ�
//1,ʧ��
u8 exfuns_init(void)
{
	u8 i;
	for(i=0;i<_VOLUMES;i++)
	{
		fs[i]=(FATFS*)mymalloc(SRAMIN,sizeof(FATFS));	//Ϊ����i�����������ڴ�	
		if(!fs[i])break;
	}
	file=(FIL*)mymalloc(SRAMIN,sizeof(FIL));		//Ϊfile�����ڴ�
	ftemp=(FIL*)mymalloc(SRAMIN,sizeof(FIL));		//Ϊftemp�����ڴ�
	fatbuf=(u8*)mymalloc(SRAMIN,512);				//Ϊfatbuf�����ڴ�
	if(i==_VOLUMES&&file&&ftemp&&fatbuf)return 0;  //������һ��ʧ��,��ʧ��.
	else return 1;	
}

//��Сд��ĸתΪ��д��ĸ,���������,�򱣳ֲ���.
u8 char_upper(u8 c)
{
	if(c<'A')return c;//����,���ֲ���.
	if(c>='a')return c-0x20;//��Ϊ��д.
	else return c;//��д,���ֲ���
}	      
//�����ļ�������
//fname:�ļ���
//����ֵ:0XFF,��ʾ�޷�ʶ����ļ����ͱ��.
//		 ����,����λ��ʾ��������,����λ��ʾ����С��.
u8 f_typetell(u8 *fname)
{
	u8 tbuf[5];
	u8 *attr='\0';//��׺��
	u8 i=0,j;
	while(i<250)
	{
		i++;
		if(*fname=='\0')break;//ƫ�Ƶ��������.
		fname++;
	}
	if(i==250)return 0XFF;//������ַ���.
 	for(i=0;i<5;i++)//�õ���׺��
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
 	for(i=0;i<4;i++)tbuf[i]=char_upper(tbuf[i]);//ȫ����Ϊ��д 
	for(i=0;i<FILE_MAX_TYPE_NUM;i++)	//����Ա�
	{
		for(j=0;j<FILE_MAX_SUBT_NUM;j++)//����Ա�
		{
			if(*FILE_TYPE_TBL[i][j]==0)break;//�����Ѿ�û�пɶԱȵĳ�Ա��.
			if(strcmp((const char *)FILE_TYPE_TBL[i][j],(const char *)tbuf)==0)//�ҵ���
			{
				return (i<<4)|j;
			}
		}
	}
	return 0XFF;//û�ҵ�		 			   
}	 

//�õ�����ʣ������
//drv:���̱��("0:"/"1:")
//total:������	 ����λKB��
//free:ʣ������	 ����λKB��
//����ֵ:0,����.����,�������
u8 exf_getfree(u8 *drv,u32 *total,u32 *free)
{
	FATFS *fs1;
	u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //�õ�������Ϣ�����д�����
    res =(u32)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
    if(res==0)
	{											   
	    tot_sect=(fs1->n_fatent-2)*fs1->csize;	//�õ���������
	    fre_sect=fre_clust*fs1->csize;			//�õ�����������	   
#if _MAX_SS!=512				  				//������С����512�ֽ�,��ת��Ϊ512�ֽ�
		tot_sect*=fs1->ssize/512;
		fre_sect*=fs1->ssize/512;
#endif	  
		*total=tot_sect>>1;	//��λΪKB
		*free=fre_sect>>1;	//��λΪKB 
 	}
	return res;
}	

//path:��ʾҪд�ļ����ļ�������·��  str: Ҫд����
//path: /* Pointer to the file path */ 
//str: write data
//MAX_file_size:����ļ��ߴ��С
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
	//OPEN SD file  ���ļ�
	if(!f_open(&fp,path,FA_WRITE|FA_READ)){
		//�ж��ļ���С �ļ��ߴ�<MAX_file_size ����д�� �ļ��ߴ�>MAX_file_size �����ļ�
		   if(f_size(&fp)<MAX_file_size){
				 //Seek File R/W Pointer 				 				 
				 if(!f_lseek(&fp, f_size(&fp))){
//					 printf("file size:%d\r\n",f_size(&fp));
					 //write file  д���ݵ��ļ�
				   if(!f_write (&fp,str,write_data_len,&pbw)){
						 //д�ļ��ɹ�
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
					 //׷���ļ����ɹ� 
			     return 0;	
				 }					 
		 }
		 else{
			 //�����ļ�
		   f_close(&fp);
			 return 2;
		 }
	 }
	else{
		f_close(&fp);
		return 0;	
	}		
}

//path:��ʾҪд�ļ����ļ�������·��  str: ���ݻ��� offset:�ļ��ж�ȡ���ݵ�λ��  length ��ȡ���ݳ���
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

//����ļ��Ƿ�������������򴴽��ļ������������贴���˳�
uint8_t SD_Creat_File(const char *path)
{
  FILINFO fno;
	FIL fp;
	FRESULT res;
	
	res = f_stat(path,&fno);
	if(res==FR_OK){
		 res=f_close (&fp) ;
		//�ļ��Ѿ�����
		return 1;
	}
	else{
		//�����ļ� creat file
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
//		//�����ļ�
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







