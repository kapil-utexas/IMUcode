/********************************GPSЭ��˵��*************************/
/*GPSʹ��STM32F4����1 ����Ϊ PA2 PA3 
GPS����ͨ��Ĭ�ϲ���Ϊ�������ʣ�9600bps,����λ��8bit,��ʼλ=1bit,ֹͣλ��1bit,����żУ�顣
GPS֡��ʽ���磺$aaccc,ddd,ddd,��,ddd*hh<CR><LF>
1.��$��-- ֡������ʼλ
2.aaccc-- ��ַ��ǰ��λΪʶ���������λΪ�����
3.ddd��ddd�� ����
4.��* ���� У���ǰ׺
5.hh�� У��ͣ� $ ��* ֮�������ַ������У��ͣ����ֽ���������㣬�õ�У��ͺ���ת��16 ���Ƹ�ʽ��ASCII �ַ�����
6.<CR><LF>-- ֡�������س��ͻ���
*/

/******************************** GPS  NMEA0183 protocol description *************************/
/*GPS use STM32F4 serial port 1 , GPIO PA2 PA3  GPS communication protocols use NMEA0183 standards
GPS serial communication: baud rate = 9600 BPS, data = 8 bit, began a = 1 bit, stop bit = 1 bit, NO parity checking.
GPS frame format: $aaccc,ddd,ddd,��,ddd*hh<CR><LF>
1.��$��-- Frame start bit
2.aaccc-- the address domain,The first two is the identifier��After three's statement
3.ddd��ddd�� data
4.��* ���� Check the prefix
5.hh�� Check�� Between $* all character code checksum (exclusive or operation, each byte do after get the checksum, 
  ASCII characters to convert hexadecimal format.)
6.<CR><LF>-- Frame stop
*/


#include "GPS_decode.h"


/* DMA���ջ���   DMA receive buffer*/
char gps_recv_buff[GPS_RBUFF_SIZE];
/* DMA���������־   End of the DMA transfers*/
__IO uint8_t GPS_TransferEnd = 0, GPS_HalfTransferEnd = 0;
/*�ݴ涺��֮������ݷ���ת��    Comma between data storage. The convenient conversion */
static Parse_Arry  pack[20] = {0x00}; 
nmeaINFO info;

/******************************************************************************************************** 
**     ��������:            bit        IsLeapYear(uint8_t    iYear) 
**    ��������:            �ж�����(�������2000�Ժ�����) 
**    ��ڲ�����            iYear    ��λ���� 
**    ���ڲ���:            uint8_t        1:Ϊ����    0:Ϊƽ�� 
********************************************************************************************************/ 
static uint8_t IsLeapYear(uint8_t iYear) 
{ 
    uint16_t    Year; 
    Year    =    2000+iYear; 
    if((Year&3)==0) 
    { 
        return ((Year%400==0) || (Year%100!=0)); 
    } 
     return 0; 
} 

/******************************************************************************************************** 
**     ��������:            void    GMTconvert(uint8_t *DT,uint8_t GMT,uint8_t AREA) 
**    ��������:            ��������ʱ�任�������ʱ��ʱ�� 
**    ��ڲ�����            *DT:    ��ʾ����ʱ������� ��ʽ YY,MM,DD,HH,MM,SS 
**                        GMT:    ʱ���� 
**                        AREA:    1(+)���� W0(-)���� 
********************************************************************************************************/ 
void    GMTconvert(nmeaTIME *SourceTime, nmeaTIME *ConvertTime, uint8_t GMT,uint8_t AREA) 
{ 
    uint32_t    YY,MM,DD,hh,mm,ss;        //������ʱ�����ݴ���� 
     
    if(GMT==0)    return;                //�������0ʱ��ֱ�ӷ��� 
    if(GMT>12)    return;                //ʱ�����Ϊ12 �����򷵻�         

    YY    =    SourceTime->year;                //��ȡ�� 
    MM    =    SourceTime->mon;                 //��ȡ�� 
    DD    =    SourceTime->day;                 //��ȡ�� 
    hh    =    SourceTime->hour;                //��ȡʱ 
    mm    =    SourceTime->min;                 //��ȡ�� 
    ss    =    SourceTime->sec;                 //��ȡ�� 

    if(AREA)                        //��(+)ʱ������ 
    { 
        if(hh+GMT<24)    hh    +=    GMT;//������������ʱ�䴦��ͬһ�������Сʱ���� 
        else                        //����Ѿ����ڸ�������ʱ��1����������ڴ��� 
        { 
            hh    =    hh+GMT-24;        //�ȵó�ʱ�� 
            if(MM==1 || MM==3 || MM==5 || MM==7 || MM==8 || MM==10)    //���·�(12�µ�������) 
            { 
                if(DD<31)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==4 || MM==6 || MM==9 || MM==11)                //С�·�2�µ�������) 
            { 
                if(DD<30)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==2)    //����2�·� 
            { 
                if((DD==29) || (DD==28 && IsLeapYear(YY)==0))        //��������������2��29�� ���߲�����������2��28�� 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
                else    DD++; 
            } 
            else if(MM==12)    //����12�·� 
            { 
                if(DD<31)    DD++; 
                else        //�������һ�� 
                {               
                    DD    =    1; 
                    MM    =    1; 
                    YY    ++; 
                } 
            } 
        } 
    } 
    else 
    {     
        if(hh>=GMT)    hh    -=    GMT;    //������������ʱ�䴦��ͬһ�������Сʱ���� 
        else                        //����Ѿ����ڸ�������ʱ��1����������ڴ��� 
        { 
            hh    =    hh+24-GMT;        //�ȵó�ʱ�� 
            if(MM==2 || MM==4 || MM==6 || MM==8 || MM==9 || MM==11)    //�����Ǵ��·�(1�µ�������) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    31; 
                    MM    --; 
                } 
            } 
            else if(MM==5 || MM==7 || MM==10 || MM==12)                //������С�·�2�µ�������) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    30; 
                    MM    --; 
                } 
            } 
            else if(MM==3)    //�����ϸ�����2�·� 
            { 
                if((DD==1) && IsLeapYear(YY)==0)                    //�������� 
                { 
                    DD    =    28; 
                    MM    --; 
                } 
                else    DD--; 
            } 
            else if(MM==1)    //����1�·� 
            { 
                if(DD>1)    DD--; 
                else        //�����һ�� 
                {               
                    DD    =    31; 
                    MM    =    12; 
                    YY    --; 
                } 
            } 
        } 
    }         

    ConvertTime->year   =    YY;                //������ 
    ConvertTime->mon    =    MM;                //������ 
    ConvertTime->day    =    DD;                //������ 
    ConvertTime->hour   =    hh;                //����ʱ 
    ConvertTime->min    =    mm;                //���·� 
    ConvertTime->sec    =    ss;                //������ 
}

/**
  * @brief  GPS_DMA_Config  GPS receiving configuration DMA interrupt 
  * @param  ��
  * @retval ��
  */
static void GPS_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
		/*����DMAʱ��*/
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	
    DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
		/*����DMAԴ���������ݼĴ�����ַ*/
		DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(USART1->DR));	   
		/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
		DMA_InitStructure.DMA_Memory0BaseAddr  = (u32)gps_recv_buff;
		/*���򣺴��ڴ浽����*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
		/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = GPS_RBUFF_SIZE;
		/*�����ַ����*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
		/*�ڴ��ַ����*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
		/*�������ݵ�λ*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		/*�ڴ����ݵ�λ 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
		/*DMAģʽ������ѭ��*/
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 
		/*���ȼ�����*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
		/*��ֹFIFOģʽ����	*/
		DMA_InitStructure.DMA_FIFOMode  = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold   = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst    = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
		/*����DMA��ͨ��*/		   
		DMA_Init(DMA2_Stream2, &DMA_InitStructure); 	   
    
    // DMA2 Channel Interrupt ENABLE
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
    DMA_ITConfig(DMA2_Stream2,DMA_IT_TC|DMA_IT_HT,ENABLE);  //����DMA������ɺ�����ж�
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_HTIF2);
		/*ʹ��DMA*/
		DMA_Cmd (DMA2_Stream2,ENABLE);		
    
    /* ���ô��� �� DMA����TX���� */
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);   
}


/**
  * @brief  GPS_Config  gps ��ʼ��
  * @param  ��
  * @retval ��
**/
//GPS���� ����1��ֹ�ж� ������Ϊ9600 DMA��������ж�
void GPS_Config(void)
{
	//serial port 1 baud rate 9600, enable interrupts
  USART_INIT(9600,1,ENABLE);
	//GPS reception is complete and a half to complete interrupts 
  GPS_DMA_Config();  
  
}

/**
  * @brief  GPS DMA�жϷ�����  GPS DMA interrupt service function 
  * @param  None.
  * @retval None.
  */
void DMA2_Stream2_IRQHandler(void)
{ 
  if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2))     /* DMA ������� */
  {
    GPS_TransferEnd = 1;                    //���ô�����ɱ�־λ
    DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_TCIF2);
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
  }
	if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_HTIF2))     /* DMA �봫����� */
  {
    GPS_HalfTransferEnd = 1;                    //���ð봫����ɱ�־λ
    DMA_ClearFlag(DMA2_Stream2,DMA_FLAG_HTIF2);
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_HTIF2);
   }
}

/**
 * \brief Define packet type by header (nmeaPACKTYPE).
 * @param buff a constant character pointer of packet buffer.
 * @return The defined packet type
 * @see nmeaPACKTYPE
 */
GPS_Parse_Type nmea_pack_type(const char *buff)
{
    static const char *pheads[] = {"$GPGGA","$GPGSA", "$GPGSV","$GPRMC", "$GPVTG","$GPGLL"};
    if(0 == memcmp(buff, pheads[0], 6))
        return GPGGA;  // GPGGA pack type
    else if(0 == memcmp(buff, pheads[1], 6))
        return GPGSA;  // GPGSA pack type
    else if(0 == memcmp(buff, pheads[2], 6))
        return GPGSV;  // GPGSV pack type
    else if(0 == memcmp(buff, pheads[3], 6))
        return GPRMC;  // GPRMC pack type
    else if(0 == memcmp(buff, pheads[4], 6))
        return GPVTG;  // GPVTG pack type
    else if(0 == memcmp(buff, pheads[5], 6))
        return GPGLL;  // GPGLL pack type
    return GPNON;  // no type
} 


/**GPS У��ͼ��㼰�ж�
 * \brief DGPS checksum calculation and judgment
 * @param buff a constant character pointer of packet buffer.
 * @return Check whether the correct 
 * @see nmeaPACKTYPE
 */
uint8_t GPS_Check_CRC(char *gps_data)
{
	uint16_t i;
	char *ptr1 , *ptr2 , crc , crc_ascii[4];
	ptr1 = strchr(gps_data,'$');  //��ȡ'$'�ĵ�ַλ  get '$' address
	ptr2 = strchr(gps_data,'*');  //��ȡ'*'�ĵ�ַλ  get '*' address
	//��ѯ����ͷβ���ش���  no head or tail  
	if((ptr1==NULL)||(ptr2==NULL))
		return FALSE; 
	//'*'�ĵ�ַλ��'$'�ĵ�ַλ�Ĳ�ֵΪʵ�����ݳ���
	for(i=1;i<=(ptr2-ptr1);i++){
		crc ^= ptr1[i];
	}
	
	sprintf(crc_ascii,"%2X",crc);
	ptr2++;
	if(strncmp(crc_ascii,ptr2,2)==0)
		return TURE;  //check succeed
	else return FALSE; //Check failure
}

/**�ַ��������radixת��Ϊ����
 * \brief Convert alphanumeric to integer
 * @param str: Alphabetic character array does not contain the '\ 0'
 * @param str_sz:  Alphabetic character array length
 * @param radix: Hexadecimal Radix 10 means decimal,Radix 16 means hexadecimal
 * @return The converted integer
 */
int nmea_atoi(const char *str, int str_sz, int radix)
{
    char buff[20];
    int res = 0;
    if(str_sz < 20)
    {
        memcpy(&buff[0], str, str_sz);
        buff[str_sz] = '\0';
        res = strtol(&buff[0], NULL, radix);
    }
    return res;
}

/** �ַ�����ת��Ϊ������
 * \brief Convert alphanumeric to double
 * @param buffer: Alphabetic character array does not contain the '\ 0'
 * @param recv_len:  Alphabetic character array length
 * @return The converted double
 */
double nmea_atof(const char *buffer,uint8_t recv_len)
{
  double res;
	char temp[PARSE_MAX_NUM];
	
	if((recv_len<PARSE_MAX_NUM)&&(recv_len)){
	   memcpy(temp, buffer, recv_len);
		 temp[recv_len] = '\0';
//		 res = atof(temp);
		 res = strtod(temp, NULL);
	}
  return res;	
}

/**
 * \brief Time array is converted into structure nmeaTIME
 * @param recv_time: Time array(ASCII)
 * @param time_buff_len:  Time array length
 * @param time: The structure after conversion
 * @param time_type: TIME_TYPE_HHMMSS means (hour)(minute)(second),TIME_TYPE_DDMMYY means (date)(month)(year)
 * @return null 
 */
void GET_NMEA_TIME(const char *recv_time,uint8_t time_buff_len,nmeaTIME *time,uint8_t time_type)
{
	uint8_t i;
	char temp[6];
	if(time_buff_len>=6){
    if(time_type==TIME_TYPE_HHMMSS){
		  time->hour= (recv_time[0] - '0')*10 + (recv_time[1] - '0');//ʱ
			time->min= (recv_time[2] - '0')*10 + (recv_time[3] - '0');//��
			time->sec= (recv_time[4] - '0')*10 + (recv_time[5] - '0');//��
			if(recv_time[6]=='.'){//С������漸λ������ȡ
				for(i=0;i<=(time_buff_len-7);i++)
			    temp[i] = recv_time[7+i];
				temp[i] = '\0';
				time->hsec = atoi(temp);
			}
			else
				time->hsec= 0x00 ;
	  } 
	  else if(time_type==TIME_TYPE_DDMMYY){
	    time->day= (recv_time[0] - '0')*10 + (recv_time[1] - '0');//��
			time->mon= (recv_time[2] - '0')*10 + (recv_time[3] - '0');//��
			time->year= (recv_time[4] - '0')*10 + (recv_time[5] - '0');//��
	  }
  }
}

/* ��ȡ���䶺��ֱ�ӵ�����  obtain a comma between the data 
 * ���ݱ��浽ȫ�ֱ���  pack 
 * \brief obtain a comma between the data
 * @param  use_buffer:  Between the '$' and '*' data buffer
 * @param use_len: Between the '$' and '*' data length
 * @return  The number of data in the comma between 
*/
uint8_t GET_Comma_Between_Paragraphs(const char *use_buffer,uint8_t use_len)
{
  uint8_t tail,head,i,pack_num = 0;
	//���仺��������
	for(i=0;i<20;i++){
	  memset(pack[i].buffer,0,PARSE_MAX_SIZE);
		pack[i].len = 0x00;
	}
	//��ȡԭ��������ѭ������
	tail = head = 0x00;
	for(i=0,pack_num=0;i<use_len;i++){
	  if((*(use_buffer+i)==',')||(i==use_len-1)){
		  if(tail == head)
				tail=i;
			else
			{
			  head = tail;
				tail = i;
				//ͷβ����Ϊ1��ʾ�м�������
				pack[pack_num].len = tail - head -1;
				memcpy(pack[pack_num].buffer,use_buffer+head+1,pack[pack_num].len);
				pack_num++;
			}			
		}
	}
	return pack_num; 
}

/*
* \brief  GPGGA paragraph analysis
 * @param  use_buffer:  Between the '$' and '*' data buffer
 * @param use_len: Between the '$' and '*' data length
 * @param nmea_gga: GPGGA data 
 * @return  null
*/
void NMEA_Parse_GPGGA(const char *use_buffer,uint8_t use_len,nmeaGPGGA *nmea_gga)
{
	uint8_t i,Paragraph_num;
	memset(nmea_gga,0,sizeof(nmeaGPGGA));
	Paragraph_num = GET_Comma_Between_Paragraphs(use_buffer,use_len);
	if(Paragraph_num == 0x0e){ //GPGGA�ܹ�14������
		// UTC ʱ�䣬��ʽΪhhmmss.sss��ȡUTCʱ��
		if(pack[0].len)
			GET_NMEA_TIME(pack[0].buffer,pack[0].len,&(nmea_gga->utc),TIME_TYPE_HHMMSS); 
		//γ�ȣ���ʽΪddmm.mmmm
		if(pack[1].len)
			nmea_gga->lat = nmea_atof(pack[1].buffer,pack[1].len);
		//γ�Ȱ���N ��S(��γ����γ)
		if(pack[2].len)
			nmea_gga->ns = pack[2].buffer[0]; 
		//���ȣ���ʽΪddmm.mmmm
		if(pack[3].len)
			nmea_gga->lon = nmea_atof(pack[3].buffer,pack[3].len); 
		//���Ȱ���E ��W(����������)
		if(pack[4].len)
			nmea_gga->ew = pack[4].buffer[0]; 
		//��λ����ָʾ��0=��λ��Ч��1=��λ��Ч
		if(pack[5].len){
			nmea_gga->sig = nmea_atoi(pack[5].buffer,pack[5].len, 16);
//			printf("pack[5].len:%d\r\n",pack[5].len);
		}
		//ʹ��������������00��12(��һ����Ҳ������)
		if(pack[6].len){
//			printf("pack[6].len:%d\r\n",pack[6].len);
			nmea_gga->satinuse = nmea_atoi(pack[6].buffer,pack[6].len, 16); 
		}			
		//ˮƽ��ȷ�ȣ�0.5��99.9
		if(pack[7].len){
//			printf("pack[7].len:%d\r\n",pack[7].len);
			nmea_gga->HDOP = nmea_atof(pack[7].buffer,pack[7].len); 
//			printf("nmea_gga->HDOP:%f\r\n",nmea_gga->HDOP);
		}
		 //�����뺣ƽ��ĸ߶ȣ�-9999.9��9999.9��
		if(pack[8].len)
			nmea_gga->elv = pack[8].buffer[0]; 
		//ָ��λ��
		if(pack[9].len)
			nmea_gga->elv_units = nmea_atof(pack[9].buffer,pack[9].len);
		//���ˮ׼��߶ȣ�-9999.9��9999.9��
		if(pack[10].len)
			nmea_gga->diff = nmea_atof(pack[10].buffer,pack[10].len);
		//ָ��λ��
		if(pack[11].len)
			nmea_gga->diff_units = pack[11].buffer[0];
		//���GPS��������
		if(pack[12].len){
			nmea_gga->dgps_age = nmea_atof(pack[12].buffer,pack[12].len);
//			printf("pack[12].len:%d\r\n",pack[12].len);
		}
		// ��ֲο���վ���
		if(pack[13].len)
			nmea_gga->dgps_sid = nmea_atoi(pack[13].buffer,pack[13].len, 16);
  }
}
/*
* \brief  GPGSA paragraph analysis
 * @param  use_buffer:  Between the '$' and '*' data buffer
 * @param use_len: Between the '$' and '*' data length
 * @param nmea_gsa: GPGSA data 
 * @return  null
*/
void NMEA_Parse_GPGSA(const char *use_buffer,uint8_t use_len,nmeaGPGSA *nmea_gsa)
{
	uint8_t i,Paragraph_num;
	memset(nmea_gsa,0,sizeof(nmeaGPGSA));
  Paragraph_num = GET_Comma_Between_Paragraphs(use_buffer,use_len);
	if(Paragraph_num == 0x11){ //GPGSA�ܹ�14������
		// ģʽ2��M = �ֶ��� A = �Զ�
		if(pack[0].len)
			nmea_gsa->fix_mode = pack[0].buffer[0];
		//ģʽ1����λ��ʽ1 = δ��λ��2 = ��ά��λ��3 = ��ά��λ��
		if(pack[1].len)
			nmea_gsa->fix_type = nmea_atoi(pack[1].buffer,pack[1].len, 16);
		//����PRN����
		for(i=0;i<12;i++){
			if(pack[i+2].len)
			  nmea_gsa->sat_prn[i] = nmea_atoi(pack[i+2].buffer,pack[i+2].len, 16);
		}	
		// PDOP�ۺ�λ�þ�������
		if(pack[14].len)
			nmea_gsa->PDOP = nmea_atof(pack[14].buffer,pack[14].len); 
		//HDOPˮƽ�������ӣ�0.5 �\ 99.9��
		if(pack[15].len){
			nmea_gsa->HDOP = nmea_atof(pack[15].buffer,pack[15].len); 
//			printf("nmea_gsa->HDOP:%f\r\n",nmea_gsa->HDOP);
		}
		// VDOP��ֱ�������ӣ�0.5 �\ 99.9��
		if(pack[16].len)
			nmea_gsa->VDOP = nmea_atof(pack[16].buffer,pack[16].len);  		
	}
}
/*
* \brief  GPGSV paragraph analysis
 * @param  use_buffer:  Between the '$' and '*' data buffer
 * @param use_len: Between the '$' and '*' data length
 * @param nmea_gsv: GPGSV data 
 * @return  null
*/
void NMEA_Parse_GPGSV(const char *use_buffer,uint8_t use_len,nmeaGPGSV *nmea_gsv)
{
	uint8_t i,sat_num,Paragraph_num;
	memset(nmea_gsv,0,sizeof(nmeaGPGSV));
  Paragraph_num = GET_Comma_Between_Paragraphs(use_buffer,use_len);
	//GPGSV�ܹ����յ�����������Ϣ
	sat_num = Paragraph_num/4;
	if(sat_num){
	  // GSV��������
		if(pack[0].len)
			nmea_gsv->pack_count = nmea_atoi(pack[0].buffer,pack[0].len, 16);
		//����GSV�ı��
		if(pack[1].len)
			nmea_gsv->pack_index = nmea_atoi(pack[1].buffer,pack[1].len, 16);
		//�ɼ����ǵ�������00~12��ǰ���0Ҳ�������䣩
		if(pack[2].len)
			nmea_gsv->sat_count = nmea_atoi(pack[2].buffer,pack[2].len, 16);
		for(i=0;i<sat_num;i++){
			//���Ǳ�ţ�01��32
			if(pack[3+i*4].len)
				nmea_gsv->sat_data[i].id = nmea_atoi(pack[3+i*4].buffer,pack[3+i*4].len, 16);
			//�������ǣ�00��90��
			if(pack[4+i*4].len)
				nmea_gsv->sat_data[i].elv = nmea_atoi(pack[4+i*4].buffer,pack[4+i*4].len, 16); 
			//���Ƿ�λ�ǣ�000��359�ȡ�ʵ��ֵ��
			if(pack[5+i*4].len)
				nmea_gsv->sat_data[i].azimuth = nmea_atoi(pack[5+i*4].buffer,pack[5+i*4].len, 16);
			//����ȣ�C/No����00��99dB���ޱ�δ���յ�Ѷ��
			if(pack[6+i*4].len)
				nmea_gsv->sat_data[i].sig =nmea_atoi(pack[6+i*4].buffer,pack[6+i*4].len, 16);	
		}		
	}
}

/*
* \brief  GPRMC paragraph analysis
 * @param  use_buffer:  Between the '$' and '*' data buffer
 * @param use_len: Between the '$' and '*' data length
 * @param nmea_rmc: GPRMC data 
 * @return  null
*/
void NMEA_Parse_GPRMC(const char *use_buffer,uint8_t use_len,nmeaGPRMC *nmea_rmc)
{
	uint8_t i,Paragraph_num;
	memset(nmea_rmc,0,sizeof(nmeaGPRMC));
  Paragraph_num = GET_Comma_Between_Paragraphs(use_buffer,use_len);
  if(Paragraph_num == 0x0c){
	  //  UTC��Coordinated Universal Time��ʱ�䣬hhmmss��ʱ���룩��ʽ 
		if(pack[0].len)
			GET_NMEA_TIME(pack[0].buffer, pack[0].len,&(nmea_rmc->utc),TIME_TYPE_HHMMSS); 
		//��λ״̬��A=��Ч��λ��V=��Ч��λ
		if(pack[1].len)
			nmea_rmc->status =pack[1].buffer[0];
		//Latitude��γ��ddmm.mmmm���ȷ֣���ʽ��ǰ��λ��������0��
		if(pack[2].len)
			nmea_rmc->lat = nmea_atof(pack[2].buffer, pack[2].len);
		//γ�Ȱ���N ��S(��γ����γ)
		if(pack[3].len)
			nmea_rmc->ns = pack[3].buffer[0];
		//���ȣ���ʽΪddmm.mmmm
		if(pack[4].len)
			nmea_rmc->lon = nmea_atof(pack[4].buffer,pack[4].len); 
		//���Ȱ���E ��W(����������)
		if(pack[5].len)
			nmea_rmc->ew = pack[5].buffer[0]; 
		//�������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩
		if(pack[6].len)
			nmea_rmc->speed = nmea_atof(pack[6].buffer,pack[6].len); 
		//���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼��ǰ���0Ҳ�������䣩
		if(pack[7].len)
			nmea_rmc->direction = nmea_atof(pack[7].buffer,pack[7].len);  
		//UTC���ڣ�ddmmyy�������꣩��ʽ
		if(pack[8].len)
			GET_NMEA_TIME(pack[8].buffer, pack[8].len,&(nmea_rmc->utc),TIME_TYPE_DDMMYY);
		//��ƫ�ǣ�000.0~180.0�ȣ�ǰ���0Ҳ�������䣩
		if(pack[9].len)
			nmea_rmc->declination = nmea_atof(pack[9].buffer,pack[9].len); 
		 //��ƫ�Ƿ���E��������W������
		if(pack[10].len)
			nmea_rmc->declin_ew = pack[10].buffer[0]; 
		//ģʽָʾ����NMEA01833.00�汾�����A=������λ��D=��֣�E=���㣬N=������Ч
		if(pack[11].len)
			nmea_rmc->mode = pack[11].buffer[0]; ;
	}
}
/*
* \brief  GPVTG paragraph analysis
 * @param  use_buffer:  Between the '$' and '*' data buffer
 * @param use_len: Between the '$' and '*' data length
 * @param nmea_vtg: GPVTG data 
 * @return  null
*/
void NMEA_Parse_GPVTG(const char *use_buffer,uint8_t use_len,nmeaGPVTG *nmea_vtg)
{
	uint8_t i,Paragraph_num;
	memset(nmea_vtg,0,sizeof(nmeaGPVTG));
  Paragraph_num = GET_Comma_Between_Paragraphs(use_buffer,use_len);
	if(Paragraph_num == 0x09){
		// ���汱Ϊ�ο���׼�ĵ��溽��000~359��
		if(pack[0].len)
			nmea_vtg->dir = nmea_atof(pack[2].buffer, pack[2].len); 
		//T
		if(pack[1].len)
			nmea_vtg->dir_t =pack[1].buffer[0];
		//�Դű�Ϊ�ο���׼�ĵ��溽��000~359��
		if(pack[2].len)
			nmea_vtg->dec = nmea_atof(pack[2].buffer, pack[2].len);
		//M
		if(pack[3].len)
			nmea_vtg->dec_m = pack[3].buffer[0];
		//�������ʣ�000.0~999.9��
		if(pack[4].len)
			nmea_vtg->spn = nmea_atof(pack[4].buffer,pack[4].len); 
		//N
		if(pack[5].len)
			nmea_vtg->spn_n = pack[5].buffer[0]; 
		//�������ʣ�0000.0~1851.8����/Сʱ
		if(pack[6].len)
			nmea_vtg->spk = nmea_atof(pack[6].buffer,pack[6].len); 
		//K
		if(pack[7].len)
			nmea_vtg->spk_k = pack[7].buffer[0];  
		//K
		if(pack[8].len)
			nmea_vtg->mode = pack[8].buffer[0]; 
	}
}

/*
* \brief  GPGLL paragraph analysis
 * @param  use_buffer:  Between the '$' and '*' data buffer
 * @param use_len: Between the '$' and '*' data length
 * @param nmea_gll: GPGLL data 
 * @return  null
*/
void NMEA_Parse_GPGLL(const char *use_buffer,uint8_t use_len,nmeaGPGLL *nmea_gll)
{
	uint8_t i,Paragraph_num;
	memset(nmea_gll,0,sizeof(nmeaGPGLL));
  Paragraph_num = GET_Comma_Between_Paragraphs(use_buffer,use_len);
	if(Paragraph_num == 0x07){
		//Latitude��γ��ddmm.mmmm���ȷ֣���ʽ��ǰ��λ��������0��
		if(pack[0].len)
			nmea_gll->lat = nmea_atof(pack[0].buffer, pack[0].len);
		//γ�Ȱ���N ��S(��γ����γ)
		if(pack[1].len)
			nmea_gll->ns = pack[1].buffer[0];
		//���ȣ���ʽΪddmm.mmmm
		if(pack[2].len)
			nmea_gll->lon = nmea_atof(pack[2].buffer,pack[2].len); 
		//���Ȱ���E ��W(����������)
		if(pack[3].len)
			nmea_gll->ew = pack[3].buffer[0];
		//UTCʱ�䣬hhmmss��ʱ���룩��ʽ
		if(pack[4].len)
			GET_NMEA_TIME(pack[4].buffer, pack[4].len,&(nmea_gll->utc),TIME_TYPE_HHMMSS); 
		//��λ״̬��A=��Ч��λ��V=��Ч��λ
		if(pack[5].len)
			nmea_gll->status =pack[5].buffer[0];
    //ģʽָʾ
		if(pack[6].len)
			nmea_gll->mode =pack[6].buffer[0];		
	}
}
//��ȡ������Ϣ��У��CRC   Paragraph information and check CRC 
int Draw_Parse_Message_CRC(char *gps_buffer,parse_store *pack,uint16_t total_size)  
{
	uint8_t start_falg = 0;
	int crc = 0,recv_crc = 0;
	int result = -1;
  for(;pack->tail_size<=total_size;pack->tail_size++,gps_buffer++){
		if(*gps_buffer =='$'){
			pack->buffer_size = 0x00;
			start_falg = 0x01; //��ʾ�ҵ�'$'Э��ͷ
			pack->buffer[pack->buffer_size++] = *gps_buffer ;
		}
		else if((*gps_buffer =='*')&&(start_falg)){
			if((gps_buffer[3] =='\r')&&(gps_buffer[4] =='\n')){
			   start_falg = 0x00;
				 recv_crc=nmea_atoi(gps_buffer+1, 2, 16);
				 if(recv_crc == crc){
					 pack->buffer[pack->buffer_size++] = *gps_buffer ;
					 result = pack->tail_size;
					 break;
				 }
			}
			
		}
		else if(start_falg){
		  crc ^= *gps_buffer ;
			pack->buffer[pack->buffer_size++] = *gps_buffer ;
		}
	}
	return result;
}

//��׼NMEA-0183 Э�����   NMEA-0183 protocol analysis 
/*
* \brief   NMEA-0183 protocol analysis 
 * @param  gps_buffer:  GPS receive all the data
 * @param  total_size:  GPS receive data length
 * @param  nmea_info: Protocol parsing extract useful data  
 * @return  Whether to receive useful data  
*/
uint8_t NMEA0183_protocol_analysis(char *gps_buffer,uint16_t total_size,nmeaINFO *nmea_info)
{
	nmeaGPGGA nmea_gga;
	nmeaGPGSA nmea_gsa;
	nmeaGPGSV nmea_gsv;
	nmeaGPRMC nmea_rmc;
	nmeaGPVTG nmea_vtg;
	nmeaGPGLL nmea_gll;
  static parse_store  parser = {0x00};
	//Extract paragraphs and check CRC  
	if(Draw_Parse_Message_CRC(gps_buffer,&parser,total_size)!=-1){
		//Paragraph types
	   parser.parse_type = nmea_pack_type(parser.buffer);
				//�ж϶������ͣ����ݲ�ͬ���ͽ��н���
				switch(parser.parse_type){
					case GPGGA:
            //����2��ӡ����(����)						
						USART_Write_LEN((u8 *)parser.buffer,parser.buffer_size,2);
					  printf("\r\n");
					  //��������
						NMEA_Parse_GPGGA(parser.buffer,parser.buffer_size,&nmea_gga);
					  memcpy(&nmea_info->utc,&nmea_gga.utc,sizeof(nmeaTIME));
					  nmea_info->lat = nmea_gga.lat;
					  nmea_info->lon = nmea_gga.lon;
					  nmea_info->HDOP = nmea_gga.HDOP;
            break;
					
					case GPGSA:	
						//����2��ӡ����(����)						
						USART_Write_LEN((u8 *)parser.buffer,parser.buffer_size,2);
					  printf("\r\n");
					  //��������
  				  NMEA_Parse_GPGSA(parser.buffer,parser.buffer_size,&nmea_gsa);
  					nmea_info->HDOP = nmea_gsa.HDOP;
					  nmea_info->PDOP = nmea_gsa.PDOP;
					  nmea_info->VDOP = nmea_gsa.VDOP;
            break;
					
          case GPGSV:	
						//����2��ӡ����(����)						
						USART_Write_LEN((u8 *)parser.buffer,parser.buffer_size,2);
					  printf("\r\n");
					  //��������						
						NMEA_Parse_GPGSV(parser.buffer,parser.buffer_size,&nmea_gsv);
            break;	
					
					case GPRMC:	
						//����2��ӡ����(����)						
						USART_Write_LEN((u8 *)parser.buffer,parser.buffer_size,2);
					  printf("\r\n");
					  //��������
  					NMEA_Parse_GPRMC(parser.buffer,parser.buffer_size,&nmea_rmc);
					  nmea_info->speed = nmea_rmc.speed;
					  nmea_info->direction = nmea_rmc.direction;
					  nmea_info->declination = nmea_rmc.declination;
            break;
					
					case GPVTG:
						//����2��ӡ����(����)						
						USART_Write_LEN((u8 *)(parser.buffer),parser.buffer_size,2);
					  printf("\r\n");
					  //��������
   					NMEA_Parse_GPVTG(parser.buffer,parser.buffer_size,&nmea_vtg);
            break;
					
					case GPGLL:
  					//����2��ӡ����(����)						
						USART_Write_LEN((u8 *)parser.buffer,parser.buffer_size,2);
					  printf("\r\n");
					  //��������	
						NMEA_Parse_GPGLL(parser.buffer,parser.buffer_size,&nmea_gll);
            break;
					
					case GPNON:	
						parser.tail_size = 0x00;
						return FALSE;
            break;
				
				}
				
			}
	else{
	  parser.tail_size = 0x00;
		return FALSE;
	}

}
	
//GPS���ݽ��� ��ȡGPS��������
void GPS_Parse_Decode(void)
{	
		//The DMA to receive all completed
  if(GPS_TransferEnd){		
		//Use a serial port 3 print receiving data
//	   USART_Write_LEN(gps_recv_buff,GPS_RBUFF_SIZE,3);
		// NMEA0183 protocol analysis
		 if(NMEA0183_protocol_analysis(gps_recv_buff,GPS_RBUFF_SIZE,&info)){
			 
			 printf("\r\nTime:%d-%d-%d %d:%d:%d",info.utc.year,info.utc.mon,info.utc.day,info.utc.hour,info.utc.min,info.utc.sec);
			// printf("lon = %f, lat = %f,info.HDOP:%f\r\n ",info.lon,info.lat,info.HDOP);
			 printf("lon = %f, lat = %f \r\n ",info.lon,info.lat); 
		

			 //printf("info.speed:%f,info.direction:%f,info.declination:%f\r\n ",info.speed,info.direction,info.declination);
		 }
	   GPS_TransferEnd = 0x00;
	}
	//The DMA half receive complete
	if(GPS_HalfTransferEnd){
		//Use a serial port 3 print receiving data
//	   USART_Write_LEN(gps_recv_buff,HALF_GPS_RBUFF_SIZE,3);
		 // NMEA0183 protocol analysis
		 if(NMEA0183_protocol_analysis(gps_recv_buff,HALF_GPS_RBUFF_SIZE,&info)){		
//		   printf("Time:%d-%d-%d %d:%d:%d",info.utc.year,info.utc.mon,info.utc.day,info.utc.hour,info.utc.min,info.utc.sec);
//			 printf("info.lon:%f,info.lat:%f,info.HDOP:%f\r\n ",info.lon,info.lat,info.HDOP); 
//			 printf("info.speed:%f,info.direction:%f,info.declination:%f\r\n ",info.speed,info.direction,info.declination); 
			 printf("lon = %f, lat = %f \r\n ",info.lon,info.lat);
 	
		 }
	   GPS_HalfTransferEnd = 0x00;
	}
}









