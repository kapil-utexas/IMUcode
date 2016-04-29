#ifndef __GPS_DECODE_H
#define __GPS_DECODE_H

//include library file 
#include "stm32f4xx_conf.h"
#include "stm32f4xx_dma.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

#include "stdio.h"
#include "usart.h"
#include "sdio_sdcard.h"
#include "exfuns.h" 

#define NMEA_SIG_BAD        (0)
#define NMEA_SIG_LOW        (1)
#define NMEA_SIG_MID        (2)
#define NMEA_SIG_HIGH       (3)

#define NMEA_FIX_BAD        (1)
#define NMEA_FIX_2D         (2)
#define NMEA_FIX_3D         (3)

#define NMEA_MAXSAT         (12)
#define NMEA_SATINPACK      (4)
#define NMEA_NSATPACKS      (NMEA_MAXSAT / NMEA_SATINPACK)

/* GPS接口配置 使用不同的串口时，要修改对应的接口 */
#define GPS_RBUFF_SIZE            512                   //串口接收缓冲区大小
#define HALF_GPS_RBUFF_SIZE       (GPS_RBUFF_SIZE/2)    //串口接收缓冲区一半  
#define PARSE_BUFER_SIZE    128
#define PARSE_MAX_SIZE    20
#define PARSE_MAX_NUM     20

#define  TIME_TYPE_DDMMYY  2
#define  TIME_TYPE_HHMMSS  1

#pragma pack(1)
/**
 * Information about satellite
 * @see nmeaSATINFO
 * @see nmeaGPGSV
 */
typedef struct _nmeaSATELLITE
{
    int     id;         /**< Satellite PRN number */
    int     in_use;     /**< Used in position fix */
    int     elv;        /**< Elevation in degrees, 90 maximum */
    int     azimuth;    /**< Azimuth, degrees from true north, 000 to 359 */
    int     sig;        /**< Signal, 00-99 dB */

} nmeaSATELLITE;

/**
 * Date and time data
 * @see nmea_time_now
 */
typedef struct _nmeaTIME
{
    int     year;       /**< Years since 1900 */
    int     mon;        /**< Months since January - [0,11] */
    int     day;        /**< Day of the month - [1,31] */
    int     hour;       /**< Hours since midnight - [0,23] */
    int     min;        /**< Minutes after the hour - [0,59] */
    int     sec;        /**< Seconds after the minute - [0,59] */
    int     hsec;       /**< Hundredth part of second - [0,99] */

} nmeaTIME;

/**
 * GGA packet information structure (Global Positioning System Fix Data)
 */
typedef struct _nmea_GPGGA{
  nmeaTIME utc;       /**< UTC of position (just time) */
	double  lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
    char    ns;         /**< [N]orth or [S]outh */
	double  lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
    char    ew;         /**< [E]ast or [W]est */
    int     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
	int     satinuse;   /**< Number of satellites in use (not those in view) */
    double  HDOP;       /**< Horizontal dilution of precision */
    double  elv;        /**< Antenna altitude above/below mean sea level (geoid) */
    char    elv_units;  /**< [M]eters (Antenna height unit) */
    double  diff;       /**< Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level. '-' = geoid is below WGS-84 ellipsoid) */
    char    diff_units; /**< [M]eters (Units of geoidal separation) */
    double  dgps_age;   /**< Time in seconds since last DGPS update */
    int     dgps_sid;   /**< DGPS station ID number */

} nmeaGPGGA;

/**
 * GSA packet information structure (Satellite status)
 */
typedef struct _nmeaGPGSA
{
    char    fix_mode;   /**< Mode (M = Manual, forced to operate in 2D or 3D; A = Automatic, 3D/2D) */
    int     fix_type;   /**< Type, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */
    int     sat_prn[NMEA_MAXSAT]; /**< PRNs of satellites used in position fix (null for unused fields) */
    double  PDOP;       /**< Dilution of precision */
    double  HDOP;       /**< Horizontal dilution of precision */
    double  VDOP;       /**< Vertical dilution of precision */

} nmeaGPGSA;

/**
 * GSV packet information structure (Satellites in view)
 */
typedef struct _nmeaGPGSV
{
    int     pack_count; /**< Total number of messages of this type in this cycle */
    int     pack_index; /**< Message number */
    int     sat_count;  /**< Total number of satellites in view */
    nmeaSATELLITE sat_data[NMEA_SATINPACK];

} nmeaGPGSV;

/**
 * RMC packet information structure (Recommended Minimum sentence C)
 */
typedef struct _nmeaGPRMC
{
    nmeaTIME utc;       /**< UTC of position */
    char    status;     /**< Status (A = active or V = void) */
	double  lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
    char    ns;         /**< [N]orth or [S]outh */
	double  lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
    char    ew;         /**< [E]ast or [W]est */
    double  speed;      /**< Speed over the ground in knots */
    double  direction;  /**< Track angle in degrees True */
    double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */
    char    declin_ew;  /**< [E]ast or [W]est */
    char    mode;       /**< Mode indicator of fix type (A = autonomous, D = differential, E = estimated, N = not valid, S = simulator) */

} nmeaGPRMC;

/**
 * VTG packet information structure (Track made good and ground speed)
 */
typedef struct _nmeaGPVTG
{
    double  dir;        /**< True track made good (degrees) */
    char    dir_t;      /**< Fixed text 'T' indicates that track made good is relative to true north */
    double  dec;        /**< Magnetic track made good */
    char    dec_m;      /**< Fixed text 'M' */
    double  spn;        /**< Ground speed, knots */
    char    spn_n;      /**< Fixed text 'N' indicates that speed over ground is in knots */
    double  spk;        /**< Ground speed, kilometers per hour */
    char    spk_k;      /**< Fixed text 'K' indicates that speed over ground is in kilometers/hour */
    char    mode;
} nmeaGPVTG;

/**
 * GLL packet information structure (Track made good and ground speed)
 */
typedef struct _nmeaGPGLL
{
	double  lat;        /**< Latitude in NDEG - [degree][min].[sec/60] */
    char    ns;         /**< [N]orth or [S]outh */
	double  lon;        /**< Longitude in NDEG - [degree][min].[sec/60] */
    char    ew;         /**< [E]ast or [W]est */
   nmeaTIME utc;       /**< UTC of position */
    char    status;     /**< Status (A = active or V = void) */
	  char   mode;
} nmeaGPGLL;

/**
 * Information about all satellites in view
 * @see nmeaINFO
 * @see nmeaGPGSV
 */
typedef struct _nmeaSATINFO
{
    int     inuse;      /**< Number of satellites in use (not those in view) */
    int     inview;     /**< Total number of satellites in view */
    nmeaSATELLITE sat[NMEA_MAXSAT]; /**< Satellites information */

} nmeaSATINFO;


/**
 * Summary GPS information from all parsed packets,
 * used also for generating NMEA stream
 * @see nmea_parse
 * @see nmea_GPGGA2info,  nmea_...2info
 */
typedef struct _nmeaINFO
{
    int     smask;      /**< Mask specifying types of packages from which data have been obtained */

    nmeaTIME utc;       /**< UTC of position */

    int     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
    int     fix;        /**< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */

    double  PDOP;       /**< Position Dilution Of Precision */
    double  HDOP;       /**< Horizontal Dilution Of Precision */
    double  VDOP;       /**< Vertical Dilution Of Precision */

    double  lat;        /**< Latitude in NDEG - +/-[degree][min].[sec/60] */
    double  lon;        /**< Longitude in NDEG - +/-[degree][min].[sec/60] */
    double  elv;        /**< Antenna altitude above/below mean sea level (geoid) in meters */
    double  speed;      /**< Speed over the ground in kilometers/hour */
    double  direction;  /**< Track angle in degrees True */
    double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */

    nmeaSATINFO satinfo; /**< Satellites information */

} nmeaINFO;
//枚举定义GPS协议地址
typedef enum{GPNON,GPGGA,GPGSA,GPGSV,GPRMC,GPVTG,GPGLL}GPS_Parse_Type;

typedef struct _parse_store{
	
	char buffer[PARSE_BUFER_SIZE];
	uint8_t buffer_size;
	uint16_t tail_size; //当前尾部所在的位置长度
	GPS_Parse_Type  parse_type;
	
}parse_store;

typedef struct _parse_arry{	
	char buffer[PARSE_MAX_SIZE];
	uint8_t len;	
}Parse_Arry;


void GPS_Config(void);
void GPS_Parse_Decode(void);


#endif



