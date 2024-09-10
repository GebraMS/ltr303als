/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#ifndef	__SHT35_H__
#define	__SHT35_H__
#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "i2c.h"
/************************************************
 *              USER REGISTER MAP               *
 ***********************************************/ 
#define SHT35_I2C		              		    						    &hi2c1
#define SHT35_SOFT_RESET_TIME                             5
#define SHT35_ADDRESS 									  						    0x44
#define SHT35_WRITE_ADDRESS 			  		 							   ((SHT35_ADDRESS<<1)|0)
#define SHT35_READ_ADDRESS 						   							   ((SHT35_ADDRESS<<1)|1)
#define SHT35_READOUT_PERIODIC_MEASUREMENT_RESULT 			  0xE000
#define SHT35_ART 									  										0x2B32
#define SHT35_BREAK_STOP									  							0x3093
#define SHT35_SOFT_RESET 									  							0x30A2
#define SHT35_HEATER_ON 								 									0x306D
#define SHT35_HEATER_OFF 							   									0x3066
#define SHT35_READOUT_STATUS_REGISTER 										0xF32D
#define SHT35_CLEAR_STATUS_REGISTER 										  0x3041
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 
 /************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
#define ADC_RAW_DATA_BUFFER_SIZE              6
/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum Ability
{  
	Disable = 0     ,                      
	Enable     
}HTU31D_Ability; 
/**************************************************
 *    		 Values For Single Shot Mode			      *
 **************************************************/ 
typedef enum Single_Shot_Mode
{
 SHT35_HIGH_REPEATABILITY_CLOCK_STRETCHING      =  0x2C06 ,
 SHT35_MEDIUM_REPEATABILITY_CLOCK_STRETCHING    =  0x2C0D ,
 SHT35_LOW_REPEATABILITY_CLOCK_STRETCHING       =  0x2C10 ,
 SHT35_HIGH_REPEATABILITY_NO_CLOCK_STRETCHING   =  0x2400 ,
 SHT35_MEDIUM_REPEATABILITY_NO_CLOCK_STRETCHING =  0x240B ,
 SHT35_LOW_REPEATABILITY_NO_CLOCK_STRETCHING    =  0x2416
}SHT35_Single_Shot_Mode;
/**************************************************
 *  Values For Periodic Data Acquisition Mode			*
 **************************************************/ 
typedef enum Periodic_Data_Acquisition_Mode
{
 SHT35_HIGH_REPEATABILITY_0P5_MPS_FREQUENCY      =  0x2032 ,
 SHT35_MEDIUM_REPEATABILITY_0P5_MPS_FREQUENCY    =  0x2024 ,
 SHT35_LOW_REPEATABILITY_0P5_MPS_FREQUENCY       =  0x202F ,
 SHT35_HIGH_REPEATABILITY_1_MPS_FREQUENCY   		 =  0x2130 ,
 SHT35_MEDIUM_REPEATABILITY_1_MPS_FREQUENCY 		 =  0x2126 ,
 SHT35_LOW_REPEATABILITY_1_MPS_FREQUENCY    		 =  0x212D ,
 SHT35_HIGH_REPEATABILITY_2_MPS_FREQUENCY        =  0x2236 ,
 SHT35_MEDIUM_REPEATABILITY_2_MPS_FREQUENCY      =  0x2220 ,
 SHT35_LOW_REPEATABILITY_2_MPS_FREQUENCY         =  0x222B ,
 SHT35_HIGH_REPEATABILITY_4_MPS_FREQUENCY   		 =  0x2334 ,
 SHT35_MEDIUM_REPEATABILITY_4_MPS_FREQUENCY 		 =  0x2322 ,
 SHT35_LOW_REPEATABILITY_4_MPS_FREQUENCY    		 =  0x2329 ,
 SHT35_HIGH_REPEATABILITY_10_MPS_FREQUENCY   		 =  0x2737 ,
 SHT35_MEDIUM_REPEATABILITY_10_MPS_FREQUENCY 		 =  0x2721 ,
 SHT35_LOW_REPEATABILITY_10_MPS_FREQUENCY    		 =  0x272A 
}SHT35_Periodic_Mode;
/**************************************************
 *  					Values For Command Status						*
 **************************************************/ 
typedef enum Command_Status
{  
	LAST_COMMAND_PROCESSED = 0     ,                      
	LAST_COMMAND_NOT_PROCESSED   = 1    
}SHT35_Command_Status; 
/**************************************************
 *  					Values For Checksum Status					*
 **************************************************/ 
typedef enum Checksum_Status
{  
	LAST_WRITE_CHECKSUM_CORRECT = 0     ,                      
	LAST_WRITE_CHECKSUM_FAILED  = 1    
}SHT35_Checksum_Status; 
/**************************************************
 *  					Values For Measurement Time 			  *
 **************************************************/ 
typedef enum Measurement_Time
{
  HIGH_REPEATABILITY_15_mS      =  15 ,
  MEDIUM_REPEATABILITY_6_mS     =  6  ,
  LOW_REPEATABILITY_4_mS        =  4  ,
}SHT35_Measurement_Time;

/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum Reset
{  
  NOT_DETECTED = 0     ,                      
	DETECTED         
}SHT35_Reset;
/*************************************************
 *         			  Values For Heater			         *
 **************************************************/ 
typedef enum Heater 
{  
	HEATER_ENABLE  = SHT35_HEATER_ON     ,                      
	HEATER_DISABLE = SHT35_HEATER_OFF      
}SHT35_Heater;
/**************************************************
 *  			Values For Alert Pending Status					*
 **************************************************/ 
typedef enum Alert_Pending_Status 
{
  NO_PENDING_ALERT   					 = 0,
  AT_LEAST_ONE_PENDING_ALERT   = 1
} SHT35_Alert_Pending_Status;
/**************************************************
 *  								Values For Alert							*
 **************************************************/ 
typedef enum Alert 
{
  NO_ALERT = 0,
  ALERT    = 1
} SHT35_Alert;

/*************************************************
 *           Values For CRC Status    	         *
 **************************************************/ 
typedef enum CRC_Status 
{  
	CRC_ERROR = 0     ,                      
	CRC_OK     
}SHT35_CRC_Status;

 /*************************************************
 *  Defining SHT35 Register & Data As Struct   *
 **************************************************/
typedef	struct SHT35
{
	  uint8_t                       	   Register_Cache;
		SHT35_Reset					       				 RESET;
	  uint16_t													 COMMAND;
	  SHT35_Single_Shot_Mode             SINGLE_SHOT_MODE;
	  SHT35_Periodic_Mode                PERIODIC_MODE;
		SHT35_Measurement_Time             MEASUREMENT_TIME;
	  HTU31D_Ability                     ART;
	  uint16_t													 STATUS_REGISTER;
	  SHT35_Alert_Pending_Status         ALERT_PENDING;
	  SHT35_Heater								       ON_CHIP_HEATER;
	  SHT35_Alert												 HUMIDITY_ALERT;
	  SHT35_Alert												 TEMPERATURE_ALERT;
	  SHT35_Command_Status               COMMAND_STATUS;
	  SHT35_Checksum_Status 						 CHECKSUM;  
	  uint8_t 													 SHT35_CRC;
	  SHT35_CRC_Status									 CRC_CHECK;
		uint8_t                            ADC_RAW_DATA[ADC_RAW_DATA_BUFFER_SIZE];
		uint16_t                           RAW_TEMPERATURE;
		uint16_t													 RAW_HUMIDITY;
    float 														 TEMPERATURE;
		float 														 HUMIDITY;
//	  double														 PARTIAL_PRESSURE;
//		double 														 DEW_POINT;
}GebraBit_SHT35;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *  Declare Read&Write SHT35 Register Values Functions *
 ********************************************************/
extern void GB_SHT35_Write_Command(GebraBit_SHT35 * SHT35 , uint16_t cmd);
/********************************************************
 *       Declare MS5611 Configuration Functions         *
 ********************************************************/
extern void GB_SHT35_Soft_Reset ( GebraBit_SHT35 * SHT35 )  ;
extern void GB_SHT35_CRC_Check( GebraBit_SHT35 * SHT35 , uint16_t value, uint8_t crc) ;
extern void GB_SHT35_On_Chip_Heater ( GebraBit_SHT35 * SHT35 , SHT35_Heater heater )   ;
extern void GB_SHT35_Read_Serial_Number ( GebraBit_SHT35 * SHT35  )    ;
extern void GB_SHT35_Read_Diagnostic ( GebraBit_SHT35 * SHT35  )   ;
extern void GB_SHT35_Configuration(GebraBit_SHT35 * SHT35)  ;
extern void GB_SHT35_Start_Conversion ( GebraBit_SHT35 * SHT35   )   ;
extern void GB_SHT35_Read_Raw_Temperature_Humidity( GebraBit_SHT35 * SHT35 )  ;
extern void GB_SHT35_Temperature ( GebraBit_SHT35 * SHT35 )  ;
extern void GB_SHT35_Humidity ( GebraBit_SHT35 * SHT35 )   ;
extern void GB_SHT35_Dew_Point( GebraBit_SHT35 * SHT35  ) ;
extern void GB_SHT35_initialize( GebraBit_SHT35 * SHT35 )  ;
extern void GB_SHT35_Get_Data(GebraBit_SHT35 * SHT35);
#endif
