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
#include "GebraBit_SHT35.h"

extern I2C_HandleTypeDef 		hi2c1;

/*=========================================================================================================================================
 * @param     cmd    Command that will be writen 
 * @return    None
 ========================================================================================================================================*/
void GB_SHT35_Write_Command(GebraBit_SHT35 * SHT35 , uint16_t cmd)
{
	SHT35->COMMAND = cmd ;
	uint8_t cmd_buffer[2]= {(cmd & 0xFF00) >> 8, cmd & 0xFF};;
	HAL_I2C_Master_Transmit(SHT35_I2C,SHT35_WRITE_ADDRESS,cmd_buffer,2,100);
}
/*=========================================================================================================================================
 * @brief     Check CRC
 * @param     SHT35   SHT35 Struct  CRC_CHECK variable
 * @param     value        Value that must be compare with crc
 * @param     crc          CRC Value
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_CRC_Check( GebraBit_SHT35 * SHT35 , uint16_t value, uint8_t crc)
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)value<<8; // Pad with zeros as specified in spec
	
	while( msb != 0x80 ) {
		
		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);
			
		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	if( result == crc )
		SHT35->CRC_CHECK = CRC_OK;
	else
		SHT35->CRC_CHECK = CRC_ERROR;
} 

/*=========================================================================================================================================
 * @brief     Reset SHT35
 * @param     SHT35   SHT35 Struct RESET variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Soft_Reset ( GebraBit_SHT35 * SHT35 )  
{
  GB_SHT35_Write_Command( SHT35 ,SHT35_SOFT_RESET);
	HAL_Delay(SHT35_SOFT_RESET_TIME);
	SHT35->RESET = DETECTED ;
}

/*=========================================================================================================================================
 * @brief     Enable Or Disable On Chip Heater
 * @param     SHT35   SHT35 Struct ON_CHIP_HEATER variable
 * @param     heater        Value is from SHT35_Heater Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_On_Chip_Heater ( GebraBit_SHT35 * SHT35 , SHT35_Heater heater )  
{
  GB_SHT35_Write_Command( SHT35 ,heater);
  SHT35->ON_CHIP_HEATER= heater ; 
}
/*=========================================================================================================================================
 * @brief     Start 4Hz Aquiring Data
 * @param     SHT35   SHT35 Struct ART variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Start_4Hz_Aquiring_Data(GebraBit_SHT35 * SHT35)
{
  GB_SHT35_Write_Command( SHT35 ,SHT35_ART);
	SHT35->ART = Enable;
}
/*=========================================================================================================================================
 * @brief     Clear Status Register
 * @param     SHT35   SHT35 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Clear_Status_Register ( GebraBit_SHT35 * SHT35  ) 
{
	GB_SHT35_Write_Command( SHT35 ,SHT35_CLEAR_STATUS_REGISTER);
}	
/*=========================================================================================================================================
 * @brief     Read SHT35 Status Register
 * @param     SHT35   SHT35 Struct  variables
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Read_Status_Register ( GebraBit_SHT35 * SHT35  ) 
{
 uint8_t temp_buffer[3];
 HAL_I2C_Mem_Read(SHT35_I2C, SHT35_READ_ADDRESS, SHT35_READOUT_STATUS_REGISTER, I2C_MEMADD_SIZE_16BIT, temp_buffer ,3, 200);
 SHT35->STATUS_REGISTER = ((uint16_t)temp_buffer[0]<<8)|((uint16_t)temp_buffer[1]) ;
 SHT35->ALERT_PENDING 							= (SHT35->STATUS_REGISTER & 0x8000)>>15 ;
 SHT35->ON_CHIP_HEATER 						  = ((SHT35->STATUS_REGISTER& 0x2000)>>13)?(HEATER_ENABLE):(HEATER_DISABLE);
 SHT35->HUMIDITY_ALERT              = (SHT35->STATUS_REGISTER & 0x0800)>>11 ;
 SHT35->TEMPERATURE_ALERT           = (SHT35->STATUS_REGISTER & 0x0400)>>10 ;
 SHT35->RESET   										= (SHT35->STATUS_REGISTER & 0x10  )>>4  ;
 SHT35->COMMAND_STATUS						  = (SHT35->STATUS_REGISTER & 0x02  )>>2 ;
 SHT35->CHECKSUM									  = (SHT35->STATUS_REGISTER & 0x01  )>>1 ;
 SHT35->SHT35_CRC = temp_buffer[2] ;
 GB_SHT35_Clear_Status_Register(SHT35);
 GB_SHT35_CRC_Check( SHT35 , (uint16_t)SHT35->STATUS_REGISTER , SHT35->SHT35_CRC) ;
}

/*=========================================================================================================================================
 * @brief     Stop Periodic Data Acquisition
 * @param     SHT35   SHT35 Struct 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Break_Stop_Periodic_Data_Acquisition(GebraBit_SHT35 * SHT35)
{
  GB_SHT35_Write_Command( SHT35 ,SHT35_BREAK_STOP);
}
/*=========================================================================================================================================
 * @brief     Set periodic Data Acquisition
 * @param     SHT35   SHT35 Struct PERIODIC_MODE variable
 * @param     mode        Value is from SHT35_Periodic_Mode Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Periodic_Data_Acquisition(GebraBit_SHT35 * SHT35 , SHT35_Periodic_Mode mode)
{
  GB_SHT35_Write_Command( SHT35 ,mode);
	SHT35->PERIODIC_MODE = mode ; 
}
/*=========================================================================================================================================
 * @brief     initialize SHT35
 * @param     SHT35     SHT35 Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_SHT35_initialize( GebraBit_SHT35 * SHT35 )
{
  GB_SHT35_Soft_Reset   ( SHT35 ) ;
  GB_SHT35_Read_Status_Register ( SHT35 ) ;
}
/*=========================================================================================================================================
 * @brief     Configure SHT35
 * @param     SHT35  Configure SHT35 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_SHT35_Configuration(GebraBit_SHT35 * SHT35)
{
  GB_SHT35_On_Chip_Heater( SHT35 , HEATER_DISABLE ) ;
	GB_SHT35_Periodic_Data_Acquisition( SHT35 , SHT35_MEDIUM_REPEATABILITY_10_MPS_FREQUENCY );
} 
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     Read SHT35 ADC Temperature , Humidity Raw Data
 * @param     SHT35   SHT35 Struct RAW_HUMIDITY variable and RAW_TEMPERATURE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Read_Raw_Temperature_Humidity ( GebraBit_SHT35 * SHT35 ) 
{
 HAL_I2C_Mem_Read(SHT35_I2C,SHT35_READ_ADDRESS,SHT35_READOUT_PERIODIC_MEASUREMENT_RESULT,I2C_MEMADD_SIZE_16BIT,SHT35->ADC_RAW_DATA,ADC_RAW_DATA_BUFFER_SIZE,200);
 SHT35->RAW_TEMPERATURE = ((uint16_t)SHT35->ADC_RAW_DATA[0]<< 8) | ((uint16_t)SHT35->ADC_RAW_DATA[1]);
 SHT35->SHT35_CRC = SHT35->ADC_RAW_DATA[2] ;
 GB_SHT35_CRC_Check( SHT35 , SHT35->RAW_TEMPERATURE , SHT35->SHT35_CRC) ;	
 SHT35->RAW_HUMIDITY    = ((uint16_t)SHT35->ADC_RAW_DATA[3]<< 8) | ((uint16_t)SHT35->ADC_RAW_DATA[4]); 
 SHT35->SHT35_CRC = SHT35->ADC_RAW_DATA[5] ;
 GB_SHT35_CRC_Check( SHT35 , SHT35->RAW_HUMIDITY , SHT35->SHT35_CRC) ;
 HAL_Delay(200);
}
/*=========================================================================================================================================
 * @brief     Calculate Temperature
 * @param     SHT35   SHT35 Struct TEMPERATURE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Temperature ( GebraBit_SHT35 * SHT35  ) 
{
	SHT35->TEMPERATURE = ((float)SHT35->RAW_TEMPERATURE / 65535.0f) * 175.0f - 45.0f;	
}
/*=========================================================================================================================================
 * @brief     Calculate HUMIDITY
 * @param     SHT35   SHT35 Struct TEMPERATURE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Humidity ( GebraBit_SHT35 * SHT35  )  
{
	SHT35->HUMIDITY   =  ((float)SHT35->RAW_HUMIDITY / 65535.0f) * 100.0f;	
}
/*=========================================================================================================================================
 * @brief     Calculate Dew Point
 * @param     SHT35   SHT35 Struct DEW_POINT variable and PARTIAL_PRESSURE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_SHT35_Dew_Point( GebraBit_SHT35 * SHT35  ) 
{
	double partial_pressure;
	double dew_point;
	
	// Missing power of 10
//	SHT35->PARTIAL_PRESSURE = pow( 10, SHT35_CONSTANT_A - SHT35_CONSTANT_B / (SHT35->TEMPERATURE + SHT35_CONSTANT_C) );
//	SHT35->DEW_POINT        = - SHT35_CONSTANT_B / (log10( SHT35->COMPANSATED_HUMIDITY * partial_pressure / 100) - SHT35_CONSTANT_A) - SHT35_CONSTANT_C;
	
}
/*=========================================================================================================================================
 * @brief     Get Data  
 * @param     SHT35     GebraBit_SHT35 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_SHT35_Get_Data(GebraBit_SHT35 * SHT35)
{
	GB_SHT35_Read_Raw_Temperature_Humidity(SHT35);
	GB_SHT35_Read_Status_Register ( SHT35 ) ;
	GB_SHT35_Temperature( SHT35  );
	GB_SHT35_Humidity( SHT35  );
//	GB_SHT35_Dew_Point( SHT35  );
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/
//  GB_SHT35_Read_User_Register(&data);
//	GB_SHT35_Write_User_Register(0x83);
//	GB_SHT35_Read_User_Register_Bits(START_MSB_BIT_AT_7, BIT_LENGTH_4, &SHT35_Module.Register_Cache);
//	GB_SHT35_Read_User_Register_Bits(START_MSB_BIT_AT_3, BIT_LENGTH_4, &SHT35_Module.Register_Cache);
//	GB_SHT35_Write_User_Register_Bits(START_MSB_BIT_AT_7, BIT_LENGTH_4, 0);
//	GB_SHT35_Write_User_Register_Bits(START_MSB_BIT_AT_3, BIT_LENGTH_4, 3);
//	GB_SHT35_Read_User_Register(&data);