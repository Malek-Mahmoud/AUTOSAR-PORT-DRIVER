 /******************************************************************************
 *
 * Module: PORT 
 *
 * File Name: PORT_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - PORT Driver
 *
 * Author: Malek Mahmoud Mohammed
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/* Pre-compile option for presence of Port_SetPinDirection API */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

/* Pre-compile option for presence of Port_SetPinMode API */
#define PORT_SET_PIN_MODE_API                (STD_ON)

/* Number of the available Port Pinss */
#define PORT_numOfPins                       (43U)
/* Number of Configured Modess */
#define PORT_numOfConfiguriedMode            (10U)
   
/*conigured Modes*/
#define PORT_PIN_DIO_MODE                     (uint8)0X00
#define PORT_PIN_ADC_MODE                     (uint8)0X00   
#define PORT_PIN_UART_MODE                    (uint8)0X01   
#define PORT_PIN_SSI_MODE                     (uint8)0X02   
#define PORT_PIN_I2C_MODE                     (uint8)0X03
#define PORT_PIN_PWM0_MODE                    (uint8)0X04
#define PORT_PIN_FAULT_MODE                   (uint8)0X04   
#define PORT_PIN_PWM1_MODE                    (uint8)0X05
#define PORT_PIN_QEI_MODE                     (uint8)0X06
#define PORT_PIN_CPP_MODE                     (uint8)0X07
#define PORT_PIN_CAN_MODE                     (uint8)0X08
#define PORT_PIN_NMI_MODE                     (uint8)0X08
#define PORT_PIN_UART_CONTROL_MODE            (uint8)0X08   
#define PORT_PIN_USB_MODE                     (uint8)0X08   
#define PORT_PIN_ANALOG_COMP_MODE             (uint8)0X09

/*Configurate Defualt Mode as a DIO Mode*/
#define PORT_PIN_MODE_DEFUALT                 PORT_PIN_DIO_MODE
/*Configurate Defualt Direction as an Input Direction*/
#define PORT_PIN_DIR_DEFUALT                  (uint8)0X00 
/*Configurate Defualt Internal Resistor To OFF*/
#define PORT_PIN_Internal_Resistor_DEFUALT    (uint8)OFF
/*Configurate Defualt Initial value as a LOW*/
#define PORT_PIN_Initial_value_DEFUALT        (uint8)PORT_PIN_LOW
   
/* PINS Index in the array of structures in PORT_PBcfg.c */
//PORTA
#define PORTA_ONSET_INDEX                               (uint8)0x00
#define PORTA_PIN0_ID_INDEX				(uint8)0x00
#define PORTA_PIN1_ID_INDEX				(uint8)0x01
#define PORTA_PIN2_ID_INDEX				(uint8)0x02
#define PORTA_PIN3_ID_INDEX				(uint8)0x03
#define PORTA_PIN4_ID_INDEX				(uint8)0x04
#define PORTA_PIN5_ID_INDEX				(uint8)0x05
#define PORTA_PIN6_ID_INDEX				(uint8)0x06
#define PORTA_PIN7_ID_INDEX				(uint8)0x07
#define PORTA_END_INDEX		        		(uint8)0x07
//PORTB
#define PORTB_ONSET_INDEX                               (uint8)0x08   
#define PORTB_PIN0_ID_INDEX				(uint8)0x08
#define PORTB_PIN1_ID_INDEX				(uint8)0x09
#define PORTB_PIN2_ID_INDEX				(uint8)0x0A
#define PORTB_PIN3_ID_INDEX				(uint8)0x0B
#define PORTB_PIN4_ID_INDEX				(uint8)0x0C
#define PORTB_PIN5_ID_INDEX				(uint8)0x0D
#define PORTB_PIN6_ID_INDEX				(uint8)0x0E
#define PORTB_PIN7_ID_INDEX				(uint8)0x0F
#define PORTB_END_INDEX		        		(uint8)0x0F   
//PORTC
#define PORTC_ONSET_INDEX                               (uint8)0x10
#define PORTC_PIN0_ID_INDEX				(uint8)0x10
#define PORTC_PIN1_ID_INDEX				(uint8)0x11
#define PORTC_PIN2_ID_INDEX				(uint8)0x12
#define PORTC_PIN3_ID_INDEX				(uint8)0x13
#define PORTC_PIN4_ID_INDEX				(uint8)0x14
#define PORTC_PIN5_ID_INDEX				(uint8)0x15
#define PORTC_PIN6_ID_INDEX				(uint8)0x16
#define PORTC_PIN7_ID_INDEX				(uint8)0x17
#define PORTC_END_INDEX			        	(uint8)0x17   
//PORTD
#define PORTD_ONSET_INDEX                               (uint8)0x18
#define PORTD_PIN0_ID_INDEX				(uint8)0x18
#define PORTD_PIN1_ID_INDEX				(uint8)0x19
#define PORTD_PIN2_ID_INDEX				(uint8)0x1A
#define PORTD_PIN3_ID_INDEX				(uint8)0x1B
#define PORTD_PIN4_ID_INDEX				(uint8)0x1C
#define PORTD_PIN5_ID_INDEX				(uint8)0x1D
#define PORTD_PIN6_ID_INDEX				(uint8)0x1E
#define PORTD_PIN7_ID_INDEX				(uint8)0x1F
#define PORTD_END_INDEX			        	(uint8)0x1F
//PORTE
#define PORTE_ONSET_INDEX                               (uint8)0x20				                   
#define PORTE_PIN0_ID_INDEX				(uint8)0x20
#define PORTE_PIN1_ID_INDEX				(uint8)0x21
#define PORTE_PIN2_ID_INDEX				(uint8)0x22
#define PORTE_PIN3_ID_INDEX				(uint8)0x23
#define PORTE_PIN4_ID_INDEX				(uint8)0x24
#define PORTE_PIN5_ID_INDEX				(uint8)0x25
#define PORTE_END_INDEX			        	(uint8)0x25   
//PORTF
#define PORTF_ONSET_INDEX                               (uint8)0x26				                   
#define PORTF_PIN0_ID_INDEX				(uint8)0x26
#define PORTF_PIN1_ID_INDEX				(uint8)0x27
#define PORTF_PIN2_ID_INDEX				(uint8)0x28
#define PORTF_PIN3_ID_INDEX				(uint8)0x29
#define PORTF_PIN4_ID_INDEX				(uint8)0x2A
#define PORTF_END_INDEX			        	(uint8)0x2A   


/* PORT Configured PINS ID's */
/*PORTA*/
#define PORTA_PIN0_ID_NUM		(uint8)0x00
#define PORTA_PIN0_MODE			PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN0_DIR			PORT_PIN_DIR_DEFUALT
#define PORTA_PIN0_INITIAL_VALUE	PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN0_INTERNAL_RESESTOR	PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN0_DIR_CHANGABLE	STD_OFF
#define PORTA_PIN0_MODE_CHANGABLE	STD_OFF

#define PORTA_PIN1_ID_NUM		(uint8)0x01
#define PORTA_PIN1_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN1_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTA_PIN1_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN1_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN1_DIR_CHANGABLE        STD_OFF
#define PORTA_PIN1_MODE_CHANGABLE       STD_OFF

#define PORTA_PIN2_ID_NUM		(uint8)0x02
#define PORTA_PIN2_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN2_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTA_PIN2_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN2_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN2_DIR_CHANGABLE        STD_OFF
#define PORTA_PIN2_MODE_CHANGABLE       STD_OFF

#define PORTA_PIN3_ID_NUM		(uint8)0x03
#define PORTA_PIN3_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN3_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTA_PIN3_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN3_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN3_DIR_CHANGABLE        STD_OFF
#define PORTA_PIN3_MODE_CHANGABLE       STD_OFF

#define PORTA_PIN4_ID_NUM		(uint8)0x04
#define PORTA_PIN4_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN4_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTA_PIN4_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN4_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN4_DIR_CHANGABLE        STD_OFF
#define PORTA_PIN4_MODE_CHANGABLE       STD_OFF

#define PORTA_PIN5_ID_NUM		(uint8)0x05
#define PORTA_PIN5_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN5_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTA_PIN5_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN5_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN5_DIR_CHANGABLE        STD_OFF
#define PORTA_PIN5_MODE_CHANGABLE       STD_OFF

#define PORTA_PIN6_ID_NUM		(uint8)0x06
#define PORTA_PIN6_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN6_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTA_PIN6_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN6_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN6_DIR_CHANGABLE        STD_OFF
#define PORTA_PIN6_MODE_CHANGABLE       STD_OFF

#define PORTA_PIN7_ID_NUM		(uint8)0x07
#define PORTA_PIN7_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTA_PIN7_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTA_PIN7_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTA_PIN7_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTA_PIN7_DIR_CHANGABLE        STD_OFF
#define PORTA_PIN7_MODE_CHANGABLE       STD_OFF
//PORTB
#define PORTB_PIN0_ID_NUM		(uint8)0x00			                    
#define PORTB_PIN0_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN0_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN0_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN0_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN0_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN0_MODE_CHANGABLE       STD_OFF
		
#define PORTB_PIN1_ID_NUM		(uint8)0x01
#define PORTB_PIN1_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN1_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN1_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN1_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN1_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN1_MODE_CHANGABLE       STD_OFF
		
#define PORTB_PIN2_ID_NUM		(uint8)0x02
#define PORTB_PIN2_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN2_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN2_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN2_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN2_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN2_MODE_CHANGABLE       STD_OFF
		
#define PORTB_PIN3_ID_NUM		(uint8)0x03
#define PORTB_PIN3_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN3_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN3_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN3_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN3_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN3_MODE_CHANGABLE       STD_OFF
		
#define PORTB_PIN4_ID_NUM		(uint8)0x04
#define PORTB_PIN4_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN4_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN4_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN4_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN4_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN4_MODE_CHANGABLE       STD_OFF
		
#define PORTB_PIN5_ID_NUM		(uint8)0x05		
#define PORTB_PIN5_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN5_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN5_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN5_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN5_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN5_MODE_CHANGABLE       STD_OFF

#define PORTB_PIN6_ID_NUM		(uint8)0x06
#define PORTB_PIN6_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN6_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN6_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN6_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN6_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN6_MODE_CHANGABLE       STD_OFF
		
#define PORTB_PIN7_ID_NUM		(uint8)0x07	
#define PORTB_PIN7_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTB_PIN7_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTB_PIN7_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTB_PIN7_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTB_PIN7_DIR_CHANGABLE        STD_OFF
#define PORTB_PIN7_MODE_CHANGABLE       STD_OFF

//PORTC				                    
#define PORTC_PIN0_ID_NUM		(uint8)0x00			                    
#define PORTC_PIN0_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN0_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN0_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN0_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN0_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN0_MODE_CHANGABLE       STD_OFF
		
#define PORTC_PIN1_ID_NUM		(uint8)0x01
#define PORTC_PIN1_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN1_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN1_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN1_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN1_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN1_MODE_CHANGABLE       STD_OFF
		
#define PORTC_PIN2_ID_NUM		(uint8)0x02
#define PORTC_PIN2_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN2_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN2_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN2_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN2_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN2_MODE_CHANGABLE       STD_OFF
		
#define PORTC_PIN3_ID_NUM		(uint8)0x03
#define PORTC_PIN3_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN3_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN3_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN3_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN3_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN3_MODE_CHANGABLE       STD_OFF
		
#define PORTC_PIN4_ID_NUM		(uint8)0x04
#define PORTC_PIN4_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN4_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN4_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN4_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN4_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN4_MODE_CHANGABLE       STD_OFF
		
#define PORTC_PIN5_ID_NUM		(uint8)0x05		
#define PORTC_PIN5_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN5_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN5_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN5_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN5_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN5_MODE_CHANGABLE       STD_OFF
		
#define PORTC_PIN6_ID_NUM		(uint8)0x06
#define PORTC_PIN6_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN6_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN6_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN6_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN6_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN6_MODE_CHANGABLE       STD_OFF
		
#define PORTC_PIN7_ID_NUM		(uint8)0x07	
#define PORTC_PIN7_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTC_PIN7_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTC_PIN7_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTC_PIN7_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTC_PIN7_DIR_CHANGABLE        STD_OFF
#define PORTC_PIN7_MODE_CHANGABLE       STD_OFF

//PORTD				                    
#define PORTD_PIN0_ID_NUM		(uint8)0x00			                    
#define PORTD_PIN0_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN0_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN0_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN0_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN0_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN0_MODE_CHANGABLE       STD_OFF
		
#define PORTD_PIN1_ID_NUM		(uint8)0x01
#define PORTD_PIN1_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN1_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN1_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN1_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN1_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN1_MODE_CHANGABLE       STD_OFF
		
#define PORTD_PIN2_ID_NUM	        (uint8)0x02
#define PORTD_PIN2_MODE 	        PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN2_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN2_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN2_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN2_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN2_MODE_CHANGABLE       STD_OFF
		
#define PORTD_PIN3_ID_NUM		(uint8)0x03
#define PORTD_PIN3_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN3_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN3_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN3_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN3_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN3_MODE_CHANGABLE       STD_OFF
		
#define PORTD_PIN4_ID_NUM		(uint8)0x04
#define PORTD_PIN4_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN4_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN4_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN4_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN4_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN4_MODE_CHANGABLE       STD_OFF
		
#define PORTD_PIN5_ID_NUM		(uint8)0x05		
#define PORTD_PIN5_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN5_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN5_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN5_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN5_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN5_MODE_CHANGABLE       STD_OFF
		
#define PORTD_PIN6_ID_NUM		(uint8)0x06
#define PORTD_PIN6_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN6_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN6_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN6_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN6_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN6_MODE_CHANGABLE       STD_OFF
		
#define PORTD_PIN7_ID_NUM	        (uint8)0x07	
#define PORTD_PIN7_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTD_PIN7_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTD_PIN7_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTD_PIN7_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTD_PIN7_DIR_CHANGABLE        STD_OFF
#define PORTD_PIN7_MODE_CHANGABLE       STD_OFF
//PORTE				                   
#define PORTE_PIN0_ID_NUM		(uint8)0x00			                    
#define PORTE_PIN0_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTE_PIN0_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTE_PIN0_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTE_PIN0_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTE_PIN0_DIR_CHANGABLE        STD_OFF
#define PORTE_PIN0_MODE_CHANGABLE       STD_OFF
		
#define PORTE_PIN1_ID_NUM		(uint8)0x01
#define PORTE_PIN1_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTE_PIN1_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTE_PIN1_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTE_PIN1_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTE_PIN1_DIR_CHANGABLE        STD_OFF
#define PORTE_PIN1_MODE_CHANGABLE       STD_OFF
		
#define PORTE_PIN2_ID_NUM		(uint8)0x02
#define PORTE_PIN2_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTE_PIN2_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTE_PIN2_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTE_PIN2_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTE_PIN2_DIR_CHANGABLE        STD_OFF
#define PORTE_PIN2_MODE_CHANGABLE       STD_OFF
		
#define PORTE_PIN3_ID_NUM		(uint8)0x03
#define PORTE_PIN3_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTE_PIN3_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTE_PIN3_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTE_PIN3_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTE_PIN3_DIR_CHANGABLE        STD_OFF
#define PORTE_PIN3_MODE_CHANGABLE       STD_OFF
		
#define PORTE_PIN4_ID_NUM		(uint8)0x04
#define PORTE_PIN4_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTE_PIN4_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTE_PIN4_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTE_PIN4_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTE_PIN4_DIR_CHANGABLE        STD_OFF
#define PORTE_PIN4_MODE_CHANGABLE       STD_OFF
		
#define PORTE_PIN5_ID_NUM		(uint8)0x05		
#define PORTE_PIN5_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTE_PIN5_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTE_PIN5_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTE_PIN5_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTE_PIN5_DIR_CHANGABLE        STD_OFF
#define PORTE_PIN5_MODE_CHANGABLE       STD_OFF
//PORTF				                   
#define PORTF_PIN0_ID_NUM		(uint8)0x00			                    
#define PORTF_PIN0_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTF_PIN0_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTF_PIN0_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTF_PIN0_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTF_PIN0_DIR_CHANGABLE        STD_OFF
#define PORTF_PIN0_MODE_CHANGABLE       STD_OFF
/*Configured LED*/		
#define PORTF_LED1_ID_NUM		(uint8)0x01
#define PORTF_LED1_MODE 		PORT_PIN_DIO_MODE 
#define PORTF_LED1_DIR                  PORT_PIN_OUT
#define PORTF_LED1_INITIAL_VALUE        PORT_PIN_LOW 
#define PORTF_LED1_INTERNAL_RESESTOR    OFF
#define PORTF_LED1_DIR_CHANGABLE        STD_OFF
#define PORTF_LED1_MODE_CHANGABLE       STD_OFF

#define PORTF_PIN2_ID_NUM		(uint8)0x02
#define PORTF_PIN2_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTF_PIN2_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTF_PIN2_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTF_PIN2_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTF_PIN2_DIR_CHANGABLE        STD_OFF
#define PORTF_PIN2_MODE_CHANGABLE       STD_OFF
		
#define PORTF_PIN3_ID_NUM		(uint8)0x03
#define PORTF_PIN3_MODE 		PORT_PIN_MODE_DEFUALT 
#define PORTF_PIN3_DIR                  PORT_PIN_DIR_DEFUALT
#define PORTF_PIN3_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTF_PIN3_INTERNAL_RESESTOR    PORT_PIN_Internal_Resistor_DEFUALT
#define PORTF_PIN3_DIR_CHANGABLE        STD_OFF
#define PORTF_PIN3_MODE_CHANGABLE       STD_OFF
/*Configured Switch*/		
#define PORTF_SW1_ID_NUM	       (uint8)0x04
#define PORTF_SW1_MODE 		       PORT_PIN_DIO_MODE
#define PORTF_SW1_DIR                  PORT_PIN_IN
#define PORTF_SW1_INITIAL_VALUE        PORT_PIN_Initial_value_DEFUALT 
#define PORTF_SW1_INTERNAL_RESESTOR    PULL_UP
#define PORTF_SW1_DIR_CHANGABLE        STD_OFF
#define PORTF_SW1_MODE_CHANGABLE       STD_OFF


#endif /* DIO_CFG_H */