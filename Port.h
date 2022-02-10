 /******************************************************************************
 *
 * Module: PORT
 *
 * File Name: PORT.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - PORT Driver
 *
 * Author: Malek Mahmoud Mohammed
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 */
#define PORT_VENDOR_ID    (1000U)

/* PORT Module Id */
#define PORT_MODULE_ID    (120U)

/* Dio Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Dio Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Dio Pre-Compile Configuration Header file */
#include " PORT_Cfg.h"

/* AUTOSAR Version checking between PORT_Cfg.h and Dio.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PORT_Cfg.h does not match the expected version"
#endif

/* Software Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PORT_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"
#include "Port_Typedefs.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
#define Port_Init_SID                       (uint8)0x00
#define Port_SetPinDirection_SID            (uint8)0x01
#define Port_RefreshPortDirection_SID       (uint8)0x02           
#define Port_GetVersionInfo_SID             (uint8)0x03
#define Port_SetPinMode_SID                 (uint8)0x04
/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
#define PORT_E_PARAM_PIN                        (uint8)0x0A
#define PORT_E_DIRECTION_UNCHANGEABLE           (uint8)0x0B
#define PORT_E_PARAM_CONFIG                     (uint8)0x0C
#define PORT_E_PARAM_INVALID_MODE               (uint8)0x0D
#define PORT_E_MODE_UNCHANGEABLE                (uint8)0x0E
#define PORT_E_UNINIT                           (uint8)0x0F
#define PORT_E_PARAM_POINTER                    (uint8)0x10
/*******************************************************************************
 *                     Preprocessor Definations                                *
 *******************************************************************************/
#define PORTA_ID                               (uint8)0x00
#define PORTB_ID                               (uint8)0x01
#define PORTC_ID                               (uint8)0x02
#define PORTD_ID                               (uint8)0x03
#define PORTE_ID                               (uint8)0x04
#define PORTF_ID                               (uint8)0x05   
 
#define PORT_UNLOCK_PASS                       (uint32)0x4C4F434B
   
#define PORT_CTRL_REG_SHIFT_ZERO_VALUE          (uint32)0x0000000F
#define PORT_CTRL_REG_SHIFT_ONE_VALUE           (uint32)0x0000000E 
#define PORT_CTRL_REG_SHIFT_TWO_VALUE           (uint32)0x0000000D 
#define PORT_CTRL_REG_SHIFT_THREE_VALUE         (uint32)0x0000000C
#define PORT_CTRL_REG_SHIFT_FOUR_VALUE          (uint32)0x0000000B 
#define PORT_CTRL_REG_SHIFT_FIVE_VALUE          (uint32)0x0000000A 
#define PORT_CTRL_REG_SHIFT_SIX_VALUE           (uint32)0x00000009 
#define PORT_CTRL_REG_SHIFT_SEVEN_VALUE         (uint32)0x00000008
#define PORT_CTRL_REG_SHIFT_EGIHT_VALUE         (uint32)0x00000007
#define PORT_CTRL_REG_SHIFT_NINE_VALUE          (uint32)0x00000006    
/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
void Port_Init(
               const Port_ConfigType* ConfigPtr
               );      
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(
                          Port_PinType Pin, 
                          Port_PinDirectionType Direction
                          );
#endif
void Port_RefreshPortDirection(
                               void
                               );  
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(
                         Std_VersionInfoType* versioninfo
                         );
#endif
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(
                     Port_PinType Pin, 
                     Port_PinModeType Mode
                     );
#endif
/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_configration;

#endif /* PORT_H */
