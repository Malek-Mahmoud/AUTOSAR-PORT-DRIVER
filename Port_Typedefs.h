 /******************************************************************************
 *
 * Module: PORT
 *
 * File Name: Port_Typedefs.h
 *
 * Description: Typedefs Header file for TM4C123GH6PM Microcontroller - PORT Driver
 *
 * Author: Malek Mahmoud Mohammed
 ******************************************************************************/
#ifndef PORT_TYPEDEFS_H
#define PORT_TYPEDEFS_H
/* Include STD_TYPES.h*/
#include "STD_TYPES.h"
/* Include PORT_Cfg.h*/
#include "PORT_Cfg.h"
/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
typedef uint8 Port_PinType;

typedef boolean Pin_dirChangeableType;

typedef boolean Pin_modeChangeableType;
/* Description: Enum to hold PIN direction */
typedef enum
{
  PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;
/* Description: Enum to hold initial value for PIN */
typedef enum
{
  PORT_PIN_LOW,PORT_PIN_HIGH
}Port_PinInitialValueType;
/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistorType;

typedef uint8 Port_PinModeType
/* Description: Structure to configure each individual PIN:
 *	1. the number of the pin in the PORT.
 *      2. the mode of the pin.
 *      3. the direction of pin --> INPUT or OUTPUT.
 *      4. the initial value of the output direction pin.
 *      5. the internal resistor --> Disable, Pull up or Pull down
 *      6. the changablitiy of the direction in the run time.
 *      7. the changablitiy of the mode in the run time.
 */
typedef struct
{
  Port_PinType Pin_Num;
  Port_PinModeType Pin_Mode;
  Port_PinDirectionType Pin_Dir;
  Port_PinInitialValueType Pin_initalValue;
  Port_InternalResistorType Pin_internalResistor;
  Pin_dirChangeableType Pin_dirChangeable;
  Pin_modeChangeableType Pin_modeChangeable;
}Port_PinConfigType;

typedef struct
{
  Port_PinConfigType Pins[PORT_numOfPins];
}Port_ConfigType;

#endif /* PORT_TYPEDEFS_H */