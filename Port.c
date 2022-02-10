 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Malek Mahmoud Mohammed
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (Port_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != Port_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != Port_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != Port_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif
/*******************************************************************************
 *                       Static Variables                                    *
 *******************************************************************************/
STATIC const Port_PinConfigType * Port_PinSetting = NULL_PTR;
STATIC uint8 Port_Status = Port_NOT_INITIALIZED;

/*******************************************************************************
 *                      Function Definitions                                   *
 *******************************************************************************/
void Port_Init(
               const Port_ConfigType* ConfigPtr
               )
{
  
  Port_PinSetting =  ConfigPtr->Pins;
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  volatile uint32 delay = 0;
  volatile uint8 count;
  
#if (DIO_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
		     PORT_E_PARAM_CONFIG);
	}
	else
#endif
	{
          for(count =0;count < PORT_numOfPins; count++)
          {
            /*setting the base address*/
            if ((count >= PORTA_ONSET_INDEX) && (count <= PORTA_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */
             /* Enable clock for PORTA and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTA_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTB_ONSET_INDEX) && (count <= PORTB_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
             /* Enable clock for PORTB and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTB_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTC_ONSET_INDEX) && (count <= PORTC_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
             /*Protection for JTAG pins*/
             switch(count)
             {
             case PORTC_PIN0_ID_INDEX: continue;
                                       break;
             case PORTC_PIN1_ID_INDEX: continue;
                                       break;
             case PORTC_PIN2_ID_INDEX: continue;
                                       break;
             case PORTC_PIN3_ID_INDEX: continue;
                                       break;
             }
             /* Enable clock for PORTC and allow time for clock to start*/
             SYSCTL_REGCGC2_REG |= (1<<PORTC_ID);
             delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTD_ONSET_INDEX) && (count <= PORTD_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
             /* Enable clock for PORTD and allow time for clock to start*/
             SYSCTL_REGCGC2_REG |= (1<<PORTD_ID);
             delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTE_ONSET_INDEX) && (count <= PORTE_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
             /* Enable clock for PORTE and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTE_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else if ((count >= PORTF_ONSET_INDEX) && (count <= PORTG_END_INDEX))
           {
             PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTF Base Address */
             /* Enable clock for PORTF and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1<<PORTF_ID);
            delay = SYSCTL_REGCGC2_REG;
           }
           else
           {
             /* Do Nothing*/
           }
           
           if((count == PORTD_PIN7_ID_INDEX)||(count == PORTF_PIN0_ID_INDEX))
           {
             /* Unlock the GPIOCR register */ 
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = PORT_UNLOCK_PASS;
            /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PinSetting[count].Pin_Num);  
           }
           else
           {
             /* Do Nothing*/
           }
           
           /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
           /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
           
           switch (Port_PinSetting[count].Pin_Mode)
           {
           case PORT_PIN_DIO_MODE:
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ZERO_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_ADC_MODE:
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                  Set_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ZERO_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_UART_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if((count == PORTC_PIN4_ID_INDEX) || ((count == PORTC_PIN5_ID_INDEX)))
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_TWO_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ONE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  break;
           case PORT_PIN_SSI_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if((count >= PORTD_PIN0_ID_INDEX) && ((count <= PORTD_PIN3_ID_INDEX)))
                  {
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ONE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_TWO_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  break;
           case PORT_PIN_I2C_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_THREE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_PWM0_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FOUR_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_FAULT_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if(count == PORTF_PIN4_ID_INDEX4)
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FIVE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FOUR_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  break;
           case PORT_PIN_PWM1_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FIVE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_QEI_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_SIX_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_CPP_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_SEVEN_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_CAN_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if((count == PORTF_PIN0_ID_INDEX) || (count == PORTF_PIN3_ID_INDEX))
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_THREE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  break;
           case PORT_PIN_NMI_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_UART_CONTROL_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_USB_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  break;
           case PORT_PIN_ANALOG_COMP_MODE:
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[count].Pin_Num);
                  if((count == PORTF_PIN0_ID_INDEX) || ((count == PORTF_PIN1_ID_INDEX)))
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ONE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  else
                  {
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_NINE_VALUE << (Port_PinSetting[count].Pin_Num * 4));
                  }
                  break;
            default:
                  /*Do Nothing*/
                  break;
           }
           if(Port_PinSetting[count].Pin_Dir == PORT_PIN_OUT)
           {
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
             if(Port_PinSetting[count].Pin_initalValue == PORT_PIN_HIGH)
             {
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinSetting[count].Pin_Num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
             }
             else
             {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PinSetting[count].Pin_Num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
             }
           }
           else if (Port_PinSetting[count].Pin_Dir == PORT_PIN_IN)
           {
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
             
             if(Port_PinSetting[count].Pin_internalResistor == PULL_UP)
             {
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinSetting[count].Pin_Num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
             }
             else if(Port_PinSetting[count].Pin_internalResistor == PULL_DOWN)
             {
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinSetting[count].Pin_Num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
             }
             else
             {
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PinSetting[count].Pin_Num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PinSetting[count].Pin_Num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
             }
           }
           else
           {
             /*Do Nothing*/
           }
          }       
          Port_Status = PORT_INITIALIZED;
	}
}
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(
                          Port_PinType Pin, 
                          Port_PinDirectionType Direction
                          )
{
  boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
  {
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == Port_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinDirection_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Pin >= PORT_numOfPins)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinDirection_SID,PORT_E_PARAM_PIN);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Port_PinSetting[Pin].Pin_dirChangeable == STD_OFF)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinDirection_SID,PORT_E_DIRECTION_UNCHANGEABLE);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
  }
#endif
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  if(error == FALSE)
  {
      /*setting the base address*/
      if ((Pin >= PORTA_ONSET_INDEX) && (Pin <= PORTA_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */
     }
     else if ((Pin >= PORTB_ONSET_INDEX) && (Pin <= PORTB_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
     }
     else if ((Pin >= PORTC_ONSET_INDEX) && (Pin <= PORTC_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
     }
     else if ((Pin >= PORTD_ONSET_INDEX) && (Pin <= PORTD_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
     }
     else if ((Pin >= PORTE_ONSET_INDEX) && (Pin <= PORTE_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
     }
     else if ((Pin >= PORTF_ONSET_INDEX) && (Pin <= PORTG_END_INDEX))
     {
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTF Base Address */
     }
     else
     {
       /* Do Nothing*/
     }
     if(Direction == PORT_PIN_OUT)
     {
       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);     /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
     }
     else
     {
       CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num);   /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
     }
  }
  else
  {
    /* Do Nothing */
  }
}
#endif
void Port_RefreshPortDirection(
                               void
                               )
{
  boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == Port_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_RefreshPortDirection_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
#endif
    
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  volatile uint8 count;
  if(error == false)
  {
    for(count = 0; count <= PORT_numOfPins; count++)
    {
      if (Port_PinSetting[count].Pin_dirChangeable == STD_ON)
      {
        continue;
      }
      else
      {
        /*setting the base address*/
        if ((count >= PORTA_ONSET_INDEX) && (count <= PORTA_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */
        }
        else if ((count >= PORTB_ONSET_INDEX) && (count <= PORTB_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        }
        else if ((count >= PORTC_ONSET_INDEX) && (count <= PORTC_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        }
        else if ((count >= PORTD_ONSET_INDEX) && (count <= PORTD_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        }
        else if ((count >= PORTE_ONSET_INDEX) && (count <= PORTE_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        }
        else if ((count >= PORTF_ONSET_INDEX) && (count <= PORTG_END_INDEX))
        {
          PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTF Base Address */
        }
        else
        {
          /* Do Nothing*/
        }
        /* Refresh Pins Direction */
        if(Port_PinSetting[count].Pin_Dir == PORT_PIN_OUT)
        {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        }
        else
        {
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PinSetting[count].Pin_Num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        }
      }
    }
  }
  else
  {
    /* Do Nothing */
  }
}
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(
                         Std_VersionInfoType* versioninfo
                         )
{
    boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == Port_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_GetVersionInfo_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    /* Check if the parameter versioninfo value is a NULL pointer */
    if(versioninfo == NULL_PTR)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_GetVersionInfo_SID,PORT_E_PARAM_POINTER);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
#endif
    if(error == FALSE)
    {
      /* Copy the vendor Id */
      versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
      /* Copy the module Id */
      versioninfo->moduleID = (uint16)PORT_MODULE_ID;
      /* Copy Software Major Version */
      versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
      /* Copy Software Minor Version */
      versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
      /* Copy Software Patch Version */
      versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
    else
    {
      /*Do Nothing*/
    }
}
#endif
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(
                     Port_PinType Pin, 
                     Port_PinModeType Mode
                     )
{
  boolean error = FALSE;
#if(PORT_DEV_ERROR_DETECT == STD_ON)
  {
    /* Check if the Driver is initialized before using this function */
    if(Port_Status == Port_NOT_INITIALIZED)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_UNINIT);
    error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Pin >= PORT_numOfPins)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_PARAM_PIN);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Port_PinSetting[Pin].Pin_modeChangeable == STD_OFF)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_DIRECTION_UNCHANGEABLE);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
    if(Mode >= PORT_numOfConfiguriedMode)
    {
     Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                     Port_SetPinMode_SID,PORT_E_PARAM_INVALID_MODE);
     error = TRUE;
    }
    else
    {
      /*Do Nothing*/
    }
  }
#endif
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  if(error == FALSE)
  {
      /*setting the base address*/
      if ((Pin >= PORTA_ONSET_INDEX) && (Pin <= PORTA_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;  /* PORTA Base Address */ 
     } 
     else if ((Pin >= PORTB_ONSET_INDEX) && (Pin <= PORTB_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */ 
     } 
     else if ((Pin >= PORTC_ONSET_INDEX) && (Pin <= PORTC_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */ 
     } 
     else if ((Pin >= PORTD_ONSET_INDEX) && (Pin <= PORTD_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */ 
     } 
     else if ((Pin >= PORTE_ONSET_INDEX) && (Pin <= PORTE_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */ 
     } 
     else if ((Pin >= PORTF_ONSET_INDEX) && (Pin <= PORTG_END_INDEX)) 
     { 
       PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTF Base Address */ 
     } 
     else 
     { 
       /* Do Nothing*/ 
     }
     switch (Mode)
     {
       case PORT_PIN_DIO_MODE: 
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ZERO_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_ADC_MODE: 
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */ 
              Set_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */ 
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ZERO_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_UART_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              if((Pin == PORTC_PIN4_ID_INDEX) || ((Pin == PORTC_PIN5_ID_INDEX))) 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_TWO_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              else 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ONE_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              break; 
       case PORT_PIN_SSI_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              if((Pin >= PORTD_PIN0_ID_INDEX) && ((Pin <= PORTD_PIN3_ID_INDEX))) 
              { 
                 *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ONE_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              else 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_TWO_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              break; 
       case PORT_PIN_I2C_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_THREE_VALUE<< (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_PWM0_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FOUR_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_FAULT_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              if(Pin == PORTF_PIN4_ID_INDEX4) 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FIVE_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              else 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FOUR_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              break; 
       case PORT_PIN_PWM1_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_FIVE_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_QEI_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_SIX_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_CPP_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_SEVEN_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_CAN_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              if((Pin == PORTF_PIN0_ID_INDEX) || (Pin == PORTF_PIN3_ID_INDEX)) 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_THREE_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              else 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              break; 
       case PORT_PIN_NMI_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_UART_CONTROL_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_USB_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_EGIHT_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              break; 
       case PORT_PIN_ANALOG_COMP_MODE: 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PinSetting[Pin].Pin_Num); 
              if((Pin == PORTF_PIN0_ID_INDEX) || ((Pin == PORTF_PIN1_ID_INDEX))) 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_ONE_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              else 
              { 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_CTRL_REG_SHIFT_NINE_VALUE << (Port_PinSetting[Pin].Pin_Num * 4)); 
              } 
              break; 
        default: 
              /*Do Nothing*/ 
              break; 
     }
  }
  else
  {
    /*Do Nothing*/
  }
}
#endif
