create Newfolder in Core/TinyUSB and move all src files in TinyUSB-Master to Core/TinyUSB
add the files on the examples of TinyUSB
    exmples/device/hid-composite
    copy the bsp folder to Core/Inc/bsp
    
    add tusb_config.h to Core/Inc and add the configuration for the STM32F4
    //--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUSB_MCU OPT_MCU_STM32F4
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

important :  Core\Src\stm32f4xx_it.c 
add #include "tusb.h" 