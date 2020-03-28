#ifndef _dfu_boot_h_
#define _dfu_boot_h_


#ifdef ENABLE_CUSTOM_DFU_BOOT
#include "regs.h"
#define HW_SW_AUX1_PIN          14
#define HW_SW_AUX2_PIN          15
#define HW_SW_AUX3_PIN          4
#define HW_SW_AUX1_PORT         GPIOC
#define HW_SW_AUX2_PORT         GPIOC
#define HW_SW_AUX3_PORT         GPIOB

static int force_dfu_gpio(){
unsigned int val, i;
    /* Enable port */
    rcc_gpio_enable(HW_SW_AUX1_PORT);
    rcc_gpio_enable(HW_SW_AUX3_PORT);

    /*Set pin as input */
	gpio_set_input_pp(HW_SW_AUX1_PORT, HW_SW_AUX1_PIN);
    gpio_set_input_pp(HW_SW_AUX3_PORT, HW_SW_AUX3_PIN);
    gpio_set_en_pu(HW_SW_AUX1_PORT, HW_SW_AUX1_PIN);
    gpio_set_en_pu(HW_SW_AUX3_PORT, HW_SW_AUX3_PIN);

	for (i = 0; i < 512; i++)
		__asm__("nop");

	val  = gpio_read(HW_SW_AUX1_PORT, HW_SW_AUX1_PIN);
    val |= gpio_read(HW_SW_AUX3_PORT, HW_SW_AUX3_PIN);	
	return val == 0;
}

#else

#include <stdint.h>
//dfu-util -a 0 -s 0x08001000 -D ../build/psu_v3.bin -R

#define DFU_MAGIC *(((uint64_t*)&_estack) - 1)

extern uint32_t _estack;

static void DFU_Enable(){
    DFU_MAGIC = 0xDEADBEEFCC00FFEEULL;    
}
#endif /* ENABLE_CUSTOM_DFU_BOOT */
#endif /* _dfu_boot_h_ */
