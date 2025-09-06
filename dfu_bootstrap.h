#ifndef _dfu_bootstrap_h_
#define _dfu_bootstrap_h_

#include <stdint.h>
#include "regs.h"

#ifdef ENABLE_CUSTOM_DFU_BOOT
#define HW_SW_AUX1_PIN          14
#define HW_SW_AUX2_PIN          15
#define HW_SW_AUX3_PIN          4
#define HW_SW_AUX1_PORT         GPIOC
#define HW_SW_AUX2_PORT         GPIOC
#define HW_SW_AUX3_PORT         GPIOB

static int force_dfu_gpio(){
unsigned int val, i;
    /* Enable port */
    gpio_periph_enable(HW_SW_AUX1_PORT);
    gpio_periph_enable(HW_SW_AUX3_PORT);

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
#endif /* ENABLE_CUSTOM_DFU_BOOT */

#endif /* _dfu_bootstrap_h_ */
