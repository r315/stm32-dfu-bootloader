#ifndef _regs_h_
#define _regs_h_

// GPIO/RCC stuff

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#define GPIOD 3
#define GPIOE 4
#define GPIOF 5

#define GPIO_CRL(x)  *((volatile uint32_t*)(x*0x400 +  0 + 0x40010800U))
#define GPIO_CRH(x)  *((volatile uint32_t*)(x*0x400 +  4 + 0x40010800U))
#define GPIO_IDR(x)  *((volatile uint32_t*)(x*0x400 +  8 + 0x40010800U))
#define GPIO_ODR(x)  *((volatile uint32_t*)(x*0x400 + 12 + 0x40010800U))
#define GPIO_BSRR(x) *((volatile uint32_t*)(x*0x400 + 16 + 0x40010800U))

#define RCC_APB2ENR  (*(volatile uint32_t*)0x40021018U)

#define gpio_set_output(a,b)    gpio_set_mode(a,b,0x2)  // output push-pull
#define gpio_set_input(a,b)     gpio_set_mode(a,b,0x4)  // input floating
#define gpio_set_input_pp(a,b)  gpio_set_mode(a,b,0x8)  // input with pull-up/down, define PU/PD on PxODR
#define gpio_set_en_pu(a,b)     GPIO_ODR(a) |= (1<<b)   // enable pull-up

#define FLASH_ACR_LATENCY         7
#define FLASH_ACR_LATENCY_2WS  0x02
#define FLASH_ACR (*(volatile uint32_t*)0x40022000U)

#define RCC_CFGR_HPRE_SYSCLK_NODIV      0x0
#define RCC_CFGR_PPRE1_HCLK_DIV2        0x4
#define RCC_CFGR_PPRE2_HCLK_NODIV       0x0
#define RCC_CFGR_ADCPRE_PCLK2_DIV8      0x3
#define RCC_CFGR_PLLMUL_PLL_CLK_MUL6    0x4
#define RCC_CFGR_PLLMUL_PLL_CLK_MUL9    0x7
#define RCC_CFGR_PLLSRC_HSE_CLK         0x1
#define RCC_CFGR_PLLXTPRE_HSE_CLK       0x0
#define RCC_CFGR_SW_SYSCLKSEL_PLLCLK    0x2
#define RCC_CFGR_SW_SHIFT                 0
#define RCC_CFGR_SW (3 << RCC_CFGR_SW_SHIFT)

#define RCC_CR_HSEON    (1 << 16)
#define RCC_CR_HSERDY   (1 << 17)
#define RCC_CR_PLLON    (1 << 24)
#define RCC_CR_PLLRDY   (1 << 25)
#define RCC_CR       (*(volatile uint32_t*)0x40021000U)
#define RCC_CFGR     (*(volatile uint32_t*)0x40021004U)


#endif //_regs_h_
