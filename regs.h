#ifndef _regs_h_
#define _regs_h_

// GPIO/RCC stuff

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#define GPIOD 3
#define GPIOE 4
#define GPIOF 5
#define RCC_USB   23

#define gpio_set_output(a,b)    gpio_set_mode(a,b,0x2)  // output push-pull
#define gpio_set_input(a,b)     gpio_set_mode(a,b,0x4)  // input floating
#define gpio_set_af(a,b)        gpio_set_mode(a,b,0xA)  // Alternative mode pp
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

/* Fields */
#define RCC_CR_HSION        (1 << 0)
#define RCC_CR_HSIRDY       (1 << 1)
#define RCC_CR_HSEON        (1 << 16)
#define RCC_CR_HSERDY       (1 << 17)
#define RCC_CR_HSEBYP       (1 << 18)
#define RCC_CR_CSSON        (1 << 19)
#define RCC_CR_PLLON        (1 << 24)
#define RCC_CR_PLLRDY       (1 << 25)

#define RCC_CFG_MCO         (15 << 24)
#define RCC_CFG_OTGFSPRE    (1 << 22)
#define RCC_CFG_PLLMUL      (15 << 18)
#define RCC_CFGR_PLLXTPRE   (1 << 17)
#define RCC_CFGR_PLLSRC     (1 << 16)
#define RCC_CFGR_SW         (3 << 0)
#define RCC_CFGR_SWS        (3 << 2)

/* Configuration bits */
#define RCC_CFG_MCO_PLL_2   (7 << 24)
#define RCC_CFG_MCO_HSE     (6 << 24)
#define RCC_CFG_MCO_HSI     (5 << 24)
#define RCC_CFG_MCO_SYSCLK  (4 << 24)
#define RCC_CFGR_PLLMUL65   (13 << 18)
#define RCC_CFGR_PLLMUL9    (7 << 18)
#define RCC_CFGR_PLLMUL8    (6 << 18)
#define RCC_CFGR_PLLMUL7    (5 << 18)
#define RCC_CFGR_PLLMUL6    (4 << 18)
#define RCC_CFGR_PLLMUL5    (3 << 18)
#define RCC_CFGR_PLLMUL4    (2 << 18)
#define RCC_CFGR_PLLSRC_HSE (1 << 16)
#define RCC_CFGR_ADCPRE_8   (3 << 14)
#define RCC_CFGR_ADCPRE_6   (2 << 14)
#define RCC_CFGR_ADCPRE_4   (1 << 14)
#define RCC_CFGR_ADCPRE_2   (0 << 14)
#define RCC_CFGR_PPRE2_16   (7 << 11)
#define RCC_CFGR_PPRE2_8    (6 << 11)
#define RCC_CFGR_PPRE2_4    (5 << 11)
#define RCC_CFGR_PPRE2_2    (4 << 11)
#define RCC_CFGR_PPRE2_1    (0 << 11)
#define RCC_CFGR_PPRE1_16   (7 << 8)
#define RCC_CFGR_PPRE1_8    (6 << 8)
#define RCC_CFGR_PPRE1_4    (5 << 8)
#define RCC_CFGR_PPRE1_2    (4 << 8)
#define RCC_CFGR_PPRE1_1    (0 << 8)
#define RCC_CFGR_HPRE_512   (15 << 4)
#define RCC_CFGR_HPRE_256   (14 << 4)
#define RCC_CFGR_HPRE_128   (13 << 4)
#define RCC_CFGR_HPRE_64    (12 << 4)
#define RCC_CFGR_HPRE_16    (11 << 4)
#define RCC_CFGR_HPRE_8     (10 << 4)
#define RCC_CFGR_HPRE_4     (9 << 4)
#define RCC_CFGR_HPRE_2     (8 << 4)
#define RCC_CFGR_HPRE_1     (0 << 4)
#define RCC_CFGR_SW_PLL     (2 << 0)
#define RCC_CFGR_SW_HSE     (1 << 0)
#define RCC_CFGR_SW_HSI     (0 << 0)



#define gpio_periph_enable(gpion)   RCC_APB2ENR |= (1 << (gpion + 2));
#define usb_periph_enable(pn)       RCC_APB1ENR |= (1 << (pn));
#define gpio_clear(gpiodev, gpion)  GPIO_BSRR(gpiodev) = (1 << (16 + gpion))
#define gpio_set(gpiodev, gpion)    GPIO_BSRR(gpiodev) = (1 << (gpion))
#define gpio_read(gpiodev, gpion)   (GPIO_IDR(gpiodev) & (1 << (gpion)))


#define GPIO_SIZE           0x100
#define GPIO_BASE           (volatile uint32_t*)0x40010800UL

#define GPIO_CRL(x)         (*(volatile uint32_t*)(GPIO_BASE + (x * GPIO_SIZE) + 0))
#define GPIO_CRH(x)         (*(volatile uint32_t*)(GPIO_BASE + (x * GPIO_SIZE) + 1))
#define GPIO_IDR(x)         (*(volatile uint32_t*)(GPIO_BASE + (x * GPIO_SIZE) + 2))
#define GPIO_ODR(x)         (*(volatile uint32_t*)(GPIO_BASE + (x * GPIO_SIZE) + 3))
#define GPIO_BSRR(x)        (*(volatile uint32_t*)(GPIO_BASE + (x * GPIO_SIZE) + 4))
#define RCC_APB2ENR         (*(volatile uint32_t*)0x40021018UL)
#define RCC_CR              (*(volatile uint32_t*)0x40021000UL)
#define RCC_CFGR            (*(volatile uint32_t*)0x40021004UL)
#define RCC_APB1ENR         (*(volatile uint32_t*)0x4002101CUL)

#endif //_regs_h_
