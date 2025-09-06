/*
 * Copyright (C) 2018 David Guillen Fandos <david@davidgf.net>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.     See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "usb.h"
#include "flash_config.h"
#include "flash.h"
#include "watchdog.h"
#include "regs.h"
#include "dfu_boot.h"

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR    0x21
#define CMD_ERASE    0x41
// Payload/app comes inmediately after Bootloader
#define APP_ADDRESS (FLASH_BASE_ADDR + (FLASH_BOOTLDR_SIZE_KB) * 1024)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

static struct block_info{
    uint8_t buf[DFU_TRANSFER_SIZE];
    uint16_t len;
    uint32_t addr;
    uint16_t blocknum;
}prog;

// USB control data buffer
uint8_t usbd_control_buffer[DFU_TRANSFER_SIZE];

// DFU state
static enum dfu_state usbdfu_state;
// Char set for unique id
static const char hcharset[16] = "0123456789abcdef";
// Serial number to expose via USB
static char serial_no[25];
// Descriptor strings
const char * const _usb_strings[5] = {
    "davidgf.net (libopencm3 based)", // iManufacturer
    "DFU bootloader [" VERSION "]", // iProduct
    serial_no, // iSerialNumber
    // Interface desc string
    /* This string is used by ST Microelectronics' DfuSe utility. */
    /* Change check_do_erase() accordingly */
    "@Internal Flash /0x08000000/"
      STR(FLASH_BOOTLDR_SIZE_KB) "*001Ka,"
      STR(FLASH_BOOTLDR_PAYLOAD_SIZE_KB) "*001Kg",
    // Config desc string
    "Bootloader config: "
    #ifdef ENABLE_WATCHDOG
    "WtDg[" STR(ENABLE_WATCHDOG) "s] "
    #endif
    #ifdef ENABLE_SAFEWRITE
    "SafeWr "
    #endif
    #ifdef ENABLE_PROTECTIONS
    "RDO/DBG "
    #endif
    #ifdef ENABLE_CHECKSUM
    "FW-CRC "
    #endif
};

static void gpio_set_mode(uint32_t gpio, uint16_t pin, uint32_t mode) {
    uint8_t shift = (pin & 7) << 2; // shift = pin % 8 * 4
    uint32_t mask = (15 << shift);
    uint32_t reg_value;
    volatile uint32_t* reg_base = GPIO_BASE + (gpio * GPIO_SIZE);

    if (pin > 7){
        reg_base += 1;
    }

    reg_value = *reg_base & ~mask;
    *reg_base = reg_value | (mode << shift);
}

// Clears reboot information so we reboot in "normal" mode
static void reboot_clear_flags(void) {
	*(volatile uint64_t*)&_magic = 0;
}

// Returns whether we were rebooted into DFU mode
static int reboot_check_dfu(void) {
	return (*(volatile uint64_t*)&_magic == *(volatile uint64_t*)&DFU_MAGIC_STR);
}

static void _full_system_reset() {
    // Reset and wait for it!
    volatile uint32_t *_scb_aircr = (uint32_t*)0xE000ED0CU;
    *_scb_aircr = 0x05FA0000 | 0x4;
    while(1);
    __builtin_unreachable();
}

static void get_dev_unique_id(char *s) {
    volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
    /* Fetch serial number from chip's unique ID */
    for (int i = 0; i < 24; i += 2) {
        s[i]   = hcharset[(*unique_id >> 4) & 0xF];
        s[i+1] = hcharset[*unique_id++ & 0xF];
    }
}

static void usbdfu_unplug(void){
    /* Disable USB peripheral as it overrides GPIO settings */
    *USB_CNTR_REG = USB_CNTR_PWDN;

    /*
     * Vile hack to reenumerate, physically _drag_ d+ low.
     * (need at least 2.5us to trigger usb disconnect)
     */
    gpio_set_output(GPIOA, 12);
    gpio_clear(GPIOA, 12);
    for (unsigned int i = 0; i < 100000; i++)
        __asm__("nop");

    gpio_set_input(GPIOA, 11);
    gpio_set_input(GPIOA, 12);
}

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout) {
    switch (usbdfu_state) {
    case STATE_DFU_DNLOAD_SYNC:
        usbdfu_state = STATE_DFU_DNBUSY;
        *bwPollTimeout = 100;
        return DFU_STATUS_OK;
    case STATE_DFU_MANIFEST_SYNC:
        // Device will reset when read is complete.
        usbdfu_state = STATE_DFU_MANIFEST;
        return DFU_STATUS_OK;
    case STATE_DFU_ERROR:
        return STATE_DFU_ERROR;
    default:
        return DFU_STATUS_OK;
    }
}

static void usbdfu_getstatus_complete(struct usb_setup_data *req) {
    (void)req;

    // Protect the flash by only writing to the valid flash area
    const uint32_t start_addr = 0x08000000 + (FLASH_BOOTLDR_SIZE_KB*1024);
    const uint32_t end_addr   = 0x08000000 + (        FLASH_SIZE_KB*1024);

    switch (usbdfu_state) {
    case STATE_DFU_DNBUSY:
        _flash_unlock(0);
        if (prog.blocknum == 0) {
            switch (prog.buf[0]) {
            case CMD_ERASE: {
                #ifdef ENABLE_SAFEWRITE
                check_do_erase();
                #endif

                // Clear this page here.
                uint32_t baseaddr = *(uint32_t *)(prog.buf + 1);
                if (baseaddr >= start_addr && baseaddr + DFU_TRANSFER_SIZE <= end_addr) {
                    if (!_flash_page_is_erased(baseaddr))
                        _flash_erase_page(baseaddr);
                }
                } break;
            case CMD_SETADDR:
                // Assuming little endian here.
                prog.addr = *(uint32_t *)(prog.buf + 1);
                break;
            }
        } else {
            #ifdef ENABLE_SAFEWRITE
            check_do_erase();
            #endif

            // From formula Address_Pointer + ((wBlockNum - 2)*wTransferSize)
            uint32_t baseaddr = prog.addr + ((prog.blocknum - 2) * DFU_TRANSFER_SIZE);

            if (baseaddr >= start_addr && baseaddr + prog.len <= end_addr) {
                // Program buffer in one go after erasing.
                if (!_flash_page_is_erased(baseaddr))
                    _flash_erase_page(baseaddr);
                _flash_program_buffer(baseaddr, (uint16_t*)prog.buf, prog.len);
            }
        }
        _flash_lock();

        /* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
        usbdfu_state = STATE_DFU_DNLOAD_IDLE;
        return;
    case STATE_DFU_MANIFEST:
        return;  // Reset placed in main loop.
    default:
        return;
    }
}

enum usbd_request_return_codes
usbdfu_control_request(struct usb_setup_data *req,
        uint16_t *len, void (**complete)(struct usb_setup_data *req)) {
    switch (req->bRequest) {
    case DFU_DNLOAD:
        if ((len == NULL) || (*len == 0)) {
            // wLength = 0 means leave DFU
            usbdfu_state = STATE_DFU_MANIFEST_SYNC;
            return USBD_REQ_HANDLED;
        } else {
            /* Copy download data for use on GET_STATUS. */
            prog.blocknum = req->wValue;
            // Beware overflows!
            prog.len = *len;
            if (prog.len > sizeof(prog.buf))
                prog.len = sizeof(prog.buf);
            memcpy(prog.buf, usbd_control_buffer, prog.len);
            usbdfu_state = STATE_DFU_DNLOAD_SYNC;
            return USBD_REQ_HANDLED;
        }
    case DFU_CLRSTATUS:
        // Just clears errors.
        if (usbdfu_state == STATE_DFU_ERROR)
            usbdfu_state = STATE_DFU_IDLE;
        return USBD_REQ_HANDLED;
    case DFU_ABORT:
        // Abort just returns to IDLE state.
        usbdfu_state = STATE_DFU_IDLE;
        return USBD_REQ_HANDLED;
    case DFU_DETACH:
        usbdfu_state = STATE_DFU_MANIFEST;
        return USBD_REQ_HANDLED;
    case DFU_UPLOAD:
        // Send data back to host by reading the image.
        usbdfu_state = STATE_DFU_UPLOAD_IDLE;
        if (!req->wValue) {
            // Send back supported commands.
            usbd_control_buffer[0] = 0x00;
            usbd_control_buffer[1] = CMD_SETADDR;
            usbd_control_buffer[2] = CMD_ERASE;
            *len = 3;
            return USBD_REQ_HANDLED;
        } else {
            // Send back data if only if we enabled that.
            #ifndef ENABLE_DFU_UPLOAD
            usbdfu_state = STATE_DFU_ERROR;
            *len = 0;
            #else
            // From formula Address_Pointer + ((wBlockNum - 2)*wTransferSize)
            uint32_t baseaddr = prog.addr + ((req->wValue - 2) * DFU_TRANSFER_SIZE);
            const uint32_t start_addr = 0x08000000 + (FLASH_BOOTLDR_SIZE_KB*1024);
            const uint32_t end_addr   = 0x08000000 + (        FLASH_SIZE_KB*1024);
            if (baseaddr >= start_addr && baseaddr + DFU_TRANSFER_SIZE <= end_addr) {
                memcpy(usbd_control_buffer, (void*)baseaddr, DFU_TRANSFER_SIZE);
                *len = DFU_TRANSFER_SIZE;
            } else {
                usbdfu_state = STATE_DFU_ERROR;
                *len = 0;
            }
            #endif
        }
        return USBD_REQ_HANDLED;
    case DFU_GETSTATUS: {
        // Perfom the action and register complete callback.
        uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */
        usbd_control_buffer[0] = usbdfu_getstatus(&bwPollTimeout);
        usbd_control_buffer[1] = bwPollTimeout & 0xFF;
        usbd_control_buffer[2] = (bwPollTimeout >> 8) & 0xFF;
        usbd_control_buffer[3] = (bwPollTimeout >> 16) & 0xFF;
        usbd_control_buffer[4] = usbdfu_state;
        usbd_control_buffer[5] = 0; /* iString not used here */
        *len = 6;
        *complete = usbdfu_getstatus_complete;
        return USBD_REQ_HANDLED;
        }
    case DFU_GETSTATE:
        // Return state with no state transision.
        usbd_control_buffer[0] = usbdfu_state;
        *len = 1;
        return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NEXT_CALLBACK;
}

#if defined(GPIO_DFU_BOOT_PORT) && defined(GPIO_DFU_BOOT_PIN)
int force_dfu_gpio(void) {
    gpio_periph_enable(GPIO_DFU_BOOT_PORT);
    gpio_set_input_pp(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
    //gpio_clear(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
    gpio_set(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
    for (unsigned int i = 0; i < 512; i++)
        __asm__("nop");
    uint16_t val = gpio_read(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
    gpio_set_input(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
    return val == 0;
}
#elif defined(ENABLE_CUSTOM_DFU_BOOT)
#include "dfu_bootstrap.h"
#else
__attribute__ ((weak)) int force_dfu_gpio(void){ return 0; }
#endif

static void clock_setup_72mhz_from_hse(void) {
    /**
     * Useless to use HSI as pll source clock,
     * despite to be possible to generate 48MHz
     * USB never makes proper connection
    */

    /* Enable external high-speed oscillator 8MHz. */
    RCC_CR |= RCC_CR_HSEON;
    while (!(RCC_CR & RCC_CR_HSERDY));
    /* Switch system to HSI */
    RCC_CFGR &= ~RCC_CFGR_SW;

    /* Ensure that PLL is off */
    RCC_CR &= ~(RCC_CR_PLLON);

    /*
     * Configure pll, system clock divider
     * and select PLL clock source
     */
    RCC_CFGR = (RCC_CFGR & 0x0FC00000) |
            #ifdef HSE12MHZ
                RCC_CFGR_PLLMUL6 |
            #else
                RCC_CFGR_PLLMUL9 | /* Assumes 8MHz */
            #endif
                RCC_CFGR_PLLSRC_HSE |
                RCC_CFGR_ADCPRE_2 |
                RCC_CFGR_PPRE2_1 |
                RCC_CFGR_PPRE1_2 |
                RCC_CFGR_HPRE_1 |
                RCC_CFGR_SW_HSE;

    // 0WS from 0-24MHz
    // 1WS from 24-48MHz
    // 2WS from 48-72MHz
    FLASH_ACR = (FLASH_ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_2WS;

    /* Enable PLL oscillator and wait for it to stabilize. */
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY));

    // Select PLL as SYSCLK source.
    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
}

int main(void) {
    /* Boot the application if it seems valid and we haven't been
     * asked to reboot into DFU mode. This should make the CPU to
     * boot into DFU if the user app has been erased. */

    #ifdef ENABLE_PROTECTIONS
    // Check for RDP protection, and in case it's not enabled, do it!
    volatile uint32_t *_flash_obr = (uint32_t*)0x4002201CU;
    if (!((*_flash_obr) & 0x2)) {
        // Read protection NOT enabled ->

        // Unlock option bytes
        _flash_unlock(1);

        // Delete them all
        _flash_erase_option_bytes();

        // Now write a pair of bytes that are complentary [RDP, nRDP]
        _flash_program_option_bytes(0x1FFFF800U, 0x33CC);

        usbdfu_unplug();

        // Now reset, for RDP to take effect. We should not re-enter this path
        _full_system_reset();
    }

    // Disable JTAG and SWD to prevent debugging/readout
    volatile uint32_t *_AFIO_MAPR = (uint32_t*)0x40010004U;
    *_AFIO_MAPR = (*_AFIO_MAPR & ~(0x7 << 24)) | (0x4 << 24);
    #endif

    volatile const uint32_t *base_addr = (volatile const uint32_t*)APP_ADDRESS;

    #ifdef ENABLE_CHECKSUM
    uint32_t imagesize = base_addr[0x20 / 4];
    #else
    uint32_t imagesize = 0;
    #endif

    int go_dfu = 0;
    if(reboot_check_dfu()){
        reboot_clear_flags();
        go_dfu = 1;
    }else{
        go_dfu =
        #ifdef ENABLE_WATCHDOG
            reset_due_to_watchdog() ||
        #endif
            (imagesize > FLASH_BOOTLDR_PAYLOAD_SIZE_KB * (1024/4)) ||
            force_dfu_gpio();
    }

    if (!go_dfu &&
       (*(volatile uint32_t *)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {

        // Do some simple XOR checking
        uint32_t xorv = 0;
        for (uint32_t i = 0; i < imagesize; i++)
            xorv ^= base_addr[i];

        if (xorv == 0) {  // Matches!
            // Clear flags
            reboot_clear_flags();
            #ifdef ENABLE_WATCHDOG
            // Enable the watchdog
            enable_iwdg(4096 * ENABLE_WATCHDOG / 26);
            #endif
            // Set vector table base address.
            volatile uint32_t *_csb_vtor = (uint32_t*)0xE000ED08U;
            *_csb_vtor = APP_ADDRESS & 0xFFFF;
            // Initialise master stack pointer.
            __asm__ volatile("msr msp, %0"::"g"
                     (*(volatile uint32_t *)APP_ADDRESS));
            // Jump to application.
            (*(void (**)())(APP_ADDRESS + 4))();
        }
    }

    gpio_periph_enable(GPIOA);

    #ifdef ENABLE_MCO
    RCC_CFGR = (RCC_CFGR & ~RCC_CFG_MCO) | RCC_CFG_MCO_PLL_2;
    gpio_set_mode(GPIOA, 8, 0xA);
    #endif

    clock_setup_72mhz_from_hse();

    usbdfu_unplug();

    gpio_set_af(GPIOA, 11);
    gpio_set_af(GPIOA, 12);

    usbdfu_state = STATE_DFU_IDLE;
    get_dev_unique_id(serial_no);
    usb_init();

    while (1) {
        // Poll based approach
        do_usb_poll();
        if (usbdfu_state == STATE_DFU_MANIFEST) {
            usbdfu_unplug();
            // USB device must detach, we just reset...
            _full_system_reset();
        }
    }
}

// Implement this here to save space, quite minimalistic :D
void *memcpy(void * dst, const void * src, size_t count) {
    uint8_t * dstb = (uint8_t*)dst;
    uint8_t * srcb = (uint8_t*)src;
    while (count--)
        *dstb++ = *srcb++;
    return dst;
}





