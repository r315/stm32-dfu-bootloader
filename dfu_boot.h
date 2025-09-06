#ifndef _dfu_boot_h_
#define _dfu_boot_h_

#include <stdint.h>

//dfu-util -a 0 -s 0x08001000 -D ../build/psu_v3.bin -R

#define DFU_MAGIC_STR   "_DFU_BOOT_MAGIC_"

extern uint64_t _magic;

// Reboots the system into the bootloader, making sure
// it enters in DFU mode.
static inline void reboot_into_dfu() {
	*(volatile uint64_t*)&_magic = *(volatile uint64_t*)&DFU_MAGIC_STR;
}

#endif /* _dfu_boot_h_ */
