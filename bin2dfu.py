import sys
import struct
import os
import zlib

DFU_SUFFIX_LEN = 16

# USB DFU specification constants
DFU_SIGNATURE = b'UFD'  # Actually 'DFU' in little endian
DFU_VERSION = 0x011A    # Version 1.1a
DFUSE_VERSION = 0x01    # DfuSe file format version

# TODO Set szTargetName name on each target prefix with elementAddress
def create_dfu(segments, dfu_file, target_name=b"@Internal Flash   /0x08000000/"):
    """
    segments: list of tuples (bin_file, address)
    target_name: up to 255 bytes, zero-padded
    """
    elements = []
    total_size = 0

    for bin_file, address in segments:
        with open(bin_file, "rb") as f:
            data = f.read()
        size = len(data)
        element = struct.pack('<I', address)  # Start address
        element += struct.pack('<I', size)    # Size
        element += data                       # Firmware content
        elements.append(element)
        total_size += len(element)

    # === DFU PREFIX (11 bytes total) ===
    prefix = b'DfuSe'                          # Signature (5 bytes)
    prefix += struct.pack('<B', DFUSE_VERSION) # Version (1 byte)
    prefix += struct.pack('<I', 11 + 274 + total_size)  # Total size (4 bytes)
    prefix += struct.pack('<B', 1)             # Number of targets (1)

    # === TARGET PREFIX (274 bytes total) ===
    target_prefix = b'Target'                  # Signature (6 bytes)
    target_prefix += struct.pack('<B', 0)      # Alternate setting
    target_prefix += struct.pack('<I', 1)      # Named flag (1 = has name, 4 bytes)
    target_prefix += target_name.ljust(255, b'\x00')      # Target name (255 bytes)
    target_prefix += struct.pack('<I', total_size)  # Target size
    target_prefix += struct.pack('<I', len(elements))     # Number of elements

    # === ELEMENTS ===
    elements_blob = b''.join(elements)

    # === DFU SUFFIX (16 bytes) ===
    suffix = struct.pack('<H', 0)        # Firmware version (2 bytes)
    suffix += struct.pack('<H', 0)       # Target PID (2 bytes)
    suffix += struct.pack('<H', 0x0483)       # Target VID (2 bytes)
    suffix += struct.pack('<H', DFU_VERSION)  # DFU spec version (2 bytes)
    suffix += b'UFD'                          # Signature (3 bytes)
    suffix += struct.pack('<B', DFU_SUFFIX_LEN)  # Length (1 byte)
    suffix += struct.pack('<I', 0)            # Placeholder CRC (4 bytes)

    # Build file in memory for CRC
    full_dfu = prefix + target_prefix + elements_blob + suffix

    # Compute CRC32/JAMCRC of entire file minus the last 4 bytes (the CRC field itself)
    crc = 0xFFFFFFFF - zlib.crc32(full_dfu[:-4])

    # Replace placeholder with real CRC
    suffix = suffix[:-4] + struct.pack('<I', crc)

    # Write final file
    with open(dfu_file, "wb") as f:
        f.write(prefix)
        f.write(target_prefix)
        f.write(elements_blob)
        f.write(suffix)

    print(f"[+] DFU file created: {dfu_file}, size={os.path.getsize(dfu_file)} bytes, CRC=0x{crc:08X}")


def main():
    if len(sys.argv) < 4 or len(sys.argv) % 2 != 0:
        print(f"Usage: {sys.argv[0]} output.dfu input1.bin address1 [input2.bin address2 ...]")
        print("Example: python bin_to_dfu.py firmware.dfu bootloader.bin 0x08000000 app.bin 0x08004000")
        sys.exit(1)

    dfu_file = sys.argv[1]
    segments = []
    args = sys.argv[2:]
    for i in range(0, len(args), 2):
        bin_file = args[i]
        address = int(args[i+1], 0)
        segments.append((bin_file, address))

    create_dfu(segments, dfu_file)


if __name__ == "__main__":
    main()
