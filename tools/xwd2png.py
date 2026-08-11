#!/usr/bin/env python3
"""Parse X11 XWD file and convert to PNG. Handles 24/32-bit ZPixmap."""
import struct
import sys
from PIL import Image

def parse_xwd(path, out):
    with open(path, "rb") as f:
        data = f.read()
    # XWD v7 header: 22 CARD32, big-endian
    hdr = struct.unpack(">22I", data[:88])
    (header_size, file_version, pixmap_format, depth, width, height,
     xoffset, byte_order, bitmap_unit, bitmap_bit_order, bitmap_pad,
     bits_per_pixel, bytes_per_line, visual_class,
     red_mask, green_mask, blue_mask, bits_per_rgb,
     colormap_entries, ncolors, window_width, window_height) = hdr
    print(f"version={file_version} fmt={pixmap_format} depth={depth} "
          f"{width}x{height} bpp={bits_per_pixel} bpl={bytes_per_line} "
          f"byte_order={'MSB' if byte_order else 'LSB'} "
          f"masks=#{red_mask:06x}/#{green_mask:06x}/#{blue_mask:06x} "
          f"ncolors={ncolors} header_size={header_size}", file=sys.stderr)

    pixel_start = header_size
    bpp = bits_per_pixel
    if pixmap_format != 2:  # ZPixmap
        print("not ZPixmap", file=sys.stderr)
        return 1

    if ncolors > 0:
        # colormap entries follow header; pixel data after them.
        # Each entry: 12 bytes (CARD32 pixel, CARD16 red/green/blue, CARD8 flags/pad)
        pixel_start = header_size + ncolors * 12

    raw = data[pixel_start:pixel_start + bytes_per_line * height]

    if bpp == 32:
        # decide channel order by masks (value in the 32-bit word, byte order MSB)
        if byte_order == 1:  # MSBFirst: word bytes are MSB..LSB
            # e.g. xRGB word = 0x00RRGGBB -> bytes [00, R, G, B]
            if red_mask == 0x00ff0000:
                mode = "XRGB"
            elif red_mask == 0xff000000:
                mode = "RGBX"
            else:
                mode = "RGBX"
            img = Image.frombytes("RGB", (width, height), raw, "raw", mode)
        else:  # LSBFirst: word bytes are LSB..MSB, xRGB -> [B, G, R, 00]
            img = Image.frombytes("RGB", (width, height), raw, "raw", "BGRX")
    elif bpp == 24:
        img = Image.frombytes("RGB", (width, height), raw, "raw", "RGB")
    elif bpp == 16:
        img = Image.frombytes("RGB", (width, height), raw, "raw", "BGR;16")
    else:
        print(f"unsupported bpp={bpp}", file=sys.stderr)
        return 1

    img.save(out, optimize=True)
    print(f"saved {out} {img.size}", file=sys.stderr)
    return 0

if __name__ == "__main__":
    sys.exit(parse_xwd(sys.argv[1], sys.argv[2]))
