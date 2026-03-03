#!/usr/bin/env python3
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to you under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied.  See the License for the specific language governing
# permissions and limitations under the License.

"""
png2code.py - Convert LCD-previewed PNG assets to LVGL C arrays,
              and reconstruct images from C arrays for display emulation.

Mode 1 - PNG to C (default):
    Scans DEFAULT_PNG_DIR recursively for *_lcd.png files.
    Generates <symbol>.c in DEFAULT_OUT_DIR.

    python3 png2code.py
    python3 png2code.py --fit contain

Mode 2 - C to PNG (reverse):
    Scans DEFAULT_OUT_DIR for *.c files containing LVGL arrays.
    Reconstructs _reverse.png next to each .c file.

    python3 png2code.py --reverse

Dependencies:
    pip install Pillow
"""

import sys
import os
import re
import argparse
from datetime import datetime

try:
    from PIL import Image
except ImportError:
    print("ERROR: Pillow not installed. Run: pip install Pillow")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Project paths - adjust if repository moves
# ---------------------------------------------------------------------------

_BASE = (
    "/home/vinicius/git/nuttxspace/stm32h753zi"
    "/apps/hmi_manager/uiux"
)

DEFAULT_PNG_DIR = os.path.join(_BASE, "assets/png")
DEFAULT_OUT_DIR = os.path.join(_BASE, "assets")

DISPLAY_W      = 480
DISPLAY_H      = 320
BYTES_PER_LINE = 12

# ---------------------------------------------------------------------------
# RGB565 conversion
# ---------------------------------------------------------------------------

def rgb888_to_rgb565(r, g, b):
    """Pack R8 G8 B8 into uint16_t RGB565 (little-endian)."""
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def rgb565_to_rgb888(px):
    """Unpack uint16_t RGB565 to (R8, G8, B8)."""
    r = (px >> 11) & 0x1F
    g = (px >> 5)  & 0x3F
    b =  px        & 0x1F
    return (r << 3, g << 2, b << 3)

# ---------------------------------------------------------------------------
# Fit modes (same as preview.py for consistency)
# ---------------------------------------------------------------------------

def fit_stretch(img):
    return img.resize((DISPLAY_W, DISPLAY_H), Image.LANCZOS)


def fit_contain(img):
    img.thumbnail((DISPLAY_W, DISPLAY_H), Image.LANCZOS)
    canvas = Image.new('RGB', (DISPLAY_W, DISPLAY_H), (0, 0, 0))
    x = (DISPLAY_W - img.width)  // 2
    y = (DISPLAY_H - img.height) // 2
    canvas.paste(img, (x, y))
    return canvas


def fit_crop(img):
    ratio = max(DISPLAY_W / img.width, DISPLAY_H / img.height)
    nw    = int(img.width  * ratio)
    nh    = int(img.height * ratio)
    img   = img.resize((nw, nh), Image.LANCZOS)
    x     = (nw - DISPLAY_W) // 2
    y     = (nh - DISPLAY_H) // 2
    return img.crop((x, y, x + DISPLAY_W, y + DISPLAY_H))


FIT_MODES = {
    'stretch': fit_stretch,
    'contain': fit_contain,
    'crop':    fit_crop,
}

# ---------------------------------------------------------------------------
# Mode 1: PNG -> C array
# ---------------------------------------------------------------------------

def symbol_from_path(path):
    """
    Derive C symbol name from file path.

    'assets/png/themes/main_cyberPunk_lcd.png' -> 'img_main_cyberpunk'

    Rules:
      - strip _lcd suffix
      - lowercase
      - replace non-alphanumeric with _
      - prefix with img_
    """
    base   = os.path.splitext(os.path.basename(path))[0]
    base   = re.sub(r'_lcd$', '', base)
    base   = re.sub(r'[^a-zA-Z0-9]', '_', base).lower()
    return f"img_{base}"


def img_to_c(src, fit, out_dir):
    """
    Convert a *_lcd.png file to LVGL C array.

    Generates <out_dir>/<symbol>.c
    Returns (symbol, c_path, total_kb).
    """
    img = Image.open(src)

    if img.mode != 'RGB':
        img = img.convert('RGB')

    # _lcd.png files should already be 480x320 from preview.py,
    # but resize anyway to be safe.

    img = FIT_MODES[fit](img)

    w, h = img.size

    # Build pixel list and byte array (little-endian uint16_t)

    raw = []
    for y in range(h):
        for x in range(w):
            r, g, b = img.getpixel((x, y))
            px = rgb888_to_rgb565(r, g, b)
            raw.append(px & 0xFF)
            raw.append((px >> 8) & 0xFF)

    symbol   = symbol_from_path(src)
    total    = len(raw)
    total_kb = total / 1024
    ts       = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    src_name = os.path.basename(src)

    # Build array body

    rows = []
    for i in range(0, total, BYTES_PER_LINE):
        chunk = raw[i:i + BYTES_PER_LINE]
        comma = "," if (i + BYTES_PER_LINE) < total else ""
        rows.append(
            "    " +
            ", ".join(f"0x{b:02X}" for b in chunk) +
            comma
        )

    array_body = "\n".join(rows)

    c_content = "\n".join([
        "/**" + "*" * 71,
        f" * {symbol}.c",
        " *",
        f" * Auto-generated by png2code.py on {ts}",
        f" * Source     : {src_name}",
        f" * Resolution : {w}x{h} px",
        " * Format     : RGB565, little-endian",
        f" * Flash usage: {total} bytes ({total_kb:.1f} KB)",
        " *",
        " * DO NOT EDIT MANUALLY.",
        f" * Regenerate : python3 png2code.py",
        " *" + "*" * 71 + "/",
        "",
        "#include <lvgl/lvgl.h>",
        "",
        "/* Pixel data in flash (.rodata) - never copied to RAM */",
        "",
        f"static const uint8_t {symbol}_map[] =",
        "{",
        array_body,
        "};",
        "",
        f"/* Pass &{symbol} to lv_img_set_src() */",
        "",
        f"const lv_img_dsc_t {symbol} =",
        "{",
        "    .header =",
        "    {",
        "        .cf = LV_COLOR_FORMAT_RGB565,",
        f"        .w  = {w}U,",
        f"        .h  = {h}U,",
        "    },",
        f"    .data_size = {total}U,",
        f"    .data      = {symbol}_map,",
        "};",
        "",
    ])

    os.makedirs(out_dir, exist_ok=True)
    c_path = os.path.join(out_dir, f"{symbol}.c")

    with open(c_path, 'w') as f:
        f.write(c_content)

    return symbol, c_path, total_kb


def run_png_to_c(fit):
    """
    Scan DEFAULT_PNG_DIR for *_lcd.png and convert all to C arrays.
    """
    if not os.path.isdir(DEFAULT_PNG_DIR):
        print(f"ERROR: PNG dir not found: {DEFAULT_PNG_DIR}")
        sys.exit(1)

    sources = []
    for root, _dirs, files in os.walk(DEFAULT_PNG_DIR):
        for fname in sorted(files):
            if fname.endswith('_lcd.png'):
                sources.append(os.path.join(root, fname))

    if not sources:
        print(f"No *_lcd.png files found in: {DEFAULT_PNG_DIR}")
        print("Run preview.py first to generate _lcd.png files.")
        sys.exit(0)

    print(f"Mode   : PNG -> C array (LVGL)")
    print(f"Source : {DEFAULT_PNG_DIR}")
    print(f"Output : {DEFAULT_OUT_DIR}")
    print(f"Fit    : {fit}")
    print(f"Found  : {len(sources)} *_lcd.png file(s)")
    print()

    ok  = 0
    err = 0

    for src in sources:
        try:
            sym, cp, kb = img_to_c(src, fit, DEFAULT_OUT_DIR)
            print(
                f"  OK  {os.path.basename(src)}"
                f" -> {os.path.basename(cp)}"
                f" ({kb:.1f} KB flash)"
            )
            ok += 1
        except Exception as e:
            print(f"  ERR {os.path.basename(src)}: {e}")
            err += 1

    print()
    print(f"Done. {ok} converted, {err} errors.")

# ---------------------------------------------------------------------------
# Mode 2: C array -> PNG (reverse / display emulation)
# ---------------------------------------------------------------------------

def extract_array_from_c(path):
    """
    Parse a generated .c file and extract:
      - width, height from lv_img_dsc_t header
      - raw bytes from the _map[] array

    Returns (width, height, bytes_list) or None if not a valid asset.
    """
    content = open(path).read()

    # Extract width and height

    m_w = re.search(r'\.w\s*=\s*(\d+)U', content)
    m_h = re.search(r'\.h\s*=\s*(\d+)U', content)

    if not m_w or not m_h:
        return None

    w = int(m_w.group(1))
    h = int(m_h.group(1))

    # Extract hex bytes from _map[] array body

    m_arr = re.search(
        r'_map\[\]\s*=\s*\{([^}]+)\}', content, re.DOTALL
    )

    if not m_arr:
        return None

    hex_vals = re.findall(r'0x([0-9A-Fa-f]{2})', m_arr.group(1))

    if not hex_vals:
        return None

    raw = [int(h, 16) for h in hex_vals]
    return w, h, raw


def c_to_img(path):
    """
    Reconstruct PNG from a generated LVGL C array file.
    Output saved next to the .c file with _reverse.png suffix.
    Returns output path.
    """
    result = extract_array_from_c(path)

    if result is None:
        raise ValueError("Not a valid png2code-generated C file")

    w, h, raw = result

    if len(raw) != w * h * 2:
        raise ValueError(
            f"Byte count mismatch: got {len(raw)}, "
            f"expected {w * h * 2}"
        )

    img        = Image.new('RGB', (w, h))
    pixels_out = img.load()

    idx = 0
    for y in range(h):
        for x in range(w):
            lo  = raw[idx]
            hi  = raw[idx + 1]
            idx += 2
            px  = lo | (hi << 8)
            pixels_out[x, y] = rgb565_to_rgb888(px)

    base = os.path.splitext(path)[0]
    out  = f"{base}_reverse.png"
    img.save(out, 'PNG')
    return out


def run_c_to_png():
    """
    Scan DEFAULT_OUT_DIR for *.c files and reconstruct images.
    Skips files that don't contain valid LVGL arrays.
    """
    if not os.path.isdir(DEFAULT_OUT_DIR):
        print(f"ERROR: Output dir not found: {DEFAULT_OUT_DIR}")
        sys.exit(1)

    sources = []
    for fname in sorted(os.listdir(DEFAULT_OUT_DIR)):
        if fname.endswith('.c'):
            sources.append(
                os.path.join(DEFAULT_OUT_DIR, fname)
            )

    if not sources:
        print(f"No .c files found in: {DEFAULT_OUT_DIR}")
        sys.exit(0)

    print(f"Mode   : C array -> PNG (display emulation)")
    print(f"Source : {DEFAULT_OUT_DIR}")
    print(f"Found  : {len(sources)} .c file(s)")
    print()

    ok  = 0
    err = 0

    for src in sources:
        try:
            out = c_to_img(src)
            print(
                f"  OK  {os.path.basename(src)}"
                f" -> {os.path.basename(out)}"
            )
            ok += 1
        except Exception as e:
            print(f"  SKIP {os.path.basename(src)}: {e}")
            err += 1

    print()
    print(f"Done. {ok} reconstructed, {err} skipped.")

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "png2code: Convert *_lcd.png assets to LVGL C arrays, "
            "or reconstruct images from C arrays for display emulation."
        )
    )
    parser.add_argument(
        "--reverse", action='store_true',
        help=(
            "Reverse mode: reconstruct _reverse.png from C arrays. "
            "Default: convert *_lcd.png to C arrays."
        )
    )
    parser.add_argument(
        "--fit", choices=FIT_MODES.keys(), default='stretch',
        help="Fit mode for PNG->C conversion (default: stretch)"
    )

    args = parser.parse_args()

    if args.reverse:
        run_c_to_png()
    else:
        run_png_to_c(args.fit)


if __name__ == "__main__":
    main()
