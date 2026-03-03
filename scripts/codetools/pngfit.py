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
preview.py - Preview how an image will look on the ST7796 display.

Resizes to 480x320 and simulates RGB565 color depth.
Output suffix: _lcd.png

Single file:
    python3 preview.py <input>
    python3 preview.py <input> --fit contain

Batch (all images in directory, recursively):
    python3 preview.py --batch <directory>
    python3 preview.py --batch <directory> --fit crop

Fit modes:
    stretch  - fill 480x320 ignoring aspect ratio (default)
    contain  - fit inside 480x320 preserving aspect, black bars
    crop     - fill 480x320 preserving aspect, crop edges

Dependencies:
    pip install Pillow
"""

import sys
import os
import argparse

try:
    from PIL import Image
except ImportError:
    print("ERROR: Pillow not installed. Run: pip install Pillow")
    sys.exit(1)

DISPLAY_W        = 480
DISPLAY_H        = 320
SUPPORTED_EXTS   = {'.png', '.jpg', '.jpeg', '.bmp', '.webp'}
DEFAULT_BATCH_DIR = (
    "/home/vinicius/git/nuttxspace/stm32h753zi"
    "/apps/hmi_manager/uiux/assets/png"
)

# ---------------------------------------------------------------------------
# RGB565 simulation
# ---------------------------------------------------------------------------

def to_rgb565_and_back(img):
    """
    Simulate RGB565 color depth by quantizing each channel.

    R: 8-bit -> 5-bit -> 8-bit  (loses 3 LSBs)
    G: 8-bit -> 6-bit -> 8-bit  (loses 2 LSBs)
    B: 8-bit -> 5-bit -> 8-bit  (loses 3 LSBs)

    Shows exactly how the image will look on the display
    before burning it to flash.
    """
    out        = Image.new('RGB', img.size)
    pixels_in  = img.load()
    pixels_out = out.load()

    for y in range(img.size[1]):
        for x in range(img.size[0]):
            r, g, b = pixels_in[x, y]
            pixels_out[x, y] = (
                (r >> 3) << 3,
                (g >> 2) << 2,
                (b >> 3) << 3,
            )

    return out

# ---------------------------------------------------------------------------
# Fit modes
# ---------------------------------------------------------------------------

def fit_stretch(img):
    """Resize to exactly 480x320, ignoring aspect ratio."""
    return img.resize((DISPLAY_W, DISPLAY_H), Image.LANCZOS)


def fit_contain(img):
    """
    Fit inside 480x320 preserving aspect ratio.
    Remaining area filled with black.
    """
    img.thumbnail((DISPLAY_W, DISPLAY_H), Image.LANCZOS)
    canvas = Image.new('RGB', (DISPLAY_W, DISPLAY_H), (0, 0, 0))
    x = (DISPLAY_W - img.width)  // 2
    y = (DISPLAY_H - img.height) // 2
    canvas.paste(img, (x, y))
    return canvas


def fit_crop(img):
    """
    Fill 480x320 preserving aspect ratio, crop what overflows.
    """
    ratio_w = DISPLAY_W / img.width
    ratio_h = DISPLAY_H / img.height
    ratio   = max(ratio_w, ratio_h)
    new_w   = int(img.width  * ratio)
    new_h   = int(img.height * ratio)
    img     = img.resize((new_w, new_h), Image.LANCZOS)
    x       = (new_w - DISPLAY_W) // 2
    y       = (new_h - DISPLAY_H) // 2
    return img.crop((x, y, x + DISPLAY_W, y + DISPLAY_H))


FIT_MODES = {
    'stretch': fit_stretch,
    'contain': fit_contain,
    'crop':    fit_crop,
}

# ---------------------------------------------------------------------------
# Core conversion
# ---------------------------------------------------------------------------

def convert_one(src, fit):
    """
    Convert a single image file.

    Output saved next to source with _lcd suffix.
    Returns the output path.
    """
    base = os.path.splitext(src)[0]
    out  = f"{base}_lcd.png"

    img = Image.open(src)

    if img.mode != 'RGB':
        img = img.convert('RGB')

    img = FIT_MODES[fit](img)
    img = to_rgb565_and_back(img)
    img.save(out, 'PNG')

    return out


def collect_images(directory):
    """
    Walk directory recursively and collect supported image paths.
    Skips _lcd.png files to avoid reprocessing outputs.
    """
    found = []
    for root, _dirs, files in os.walk(directory):
        for fname in sorted(files):
            ext = os.path.splitext(fname)[1].lower()
            if ext not in SUPPORTED_EXTS:
                continue
            if fname.endswith('_lcd.png'):
                continue
            found.append(os.path.join(root, fname))
    return found

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            f"Preview images as they will appear on "
            f"ST7796 {DISPLAY_W}x{DISPLAY_H} RGB565 display. "
            f"Output suffix: _lcd.png"
        )
    )

    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument(
        "input", nargs='?', default=None,
        help="Single input image file"
    )
    group.add_argument(
        "--batch", metavar="DIR",
        help=(
            "Convert all images in directory (recursive). "
            f"Default: {DEFAULT_BATCH_DIR}"
        )
    )

    parser.add_argument(
        "--fit", choices=FIT_MODES.keys(), default='stretch',
        help="Fit mode (default: stretch)"
    )

    args = parser.parse_args()

    # No arguments: run batch on hardcoded default directory

    if args.input is None and args.batch is None:
        args.batch = DEFAULT_BATCH_DIR

    flash_kb = (DISPLAY_W * DISPLAY_H * 2) / 1024

    # --- Single file mode ---

    if args.input is not None:
        if not os.path.isfile(args.input):
            print(f"ERROR: File not found: {args.input}")
            sys.exit(1)

        img = Image.open(args.input)
        print(f"Input  : {args.input} ({img.width}x{img.height})")
        print(f"Fit    : {args.fit}")
        img.close()

        out = convert_one(args.input, args.fit)

        print(f"Output : {out}")
        print(f"Flash  : {flash_kb:.0f} KB if embedded as C array")
        print("Done.")
        return

    # --- Batch mode ---

    if not os.path.isdir(args.batch):
        print(f"ERROR: Directory not found: {args.batch}")
        sys.exit(1)

    images = collect_images(args.batch)

    if not images:
        print(f"No supported images found in: {args.batch}")
        sys.exit(0)

    print(f"Batch  : {args.batch}")
    print(f"Fit    : {args.fit}")
    print(f"Found  : {len(images)} image(s)")
    print()

    ok  = 0
    err = 0

    for src in images:
        try:
            img  = Image.open(src)
            w, h = img.width, img.height
            img.close()
            out = convert_one(src, args.fit)
            print(f"  OK  {os.path.basename(src)}"
                  f" ({w}x{h}) -> {os.path.basename(out)}")
            ok += 1
        except Exception as e:
            print(f"  ERR {os.path.basename(src)}: {e}")
            err += 1

    print()
    print(f"Done. {ok} converted, {err} errors.")
    print(
        f"Flash per image: {flash_kb:.0f} KB if embedded as C array"
    )


if __name__ == "__main__":
    main()
