#!/bin/bash
cd ~/git/nuttxspace/stm32h753zi/nuttx

for config in socketcan socketcan_example_canio socketcan_st7796_lvgl st7796_fb_spi lvgl_animation lvgl_canio lvgl_can_placeholder; do
  echo "=== Processing $config ==="
  make distclean > /dev/null 2>&1
  ./tools/configure.sh nucleo-h753zi:$config
  make savedefconfig
  lines=$(wc -l < defconfig)
  echo "Generated defconfig: $lines lines"

  if [ $lines -lt 500 ]; then
    cp defconfig boards/arm/stm32h7/nucleo-h753zi/configs/$config/defconfig
    echo "✓ $config saved (clean)"
  else
    echo "✗ $config FAILED (still dirty: $lines lines)"
  fi
  echo ""
done

echo "=== Summary ==="
wc -l boards/arm/stm32h7/nucleo-h753zi/configs/*/defconfig
