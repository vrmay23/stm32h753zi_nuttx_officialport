# HMI Manager — Master Prompt for New Theme Generation
# STM32H753ZI + Apache NuttX + LVGL 9.2 + ST7796 480x320

---

## CONTEXT

You are helping develop a professional automotive HMI (Human-Machine Interface)
for a Battery Electric Vehicle (BEV) truck instrument cluster running on:

- **MCU**: STM32H753ZI Nucleo-144 (Cortex-M7 @ 480 MHz)
- **RTOS**: Apache NuttX 12.8+
- **Display**: ST7796 SPI 480×320 @ 16bpp RGB565
- **Graphics**: LVGL 9.2
- **CAN**: FDCAN1 via SocketCAN (500 kbps, 29-bit Extended IDs)
- **Build**: NuttX Application framework (Makefile + CMakeLists.txt)

The application is called `hmi_manager` — a daemon that runs in the background
managing CAN RX/TX, LVGL rendering, button inputs, and LED control.

---

## WHAT YOU WILL RECEIVE

The user will provide:

1. **An SVG file** (`lvgl_placeholder_<theme_name>.svg`) created in Inkscape
   at exactly **126.76mm × 84.57mm** (the physical LCD dimensions).
   This SVG contains ONLY the widget placeholder shapes — coloured rectangles
   and circles drawn over a screenshot of the static background image.
   The SVG has a global group transform `translate(-231.67059, -25.820113)`
   plus an outer transform `translate(-0.28242986, -0.23069661)`.

2. **A static background PNG** (`main_<theme_name>_lcd.png`) at 480×320 pixels,
   already converted to RGB565 format and stored as a C array via `png2code.py`.
   This image is loaded once at startup and set as the background of `lv_scr_act()`.
   All LVGL widgets are layered transparently ON TOP of this image.

3. **The theme name** (e.g. `retro_t3`, `cyberpunk`, `modern`, `nordic`).

---

## HOW TO EXTRACT COORDINATES FROM THE SVG

The SVG uses mm coordinates. Convert to LCD pixels using:

```
Total offset TX = -231.67059 + (-0.28242986) = -231.95302 mm
Total offset TY = -25.820113 + (-0.23069661) = -26.050810 mm

Canvas bounding rect (black stroke in SVG):
  x=232.35825, y=26.456036, w=125.95261, h=83.762268 mm

Canvas origin in absolute SVG space:
  origin_x = 232.35825 + TX = ~0.405 mm
  origin_y = 26.456036 + TY = ~0.405 mm

Scale:
  SX = 480.0 / 125.95261 = 3.8110 px/mm
  SY = 320.0 / 83.762268 = 3.8203 px/mm

Conversion formula:
  lcd_x = round((svg_x + TX - origin_x) * SX)
  lcd_y = round((svg_y + TY - origin_y) * SY)
  lcd_w = round(svg_w * SX)
  lcd_h = round(svg_h * SY)

For circles:
  lcd_cx = round((svg_cx + TX - origin_x) * SX)
  lcd_cy = round((svg_cy + TY - origin_y) * SY)
  lcd_r  = round(svg_r * SX)
```

Always verify by drawing all widgets on a blank 480×320 image and comparing
with the reference overlay screenshot.

---

## MANDATORY WIDGETS

Every theme MUST implement ALL of the following widgets.
The SVG placeholder colours identify each widget:

### GAUGE NEEDLES (lv_line over static background)

| Widget | SVG colour | Description |
|--------|-----------|-------------|
| Speed needle | Blue circle (outer) | White, long. Pivot = outer circle centre |
| RPM needle | Blue circle (inner) | Amber, half length of speed needle. Same pivot |
| Current needle | Blue circle (right, outer) | Amber, full length. Right gauge pivot |
| KW needle | Blue circle (right, inner reference) | White, shorter. Same pivot as current. Shows electrical power |

**Needle geometry (both gauges, same sweep):**
- Start angle: **210°** (7 o'clock = zero/minimum)
- Sweep: **300°** (ends at 5 o'clock = maximum)
- Angle formula: `angle = 210 + (value / max_value) * 300`
- Tip formula: `tip_x = cx + len * sin(angle_rad)`, `tip_y = cy - len * cos(angle_rad)`
- LVGL angle convention: 0° = 12 o'clock, clockwise positive

**Speedometer needle lengths:**
- Speed needle: `r_outer - 9` px (white)
- RPM needle: `r_inner + 12` px (amber, shorter, stays inside inner ring)

**Regen gauge needle lengths:**
- Current needle: `r_outer - 9` px (amber)
- KW needle: ~50 px or `r_inner + 7` px (white, shorter)

**KW needle colour logic:**
- Positive watts (motoring) → white
- Negative watts (regenerating) → amber

**KW is computed internally** from voltage × current — no separate CAN message needed:
```c
// In widgets_set_current(amps):
watts = (g_cached_voltage_dv * amps) / 10;
widgets_set_kw(watts);
```

### TEXT / BAR WIDGETS

| Widget | SVG colour | Type | Content | Notes |
|--------|-----------|------|---------|-------|
| Gear position | Cyan | lv_label | N / D / R / P | Large font, centred in rect |
| Drive mode | Green | lv_label | STANDARD / SPORT / ECO | Small font, colour changes per mode |
| Battery % | Red | lv_bar | 0–100% fill | Colour-coded (see thresholds) |
| Odometer | Yellow | lv_label | "NNNNNN km" | 6-digit zero-padded |
| External temp | Pink | lv_label | "NN°C" | From CAN |
| Range prediction | Purple | lv_label | "NNN km" | From CAN |
| Charge time | Orange | lv_label | "NhNNm" | From CAN |
| Aux / Warning | Cream/White | lv_label | Free text | Warnings, status, future use |

**FUTURE widgets (not yet in CAN DBC — reserve space in SVG):**
- **kWh** (energy consumed — lv_label, small)
- **Regeneration total** (energy recovered — lv_label, small)
- These should have placeholder rects in the SVG even if not yet active

### BATTERY BAR COLOUR THRESHOLDS

```c
#define BAT_WARN_PCT      26   // 27–100% = GREEN  (#00CC44)
#define BAT_WARN_LOW_PCT  11   // 11–26%  = ORANGE (#FF6600)
#define BAT_CRIT_PCT       1   // 1–10%   = RED    (#FF2000)
                               // 0%      = EMPTY (indicator hidden)
```

---

## MANDATORY CODE STRUCTURE

Generate exactly these files (replace `<theme>` with theme name):

### 1. `uiux/widgets.h`

Must contain:
- `#define WIDGET_DIR_*` direction constants (0=Neutral, 1=Forward, 2=Reverse)
- `#define WIDGET_MODE_*` mode constants (0=Standard, 1=Sport, 2=Eco)
- Colour palette using `lv_color_hex()`
- ALL layout `#define` constants from SVG (named descriptively)
- Gauge geometry constants: `SPD_CX`, `SPD_CY`, `SPD_R_OUTER`, `SPD_R_INNER`,
  `SPD_NEEDLE_LEN`, `RPM_NEEDLE_LEN`, same for `REG_*`
- `SPD_ANGLE_START 210`, `SPD_ANGLE_SWEEP 300` (same for REG)
- `BAT_VOLTAGE_MAX_DV`, `BAT_VOLTAGE_MIN_DV`, threshold defines
- Full API declarations for ALL functions listed below

### 2. `uiux/widgets.c`

Must implement:
```c
int  widgets_init(void);

// Gauge needles
void widgets_set_speed(int kmh);
void widgets_set_rpm(int rpm);
void widgets_set_current(int amps);   // also computes and calls widgets_set_kw()
void widgets_set_kw(int watts);       // positive=motoring white, negative=regen amber

// Text/bar widgets
void widgets_set_direction(int dir);
void widgets_set_mode(int mode);
void widgets_set_battery_pct(int pct);
void widgets_set_voltage(int dv);     // maps dV to %, caches for kW computation
void widgets_set_odometer(uint32_t km);
void widgets_set_ext_temp(int temp_c);
void widgets_set_range(int km);
void widgets_set_chg_time(int minutes);
void widgets_set_warning(const char *msg);
// Stubs (implement when CAN DBC has these signals):
// void widgets_set_kwh(float kwh);           // energy consumed
// void widgets_set_regen_total(float kwh);   // energy recovered
```

**LVGL 9.2 requirements:**
- Use `lv_obj_set_style_*()` local style setters (NOT `lv_style_t` structs)
- Use `lv_color_hex()` for colours
- Call `lv_obj_update_layout()` before `lv_obj_get_width()` for centering
- All label backgrounds: `LV_OPA_TRANSP` (background image shows through)
- Needle lines: `lv_line_create()` + `lv_line_set_points()` with `lv_point_precise_t`
- Link with `-lm` for `sinf()`/`cosf()` (needle angle computation)

**Private static variables required:**
```c
static lv_obj_t           *g_spd_line, *g_rpm_line, *g_reg_line, *g_kw_line;
static lv_point_precise_t  g_spd_pts[2], g_rpm_pts[2], g_reg_pts[2], g_kw_pts[2];
static lv_obj_t           *g_spd_pivot, *g_reg_pivot;
static lv_obj_t           *g_gear_lbl, *g_mode_lbl, *g_bat_bar;
static lv_obj_t           *g_odo_lbl, *g_temp_lbl, *g_range_lbl;
static lv_obj_t           *g_chg_lbl, *g_aux_lbl;
static int                 g_cached_voltage_dv = 0;
```

### 3. `app_hmi_manager.c` (patches only — do not rewrite the full file)

Apply these patches to the existing file:

**a) Add globals:**
```c
static int g_rpm        = 0;
static int g_drive_mode = WIDGET_MODE_STANDARD;
```

**b) Speed CAN handler — call both set_speed AND set_rpm:**
```c
g_rpm   = (rpm < 0) ? -rpm : rpm;
g_speed = rpm_to_kmh(g_rpm);
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
widgets_set_speed(g_speed);
widgets_set_rpm(g_rpm);
#endif
```

**c) Gear direction — check drive_state, not just direction:**
```c
if (msg.demanded_drive_state == ...DISABLE_CHOICE)
  g_direction = WIDGET_DIR_NEUTRAL;
else if (msg.demanded_drive_direction == ...BACKWARD_CHOICE)
  g_direction = WIDGET_DIR_REVERSE;
else
  g_direction = WIDGET_DIR_FORWARD;
```

**d) Pedal mode button — call widgets_set_mode():**
```c
g_drive_mode = g_driver_cmd.pedal_setting;
#ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
widgets_set_mode(g_drive_mode);
#endif
```

**e) Voltage handler — voltage caches for kW:**
```c
widgets_set_voltage(g_voltage);  // caches dV internally for V*I computation
```

**f) Add current CAN handler stub** with comment explaining where to plug in
   the inverter current message when available in DBC.

### 4. `Makefile` — add `-lm`:
```makefile
LDLIBS += -lm
```

### 5. `CMakeLists.txt` — add math dependency:
```cmake
DEPENDS
  m)   # -lm for sinf/cosf in widgets.c
```

---

## CAN MESSAGE REFERENCE (DBC)

All IDs are 29-bit extended.

### TX (HMI sends)
| Message | ID | DLC | Cycle | Key signals |
|---------|-----|-----|-------|-------------|
| DriverCommands | 0x0C03FE32 | 1 | 100ms | bit0=direction, bit1=drive_state, bit2=reset_faults, bit3=dc_link_demand, bit4-7=pedal |
| HmiInfo | 0x0C0B0032 | 4 | 1000ms | bytes0-3=uptime_ms |

### RX (HMI receives)
| Message | ID | DLC | Key signals for display |
|---------|-----|-----|------------------------|
| InverterSpeedInfo | 0x0C0AFEFE | 4 | bytes2-3=em_speed_rpm (int14 LE) → speed+rpm needles |
| SystemVoltages | 0x0C06FE00 | 8 | bytes6-7=meas_batt_voltage (uint16 LE, scale 0.01V) → battery bar |
| InverterInfo | 0x0C04FE00 | 6 | byte4=mft_state, byte5=dc_link_state → LEDs |
| DriverCommands | 0x0C03FE32 | 1 | loopback → gear indicator |

### Signals not yet in DBC (reserve CAN handler stubs):
- **Motor current** → `widgets_set_current(amps)` → triggers kW computation
- **External temperature** → `widgets_set_ext_temp(temp_c)`
- **Range prediction** → `widgets_set_range(km)`
- **Charge time** → `widgets_set_chg_time(minutes)`
- **Energy consumed (kWh)** → `widgets_set_kwh(kwh)` (stub)
- **Regeneration total (kWh)** → `widgets_set_regen_total(kwh)` (stub)

---

## BUTTON MAPPING

| Button | Define | Action | Widget effect |
|--------|--------|--------|---------------|
| BTN0 | BUTTON_DC_LINK_TOGGLE | Toggle DC Link | LEDs |
| BTN1 | BUTTON_FORWARD | Enable forward (needs DC Link ON) | Gear → D |
| BTN2 | BUTTON_REVERSE | Enable reverse (needs DC Link ON) | Gear → R |
| BTN3 | BUTTON_NEUTRAL | Disable drive | Gear → N |
| BTN4 | BUTTON_RESET_FAULTS | Reset inverter faults | — |
| BTN5 | BUTTON_PEDAL_MODE | Cycle ECO→NORMAL→SPORT | Mode label |
| BTN6 | — | Carousel: DASHBOARD→THEME→SPLASH | Screen change |
| BTN7 | — | Screen OFF (backlight) | — |

---

## THREADING MODEL

```
Main thread      — command socket (hmi_ctl) + button polling
can_rx_thread    — blocking read() on CAN socket, PRIORITY_VERY_HIGH
can_tx_100ms     — cyclic DriverCommands TX, PRIORITY_HIGH
can_tx_1000ms    — cyclic HmiInfo TX, PRIORITY_HIGH
lvgl_thread      — lv_tick_inc + lv_timer_handler, PRIORITY_LOW
```

**CRITICAL**: All `widgets_set_*()` calls MUST happen in `lvgl_thread` only.
Use a `volatile bool g_ui_dirty` flag: `can_rx_thread` updates globals and sets
the flag; `lvgl_thread` reads flag and calls widgets. This prevents LVGL race
conditions.

---

## FONT REQUIREMENTS

Add to `lv_conf.h` or Kconfig:
```
CONFIG_LV_FONT_MONTSERRAT_28=y   (gear letter N/D/R/P)
CONFIG_LV_FONT_MONTSERRAT_14=y   (all small labels)
```

Optional upgrade: generate a 7-segment font from `DSEG7Classic-Bold.ttf`
at https://lvgl.io/tools/fontconverter (size=24, bpp=4) for retro look.
Replace `FONT_GEAR` and `FONT_STD` defines in `widgets.c`.

---

## FILE NAMING CONVENTION

```
uiux/assets/png/themes/main_<theme>_lcd.png   ← source PNG (480x320 RGB565)
uiux/assets/img_main_<theme>.c                ← C array from png2code.py
uiux/assets/lvgl_placeholder_<theme>.svg      ← Inkscape SVG (source of truth)
uiux/widgets.h                                ← shared API (updated per theme)
uiux/widgets.c                                ← implementation (updated per theme)
```

Run `python3 png2code.py` to regenerate C arrays after changing PNGs.
Run `python3 png2code.py --reverse` to verify alignment (generates `_reverse.png`).

---

## VERIFICATION CHECKLIST

After generating code for a new theme, verify:

- [ ] All coordinates extracted from SVG using the conversion formula
- [ ] Verification image generated (all widget boxes on blank 480×320 canvas)
- [ ] Verification image compared against reference overlay screenshot
- [ ] Needle at 0 points to 7 o'clock (210°)
- [ ] Needle at max points to 5 o'clock (510° = 150°)
- [ ] Battery bar: 0%=empty, 1-10%=red, 11-26%=orange, 27-100%=green
- [ ] Gear: N=white, D=white, R=orange
- [ ] Mode: STANDARD=green, SPORT=orange, ECO=cyan-green
- [ ] KW: positive=white needle, negative=amber needle
- [ ] All NULL guards in every `widgets_set_*()` function
- [ ] `-lm` in Makefile and CMakeLists.txt
- [ ] `lv_obj_update_layout()` called before centering labels

---

## TEST CAN FRAMES (for validation after flash)

```bash
# Speedometer sweep (0→160 km/h, every 30°)
cansend can0 0C0AFEFE#00000000   # 0   km/h  — 7h
cansend can0 0C0AFEFE#00008605   # 16  km/h  — 8h
cansend can0 0C0AFEFE#00000D0B   # 32  km/h  — 9h
cansend can0 0C0AFEFE#00009410   # 48  km/h  — 10h
cansend can0 0C0AFEFE#00001A16   # 64  km/h  — 11h
cansend can0 0C0AFEFE#0000A11B   # 80  km/h  — 12h (mid)
cansend can0 0C0AFEFE#00002821   # 96  km/h  — 1h
cansend can0 0C0AFEFE#0000AE26   # 112 km/h  — 2h
cansend can0 0C0AFEFE#0000352C   # 128 km/h  — 3h
cansend can0 0C0AFEFE#0000BC31   # 144 km/h  — 4h
cansend can0 0C0AFEFE#00004337   # 160 km/h  — 5h (max)

# Battery bar
cansend can0 0C06FE00#0000000000008025   # 100% green
cansend can0 0C06FE00#000000000000FC21   #  75% green
cansend can0 0C06FE00#000000000000781E   #  50% green
cansend can0 0C06FE00#000000000000F41A   #  25% orange
cansend can0 0C06FE00#000000000000D818   #  10% red
cansend can0 0C06FE00#000000000000701760 #   0% empty

# DC Link LEDs
cansend can0 0C04FE00#000000000601       # pre-charge → LED orange
cansend can0 0C04FE00#000000000603       # DC Link ON → LED green
cansend can0 0C04FE00#000000000600       # OFF        → LEDs off

# Gear position
cansend can0 0C03FE32#00   # N
cansend can0 0C03FE32#02   # D
cansend can0 0C03FE32#03   # R
```

---

## ATTACHED REFERENCE DOCUMENTS

The following documents describe the full application architecture and API.
Read them before generating any code:

1. `hmi_manager_api_rev.pdf` — API Reference & Developer Guide
   (CAN message reference, widget API, button system, threading model,
    build instructions, memory map, known issues)

2. `hmi_manager_complete_project.pdf` — Complete Project Documentation
   (system overview, implemented modules, roadmap, multi-display architecture,
    sensor modules, RFID authentication, recommended implementation phases)

These documents are the authoritative source for:
- CAN message IDs, signal names, and DBC encoding
- Function signatures in `can_handler.h`, `widgets.h`, `fb_handler.h`
- Kconfig option names
- Build and flash commands
- Known issues and TODO items

---

## EXAMPLE REQUEST

"Generate `widgets.h`, `widgets.c`, and the `app_hmi_manager.c` patches for
the `cyberpunk` theme using the attached SVG file `lvgl_placeholder_cyberpunk.svg`
and background image `main_cyberpunk_lcd.png`."

The output must:
1. Extract all coordinates from the SVG using the conversion formula above
2. Generate a verification image showing all widget bounding boxes
3. Implement all mandatory widgets listed in this prompt
4. Follow all LVGL 9.2 requirements
5. Include the kW, kWh, and regeneration stubs
6. Pass the verification checklist
