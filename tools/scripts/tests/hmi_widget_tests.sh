#!/bin/bash
# =============================================================================
# hmi_test_widgets.sh
# HMI Manager - Widget Visual Test Script
# Target: STM32H753ZI + ST7796 480x320 LCD
# CAN: 500kbps, 29-bit Extended IDs
#
# Usage:
#   ./hmi_test_widgets.sh [interface]
#   ./hmi_test_widgets.sh can0          (default)
#   ./hmi_test_widgets.sh can1
#
# Requirements:
#   - can-utils installed (cansend, candump)
#   - CAN interface already UP at 500kbps
#     e.g.: sudo ip link set can0 type can bitrate 500000
#           sudo ip link set can0 up
# =============================================================================

CAN=${1:-can0}
DELAY=0.6       # seconds between frames in sequences
DELAY_FAST=0.2  # seconds for sweep animations

# Colors for terminal output
RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[1;33m'
BLU='\033[0;34m'
CYN='\033[0;36m'
MAG='\033[0;35m'
WHT='\033[1;37m'
RST='\033[0m'

# =============================================================================
# Helpers
# =============================================================================

send() {
    local frame=$1
    local label=$2
    echo -e "  ${CYN}cansend ${CAN} ${frame}${RST}  ${WHT}${label}${RST}"
    cansend $CAN $frame
}

pause() {
    sleep ${1:-$DELAY}
}

header() {
    echo ""
    echo -e "${YEL}══════════════════════════════════════════════════════${RST}"
    echo -e "${YEL}  $1${RST}"
    echo -e "${YEL}══════════════════════════════════════════════════════${RST}"
}

subheader() {
    echo ""
    echo -e "${BLU}  ── $1 ──${RST}"
}

wait_key() {
    echo ""
    echo -e "${MAG}  [ENTER para continuar]${RST}"
    read -r
}

check_can() {
    if ! command -v cansend &>/dev/null; then
        echo -e "${RED}ERROR: can-utils not installed. Run: sudo apt install can-utils${RST}"
        exit 1
    fi
    if ! ip link show $CAN &>/dev/null; then
        echo -e "${RED}ERROR: interface $CAN not found${RST}"
        exit 1
    fi
    state=$(ip link show $CAN | grep -o 'state [A-Z]*' | awk '{print $2}')
    if [ "$state" != "UP" ]; then
        echo -e "${RED}ERROR: $CAN is $state — bring it up first:${RST}"
        echo -e "  sudo ip link set $CAN type can bitrate 500000"
        echo -e "  sudo ip link set $CAN up"
        exit 1
    fi
    echo -e "${GRN}  $CAN is UP — ready${RST}"
}

# =============================================================================
# Test functions
# =============================================================================

test_speedometer() {
    header "SPEEDOMETER + RPM NEEDLES (every 30°)"
    echo -e "  White needle = km/h  |  Amber needle = RPM (co-axial, shorter)"
    echo -e "  Sweep: 7h (0 km/h) → 12h (80 km/h) → 5h (160 km/h)"
    echo ""

    subheader "Step by step — 30° increments"
    send "0C0AFEFE#00000000"  "7h  →   0 km/h      0 RPM"  ; pause
    send "0C0AFEFE#00008605"  "8h  →  16 km/h   1415 RPM"  ; pause
    send "0C0AFEFE#00000D0B"  "9h  →  32 km/h   2829 RPM"  ; pause
    send "0C0AFEFE#00009410"  "10h →  48 km/h   4244 RPM"  ; pause
    send "0C0AFEFE#00001A16"  "11h →  64 km/h   5659 RPM"  ; pause
    send "0C0AFEFE#0000A11B"  "12h →  80 km/h   7074 RPM  ← MID SCALE"  ; pause
    send "0C0AFEFE#00002821"  "1h  →  96 km/h   8488 RPM"  ; pause
    send "0C0AFEFE#0000AE26"  "2h  → 112 km/h   9903 RPM"  ; pause
    send "0C0AFEFE#0000352C"  "3h  → 128 km/h  11318 RPM"  ; pause
    send "0C0AFEFE#0000BC31"  "4h  → 144 km/h  12732 RPM"  ; pause
    send "0C0AFEFE#00004337"  "5h  → 160 km/h  14147 RPM  ← FULL SCALE"  ; pause

    subheader "Sweep animation — up and down"
    echo "  Sweeping 0→160→0 km/h..."
    frames=(
        "00000000" "00008605" "00000D0B" "00009410" "00001A16"
        "0000A11B" "00002821" "0000AE26" "0000352C" "0000BC31" "00004337"
        "0000BC31" "0000352C" "0000AE26" "00002821" "0000A11B"
        "00001A16" "00009410" "00000D0B" "00008605" "00000000"
    )
    for f in "${frames[@]}"; do
        cansend $CAN 0C0AFEFE#$f
        sleep $DELAY_FAST
    done

    subheader "Back to zero"
    send "0C0AFEFE#00000000"  "→ 0 km/h (7h)"
}

test_battery() {
    header "BATTERY BAR (RED rect, bottom-left)"
    echo -e "  Color thresholds:"
    echo -e "    ${GRN}GREEN${RST}  = 27..100%"
    echo -e "    ${YEL}ORANGE${RST} = 11..26%"
    echo -e "    ${RED}RED${RST}    = 1..10%"
    echo -e "    EMPTY  = 0%"
    echo ""

    subheader "Color boundaries"
    send "0C06FE00#0000000000008025"  "100% → GREEN  (96.0V)"   ; pause
    send "0C06FE00#000000000000FC21"  " 75% → GREEN  (87.0V)"   ; pause
    send "0C06FE00#000000000000781E"  " 50% → GREEN  (78.0V)"   ; pause
    send "0C06FE00#000000000000181B"  " 26% → GREEN  (69.4V)  ← boundary"  ; pause
    send "0C06FE00#000000000000F41A"  " 25% → ORANGE (69.0V)  ← boundary"  ; pause
    send "0C06FE00#000000000000FC18"  " 11% → ORANGE (64.0V)  ← boundary"  ; pause
    send "0C06FE00#000000000000D818"  " 10% → RED    (63.6V)  ← boundary"  ; pause
    send "0C06FE00#000000000000701760" " 0% → EMPTY  (60.0V)"  ; pause

    subheader "Sweep animation — 100% down to 0%"
    echo "  Sweeping 100→0%..."
    bat_frames=(
        "8025" "FC21" "781E" "F41A" "FC18" "D818" "7017"
    )
    voltages=("96.0V=100%" "87.0V=75%" "78.0V=50%" "69.0V=25%" "64.0V=11%" "63.6V=10%" "60.0V=0%")
    for i in "${!bat_frames[@]}"; do
        f="${bat_frames[$i]}"
        cansend $CAN 0C06FE00#000000000000${f}
        echo -e "    ${voltages[$i]}"
        sleep $DELAY
    done

    subheader "Back to 100%"
    send "0C06FE00#0000000000008025"  "100% → GREEN"
}

test_dc_link() {
    header "DC LINK STATE + LEDs (InverterInfo)"
    echo -e "  LED Green  = DC Link ON"
    echo -e "  LED Orange = Pre-charging"
    echo -e "  Both off   = DC Link OFF / Discharging"
    echo ""

    send "0C04FE00#000000000600"  "DC Link OFF    → LEDs off"      ; pause 1.5
    send "0C04FE00#000000000601"  "Pre-charging   → LED ORANGE"    ; pause 1.5
    send "0C04FE00#000000000603"  "DC Link ON     → LED GREEN"     ; pause 1.5
    send "0C04FE00#000000000604"  "Discharging    → LEDs off"      ; pause 1.5
    send "0C04FE00#000000000600"  "DC Link OFF    → LEDs off"
}

test_gear() {
    header "GEAR POSITION (DriverCommands loopback)"
    echo -e "  Cyan rect inside hex frame"
    echo -e "  N = white  |  D = white  |  R = orange"
    echo -e "  Logic: drive_state=DISABLE → N (regardless of direction)"
    echo ""

    send "0C03FE32#00"  "state=Disable, dir=Forward  → N"  ; pause 1.5
    send "0C03FE32#02"  "state=Enable,  dir=Forward  → D"  ; pause 1.5
    send "0C03FE32#03"  "state=Enable,  dir=Backward → R"  ; pause 1.5
    send "0C03FE32#01"  "state=Disable, dir=Backward → N"  ; pause 1.5
    send "0C03FE32#00"  "state=Disable, dir=Forward  → N"
}

test_full_sequence() {
    header "FULL INTEGRATION SEQUENCE"
    echo "  Simulates a complete drive cycle"
    echo ""

    subheader "1. System boot — all at zero/neutral"
    send "0C0AFEFE#00000000"  "Speed=0  RPM=0"
    send "0C06FE00#0000000000008025"  "Battery=100%"
    send "0C03FE32#00"  "Gear=N"
    send "0C04FE00#000000000600"  "DC Link=OFF"
    pause 1.5

    subheader "2. DC Link pre-charge"
    send "0C04FE00#000000000601"  "Pre-charging → LED ORANGE"
    pause 2

    subheader "3. DC Link ON"
    send "0C04FE00#000000000603"  "DC Link ON → LED GREEN"
    pause 1.5

    subheader "4. Engage Drive (D)"
    send "0C03FE32#02"  "Gear=D"
    pause 1

    subheader "5. Accelerate 0→160 km/h"
    frames=(
        "00000000" "00008605" "00000D0B" "00009410" "00001A16"
        "0000A11B" "00002821" "0000AE26" "0000352C" "0000BC31" "00004337"
    )
    for f in "${frames[@]}"; do
        cansend $CAN 0C0AFEFE#$f
        sleep $DELAY_FAST
    done
    pause 1

    subheader "6. Cruise at 96 km/h"
    send "0C0AFEFE#00002821"  "96 km/h cruise"
    pause 2

    subheader "7. Decelerate to stop"
    frames=(
        "00002821" "0000A11B" "00001A16" "00009410" "00000D0B"
        "00008605" "00000000"
    )
    for f in "${frames[@]}"; do
        cansend $CAN 0C0AFEFE#$f
        sleep $DELAY_FAST
    done
    pause 1

    subheader "8. Neutral + DC Link OFF"
    send "0C03FE32#00"  "Gear=N"
    pause 0.5
    send "0C04FE00#000000000600"  "DC Link=OFF"
    pause 1

    subheader "9. Battery drain simulation"
    bat_frames=("8025" "FC21" "781E" "F41A" "FC18" "D818")
    for f in "${bat_frames[@]}"; do
        cansend $CAN 0C06FE00#000000000000${f}
        sleep 0.5
    done

    echo ""
    echo -e "  ${GRN}Full sequence complete!${RST}"
}

# =============================================================================
# Menu
# =============================================================================

show_menu() {
    echo ""
    echo -e "${WHT}╔══════════════════════════════════════════════╗${RST}"
    echo -e "${WHT}║     HMI Manager Widget Test — $CAN           ║${RST}"
    echo -e "${WHT}╠══════════════════════════════════════════════╣${RST}"
    echo -e "${WHT}║  1. Speedometer + RPM  (30° steps + sweep)   ║${RST}"
    echo -e "${WHT}║  2. Battery bar        (colors + sweep)      ║${RST}"
    echo -e "${WHT}║  3. DC Link + LEDs                           ║${RST}"
    echo -e "${WHT}║  4. Gear position      (N / D / R)           ║${RST}"
    echo -e "${WHT}║  5. Full drive cycle   (all widgets)         ║${RST}"
    echo -e "${WHT}║  6. Run ALL tests sequentially               ║${RST}"
    echo -e "${WHT}║  q. Quit                                     ║${RST}"
    echo -e "${WHT}╚══════════════════════════════════════════════╝${RST}"
    echo -ne "  Choice: "
}

# =============================================================================
# Main
# =============================================================================

echo ""
echo -e "${WHT}HMI Manager Widget Test Script${RST}"
echo -e "CAN interface: ${CYN}$CAN${RST}"
echo ""
check_can

while true; do
    show_menu
    read -r choice

    case $choice in
        1) test_speedometer   ; wait_key ;;
        2) test_battery       ; wait_key ;;
        3) test_dc_link       ; wait_key ;;
        4) test_gear          ; wait_key ;;
        5) test_full_sequence ; wait_key ;;
        6)
            test_speedometer   ; pause 1
            test_battery       ; pause 1
            test_dc_link       ; pause 1
            test_gear          ; pause 1
            test_full_sequence
            wait_key
            ;;
        q|Q)
            echo ""
            echo -e "${GRN}  Bye!${RST}"
            echo ""
            # Reset to zero/neutral on exit
            cansend $CAN 0C0AFEFE#00000000 2>/dev/null
            cansend $CAN 0C03FE32#00       2>/dev/null
            exit 0
            ;;
        *)
            echo -e "  ${RED}Invalid option${RST}"
            ;;
    esac
done
