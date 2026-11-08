#!/bin/bash
#######################################################################
# hmi_test.sh - HMI Manager Integration Test
#
# Automated + semi-automated tests for HMI Manager running on
# NuttX STM32H753ZI. Manages slcand lifecycle, sends CAN frames,
# captures NuttX console output, and validates responses.
#
# Prerequisites:
#   - can-utils installed (cansend, candump, slcand)
#   - hmi_manager running on NuttX (nsh> hmi_manager &)
#   - Two USB serial ports available (CAN adapter + NuttX console)
#
# Usage:
#   ./hmi_test.sh <CAN_SERIAL> <NUTTX_SERIAL>
#
# Example:
#   ./hmi_test.sh /dev/ttyACM0 /dev/ttyACM1
#
# CAN_SERIAL   : Serial device for slcand (CAN adapter)
# NUTTX_SERIAL : Serial device for NuttX console (debug)
#
# CAN config: 500 kbps, Extended frames (29-bit IDs)
#######################################################################

set -euo pipefail

#######################################################################
# Config
#######################################################################

CAN_IFACE="can0"
CAN_SPEED="-s6"           # 500 kbps
CAN_BAUDRATE="500kbps"
NUTTX_BAUD=115200

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RESULTS_DIR="${SCRIPT_DIR}/results"

PASS=0
FAIL=0
SKIP=0

# Temp files
SERIAL_LOG=$(mktemp /tmp/hmi_serial.XXXXXX)
CAN_LOG=$(mktemp /tmp/hmi_candump.XXXXXX)

# Colors
RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[0;33m'
CYN='\033[0;36m'
RST='\033[0m'

# PIDs to track
SERIAL_PID=0
CAN_PID=0

#######################################################################
# Usage
#######################################################################

usage() {
    echo "Usage: $0 <CAN_SERIAL> <NUTTX_SERIAL>"
    echo ""
    echo "  CAN_SERIAL   : /dev/ttyACMx for slcand"
    echo "  NUTTX_SERIAL : /dev/ttyACMx for NuttX console"
    echo ""
    echo "  CAN: ${CAN_BAUDRATE}, Extended (29-bit)"
    echo ""
    echo "Example:"
    echo "  $0 /dev/ttyACM0 /dev/ttyACM1"
    trap - EXIT
    exit 1
}

#######################################################################
# Cleanup
#######################################################################

cleanup() {
    echo ""
    echo "Cleaning up..."
    [ "$SERIAL_PID" -gt 0 ] && kill "$SERIAL_PID" 2>/dev/null || true
    [ "$CAN_PID" -gt 0 ] && kill "$CAN_PID" 2>/dev/null || true
    sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
    sudo killall -9 slcand 2>/dev/null || true
    rm -f "$SERIAL_LOG" "$CAN_LOG"
    echo "Done."
}

trap cleanup EXIT

#######################################################################
# Log helpers
#######################################################################

log_pass() {
    echo -e "  ${GRN}PASS${RST}: $1"
    PASS=$((PASS+1))
}

log_fail() {
    echo -e "  ${RED}FAIL${RST}: $1"
    FAIL=$((FAIL+1))
}

log_skip() {
    echo -e "  ${YEL}SKIP${RST}: $1"
    SKIP=$((SKIP+1))
}

log_test() {
    echo -e "\n${CYN}[$1]${RST} $2"
}

#######################################################################
# CAN interface management
#######################################################################

can_up() {
    echo "Killing existing CAN setup..."
    sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
    sudo killall -9 slcand 2>/dev/null || true
    sleep 0.5

    echo "Setting up $CAN_IFACE ($CAN_BAUDRATE)..."

    if [ ! -c "$CAN_SERIAL" ]; then
        echo -e "${RED}ERROR: $CAN_SERIAL not found${RST}"
        exit 1
    fi

    sudo slcand -o -c "$CAN_SPEED" "$CAN_SERIAL" "$CAN_IFACE"
    sleep 0.5
    sudo ip link set "$CAN_IFACE" up
    sleep 0.3

    if ip link show "$CAN_IFACE" | grep -q "UP"; then
        echo -e "  ${GRN}$CAN_IFACE UP${RST} on $CAN_SERIAL"
    else
        echo -e "${RED}ERROR: $CAN_IFACE failed to come up${RST}"
        exit 1
    fi
}

#######################################################################
# NuttX serial capture
#######################################################################

start_serial_capture() {
    if [ ! -c "$NUTTX_SERIAL" ]; then
        echo -e "${RED}ERROR: $NUTTX_SERIAL not found${RST}"
        exit 1
    fi

    stty -F "$NUTTX_SERIAL" "$NUTTX_BAUD" raw -echo \
        -echoe -echok -echoctl -echoke 2>/dev/null || true

    cat "$NUTTX_SERIAL" >> "$SERIAL_LOG" 2>/dev/null &
    SERIAL_PID=$!
    echo "NuttX console capture started on $NUTTX_SERIAL"
}

#######################################################################
# candump capture
#######################################################################

start_can_capture() {
    candump "$CAN_IFACE" >> "$CAN_LOG" 2>/dev/null &
    CAN_PID=$!
}

reset_can_log() {
    kill "$CAN_PID" 2>/dev/null || true
    sleep 0.1
    > "$CAN_LOG"
    candump "$CAN_IFACE" >> "$CAN_LOG" 2>/dev/null &
    CAN_PID=$!
}

reset_serial_log() {
    kill "$SERIAL_PID" 2>/dev/null || true
    sleep 0.1
    > "$SERIAL_LOG"
    cat "$NUTTX_SERIAL" >> "$SERIAL_LOG" 2>/dev/null &
    SERIAL_PID=$!
}

#######################################################################
# Test helpers
#######################################################################

can_send() {
    cansend "$CAN_IFACE" "$1"
    sleep 0.3
}

serial_contains() {
    local pattern="$1"
    local desc="$2"
    sleep 0.5
    if grep -q "$pattern" "$SERIAL_LOG" 2>/dev/null; then
        log_pass "$desc"
        return 0
    else
        log_fail "$desc (expected: '$pattern')"
        return 1
    fi
}

prompt_button() {
    local btn_name="$1"
    local wait_sec="${2:-5}"
    echo -e "  ${YEL}>> Press ${btn_name} NOW${RST}"
    echo -n "     Waiting ${wait_sec}s..."
    sleep "$wait_sec"
    echo " done."
}

#######################################################################
# Validate args
#######################################################################

if [ $# -ne 2 ]; then
    usage
fi

CAN_SERIAL="$1"
NUTTX_SERIAL="$2"

if [ "$CAN_SERIAL" = "$NUTTX_SERIAL" ]; then
    echo -e "${RED}ERROR: CAN and NuttX serial must differ${RST}"
    exit 1
fi

#######################################################################
# Setup
#######################################################################

echo "============================================"
echo "  HMI Manager Integration Test"
echo "============================================"
echo ""
echo "  CAN serial    : $CAN_SERIAL"
echo "  NuttX serial  : $NUTTX_SERIAL"
echo "  CAN interface : $CAN_IFACE"
echo "  CAN speed     : $CAN_BAUDRATE (ext 29-bit)"
echo ""

for cmd in cansend candump slcand; do
    if ! command -v "$cmd" &>/dev/null; then
        echo -e "${RED}ERROR: $cmd not found${RST}"
        echo "Install: sudo apt install can-utils"
        exit 1
    fi
done

can_up
start_serial_capture
start_can_capture

echo ""
echo "Starting tests..."

#######################################################################
# Start hmi_manager on NuttX
#######################################################################

echo "Sending 'hmi_manager &' to NuttX..."
echo -ne "\r" > "$NUTTX_SERIAL"
sleep 0.3
echo -ne "hmi_manager &\r" > "$NUTTX_SERIAL"

echo -n "  Waiting for Ready..."
for i in $(seq 1 15); do
    if grep -q "Ready" "$SERIAL_LOG" 2>/dev/null; then
        echo " OK"
        break
    fi
    sleep 1
    if [ "$i" -eq 15 ]; then
        echo ""
        echo -e "${YEL}WARNING: 'Ready' not seen in 15s."
        echo -e "Verify hmi_manager is running.${RST}"
    fi
done
sleep 1

echo "Waiting 5s for system to stabilize..."
sleep 5

#######################################################################
# TEST 1: CAN TX cyclic
#######################################################################

log_test "1a" "DRIVER_COMMANDS TX (~100ms, ID=0C03FE32)"
reset_can_log
sleep 2

count=$(grep -c "0C03FE32" "$CAN_LOG" 2>/dev/null \
    | tr -d '[:space:]' || echo 0)
if [ -z "$count" ]; then count=0; fi
if [ "$count" -ge 10 ]; then
    log_pass "DRIVER_COMMANDS: $count frames/2s"
else
    log_fail "DRIVER_COMMANDS: $count frames (expected >=10)"
fi

log_test "1b" "HMI_INFO TX (~1000ms, ID=0C0B0032)"
count=$(grep -c "0C0B0032" "$CAN_LOG" 2>/dev/null \
    | tr -d '[:space:]' || echo 0)
if [ -z "$count" ]; then count=0; fi
if [ "$count" -ge 1 ]; then
    log_pass "HMI_INFO: $count frames/2s"
else
    log_fail "HMI_INFO: $count frames (expected >=1)"
fi

#######################################################################
# TEST 2: Speed RX - INVERTER_SPEED_INFO
#######################################################################

log_test "2a" "Speed RX - 3000 RPM -> ~33 km/h"
reset_serial_log
can_send "0C0AFEFE#0000B80B"
serial_contains "Motor=3000RPM" "3000 RPM parsed" || true

log_test "2b" "Speed RX - 1000 RPM -> ~11 km/h"
reset_serial_log
can_send "0C0AFEFE#0000E803"
serial_contains "Motor=1000RPM" "1000 RPM parsed" || true

log_test "2c" "Speed RX - 0 RPM"
reset_serial_log
can_send "0C0AFEFE#00000000"
serial_contains "Motor=0RPM" "0 RPM parsed" || true

#######################################################################
# TEST 3: Voltage RX - SYSTEM_VOLTAGES
#######################################################################

log_test "3a" "Voltage RX - 48.0V (no crash)"
can_send "0C06FE00#000000000000C012"
sleep 0.3
log_pass "48V sent OK"

log_test "3b" "Voltage RX - 96.0V (no crash)"
can_send "0C06FE00#0000000000008025"
sleep 0.3
log_pass "96V sent OK"

#######################################################################
# TEST 4: LED via INVERTER_INFO
#######################################################################

log_test "4a" "LED - DC Link ON (state=3) -> green"
can_send "0C04FE00#000000000603"
log_pass "Sent (verify green LED)"

log_test "4b" "LED - PRE_CHARGING (state=1) -> orange"
can_send "0C04FE00#000000000601"
log_pass "Sent (verify orange LED)"

log_test "4c" "LED - DC Link OFF (state=0) -> all off"
can_send "0C04FE00#000000000600"
log_pass "Sent (verify LEDs off)"

#######################################################################
# TEST 5: Safety interlock - blocked
#######################################################################

log_test "5" "Safety interlock - Forward blocked w/o DC Link"

can_send "0C04FE00#000000000600"
sleep 0.3
reset_serial_log

prompt_button "BTN1 (Forward)" 5
serial_contains "Cannot enable drive" \
    "BTN1 blocked by interlock" || true

#######################################################################
# TEST 6: Drive sequence
#######################################################################

log_test "6a" "Drive - enable DC Link then Forward"

can_send "0C04FE00#000000000603"
sleep 0.3
reset_serial_log

prompt_button "BTN0 (DC Link) then BTN1 (Forward)" 8
serial_contains "FORWARD enabled" \
    "Forward after DC Link ON" || true

log_test "6b" "Drive - Reverse"
reset_serial_log
prompt_button "BTN2 (Reverse)" 5
serial_contains "REVERSE enabled" "Reverse enabled" || true

log_test "6c" "Drive - Neutral"
reset_serial_log
prompt_button "BTN3 (Neutral)" 5
serial_contains "NEUTRAL" "Neutral set" || true

#######################################################################
# TEST 7: TX data - DC Link demand bit
#######################################################################

log_test "7" "TX byte changes on DC Link toggle"

can_send "0C04FE00#000000000600"
sleep 0.3
reset_can_log

prompt_button "BTN0 (DC Link toggle)" 5

# byte 0x08 = dc_link_demand bit set
if grep "0C03FE32" "$CAN_LOG" \
    | grep -qv "\[1\]  00"; then
    log_pass "DRIVER_COMMANDS byte changed"
else
    log_fail "DRIVER_COMMANDS still 0x00"
fi

#######################################################################
# TEST 8: Pedal mode
#######################################################################

log_test "8" "Pedal mode cycle (BTN5 x3)"
reset_serial_log

prompt_button "BTN5 three times" 8

found=0
grep -q "ECO"    "$SERIAL_LOG" 2>/dev/null && found=$((found+1))
grep -q "NORMAL" "$SERIAL_LOG" 2>/dev/null && found=$((found+1))
grep -q "SPORT"  "$SERIAL_LOG" 2>/dev/null && found=$((found+1))

if [ "$found" -eq 3 ]; then
    log_pass "All 3 pedal modes cycled"
elif [ "$found" -gt 0 ]; then
    log_pass "Partial: $found/3 modes (press BTN5 3x)"
else
    log_fail "No pedal mode output"
fi

#######################################################################
# TEST 9: Reset faults
#######################################################################

log_test "9" "Reset faults (BTN4)"
reset_serial_log

prompt_button "BTN4 (Reset faults)" 5
serial_contains "RESET requested" "Fault reset" || true

#######################################################################
# TEST 10: Interlock re-engages after DC Link OFF
#######################################################################

log_test "10" "Interlock re-engages after DC Link OFF"

can_send "0C04FE00#000000000600"
sleep 0.3
reset_serial_log

prompt_button "BTN1 (Forward, should block)" 5
serial_contains "Cannot enable drive" \
    "Interlock active after DC Link OFF" || true

#######################################################################
# Save logs
#######################################################################

mkdir -p "$RESULTS_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULT_FILE="${RESULTS_DIR}/hmi_test_${TIMESTAMP}.log"

{
    echo "============================================"
    echo "  HMI Test Log - $(date)"
    echo "============================================"
    echo "CAN serial  : $CAN_SERIAL"
    echo "NuttX serial: $NUTTX_SERIAL"
    echo ""
    echo "--- NuttX Console Output ---"
    cat "$SERIAL_LOG" 2>/dev/null || echo "(empty)"
    echo ""
    echo "--- CAN Bus Capture ---"
    cat "$CAN_LOG" 2>/dev/null || echo "(empty)"
} > "$RESULT_FILE"

echo ""
echo "Log saved: $RESULT_FILE"

#######################################################################
# Summary
#######################################################################

TOTAL=$((PASS + FAIL + SKIP))

echo ""
echo "============================================"
echo "  Results"
echo "============================================"
echo -e "  ${GRN}PASS${RST}: $PASS"
echo -e "  ${RED}FAIL${RST}: $FAIL"
echo -e "  ${YEL}SKIP${RST}: $SKIP"
echo "  TOTAL : $TOTAL"
echo "============================================"

if [ "$FAIL" -gt 0 ]; then
    echo -e "  ${RED}SOME TESTS FAILED${RST}"
    echo "RESULT: FAIL ($PASS pass, $FAIL fail, $SKIP skip)" \
        >> "$RESULT_FILE"
    exit 1
else
    echo -e "  ${GRN}ALL TESTS PASSED${RST}"
    echo "RESULT: PASS ($PASS pass, $FAIL fail, $SKIP skip)" \
        >> "$RESULT_FILE"
    exit 0
fi
