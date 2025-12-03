
#!/usr/bin/env bash
set -euo pipefail

BUS=1
ADDR=0x2A
PERIOD=0.5   # seconds between reads
INIT=1       # set to 0 to skip initialization

# Helpers
get8() { sudo i2cget -y "$BUS" "$ADDR" "$1"; }
set8() { sudo i2cset -y "$BUS" "$ADDR" "$1" "$2"; }

# Read signed 24-bit value and return bytes + int32
read_adc24() {
    local b1=$(get8 0x12)   # MSB
    local b2=$(get8 0x13)   # MID
    local b3=$(get8 0x14)   # LSB

    # Strip 0x prefix and convert to decimal
    b1=$((b1))
    b2=$((b2))
    b3=$((b3))

    # Combine into 24-bit
    local raw=$(( (b1 << 16) | (b2 << 8) | b3 ))

    # Sign extend to 32-bit
    (( raw >= 0x800000 )) && raw=$((raw - 0x1000000))

    # Compute padding byte for Simulink (sign extension)
    local pad=$(( (b1 & 0x80) ? 0xFF : 0x00 ))

    # Print bytes and int32
    echo "$raw $b1 $b2 $b3 $pad"
}

init_registers() {
    echo "Configuring NAU7802 registers..."
    set8 0x00 0x80   # Reset
    sleep 0.5
    set8 0x00 0x86   # Power up
    set8 0x01 0x27   # Amplifier config
    set8 0x02 0x30   # Calibration, rate, channel
    set8 0x15 0x30   # ADC config
    set8 0x1C 0x80   # ADC current
    echo "Init done."
}

# Main loop
if [[ "$INIT" -eq 1 ]]; then init_registers; fi
echo "Reading ADC values from NAU7802 at $ADDR..."
echo "timestamp | int32 | bytes: MSB MID LSB PAD"
while true; do
	ts=$(date +"%Y-%m-%dT%H:%M:%S.%3N")
    read val b1 b2 b3 pad < <(read_adc24)
    echo "$ts | $val | $b1 $b2 $b3 $pad"
    sleep "$PERIOD"
done
