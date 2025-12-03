
#!/usr/bin/env bash
set -euo pipefail

BUS=1
ADDR=0x68
PERIOD=0.2   # seconds between reads
INIT=1       # set to 0 to skip initialization

# Conversion constants
ACCEL_SCALE=16384    # ±2g
GYRO_SCALE=131       # ±250 dps

# Helpers
get8() { sudo i2cget -y "$BUS" "$ADDR" "$1"; }
set8() { sudo i2cset -y "$BUS" "$ADDR" "$1" "$2"; }

# Read signed 16-bit (big-endian: High then Low)
read_s16() {
    local high=$(( $(get8 "$1") ))
    local low=$(( $(get8 "$(( $1 + 1 ))") ))
    local val=$(( (high << 8) | low ))
    (( val >= 32768 )) && val=$((val - 65536))
    echo "$val"
}

init_registers() {
    echo "Configuring MPU6050..."
    set8 0x6B 0x80   # Reset
    sleep 0.1
    set8 0x1B 0x00   # Gyro ±250 dps
    set8 0x1C 0x00   # Accel ±2g
    set8 0x6B 0x01   # Clock source = X gyro
    echo "Init done."
}

# Main loop
if [[ "$INIT" -eq 1 ]]; then init_registers; fi
echo "timestamp | Ax[g] Ay[g] Az[g] | Gx[dps] Gy[dps] Gz[dps] | Temp[C] | Bytes"
while true; do
    ts=$(date +"%Y-%m-%dT%H:%M:%S.%3N")

    # Read 14 bytes starting at 0x3B
    bytes=()
    for i in $(seq 0 13); do
        bytes+=($(( $(get8 $((0x3B + i))) )))
    done

    # Convert to signed values
    ax=$(( ((bytes[0] << 8) | bytes[1]) ))
    (( ax >= 32768 )) && ax=$((ax - 65536))
    ay=$(( ((bytes[2] << 8) | bytes[3]) ))
    (( ay >= 32768 )) && ay=$((ay - 65536))
    az=$(( ((bytes[4] << 8) | bytes[5]) ))
    (( az >= 32768 )) && az=$((az - 65536))
    temp=$(( ((bytes[6] << 8) | bytes[7]) ))
    (( temp >= 32768 )) && temp=$((temp - 65536))
    gx=$(( ((bytes[8] << 8) | bytes[9]) ))
    (( gx >= 32768 )) && gx=$((gx - 65536))
    gy=$(( ((bytes[10] << 8) | bytes[11]) ))
    (( gy >= 32768 )) && gy=$((gy - 65536))
    gz=$(( ((bytes[12] << 8) | bytes[13]) ))
    (( gz >= 32768 )) && gz=$((gz - 65536))

    # Scale
    ax_g=$(awk "BEGIN{printf \"%.3f\", $ax/$ACCEL_SCALE}")
    ay_g=$(awk "BEGIN{printf \"%.3f\", $ay/$ACCEL_SCALE}")
    az_g=$(awk "BEGIN{printf \"%.3f\", $az/$ACCEL_SCALE}")
    gx_dps=$(awk "BEGIN{printf \"%.3f\", $gx/$GYRO_SCALE}")
    gy_dps=$(awk "BEGIN{printf \"%.3f\", $gy/$GYRO_SCALE}")
    gz_dps=$(awk "BEGIN{printf \"%.3f\", $gz/$GYRO_SCALE}")
    temp_c=$(awk "BEGIN{printf \"%.2f\", $temp/340+36.53}")

    # Print (with Bytes)
    echo "$ts | $ax_g $ay_g $az_g | $gx_dps $gy_dps $gz_dps | $temp_c | ${bytes[*]}"
	
	# Print (without Bytes)
	echo "$ts | $ax_g $ay_g $az_g | $gx_dps $gy_dps $gz_dps | $temp_c"
	
    sleep "$PERIOD"
done
