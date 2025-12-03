
#!/usr/bin/env bash
set -euo pipefail

# ===== User options =====
BUS=1
PERIOD=0.5          # seconds between prints
INIT=1            # set to 0 to skip register setup
LOGFILE=""        # set to a path to also append CSV (e.g., /home/pi/lsm9ds1.csv)

# --- Sensitivities (match register config below) ---
# Accel ±2g: 0.000061 g/LSB → m/s^2 scale:
AX_MPS2_PER_LSB="9.80665*0.000061"     # ≈ 0.000598206
# Gyro 245 dps: 0.00875 dps/LSB
GY_DPS_PER_LSB="0.00875"
# Also output rad/s (π/180 conversion)
DEG2RAD="3.141592653589793/180"
# Mag ±4 gauss: 0.00014 gauss/LSB → µT (1 gauss = 100 µT)
MG_UT_PER_LSB="0.00014*100"            # = 0.014 µT/LSB

# ===== Helpers =====
hex_eq() { [[ "$1" == "$2" ]]; }

get8() { sudo i2cget -y "$BUS" "$1" "$2"; }

set8() { sudo i2cset -y "$BUS" "$1" "$2" "$3"; }

# signed 16-bit little endian (read L then H)
read_s16() {
  local dev=$1 regL=$2
  local l=$(get8 "$dev" "$regL")  # e.g., 0x3f
  local h=$(get8 "$dev" $((regL+1)))
  local v=$(( ((h & 0xFF) << 8) | (l & 0xFF) ))
  (( v >= 32768 )) && v=$((v - 65536))
  echo "$v"
}

read_3axes() {
  local dev=$1 base=$2
  local x=$(read_s16 "$dev" "$base")
  local y=$(read_s16 "$dev" $((base+2)))
  local z=$(read_s16 "$dev" $((base+4)))
  echo "$x $y $z"
}

fcalc() { awk "BEGIN{printf \"%.6f\", $1}"; }   # 6 dp by default

# ===== Auto-detect addresses =====
detect_addresses() {
  local ag_candidates=(0x6b 0x6a)
  local mag_candidates=(0x1e 0x1c)

  AG=""
  for a in "${ag_candidates[@]}"; do
    local w=$(get8 "$a" 0x0f 2>/dev/null || true)
    if hex_eq "$w" "0x68"; then AG="$a"; break; fi
  done
  [[ -z "${AG:-}" ]] && { echo "ERROR: LSM9DS1 accel/gyro not found (WHO_AM_I != 0x68)"; exit 1; }

  MAG=""
  for m in "${mag_candidates[@]}"; do
    local w=$(get8 "$m" 0x0f 2>/dev/null || true)
    if hex_eq "$w" "0x3d"; then MAG="$m"; break; fi
  done
  [[ -z "${MAG:-}" ]] && { echo "ERROR: LSM9DS1 magnetometer not found (WHO_AM_I != 0x3d)"; exit 1; }

  echo "Using AG=$AG, MAG=$MAG"
}

# ===== Initialize sensor registers (typical safe defaults) =====
# NOTE: These match the sensitivities above.
init_registers() {
  echo "Configuring LSM9DS1 registers..."

  # --- GYRO (AG addr) ---
  # CTRL_REG1_G (0x10): ODR_G=119 Hz (0b011<<5), FS_G=245 dps (00), BW default
  set8 "$AG" 0x10 0x60

  # --- ACCEL (AG addr) ---
  # CTRL_REG6_XL (0x20): ODR_XL=10 Hz (0b001<<5), FS=±2 g (00)
  set8 "$AG" 0x20 0x20

  # Optional: enable BDU/IF_INC if desired (not required for byte reads)
  # CTRL_REG8 (0x22): IF_INC=1 (bit 2) enables auto-increment for multi-byte reads
  # set8 "$AG" 0x22 0x04

  # --- MAGNETOMETER (MAG addr) ---
  # CTRL_REG3_M (0x22): continuous-conversion mode (0x00)
  set8 "$MAG" 0x22 0x00
  # CTRL_REG1_M (0x20): ODR=10 Hz (0b100<<2), XY performance=00, tempcomp=0
  set8 "$MAG" 0x20 0x10
  # CTRL_REG2_M (0x21): FS=±4 gauss (00)
  set8 "$MAG" 0x21 0x00
  # CTRL_REG4_M (0x23): Z performance=00, little endian
  set8 "$MAG" 0x23 0x00

  echo "Init done."
}

main_loop() {
  #echo "timestamp, ax_mps2, ay_mps2, az_mps2, gx_dps, gy_dps, gz_dps, gx_rps, gy_rps, gz_rps, mx_uT, my_uT, mz_uT, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, mx_raw, my_raw, mz_raw"
  echo "timestamp, ax_mps2, ay_mps2, az_mps2, gx_dps, gy_dps, gz_dps,  mx_uT, my_uT, mz_uT"
  while true; do
    ts=$(date --iso-8601=seconds)

    # Raw reads
    read ax_raw ay_raw az_raw < <(read_3axes "$AG"  0x28)  # accel
    read gx_raw gy_raw gz_raw < <(read_3axes "$AG"  0x18)  # gyro
    read mx_raw my_raw mz_raw < <(read_3axes "$MAG" 0x28)  # mag

    # Unit conversions
    ax=$(fcalc "$ax_raw * ($AX_MPS2_PER_LSB)")
    ay=$(fcalc "$ay_raw * ($AX_MPS2_PER_LSB)")
    az=$(fcalc "$az_raw * ($AX_MPS2_PER_LSB)")

    gx_dps=$(fcalc "$gx_raw * ($GY_DPS_PER_LSB)")
    gy_dps=$(fcalc "$gy_raw * ($GY_DPS_PER_LSB)")
    gz_dps=$(fcalc "$gz_raw * ($GY_DPS_PER_LSB)")

    gx_rps=$(fcalc "$gx_dps * ($DEG2RAD)")
    gy_rps=$(fcalc "$gy_dps * ($DEG2RAD)")
    gz_rps=$(fcalc "$gz_dps * ($DEG2RAD)")

    mx=$(fcalc "$mx_raw * ($MG_UT_PER_LSB)")
    my=$(fcalc "$my_raw * ($MG_UT_PER_LSB)")
    mz=$(fcalc "$mz_raw * ($MG_UT_PER_LSB)")

    # line=$(printf "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %d, %d, %d, %d, %d, %d, %d, %d, %d\n" \
    #             "$ts" "$ax" "$ay" "$az" "$gx_dps" "$gy_dps" "$gz_dps" "$gx_rps" "$gy_rps" "$gz_rps" "$mx" "$my" "$mz" \
    #             "$ax_raw" "$ay_raw" "$az_raw" "$gx_raw" "$gy_raw" "$gz_raw" "$mx_raw" "$my_raw" "$mz_raw")

    line=$(printf "%s A: %s, %s, %s, G: %s, %s, %s, M: %s, %s, %s\n" \
                 "$ts" "$ax" "$ay" "$az" "$gx_dps" "$gy_dps" "$gz_dps" "$mx" "$my" "$mz")


    echo "$line"
    [[ -n "$LOGFILE" ]] && echo "$line" >> "$LOGFILE"

    sleep "$PERIOD"
  done
}

# ===== Run =====
detect_addresses
if [[ "$INIT" -eq 1 ]]; then init_registers; fi
main_loop
