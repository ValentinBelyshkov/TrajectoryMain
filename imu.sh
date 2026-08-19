#!/usr/bin/env bash
# accel_read.sh – simple accelerometer reader for MPU9250/MPU6500 on Jetson
# Requires: i2c-tools (i2cget) installed and user in the i2c group or run with sudo.

I2C_BUS=7
I2C_ADDR=0x68
# Accelerometer output registers (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
REG_X_H=0x3B
REG_X_L=0x3C
REG_Y_H=0x3D
REG_Y_L=0x3E
REG_Z_H=0x3F
REG_Z_L=0x40

# Sensitivity for ±2 g full‑scale (default after power‑up)
# 1 g = 16384 LSB
SENSITIVE=16384

# Function to read two bytes and combine to signed 16‑bit
read_raw() {
    local high low
    high=$(i2cget -y "$I2C_BUS" "$I2C_ADDR" "$1" b 2>/dev/null)
    low=$(i2cget -y "$I2C_BUS" "$I2C_ADDR" "$2" b 2>/dev/null)
    # Combine
    local val=$(( (high << 8) | low ))
    # Sign‑extend if needed
    if (( val & 0x8000 )); then
        val=$(( val - 0x10000 ))
    fi
    echo "$val"
}

# Main loop
while true; do
    # Read raw values
    local x_raw y_raw z_raw
    x_raw=$(read_raw "$REG_X_H" "$REG_X_L")
    y_raw=$(read_raw "$REG_Y_H" "$REG_Y_L")
    z_raw=$(read_raw "$REG_Z_H" "$REG_Z_L")

    # Convert to g
    local x_g y_g z_g
    x_g=$(awk "BEGIN {printf \"%.3f\", $x_raw/$SENSITIVE}")
    y_g=$(awk "BEGIN {printf \"%.3f\", $y_raw/$SENSITIVE}")
    z_g=$(awk "BEGIN {printf \"%.3f\", $z_raw/$SENSITIVE}")

    # Print timestamp + values
    printf "[%s] Accel: X=%s g, Y=%s g, Z=%s g\n" \
        "$(date '+%H:%M:%S')" "$x_g" "$y_g" "$z_g"

    # Adjust sampling rate as you like (e.g., 0.1 s ≈ 10 Hz)
    sleep 0.1
done
