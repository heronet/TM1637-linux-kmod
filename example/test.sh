#!/bin/bash

# Find the tm1637 device
DEVICE=$(find /sys/bus/platform/devices -name "*tm1637*" | head -n 1)

if [ -z "$DEVICE" ]; then
    echo "Error: TM1637 device not found"
    exit 1
fi

echo "Found device: $DEVICE"
echo

# Test brightness levels
echo "Testing brightness levels..."
for i in 0 1 2 3 4 5 6 7 8; do
    echo $i > "$DEVICE/brightness"
    echo "Brightness set to $i"
    sleep 0.5
done
echo

# Set to full brightness
echo 8 > "$DEVICE/brightness"
echo "Set to full brightness"
echo

# Test basic numbers
echo "Testing numbers 0-9..."
for i in 0 1 2 3 4 5 6 7 8 9; do
    echo $i > "$DEVICE/message"
    echo "Displayed: $i"
    sleep 0.5
done
echo

# Test with decimal points
echo "Testing decimal points..."
echo "12.34" > "$DEVICE/message"
echo "Displayed: 12.34"
sleep 1

echo "56.78" > "$DEVICE/message"
echo "Displayed: 56.78"
sleep 1
echo

# Test letters
echo "Testing letters..."
for msg in "ABCD" "EFGH" "HELP" "COOL"; do
    echo $msg > "$DEVICE/message"
    echo "Displayed: $msg"
    sleep 1
done
echo

# Test mixed content
echo "Testing mixed alphanumeric..."
echo "A1B2" > "$DEVICE/message"
echo "Displayed: A1B2"
sleep 1

echo "C3.D4" > "$DEVICE/message"
echo "Displayed: C3.D4"
sleep 1
echo

# Test turning off
echo "Turning off display..."
echo 0 > "$DEVICE/brightness"
echo "Display off (brightness=0)"
sleep 1

echo "Turning back on..."
echo 8 > "$DEVICE/brightness"
echo "Display on"
echo

# Read current state
echo "Current state:"
echo "  Message (hex): $(cat $DEVICE/message)"
echo "  Brightness: $(cat $DEVICE/brightness)"
echo

echo "Test complete!"