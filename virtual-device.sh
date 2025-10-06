#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 [DEVICE_PATH]"
    echo "Creates a virtual serial device at the specified path"
    echo ""
    echo "Arguments:"
    echo "  DEVICE_PATH    Path to the virtual device (optional)"
    echo ""
    echo "Examples:"
    echo "  $0 /dev/ttyUSB0"
    echo "  $0 /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH06DKH3-if00-port0"
    echo ""
    echo "If no path is provided, you will be prompted to enter one."
}

# Check if help is requested
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    usage
    exit 0
fi

# Get device path from command line argument or prompt user
if [[ -n "$1" ]]; then
    DEVICE_PATH="$1"
else
    echo "Enter the path for the virtual device (e.g., /dev/ttyUSB0):"
    read -r DEVICE_PATH
    
    # Check if user provided input
    if [[ -z "$DEVICE_PATH" ]]; then
        echo "Error: No device path provided"
        echo ""
        usage
        exit 1
    fi
fi

# Validate the device path format
if [[ ! "$DEVICE_PATH" =~ ^/dev/ ]]; then
    echo "Warning: Device path should typically start with /dev/"
    echo "Continuing with provided path: $DEVICE_PATH"
fi

# Display information about what we're doing
echo "Creating virtual serial device at: $DEVICE_PATH"
echo "Use Ctrl+C to stop the virtual device"
echo "----------------------------------------"

# Create the virtual device using socat, suppress its output to avoid clutter
sudo socat PTY,link="$DEVICE_PATH",mode=666,raw,echo=0 STDIO