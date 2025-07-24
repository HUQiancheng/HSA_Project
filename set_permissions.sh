#!/bin/bash

echo "🔧 Setting up HSA Project permissions..."

# Detect if running in Docker container
if [ -f /.dockerenv ]; then
    echo "🐳 Detected Docker container environment"
    IN_DOCKER=true
else
    IN_DOCKER=false
fi

# Add current user to dialout group for serial port access
echo "➤ Adding user 'devel' to dialout group..."
sudo usermod -a -G dialout devel

# Set execute permissions for all Python scripts in the project
echo "➤ Setting execute permissions for Python scripts..."
find test_skin_ws/src -name "*.py" -exec chmod +x {} \;
find test_mcu_ws -name "*.py" -exec chmod +x {} \; 2>/dev/null || true

# Handle USB permissions differently for Docker vs native
if [ "$IN_DOCKER" = true ]; then
    echo "➤ Setting direct USB device permissions (Docker mode)..."
    # In Docker, directly set permissions on existing devices
    if [ -e /dev/ttyACM0 ]; then
        sudo chmod 666 /dev/ttyACM0
        echo "   • Set permissions for /dev/ttyACM0"
    fi
    if [ -e /dev/ttyUSB0 ]; then
        sudo chmod 666 /dev/ttyUSB0
        echo "   • Set permissions for /dev/ttyUSB0"
    fi
    echo "   • Note: In Docker, device permissions may reset on container restart"
else
    echo "➤ Setting up udev rules for persistent USB permissions..."
    # Only set up udev rules on native systems
    if [ -d /etc/udev/rules.d ]; then
        sudo tee /etc/udev/rules.d/99-hsa-project.rules > /dev/null << 'EOF'
# Arduino and similar USB serial devices
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"
# Generic USB serial adapters
KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
EOF

        # Reload udev rules if commands exist
        if command -v udevadm &> /dev/null; then
            echo "➤ Reloading udev rules..."
            sudo udevadm control --reload-rules
            sudo udevadm trigger
        fi
    else
        echo "   • Warning: /etc/udev/rules.d not found, skipping udev setup"
    fi
fi

echo ""
echo "✅ Permissions setup complete!"
echo ""
echo "📋 Summary:"
echo "   • User 'devel' added to dialout group"
echo "   • All Python scripts set to executable"
if [ "$IN_DOCKER" = true ]; then
    echo "   • USB device permissions set directly (Docker mode)"
    echo "   • Note: You may need to re-run this script if devices change"
else
    echo "   • USB serial devices configured with udev rules"
fi
echo ""
if [ "$IN_DOCKER" = false ]; then
    echo "⚠️  IMPORTANT: Please log out and log back in for group changes to take effect!"
    echo "    Or run: newgrp dialout"
else
    echo "🐳 Docker Note: Group changes should take effect immediately in container"
fi
echo ""
echo "🔍 Current USB device permissions:"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "   • No USB serial devices currently connected"
echo ""