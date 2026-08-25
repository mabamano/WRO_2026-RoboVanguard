#!/bin/bash
# ROBOVANGUARD WRO 2026 Autostart Installer & Diagnostic Script for Raspberry Pi 5

echo "========================================================="
echo "   ROBOVANGUARD WRO 2026 - Pi 5 Autostart Installer"
echo "========================================================="

# Automatically detect current script directory path, python path, and system user
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_EXEC="$(which python3)"
CURRENT_USER="$(whoami)"

echo "[INFO] Detected User:       $CURRENT_USER"
echo "[INFO] Working Directory:   $SCRIPT_DIR"
echo "[INFO] Python Executable:   $PYTHON_EXEC"

# Ensure user is in dialout group for USB serial (/dev/ttyUSB0) access
sudo usermod -a -G dialout $CURRENT_USER 2>/dev/null

SERVICE_FILE="/etc/systemd/system/wro_autostart.service"

# Create systemd service with dynamic path & headless environment flags
cat << EOF | sudo tee $SERVICE_FILE > /dev/null
[Unit]
Description=ROBOVANGUARD WRO 2026 Competition Autostart Service
After=multi-user.target serial-getty@ttyUSB0.service network.target

[Service]
Type=simple
User=$CURRENT_USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$PYTHON_EXEC $SCRIPT_DIR/competition_launcher.py --pin 17 --no-display
Restart=on-failure
RestartSec=3s
Environment=PYTHONUNBUFFERED=1
Environment=DISPLAY=:0

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd daemon & enable service
sudo systemctl daemon-reload
sudo systemctl enable wro_autostart.service

echo ""
echo "[SUCCESS] WRO Autostart Service successfully installed & enabled for user '$CURRENT_USER'!"
echo "========================================================="
echo "   DEBUGGING & LOG COMMANDS:"
echo "   1. Check Status:    sudo systemctl status wro_autostart.service"
echo "   2. View Live Logs:  sudo journalctl -u wro_autostart.service -f -n 50"
echo "   3. Test Start Now:  sudo systemctl start wro_autostart.service"
echo "   4. Stop Service:    sudo systemctl stop wro_autostart.service"
echo "========================================================="
