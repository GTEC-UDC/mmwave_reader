#!/bin/zsh

# Check if a parameter is provided
if [ $# -eq 0 ]; then
  echo "Usage: $0 <file.launch>"
  exit 1
fi

# Save the parameter
parameter=$1

# Launch the first command
echo "Radar OFF.."
sudo ~/src/rpi_usb_power_switch/usb_switch.sh off &

# Wait for 4 seconds
sleep 2

# Launch the second command
echo "Radar ON.."
sudo ~/src/rpi_usb_power_switch/usb_switch.sh on &

# Wait for 4 seconds
sleep 2

# Launch the third command with the provided parameter as the user "ubuntu"
echo "Launching: $parameter"
sudo -u ubuntu -E zsh -c "source /home/ubuntu/.zshrc && roslaunch gtec_mmwave_reader \"$parameter\""
