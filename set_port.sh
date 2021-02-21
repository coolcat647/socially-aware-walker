rule_file=/etc/udev/rules.d/50-walker_usb_port.rules
if [ ! -f "$rule_file" ]; then
    echo "$rule_file does not exist, create a new file automatically."
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"2303\", ATTRS{manufacturer}==\"Prolific Technology Inc.\", ATTRS{version}==\" 2.00\", MODE=\"0777\", SYMLINK+=\"walker_motor_driver\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"6001\", ATTRS{manufacturer}==\"FTDI\", ATTRS{idVendor}==\"0403\", MODE=\"0666\", SYMLINK+=\"walker_force_sensor\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"0000\", ATTRS{manufacturer}==\"Hokuyo Data Flex for USB\", ATTRS{idVendor}==\"15d1\", MODE=\"0666\", SYMLINK+=\"walker_laserscan_sensor\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"ea60\", ATTRS{manufacturer}==\"Silicon Labs\", ATTRS{idVendor}==\"10c4\", MODE=\"0666\", SYMLINK+=\"walker_ydlidar_sensor\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"9d0f\", ATTRS{manufacturer}==\"SparkFun\", MODE=\"0777\", SYMLINK+=\"walker_imu_sensor\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
fi

sudo udevadm trigger
# sudo service udev restart

# Command to check your device:
# udevadm info -a -p  $(udevadm info -q path -n /dev/ttyACM0)
