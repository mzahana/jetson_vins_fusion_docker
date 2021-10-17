
#!/bin/bash
# Copy the Realsense udev rules to /etc/udev/rules.d

sudo cp ${HOME}/src/jetson_vins_fusion_docker/udev/10-realsense.rules /etc/udev/rules.d

# Reread the rules; You may need to physically replug
sudo udevadm control --reload-rules 
sudo udevadm trigger
echo 'Realsense udev Rules installed'