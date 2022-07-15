#! /bin/bash
sudo cp 50-subt_port.rules /etc/udev/rules.d

#trigger
sudo udevadm trigger
sudo service udev restart

ls /dev/mmwav*