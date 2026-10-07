# 'SUBSYSTEM=="tty"  只匹配串口
echo  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="5C98184648", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_laser"' >/etc/udev/rules.d/wheeltec_lslidar.rules



service udev reload
sleep 2
service udev restart

sleep 2
udevadm control --reload-rules
sleep 2
udevadm trigger
