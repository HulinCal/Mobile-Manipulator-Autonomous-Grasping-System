# 'SUBSYSTEM=="tty"  只匹配串口
echo  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0666", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_car_controller.rules


service udev reload
sleep 2
service udev restart

sleep 2
udevadm control --reload-rules
sleep 2
udevadm trigger
