# Final Project COMP/ELEC 424/553

- Driver: 
    - device tree addition  (ok)
    - write driver to read encoder  (ok)
    - write file
    - manage speed in main loop 
    
- Stop Sign: insert logic to main loop https://github.com/fredotran/traffic-sign-detector-yolov4/blob/main/yolov4-traffic_road_signs-detection-images.ipynb

- Write hackster page



- TEST DRIVER's READING OF ENCODER (run car and check speed via kernel log)
Driver Compilation:
- sudo make (using Makefile)
- sudo modprobe -r gpiod_driver
- sudo mkdir /lib/modules/$(uname -r)/misc/
- sudo cp gpiod_driver.ko /lib/modules/$(uname -r)/misc/
- sudo depmod
- sudo modprobe gpiod_driver

Kernel log:
tail -f /var/log/kern.log
