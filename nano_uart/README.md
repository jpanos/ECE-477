This script is an example listening server to handle flower data requests via UART:

For the pyserial library on the Jetson Nano, run:
$ sudo apt-get install python3-serial

To run this script:
$ sudo python3 uart_mod.py

If not working try:
$ sudo systemctl stop nvgetty
$ sudo systemctl disable nvgetty
