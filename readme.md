# Links
https://phryniszak.github.io/stm32g-fdcan/
http://www.bittiming.can-wiki.info/
https://stackoverflow.com/questions/59802073/how-to-set-can-baud-rate-to-500k-for-stm32-controller


# Set bitrate abd bring device up
sudo ip link set can0 up type can fd on bitrate 500000 dbitrate 4000000 sample-point 0.875

# Send frame
sudo cansend can0 0000A1B2##f1234567812345671121111

# Sniff
sudo cansniffer -c -f 1 can0

# PinMux

CAN 1 RX |	PD0     |	CN9 25
CAN 1 TX |	PD1     |	CN9 27
CAN 2 RX |	PB12    |	CN7 7
CAN 2 TX |	PB6     |	CN10 14
CAN 3 RX |	PF6     |	CN10 11
CAN 3 TX |	PF7     |	CN9 26

