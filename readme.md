# Links
https://phryniszak.github.io/stm32g-fdcan/
http://www.bittiming.can-wiki.info/
https://stackoverflow.com/questions/59802073/how-to-set-can-baud-rate-to-500k-for-stm32-controller


# build release
```code
DEFMT_LOG=info cargo run  --release --bin gs_can
```

# Set bitrate abd bring device up
```code
sudo ip link set can0 up type can fd on bitrate 500000 dbitrate 4000000 sample-point 0.875
```

# Send frame
```code
sudo cansend can0 0000A1B2#deadbeef
cansend can0 0000A222##1deadbeefffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
```

# Sniff CAN
```code
sudo cansniffer -c -f 1 can0
```

# Sniff ETH
```code
sudo tcpdump -i lan host 239.74.163.2 and port 43113
```

# Send ETH frame
```code
cat single_eth_frame.bin | ncat -vu "192.168.16.65" 43113
```

# Trace ETH 
```code
socat UDP4-RECVFROM:43113,ip-add-membership=239.74.163.2:0.0.0.0,fork - |xxd
socat STDIO UDP4-DATAGRAM:239.74.163.2:43114,bind=:43114,range=192.168.16.0/24,ip-add-membership=239.74.163.2:192.168.16.32
```

# Reset the chip
```code
probe-rs reset --chip STM32H723ZGTx
```


# PinMux
| Function |    Pin     |   Header  |
| -------- | ---------- | --------- |
| CAN 1 RX |	PD0     |	CN9 25  |
| CAN 1 TX |	PD1     |	CN9 27  |
| CAN 2 RX |	PB12    |	CN7 7   |
| CAN 2 TX |	PB6     |	CN10 14 |
| CAN 3 RX |	PF6     |	CN10 11 |
| CAN 3 TX |	PF7     |	CN9 26  |

