## 2021/07/04
1. add crc16 based IBM which can response to si4463
2. add the whiten serial array
3. add makefile which can support arm-none-eabi-gcc
4. add the config to tx
   * datarate:2400
   * deviation:20000
   * carrier frequency:855,800,000
5. can send 10 bytes of fixed packet: 0x07,0x12,0x22,0x32,0x42...+crc16


