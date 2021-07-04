2021/07/04

add crc16 based IBM which can response to si4463

add the whiten serial array

add makefile which can support arm-none-eabi-gcc

add the config to tx
  datarate:2400
  deviation:20000
  carrier frequency:855,800,000

can send 10 bytes of fixed packet: 0x07,0x12,0x22,0x32,0x42...+crc16


