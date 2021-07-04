#ifndef __APP_SX126X_H__
#define __APP_SX126X_H__
void sx126x_init(void);
void sx126x_tx_packet(unsigned char *p,unsigned char len);
void sx126x_rx_packet(void);
void register_test(void);
void packet_rx(void);
unsigned int crc16(unsigned char *data, unsigned char length);

#endif