#include "app_sx126x.h"
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Ping-Pong implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <string.h>
#include <stdio.h>
#include "board.h"
#include "radio.h"

#define USE_MODEM_FSK
// #define USE_TCXO


#define RF_FREQUENCY                               855800000// 868000000//490000000 // Hz
// #define RF_FREQUENCY                               433000000// 868000000//490000000 // Hz
// #define RF_FREQUENCY                               868000000// 868000000//490000000 // Hz

#define TX_OUTPUT_POWER                             14//22        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       10        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )


#define FSK_FDEV                                    20000      // Hz 
#define FSK_DATARATE                                2400      // bps
// #define FSK_FDEV                                    5000      // Hz 
// #define FSK_DATARATE                                20000      // bps
#define FSK_BANDWIDTH                               50000     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         2          // 16 bits Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   true
#define FSK_FIX_PAYLOAD_LEN													10
#define FSK_SYNCWORD_LEN														2          // 3 bytes 
#define FSK_CRC_ON																	false

// #define FSK_FDEV                                    5000      // Hz 
// #define FSK_DATARATE                                20000      // bps
// #define FSK_BANDWIDTH                               50000     // Hz >> DSB in sx126x
// #define FSK_AFC_BANDWIDTH                           83333     // Hz
// #define FSK_PREAMBLE_LENGTH                         8          // 16 bits Same for Tx and Rx
// #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
// #define FSK_FIX_PAYLOAD_LEN													20
// #define FSK_SYNCWORD_LEN														2          // 3 bytes 
// #define FSK_CRC_ON																	false


#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

typedef enum
{
	FSK_TEST_INIT,
	FSK_TEST_START,
	FSK_TSET_INPROCESS,
	FSK_TEST_DONE
}Fsk_test_state_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

uint8_t xorbuf[]={0xff,0xe1 ,0x1d ,0x9a ,0xed ,0x85 ,0x33 ,0x24 ,0xea ,0x7a ,0xd2 ,0x39 ,0x70 ,0x97 ,0x57 ,0x0a ,0x54 ,0x7d ,0x2d ,0xd8 ,0x6d ,0x0d ,0xba ,0x8f ,0x67 ,0x59 ,0xc7 ,0xa2 ,0xbf ,0x34 ,0xca ,0x18 ,0x30 ,0x53 ,0x93 ,0xdf ,0x92 ,0xec ,0xa7 ,0x15 ,0x8a ,0xdc ,0xf4 ,0x86 ,0x55 ,0x4e ,0x18 ,0x21 ,0x40 ,0xc4 ,0xc4 ,0xd5 ,0xc6 ,0x91 ,0x8a ,0xcd ,0xe7 };
States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

TimerEvent_t fsk_tst_timer;
static volatile Fsk_test_state_t fsk_state = FSK_TEST_INIT;
#define TST_PKT_NUM			1000
static volatile uint32_t pkt_send = 0;
static volatile uint32_t pkt_rx_num = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );


void GenTriggerPulse( void )
{
	GpioWrite(&Led2, 0);
	DelayMs(1);
	GpioWrite(&Led2, 1);
	DelayMs(1);
	GpioWrite(&Led2, 0);
#if 0
	if(pkt_send < TST_PKT_NUM)
	{
		TimerStart(&fsk_tst_timer);
		pkt_send++;
		fsk_state = FSK_TSET_INPROCESS;
		//printf("PKT_Send: %d\n",pkt_send);
	}
	else
		fsk_state = FSK_TEST_DONE;
#endif
}
/**
 * @brief sx126x init
 * 
 */
void sx126x_init(void)
{
bool isMaster = true;
    uint8_t i;

    // Target board initialization
    BoardInitMcu();
    BoardInitPeriph();

    //Init the timer for fsk test
    TimerInit(&fsk_tst_timer, GenTriggerPulse);
    TimerSetValue(&fsk_tst_timer, 20);

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);

#if defined(USE_MODEM_LORA)

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Radio.Rx(0);
#elif defined(USE_MODEM_FSK)

    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, 3000);

    // Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
    //                   0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
    //                   0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_FIX_PAYLOAD_LEN, FSK_CRC_ON,
    //                   0, 0, false, true);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_FIX_PAYLOAD_LEN, FSK_CRC_ON,
                      0, 0, false, true);
    Radio.RxBoosted(0);
    //fsk_state = FSK_TEST_START;
    //TimerStart(&fsk_tst_timer);
    GenTriggerPulse();

#elif defined(USE_MODEM_GFSK)

    SX126xSetStandby(STDBY_RC);

    ModulationParams_t modulationParams;
    modulationParams.PacketType = PACKET_TYPE_GFSK;
    modulationParams.Params.Gfsk.BitRate = 38400;
    modulationParams.Params.Gfsk.Bandwidth = RX_BW_117300;
    modulationParams.Params.Gfsk.Fdev = 50000;
    modulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_03;
    SX126xSetModulationParams(&modulationParams);

    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_GFSK;
    packetParams.Params.Gfsk.PayloadLength = 52;
    packetParams.Params.Gfsk.PreambleMinDetect = 10;
    packetParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
    packetParams.Params.Gfsk.SyncWordLength = 1;
    SX126xSetPacketParams(&packetParams);
    uint8_t testsyncwd = 0xC1;
    SX126xSetSyncWord(&testsyncwd /*SyncWords*/);
    Radio.Rx(0);

#else
#error "Please define a frequency band in the compiler options."
#endif
}
/**
 * @brief sx126x tx packet
 * 
 * @param p 
 * @param len 
 */
void sx126x_tx_packet(unsigned char *p,unsigned char len)
{
    uint16_t irqRegs;
    uint16_t crc;
    int i;
    // Buffer[0]=11^0xff;
    Buffer[0]=0x07;
    for ( i = 1; i <= 7; i++)
    {
        Buffer[i]=0x12+0x10*(i-1);
        printf("the buffer[%d] is 0x%02x\n",i,Buffer[i]);
    }
    crc=crc16(Buffer,8);
    Buffer[8]=crc>>8;
    Buffer[9]=crc&0xff;
    BufferSize=10;
    // for (i = 1; i <= 10; i++)
    // {
    //     // Buffer[i]='A'+i;
    //     //Buffer[i]=0x10+i;
    //     // Buffer[i]=Buffer[i]^xorbuf[i];

    // }

    // for ( i = 0; i < 7; i++)
    // {
    //     Buffer[i]=0x12+0x10*(i);
    //     // printf("the buffer[%d] is 0x%02x\n",i,Buffer[i]);
    // }
    
    // BufferSize=7;
    Radio.Standby();
    
    DelayMs(2);
    irqRegs=SX126xGetIrqStatus();
    SX126xClearIrqStatus(irqRegs);
    Radio.Send( Buffer, BufferSize );
    i=0;
    while (1)
    {
        irqRegs=SX126xGetIrqStatus();
        // printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_TX_DONE) ==IRQ_TX_DONE)
        {
            printf("transmit finished\n");
            GpioToggle(&Led1);
            break;
        }
        else
        {
            // printf("--------\n");
            i++;
            HAL_Delay(10);
            if(i>=200)
            {
                printf("transmit timeout\n");
                break;
            }
        }
    }


}
/**
 * @brief sx126x rx packet
 * 
 */
void sx126x_rx_packet(void)
{
    printf("#TST_Start\n");
    while (1)
    {
        //TimerLowPowerHandler( );
        Radio.IrqProcess();
    }
}
/**
 * @brief 
 * 
 */
void register_test(void)
{
    //write register
    char xx[20];
    int a;
    printf("register write and read test\n");
    SX126xWriteRegister(0x06C0, 0x12);
    SX126xWriteRegister(0x06C1, 0x34);
    SX126xWriteRegister(0x06C2, 0x56);
    SX126xWriteRegister(0x06C3, 0x78);

    a=SX126xReadRegister(0x06C0);
    sprintf(xx,"the sync 0 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C1);
    sprintf(xx,"the sync 1 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C2);
    sprintf(xx,"the sync 2 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C3);
    sprintf(xx,"the sync 3 is 0x%02x\n",a);
    printf(xx);
}

/**
 * @brief receive packet
 * 
 */
void packet_rx(void)
{
    int i;
    uint16_t irqRegs = SX126xGetIrqStatus();
    uint8_t rxbuf[255];
    
    uint8_t size=0;
    PacketStatus_t radiopkstatus;
    SX126xClearIrqStatus(irqRegs);
    // Radio.Rx(0);
    Radio.RxBoosted(0);
    i=0;
    while (1)
    {
        irqRegs = SX126xGetIrqStatus();

        // printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_RX_DONE)==IRQ_RX_DONE)
        {
            printf("receive packet successfully\n");
            GpioToggle(&Led2);

            SX126xGetPayload( rxbuf, &size , 255 );
            printf("the receive length is %d\n",size);
            // size=20;
            for (i = 0; i < size; i++)
            {
                // rxbuf[i]=rxbuf[i]^xorbuf[i];
                printf("0x%02x ,",rxbuf[i]);
                // printf("%c ",rxbuf[i]);
            }
            // printf("\nthe crc is :");
            // printf("0x%02x ,",rxbuf[11]);
            // printf("0x%02x ,",rxbuf[12]);
            printf("\n");
            SX126xGetPacketStatus(&radiopkstatus);
            printf("the rssi is %d\n", radiopkstatus.Params.Gfsk.RssiAvg);

            break;
        }
        else
        {
            i++;
            HAL_Delay(1);
            if(i>=100)
            {
                printf("receive timeout\n");
                break;
            }
        }
    }
    Radio.Standby();
    HAL_Delay(10);
}

/**
 * @brief crc16 for IBM
 * which can get the same result as si4463
 * the important factor is not reverse,start from the MSB
 * @param data
 * @param length 
 * @return unsigned int 
 */
unsigned int crc16(unsigned char *data, unsigned char length)
{
  int j;
  // unsigned int reg_crc = 0xFFFF;
  unsigned int reg_crc = 0x0000;
  // unsigned int reg_crc = 0x4230;
  while (length--)
  {

    reg_crc ^= (*data++<<8);
    for (j = 0; j < 8; j++)
    {
      if (reg_crc & 0x8000)
      {
        // reg_crc = (reg_crc >> 1) ^ 0xA001;
        reg_crc = (reg_crc << 1) ^ 0x8005;
      }
      else
      {
        reg_crc = reg_crc << 1;
      }
    }
  }
  reg_crc&=0xffff;
  return reg_crc;
}


/**
 * Main application entry point.
 */
int main1( void )
{
    bool isMaster = true;
    uint8_t i;

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );
	
	  //Init the timer for fsk test
	  TimerInit(&fsk_tst_timer,GenTriggerPulse);
		TimerSetValue(&fsk_tst_timer,20);

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
		Radio.Rx( 0 );
#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );
																														
	
    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_FIX_PAYLOAD_LEN, FSK_CRC_ON,
                                  0, 0,false, true );
		Radio.RxBoosted(0);
		//fsk_state = FSK_TEST_START;
		//TimerStart(&fsk_tst_timer);
    GenTriggerPulse();

#elif defined( USE_MODEM_GFSK )

		SX126xSetStandby(STDBY_RC);
	
		ModulationParams_t modulationParams;
		modulationParams.PacketType = PACKET_TYPE_GFSK;
		modulationParams.Params.Gfsk.BitRate = 38400;
		modulationParams.Params.Gfsk.Bandwidth = RX_BW_117300;
		modulationParams.Params.Gfsk.Fdev = 50000;
		modulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_03;
		SX126xSetModulationParams(&modulationParams);
		
		PacketParams_t packetParams;
		packetParams.PacketType = PACKET_TYPE_GFSK;
		packetParams.Params.Gfsk.PayloadLength = 52;
		packetParams.Params.Gfsk.PreambleMinDetect = 10;
		packetParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
		packetParams.Params.Gfsk.SyncWordLength = 1;
		SX126xSetPacketParams(&packetParams);
		uint8_t testsyncwd = 0xC1;
		SX126xSetSyncWord(&testsyncwd/*SyncWords*/);
		Radio.Rx(0);
	  
		
#else
    #error "Please define a frequency band in the compiler options."
#endif

		printf("#TST_Start\n");
		while(1)
		{
			//TimerLowPowerHandler( );
			Radio.IrqProcess( );
		}

#if 0
    while( 1 )
    {
        switch( State )
        {
        case RX:
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PONG
                        GpioToggle( &Led1 );

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        GpioToggle( &Led2 ); // Set LED off
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PING
                        GpioToggle( &Led1 );

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            GpioToggle( &Led2 );
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                DelayMs( 1 );
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        TimerLowPowerHandler( );
        // Process Radio IRQ
        Radio.IrqProcess( );

    }
#endif
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}
void DUMP(uint8_t* payload, uint16_t len)
{
	printf("payload recieved\n");
	uint16_t i = 0;
	for(i = 0; i< len; i++)
		printf("%02x ",*payload++);
	printf("\n");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
#if 0
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
#endif
		printf("RX done\r\n");
    DUMP(payload, size);
	  //pkt_rx_num++;
	  //printf("#PKT: %d, RSSI: %d\n",++pkt_rx_num,rssi);
		State = RX;
	  Radio.Rx(0);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
#if 0
    Radio.Sleep( );
    State = RX_TIMEOUT;
#endif
		 printf("Rx timeout\n");
	 Radio.Rx(0);
}

void OnRxError( void )
{
#if 0
    Radio.Sleep( );
    State = RX_ERROR;
#endif
	 printf("Rx error\n");
	 Radio.Rx(0);
}
