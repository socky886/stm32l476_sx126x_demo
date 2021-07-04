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

#define RF_FREQUENCY                                490000000 // Hz

#define TX_OUTPUT_POWER                             22        // dBm

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

#define FSK_FDEV                                    25000      // Hz 
#define FSK_DATARATE                                50000      // bps
#define FSK_BANDWIDTH                               50000     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         2          // 16 bits Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   true
#define FSK_FIX_PAYLOAD_LEN													20
#define FSK_SYNCWORD_LEN														2          // 3 bytes 
#define FSK_CRC_ON																	false

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
 * Main application entry point.
 */
int main( void )
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
