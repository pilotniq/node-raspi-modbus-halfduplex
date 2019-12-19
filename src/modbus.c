/*
 * modbus.c
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "modbus.h"

/* 
 * types
 */

#define BAUD_RATE 9600
#define BITS_PER_BYTE 10 // start bit, 8 data bits, stop bit

typedef enum {
  MODBUS_STATE_IDLE,
  MODBUS_STATE_DEVICE_ADDRESS,
  MODBUS_STATE_FUNCTION
} ModbusState;


typedef enum {
  MBUS_FUNC_READ_COILS = 1,
  MBUS_FUNC_READ_DISCRETE = 2,
  MBUS_FUNC_READ_REGS = 3,
  MBUS_FUNC_READ_INPUT_REGS = 4,
  MBUS_FUNC_WRITE_COIL = 5,
  MBUS_FUNC_WRITE_REG = 6,
  MBUS_FUNC_READ_EXCEPT_STATUS = 7,
  MBUS_FUNC_DIAGNOSTICS = 8,
  MBUS_FUNC_GET_COMM_EVENT_COUNTER = 11,
  MBUS_FUNC_GET_COMM_EVENT_LOG = 12,
  MBUS_FUNC_WRITE_COILS = 15,
  MBUS_FUNC_WRITE_REGS = 16,
  MBUS_FUNC_READ_SLAVE_ID = 17,
  MBUS_FUNC_READ_FILE_RECORD = 20,
  MBUS_FUNC_WRITE_FILE_RECORD = 21,
  MBUS_FUNC_READ_WRITE_MASK_REGS = 22,
  MBUS_FUNC_READ_WRITE_REGS = 23,
  MBUS_FUNC_READ_FIFO_QUEUE = 24,
  MBUS_FUNC_READ_DEVICE_ID = 43,
  MBUS_FUNC_EXCEPTION = 0x81,
} Modbus_ConnectFuncType;

/*
  static variables
*/

static const uint8_t aucCRCHi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
  0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
  0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
  0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
  0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
  0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
  0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
  0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
  0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
  0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
  0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
  0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
  0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
  0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
  0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
  0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
  0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
  0x41, 0x81, 0x80, 0x40
};

static int serial_fd;
static struct gpiod_chip *gpio_chip;
static struct gpiod_line *gpio_line;

/*
 * static function prototypes
 */
static void modbus_send( const uint8_t *buffer, int dataLength );
static int modbus_read_reply( uint8_t *responseBuffer, int responseBufferLength );
static uint16_t calc_crc( const uint8_t *buffer, int dataLength );
static bool check_crc( const uint8_t *buffer, int dataLength );
static uint16_t mbus_crc16(const uint16_t crc16, const uint8_t byte);

static int serial_init( const char *portDeviceName );
static void gpio_init(int pinNr);
static void gpio_start_transmit(void);
static void gpio_stop_transmit(void);
static int set_interface_attribs (int fd, int speed, int parity);
static void set_blocking (int fd, int should_block);

/*
 * start of code
 */
static int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  if (tcgetattr (fd, &tty) != 0)
    {
      assert( false );
      // error_message ("error %d from tcgetattr", errno);
      return -1;
    }
  
  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);
  
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  
  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
      assert( false );
      // error_message ("error %d from tcsetattr", errno);
      return -1;
    }
  return 0;
}

static void set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    {
      assert( false );
      // error_message ("error %d from tggetattr", errno);
      return;
    }
  
  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  
  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    assert( false );
  // error_message ("error %d setting term attributes", errno);
}

// gpio
static void gpio_init( int dirPin )
{
  int err;
  
  gpio_chip = gpiod_chip_open_lookup( "gpiochip0" );
  assert( gpio_chip != NULL );
  
  gpio_line = gpiod_chip_get_line( gpio_chip, dirPin );
  assert( gpio_line != NULL );

  err = gpiod_line_request_output( gpio_line, "modbus", 0 );
  assert( err == 0 );

  err = gpiod_line_set_value( gpio_line, 0 ); // 0 = listening
  assert( err == 0 );
}

static void gpio_start_transmit()
{
  int err;
  
  err = gpiod_line_set_value( gpio_line, 1 ); // 0 = listening
  assert( err == 0 );
}
static void gpio_stop_transmit()
{
  int err;
  
  err = gpiod_line_set_value( gpio_line, 0 ); // 0 = listening
  assert( err == 0 );
}

void modbus_init( int dirPin )
{
  gpio_init( dirPin );

  serial_fd = serial_init( "/dev/serial0" );
}

ModbusError modbus_write_holding_register( uint8_t device, int holdingRegister, uint16_t value )
{
  uint8_t sendBuffer[8];
  uint8_t responseBuffer[8];
  int responseBufferCount;
  uint8_t responseFunction;

  sendBuffer[0] = device;
  sendBuffer[1] = MBUS_FUNC_WRITE_REG;
  sendBuffer[2] = (holdingRegister >> 8) & 0xff;
  sendBuffer[3] = (holdingRegister & 0xff);
  sendBuffer[4] = (value >> 8) & 0xff;
  sendBuffer[5] = value & 0xff;

  modbus_send( sendBuffer, 6 );

  responseBufferCount = modbus_read_reply( responseBuffer, 8 );

  // verify CRC of response
  if( responseBufferCount < 5 ) // an exception response is 5 bytes long
    return MODBUS_ERROR_RESPONSE_SHORT;
  
  if( responseBuffer[0] != device )
    return MODBUS_ERROR_RESPONSE_BAD_DEVICE;

  responseFunction = responseBuffer[1];
  if( responseFunction == (MBUS_FUNC_WRITE_REG | 0x80) )
  {
    uint8_t exceptionCode;
    
    // error response
    exceptionCode = responseBuffer[2];

    if( !check_crc( responseBuffer, 3 ) )
      return exceptionCode;
    else
      return MODBUS_ERROR_RESPONSE_BAD_CRC;
  }

  // it's not an exception response
  if( responseBufferCount < 5 ) // an exception response is 5 bytes long
    return MODBUS_ERROR_RESPONSE_SHORT;

  // check the crc
  if( !check_crc( responseBuffer, 6 ) )
    return MODBUS_ERROR_RESPONSE_BAD_CRC;
  
  if( memcmp( sendBuffer, responseBuffer, 6 ) == 0 )
    return MODBUS_ERROR_NONE;
  else
    return MODBUS_ERROR_RESPONSE_BAD;
}

ModbusError modbus_write_discrete_coil( uint8_t device, int coil, bool value )
{
  uint8_t sendBuffer[6];
  uint8_t responseBuffer[8];
  int responseBufferCount;
  uint8_t responseFunction;
  
  sendBuffer[0] = device;
  sendBuffer[1] = MBUS_FUNC_WRITE_COIL;
  sendBuffer[2] = (coil >> 8) & 0xff;
  sendBuffer[3] = (coil & 0xff);
  sendBuffer[4] = value ? 0xff : 0x00;
  sendBuffer[5] = 0x00;

  modbus_send( sendBuffer, 6 );

  responseBufferCount = modbus_read_reply( responseBuffer, 8 );

  // verify CRC of response
  if( responseBufferCount < 5 ) // an exception response is 5 bytes long
    return MODBUS_ERROR_RESPONSE_SHORT;

  if( !check_crc( responseBuffer, responseBufferCount - 2 ) )
    return MODBUS_ERROR_RESPONSE_BAD_CRC;
  
  if( responseBuffer[0] != device )
    return MODBUS_ERROR_RESPONSE_BAD_DEVICE;

  responseFunction = responseBuffer[1];
  if( responseFunction == (MBUS_FUNC_WRITE_REG | 0x80) )
  {
    uint8_t exceptionCode;
    
    // error response
    exceptionCode = responseBuffer[2];

    return exceptionCode;
  }

  // it's not an exception response
  if( responseBufferCount < 5 ) // an exception response is 5 bytes long
    return MODBUS_ERROR_RESPONSE_SHORT;

  if( memcmp( sendBuffer, responseBuffer, 6 ) == 0 )
    return MODBUS_ERROR_NONE;
  else
    return MODBUS_ERROR_RESPONSE_BAD;
}

ModbusError modbus_read_analog_register( uint8_t device, int reg, uint16_t *value )
{
  uint8_t sendBuffer[6];
  uint8_t responseBuffer[8];
  int responseBufferCount;
  uint8_t responseFunction;
  
  sendBuffer[0] = device;
  sendBuffer[1] = MBUS_FUNC_READ_INPUT_REGS;
  sendBuffer[2] = (reg >> 8) & 0xff;
  sendBuffer[3] = (reg & 0xff);
  sendBuffer[4] = 0; // number of registers requested, high byte
  sendBuffer[5] = 1; // number of registers requested, low byte

  modbus_send( sendBuffer, 6 );

  responseBufferCount = modbus_read_reply( responseBuffer, 7 );

  printf( "Response: [" );
  
  for( int i = 0; i < responseBufferCount; i++ )
  {
    printf( " %02x", responseBuffer[i] );
  }
  printf( " ]\n" );
  
  // verify CRC of response
  if( responseBufferCount < 5 ) // an exception response is 5 bytes long
    return MODBUS_ERROR_RESPONSE_SHORT;
  
  if( !check_crc( responseBuffer, responseBufferCount - 2 ) )
      return MODBUS_ERROR_RESPONSE_BAD_CRC;
      
  if( responseBuffer[0] != device )
    return MODBUS_ERROR_RESPONSE_BAD_DEVICE;

  responseFunction = responseBuffer[1];
  if( responseFunction == (MBUS_FUNC_READ_INPUT_REGS | 0x80) )
  {
    uint8_t exceptionCode;
    
    // error response
    exceptionCode = responseBuffer[2];

    return exceptionCode;
  }

  // it's not an exception response
  if( responseBufferCount < 7 ) // an proper response is 7 bytes long
    return MODBUS_ERROR_RESPONSE_SHORT;

  if( responseBuffer[2] != 2 ) // number of bytes returned
    return MODBUS_ERROR_RESPONSE_BAD;

  *value = responseBuffer[3] << 8 | responseBuffer[4];

  return MODBUS_ERROR_NONE;
}

static uint16_t calc_crc( const uint8_t *buffer, int dataLength )
{
  uint16_t crc = 0xffff;
  int i;
  for( i = 0; i < dataLength; i++ )
  {
    // crc = crc ^ buffer[i];
    
    crc = mbus_crc16(crc, buffer[i] );
  }

  return crc;
}
static bool check_crc( const uint8_t *buffer, int dataLength )
{
  uint16_t crc;
  bool match;
  
  crc = calc_crc( buffer, dataLength );
  match = (buffer[ dataLength ] == (crc & 0xff)) &&
    ((buffer[dataLength + 1] & 0xf8)) == ((crc >> 8) & 0xf8);

  if( match && (buffer[dataLength+1] != ((crc >> 8) & 0xff)))
    printf( "WARNING: Ignored CRC last three bit mismatch: got %02x expected %02x\n",
	    buffer[ dataLength + 1 ], (crc >> 8) & 0xff );
  else if( !match )
  {
    printf( "buffer [" );
    for( int i = 0; i < dataLength + 2; i++ )
      printf( " %02x", buffer[i] );

    printf( " ]: got CRC %02x %02x, expected %02x %02x\n",
	    buffer[ dataLength ], buffer[ dataLength + 1 ],
	    crc & 0xff, crc >> 8 );
  }

  return match;
}
 
static int modbus_read_reply( uint8_t *responseBuffer, int responseBufferLength )
{
  int count, n;
  
  usleep( 1e6 * 3.5 * BITS_PER_BYTE / BAUD_RATE ); // should receive a reply within this time

  for( count = 0; count < responseBufferLength; )
  {
    n = read( serial_fd, &(responseBuffer[count]), (responseBufferLength - count) );
    if( n == 0 )
      return count;

    count += n;
    usleep( 1e6 * 2 * BITS_PER_BYTE / BAUD_RATE );
  }

  return count;
}

static void modbus_send( const uint8_t *buffer, int dataLength )
{
  uint8_t crcBuf[2];
  uint16_t crc; //  = 0xffff;
  
  printf( "modbus_send: buffer=[ " );
  for( int i = 0; i < dataLength; i++ )
  {
    printf( "%02x ", buffer[i] );
    // crc = mbus_crc16(crc, buffer[i] );
  }
  
  crc = calc_crc( buffer, dataLength );
  printf( "] crc: %x\n", crc );

  crcBuf[0] = crc & 0xff;
  crcBuf[1] = (crc >> 8) & 0xff;
  
  gpio_start_transmit();
  
  write( serial_fd, buffer, dataLength );
  write( serial_fd, crcBuf, 2 );

  usleep( 1000000 * (dataLength + 2) * BITS_PER_BYTE / BAUD_RATE );

  gpio_stop_transmit();
}


/*
USHORT
usMBCRC16( UCHAR * pucFrame, USHORT usLen )
{
    UCHAR           ucCRCHi = 0xFF;
    UCHAR           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( UCHAR )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( USHORT )( ucCRCHi << 8 | ucCRCLo );
}

*/

static uint16_t   mbus_crc16(const uint16_t crc16, const uint8_t byte){
  const int index = (crc16&0xFF) ^ byte;
  return (aucCRCLo[index] << 8) | ((crc16 >> 8) ^ aucCRCHi[index]);
}

static int serial_init( const char *portname )
{
  int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
    {
      fprintf( stderr, "error %d opening %s: %s", errno, portname, strerror (errno));
      return -1;
    }
  
  set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  // set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (fd, 0);                // set no blocking

  return fd;
}
