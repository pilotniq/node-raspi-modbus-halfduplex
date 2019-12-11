# raspi-modbus-halfduplex

Communicate using half duplex modbus on the Raspberry Pi.
The library has been used with a _MAX485 TTL to RS485 converter_.
Please see the "Background and Limitations" section for how this is implemented.
Only some modbus functions are implemented, other should be easy to add. Pull requests welcome.

## Installation

To build the native code, you need to install the libgpiod-dev package, which is used to switch the
GPIO to indicate the half-duplex direction:

```
sudo apt-get install libgpiod-dev
npm install raspi-modbus-halfduplex
```

## Usage

Connect your RS-485 to TTL converter board:
* R0 to RXD0 / pin 10 / GPIO 15 of the Raspberry Pi
* RE to DE pin (both on the RS-485 board)
* DE to a GPIO pin on the Raspberry Pi. I use pin 7 / GPIO 4, but any pin should work.
* DI to TXD0 / pin 8 / GPIO 14 of the Rasperry Pi
* VCC to 3.3 or 5V (pins 1 or 2 of your Raspberry Pi). According to the module specs, you should need 5V, but it works for me on 3.3V. In theory you might risk damage to your Raspberry Pi if you power it with 5V, but I did without damanging my Pi.
* A and B to the A and B of the modbus device(s) you want to talk to
* GND to ground of the Raspberry Pi (pins 6, 9, 14, 20, 25, 30, 34 or 39)

Then write your javascript code like:
```
const modbus = require( 'raspi-modbus-halfduplex' );

modbus.init( 4 )  // 4 is the GPIO number of the pin that switches direction

console.log( modbus.read_analog_register( 1, 0 ) );
```

## API
All functions that take registers use the data address and not the
logical address, so for instance 0 rather than 40001.

All functions may throw modbus errors. Error numbers are in the src/modbus.h file.

### modbus.init( directionPin )
Initialize the modbus code. Opens the serial port, and requests access to the GPIO pin.
Must be called before the other functions

### modbus.read_analog_register( device, register )
Read an analog register over modbus.
Returns the 16 bit value as an unsigned integer.
Device is the modbus address of the device you want to talk to (an 8 bit value).

### modbus.write_holding_register( device, register, value ) {
Writes to a holding register.
Device is the modbus address of the device you want to talk to (an 8 bit value).
Register is the 16 bit register (not logical address, see above).
Value is the 16 bit unsigned value to be written.

### modbus.write_discrete_coil( device, register, value )
Writes a bit (on/off) to a "discrete coil".
Device is the modbus address of the device you want to talk to (an 8 bit value).
Register is the 16 bit register (not logical address, see above).
Value is the boolean value to be written.

## Background and Limitations

Currently this library is hard coded to 9600 bps N81, but that can be changed in the modbus.c file, lines
```
#define BAUD_RATE 9600
#define BITS_PER_BYTE 10 // start bit, 8 data bits, stop bit
```

the function _set_interface_attribs_ and
```
set_interface_attribs (fd, B9600, 0);
```

Or even better, make it configurable, and submit a pull request.

Communicating in half duplex is tricky, because as soon as the last outgoing bit has been sent,
the signal to the RS485 chip to receive instead of transmit must be switched. But there is no
way to get the feedback when that bit has been sent. This native code uses a bit of a hack; it
sleeps for as long as it thinks it will take to send the bits, then toggles the direction signal.
It works for me, but there are no guarantees.
I occasionally get a CRC error, but then I retry and it normally works.
