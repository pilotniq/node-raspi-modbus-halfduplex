/*
 *  modbus.h
 */

#ifndef MODBUS_H
#define MODBUS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MODBUS_ERROR_NONE = 0,
  MODBUS_ERROR_RESPONSE_BAD_DEVICE = 12,
  MODBUS_ERROR_RESPONSE_SHORT = 13,
  MODBUS_ERROR_RESPONSE_BAD_CRC = 14,
  MODBUS_ERROR_RESPONSE_BAD = 15
} ModbusError;

void modbus_init( int dirPin );
ModbusError modbus_read_analog_register( uint8_t device, int reg, uint16_t *value );
ModbusError modbus_write_holding_register( uint8_t device, int holdingRegister, int value );
ModbusError modbus_write_discrete_coil( uint8_t device, int coil, bool value );

#endif
