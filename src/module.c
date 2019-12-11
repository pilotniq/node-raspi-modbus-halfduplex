#include <stdio.h> // for sprintf
#include <node_api.h>

#include "modbus.h"

napi_value init(napi_env env, napi_callback_info info) {
  napi_status status;
  size_t argc = 1;
  int32_t directionPin = 0;
  napi_value argv[1];
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
  }

  status = napi_get_value_int32(env, argv[0], &directionPin);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as argument");
  }

  modbus_init( directionPin );

  return NULL;
}

napi_value read_analog_register( napi_env env, napi_callback_info info) {
  ModbusError merr;
  napi_status status;
  size_t argc = 2;
  napi_value argv[2];
  int32_t device, reg;
  uint16_t value;
  
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
  }

  status = napi_get_value_int32(env, argv[0], &device);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as device");
  }

  status = napi_get_value_int32(env, argv[1], &reg);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as register");
    return 0;
  }

  merr = modbus_read_analog_register( device, reg, &value );
  if( merr != MODBUS_ERROR_NONE )
  {
    char buffer[128];
    sprintf( buffer, "Modbus error %d", merr );
    
    napi_throw_error( env, NULL, buffer );
    return 0;
  }

  napi_value myNumber;

  status = napi_create_int32(env, value , &myNumber);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to create return value");
  }
  
  return myNumber;
}

napi_value write_holding_register( napi_env env, napi_callback_info info) {
  ModbusError merr;
  napi_status status;
  size_t argc = 3;
  napi_value argv[3];
  int32_t device, reg;
  int32_t value;
  
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
  }

  status = napi_get_value_int32(env, argv[0], &device);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as device");
  }

  status = napi_get_value_int32(env, argv[1], &reg);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as register");
    return 0;
  }

  status = napi_get_value_int32(env, argv[2], &value);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as register");
    return NULL;
  }
  
  merr = modbus_write_holding_register( device, reg, value );
  if( merr != MODBUS_ERROR_NONE )
  {
    char buffer[128];
    sprintf( buffer, "Modbus error %d", merr );
    
    napi_throw_error( env, NULL, buffer );
    return NULL;
  }

  return NULL;
}

napi_value write_discrete_coil( napi_env env, napi_callback_info info) {
  ModbusError merr;
  napi_status status;
  size_t argc = 3;
  napi_value argv[3];
  int32_t device, reg;
  bool value;
  
  status = napi_get_cb_info(env, info, &argc, argv, NULL, NULL);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Failed to parse arguments");
  }

  status = napi_get_value_int32(env, argv[0], &device);

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as device");
  }

  status = napi_get_value_int32(env, argv[1], &reg);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid number was passed as register");
    return 0;
  }

  status = napi_get_value_bool(env, argv[2], &value);
  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Invalid bool was passed as value");
    return NULL;
  }
  
  merr = modbus_write_discrete_coil( device, reg, value );
  if( merr != MODBUS_ERROR_NONE )
  {
    char buffer[128];
    sprintf( buffer, "Modbus error %d", merr );
    
    napi_throw_error( env, NULL, buffer );
    return NULL;
  }

  return NULL;
}

napi_value Init(napi_env env, napi_value exports) {
  napi_status status;
  napi_property_descriptor desc[] = {
    {"init", NULL, init, NULL, NULL, NULL, napi_default, NULL},
    {"read_analog_register", NULL, read_analog_register, NULL, NULL, NULL, napi_default, NULL},
    {"write_holding_register", NULL, write_holding_register, NULL, NULL, NULL, napi_default, NULL},
    {"write_discrete_coil", NULL, write_discrete_coil, NULL, NULL, NULL, napi_default, NULL}
  };

  status = napi_define_properties( env, exports, 4, desc );

  if (status != napi_ok) {
    napi_throw_error(env, NULL, "Unable to populate exports");
  }

  return exports;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)
