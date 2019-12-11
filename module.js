const native_module = require('./build/Release/module');

module.exports = {
    init: init( directionPin ) {
	native_module.init( directionPin );
    }

    read_analog_register: read_analog_register( device, register ) {
	return native_module.read_analog_register( device, register );
    }

    write_holding_register: write_holding_register( device, register, value ) {
	return native_module.write_holding_register( device, register, value );
    }

    write_discrete_coil: write_discrete_coil( device, register, value ) {
	return native_module.write_discrete_coil( device, register, value );
    }
}
