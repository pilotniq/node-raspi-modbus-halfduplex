const native_module = require('./build/Release/module');

module.exports = {
    init: function( directionPin ) {
	native_module.init( directionPin );
    },

    read_analog_register: function( device, register ) {
	return native_module.read_analog_register( device, register );
    },

    write_holding_register: function( device, register, value ) {
	if( value > 65535 || value < 0 )
	    throw "Invalid value " + value;
	return native_module.write_holding_register( device, register, value );
    },

    write_discrete_coil: function( device, register, value ) {
	return native_module.write_discrete_coil( device, register, value );
    }
}
