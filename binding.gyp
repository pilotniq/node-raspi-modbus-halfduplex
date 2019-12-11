{
  "targets": [
      {
            "target_name": "module",
	    "sources": [ "./src/module.c", "./src/modbus.c" ],
	    'link_settings': {
	            'libraries': ['-lgpiod'],
		          },
      }
   ]
}
