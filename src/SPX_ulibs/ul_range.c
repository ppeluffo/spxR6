/*
 * spx_range.c
 *
 *  Created on: 8 mar. 2019
 *      Author: pablo
 */


#include "spx.h"

//------------------------------------------------------------------------------------
void range_init()
{

	// Inicializo el sistema de medida de ancho de pulsos
    if ( spx_io_board == SPX_IO5CH ) {
    	RMETER_init( SYSMAINCLK );
     	xprintf_P( PSTR("RMETER init..\r\n\0"));

    }

}
//------------------------------------------------------------------------------------
void range_config_defaults(void)
{
	snprintf_P( systemVars.range_name, PARAMNAME_LENGTH, PSTR("X\0"));
}
//------------------------------------------------------------------------------------
bool range_config ( char *s_name )
{

	// Esta opcion es solo valida para IO5
	if ( spx_io_board != SPX_IO5CH ) {
		range_config_defaults();
		return(false);
	}

	snprintf_P( systemVars.range_name, PARAMNAME_LENGTH, PSTR("%s\0"), s_name );
	return(true);

}
//------------------------------------------------------------------------------------
bool range_read( int16_t *range )
{

bool retS = false;

	// Solo si el equipo es IO5CH y esta el range habilitado !!!
	if ( ( spx_io_board == SPX_IO5CH )  && ( strcmp_P( systemVars.range_name, PSTR("X\0")) != 0 ) ) {
		( systemVars.debug == DEBUG_DATA ) ?  RMETER_ping( range, true ) : RMETER_ping( range, false );
		retS = true;
	}
	return(retS);
}
//------------------------------------------------------------------------------------
void range_print(file_descriptor_t fd, uint16_t src )
{

	// Solo si el equipo es IO5CH y esta el range habilitado !!!
	if ( ( spx_io_board == SPX_IO5CH )  && ( strcmp_P( systemVars.range_name, PSTR("X\0")) != 0 ) ) {
		xfprintf_P(fd, PSTR("%s:%d;"),systemVars.range_name,src );
	}
}
//------------------------------------------------------------------------------------
uint8_t range_hash(void)
{

uint8_t hash = 0;
//char dst[32];
char *p;
uint16_t i;
int16_t free_size = sizeof(hash_buffer);

	//	char range_name[PARAMNAME_LENGTH];

	// Vacio el buffer temoral
	memset(hash_buffer,'\0', sizeof(hash_buffer));
	// Copio sobe el buffer una vista ascii ( imprimible ) de c/registro.
	i = 0;
	i += snprintf_P(hash_buffer, free_size, PSTR("%s"), systemVars.range_name);
	free_size = (  sizeof(hash_buffer) - i );
	if ( free_size < 0 ) goto exit_error;

	//xprintf_P( PSTR("DEBUG: RCKS = [%s]\r\n\0"), hash_buffer );
	// Apunto al comienzo para recorrer el buffer
	p = hash_buffer;
	// Mientras no sea NULL calculo el checksum deol buffer
	while (*p != '\0') {
		//checksum += *p++;
		hash = u_hash(hash, *p++);
	}
	//xprintf_P( PSTR("DEBUG: cks = [0x%02x]\r\n\0"), checksum );

	//xprintf_P( PSTR("COMMS: range_hash OK[%d]\r\n\0"),free_size);

	return(hash);

exit_error:
	xprintf_P( PSTR("COMMS: range_hash ERROR !!!\r\n\0"));
	return(0x00);
}
//------------------------------------------------------------------------------------


