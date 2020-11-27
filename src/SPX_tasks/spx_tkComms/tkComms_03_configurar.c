/*
 * tkComms_05_configurar.c
 *
 *  Created on: 5 mar. 2020
 *      Author: pablo
 */

#include <tkComms.h>

// La tarea no puede demorar mas de 180s.
#define WDG_COMMS_TO_CONFIG	WDG_TO180

//------------------------------------------------------------------------------------
t_comms_states tkComms_st_configurar(void)
{
	/*
	 * Configuro el dispositivo para que pueda usarse.
	 * Salidas:
	 * 	ST_MON_SQE
	 * 	ST_ENTRY
	 *
	 */

uint8_t err_code = ERR_NONE;
t_comms_states next_state = ST_PRENDER;


	xprintf_PD( DF_COMMS, PSTR("COMMS: IN st_configurar.\r\n\0"));

	ctl_watchdog_kick(WDG_COMMS, WDG_COMMS_TO_CONFIG);

	if ( xCOMMS_configurar_dispositivo( sVarsComms.simpwd, sVarsComms.apn, &err_code ) == true ) {
		next_state = ST_ESPERA_PRENDIDO;
		goto EXIT;
	}

	// Error de configuracion.
	xCOMMS_stateVars.errores_comms++;
	switch (err_code) {
	case ERR_CPIN_FAIL:
		/*
		 * No tengo el pin correcto. Apago y vuelvo a empezar.
		 */
		xprintf_P( PSTR("COMMS: ERROR cpin. !!!\r\n\0"));
		xCOMMS_apagar_dispositivo();
		next_state = ST_PRENDER;
		break;

	case ERR_CREG_FAIL:
		/*
		 * No ms puedo registrar en la red. Apago y vuelvo a empezar.
		 */
		xprintf_P( PSTR("COMMS: ERROR registration. !!!\r\n\0"));
		xCOMMS_apagar_dispositivo();
		next_state = ST_PRENDER;
		break;

	case ERR_CPSI_FAIL:
		/*
		 * La red no esta operativa
		 */
		xprintf_P( PSTR("COMMS: ERROR red NO operativa. !!!\r\n\0"));
		xCOMMS_apagar_dispositivo();
		next_state = ST_PRENDER;
		break;

	case ERR_NETATTACH_FAIL:
		xprintf_P( PSTR("COMMS: ERROR net attach.\r\n\0"));
		next_state = ST_PRENDER;
		break;

	case ERR_APN_FAIL:
		xprintf_P( PSTR("COMMS: ERROR apn.\r\n\0"));
		next_state = ST_PRENDER;
		break;

	default:
		xprintf_P( PSTR("COMMS: ERRROR no considerado. !!!\r\n\0"));
		next_state = ST_PRENDER;
		break;
	}

	// No proceso las señales:

EXIT:

	xprintf_PD( DF_COMMS, PSTR("COMMS: OUT st_configurar.\r\n\0"));
	return(next_state);
}
//------------------------------------------------------------------------------------



