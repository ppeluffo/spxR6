/*
 * tkComms_01_main.c
 *
 *  Created on: 5 mar. 2020
 *      Author: pablo
 */

#include <tkComms.h>

#define WDG_GPRSRX_TIMEOUT WDG_TO60

//------------------------------------------------------------------------------------
void tkComms(void * pvParameters)
{

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	xCOMMS_init();

	tkComms_state = ST_PRENDER;
	xprintf_P( PSTR("starting tkComms..\r\n\0"));

	// loop
	for( ;; )
	{

		switch ( tkComms_state ) {
		case ST_PRENDER:
			tkComms_state = tkComms_st_prender();
			break;
		case ST_CONFIGURAR:
			tkComms_state = tkComms_st_configurar();
			break;
		case ST_ESPERA_PRENDIDO:
			tkComms_state = tkComms_st_espera_prendido();
			break;
		case ST_DATAFRAME:
			tkComms_state = tkComms_st_dataframe();
			break;
		default:
			tkComms_state = ST_ENTRY;
			xprintf_P( PSTR("COMMS: state ERROR !!.\r\n\0"));
		}

	}
}
//------------------------------------------------------------------------------------
void tkCommsRX(void * pvParameters)
{
	// Esta tarea lee y procesa las respuestas del GPRS. Lee c/caracter recibido y lo va
	// metiendo en un buffer circular propio del GPRS que permite luego su impresion,
	// analisis, etc.

( void ) pvParameters;
char c;
uint32_t ulNotifiedValue;


	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	xprintf_P( PSTR("starting tkCommsRX..\r\n\0"));

	/*
	// loop
	for( ;; )
	{
		ctl_watchdog_kick(WDG_COMMSRX, WDG_GPRSRX_TIMEOUT );

		// Leo el UART de GPRS
		if ( frtos_read( fdGPRS, &c, 1 ) == 1 ) {
			gprs_rxBuffer_fill(c);
		}

	}
	*/
	for( ;; )	{

		ctl_watchdog_kick(WDG_COMMSRX, WDG_GPRSRX_TIMEOUT );

		if ( xCOMMS_stateVars.gprs_prendido == true ) {
			// Leo el UART de GPRS
			if ( frtos_read( fdGPRS, &c, 1 ) == 1 ) {
				gprs_rxBuffer_fill(c);
			}

		} else {

			// Espero hasta 25s o que me llegue una señal
			xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 10000 / portTICK_RATE_MS ) );

		}
	}
}
//------------------------------------------------------------------------------------


