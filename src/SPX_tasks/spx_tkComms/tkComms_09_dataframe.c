/*
 * tkComms_10_dataframe.c
 *
 *  Created on: 10 mar. 2020
 *      Author: pablo
 */

#include <tkComms.h>
#include <../spx_tkApp/tkApp.h>

// La tarea no puede demorar mas de 300s.

#define WDG_COMMS_TO_DATAFRAME	WDG_TO300

static st_dataRecord_t dataRecord;

static void data_send_record( void );
static uint8_t data_process_response_OK(void);


//------------------------------------------------------------------------------------
t_comms_states tkComms_st_dataframe(void)
{
	/* Estado en que procesa los frames de datos, los transmite y procesa
	 * las respuestas
	 * Si no hay datos para transmitir sale
	 */

// ENTRY

#ifdef BETA_TEST
	xprintf_PD( DF_COMMS, PSTR("COMMS: IN st_dataframe.[%d,%d,%d]\r\n\0"),xCOMMS_stateVars.gprs_prendido, xCOMMS_stateVars.gprs_inicializado,xCOMMS_stateVars.errores_comms);
#else
	xprintf_PD( DF_COMMS, PSTR("COMMS: IN st_dataframe.\r\n"));
#endif

#ifdef MONITOR_STACK
	debug_monitor_stack_watermarks("13");
#endif

	xprintf_P( PSTR("COMMS: dataframe.\r\n\0"));
	ctl_watchdog_kick(WDG_COMMS, WDG_COMMS_TO_DATAFRAME );

	while ( xCOMMS_datos_para_transmitir() > 0 ) {
		xCOMMS_process_frame(DATA, sVarsComms.server_ip_address, sVarsComms.server_tcp_port );
	}

	// Si llego la senal, la reseteo ya que transmiti todos los frames.
	xCOMMS_SGN_FRAME_READY();

#ifdef BETA_TEST
	xprintf_PD( DF_COMMS, PSTR("COMMS: OUT st_dataframe.[%d,%d,%d]\r\n\0"),xCOMMS_stateVars.gprs_prendido, xCOMMS_stateVars.gprs_inicializado,xCOMMS_stateVars.errores_comms);
#else
	xprintf_PD( DF_COMMS, PSTR("COMMS: OUT st_dataframe.\r\n\0"));
#endif

	return(ST_ESPERA_PRENDIDO);

}
//------------------------------------------------------------------------------------
void xDATA_FRAME_send(void)
{

uint8_t registros_trasmitidos = 0;

	// Envio un window frame
	registros_trasmitidos = 0;
	FF_rewind();

	xCOMMS_flush_RX();
	xCOMMS_flush_TX();

	xCOMMS_send_header("DATA");
	xprintf_PVD(  xCOMMS_get_fd(), DF_COMMS, PSTR("&PLOAD=\0") );

	while ( ( xCOMMS_datos_para_transmitir() > 0 ) && ( registros_trasmitidos < MAX_RCDS_WINDOW_SIZE ) ) {
		data_send_record();
		registros_trasmitidos++;
		// Espero 250ms entre records
		vTaskDelay( (portTickType)( INTER_FRAMES_DELAY / portTICK_RATE_MS ) );
	}

	xCOMMS_send_tail();

}
//------------------------------------------------------------------------------------
t_responses xDATA_FRAME_process_response(void)
{
	// Las respuestas pueden venir enganchadas por lo que hay que procesarlas todas !!

t_responses rsp = rsp_NONE;

	// Recibi un ERROR de respuesta
	if ( xCOMMS_check_response("ERROR") ) {
		xCOMMS_print_RX_buffer();
		rsp = rsp_ERROR;
		return(rsp);
	}

	// Respuesta completa del server
	if ( xCOMMS_check_response("</h1>") ) {

		xCOMMS_print_RX_buffer();

		if ( xCOMMS_check_response ("ERROR\0")) {
			// ERROR del server: salgo inmediatamente
			rsp = rsp_ERROR;
			return(rsp);
		}

		/*
		 * Lo ultimo que debo procesar es el OK !!!
		 */
		if ( xCOMMS_check_response ("RX_OK\0")) {
			// Datos procesados por el server.
			data_process_response_OK();
			rsp = rsp_OK;
			return(rsp);
		}
	}

// Exit:
	// No tuve respuesta aun
	return(rsp);
}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES
//------------------------------------------------------------------------------------
static void data_send_record( void )
{
	/* Leo un registro de la memoria haciendo el proceso inverso de
	 * cuando los grabe en spx_tkData::pv_data_guardar_en_BD y lo
	 * mando por el canal de comunicaciones
	 */

size_t bRead;
FAT_t fat;

	memset ( &fat, '\0', sizeof(FAT_t));
	memset ( &dataRecord, '\0', sizeof( st_dataRecord_t));

	// Paso1: Leo un registro de memoria
	bRead = FF_readRcd( &dataRecord, sizeof(st_dataRecord_t));
	FAT_read(&fat);

	if ( bRead == 0) {
		return;
	}

	xprintf_PVD(  xCOMMS_get_fd(), DF_COMMS, PSTR("CTL:%d;\0"),fat.rdPTR );
	xCOMMS_send_dr(DF_COMMS, &dataRecord);

}
//------------------------------------------------------------------------------------
static uint8_t data_process_response_OK(void)
{
	/*
	 * Retorno la cantidad de registros procesados ( y borrados )
	 * Recibi un OK del server y el ultimo ID de registro a borrar.
	 * Los borro de a uno.
	 */

uint8_t recds_borrados = 0;
FAT_t fat;

	memset ( &fat, '\0', sizeof( FAT_t));

	// Borro los registros.
	while (  u_check_more_Rcds4Del(&fat) ) {
		FF_deleteRcd();
		recds_borrados++;
		FAT_read(&fat);
		xprintf_PD( DF_COMMS, PSTR("COMMS: mem wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), fat.wrPTR, fat.rdPTR, fat.delPTR, fat.rcds4wr, fat.rcds4rd, fat.rcds4del );
	}

	return(recds_borrados);
}
//------------------------------------------------------------------------------------


