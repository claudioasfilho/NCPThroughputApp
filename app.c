/***********************************************************************************************//**
 * \file   app.c
 * \brief  Event handling and application code for Empty NCP Host application example
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* standard library headers */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
//#include<conio.h>

/* BG stack headers */
#include "bg_types.h"
#include "gecko_bglib.h"
#include "gatt_db.h"


/* Own header */
#include "app.h"



#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 1
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];


//bb b9 9e 70-ff f7-46 cf-ab c7-2d 32 c7 18 20 f2

/* UUIDs for the demo service and characteristics */
const uint8_t serviceUUID[] = {
		0xf2,
		0x20,
		0x18,
		0xc7,
		0x32,
		0x2d,
		0xc7,
		0xab,
		0xcf,
		0x46,
		0xf7,
		0xFF,
		0x70,
		0x9e,
		0xb9,
		0xbb };

/* ---- Application macros ---- */

/* GENERAL MACROS */
#define SOFT_TIMER_DISPLAY_REFRESH_HANDLE		0	// Handle for the display refresh
#define SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE	1 	// Handle for stopping fixed time transfer

#define DATA_SIZE			255					// Size of the arrays for sending and receiving data

#define DATA_TRANSFER_SIZE_INDICATIONS		0 // If == 0 or > MTU-3 then it will send MTU-3 bytes of data, otherwise it will use this value
#define DATA_TRANSFER_SIZE_NOTIFICATIONS	0 // If == 0 or > MTU-3 then it will calculate the data amount to send for maximum over-the-air packet usage, otherwise it will use this value

#define PHY_1M				(0x01)
#define PHY_2M				(0x02)
#define PHY_S8				(0x04)
#define PHY_S2				(0x08)

#define TX_POWER			(-50)

//#define USE_LED_FOR_CONNECTION_SIGNALING		// Define this so that LED0 is ON when connection is established and OFF when it's disconnected
//#define USE_LED_FOR_DATA_SENDING_SIGNALING 	// Define this so that LED1 is ON when data is being send

/* SLAVE SIDE MACROS */
#define NOTIFICATIONS_START				(uint32)(1 << 0)  	// Bit flag to external signal command
#define NOTIFICATIONS_END				(uint32)(1 << 1)	// Bit flag to external signal command
#define NOTIFICATIONS_TEST_STARTED				(uint32)(0x1000)  	// Bit flag to external signal command
#define NOTIFICATIONS_TEST_FINISHED				(uint32)(0x2000)  	// Bit flag to external signal command
#define NOTIFICATIONS_TEST_INTERVAL 10 							//In seconds
#define INDICATIONS_START				(uint32)(1 << 2)	// Bit flag to external signal command
#define INDICATIONS_END					(uint32)(1 << 3)	// Bit flag to external signal command
#define ADV_INTERVAL_MAX				160					// 160 * 0.625us = 100ms
#define ADV_INTERVAL_MIN				160					// 160 * 0.625us = 100ms
//#define SEND_FIXED_TRANSFER_COUNT		1000				// Uncomment this if you want to send a fixed amount of indications/notifications on each button press
//#define SEND_FIXED_TRANSFER_TIME		(32768*5)				// Uncomment this if you want to send indications/notifications for a fixed amount of time (in 32.768Hz clock ticks) on each button press

/* MASTER SIDE MACROS */
#define PHY_CHANGE						(uint32)(1 << 4)	// Bit flag to external signal command
#define WRITE_NO_RESPONSE_START			(uint32)(1 << 5)	// Bit flag to external signal command
#define WRITE_NO_RESPONSE_END			(uint32)(1 << 6)	// Bit flag to external signal command
#define CONN_INTERVAL_1MPHY_MAX			40					// 40 * 1.25ms = 50ms
#define CONN_INTERVAL_1MPHY_MIN			40					// 40 * 1.25ms = 50ms
#define SLAVE_LATENCY_1MPHY				0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_1MPHY		100					// 100 * 10ms = 1000ms
#define CONN_INTERVAL_2MPHY_MAX			20  				// 20 * 1.25ms = 25ms
#define CONN_INTERVAL_2MPHY_MIN			20					// 20 * 1.25ms = 25ms
#define SLAVE_LATENCY_2MPHY				0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_2MPHY		100					// 100 * 10ms = 1000ms
#define CONN_INTERVAL_125KPHY_MAX		160					// 160 * 1.25ms = 200ms
#define CONN_INTERVAL_125KPHY_MIN		160					// 160 * 1.25ms = 200ms
#define SLAVE_LATENCY_125KPHY			0					// How many connection intervals can the slave skip if no data is to be sent
#define SUPERVISION_TIMEOUT_125KPHY		200					// 200 * 10ms = 2000ms
#define SCAN_INTERVAL					16					// 16 * 0.625 = 10ms
#define SCAN_WINDOW						16					// 16 * 0.625 = 10ms
#define ACTIVE_SCANNING					1					// 1 = active scanning (sends scan requests), 0 = passive scanning (doesn't send scan requests)
/* -------------------- */


#if (CONN_INTERVAL_125KPHY_MAX < 32) || (CONN_INTERVAL_125KPHY_MIN < 32)
#error "Minimum connection interval for LE Coded PHY must be above 40ms according to set_phy command description in API Ref."
#endif

#if defined(SEND_FIXED_TRANSFER_COUNT) && defined(SEND_FIXED_TRANSFER_TIME)
#error "These are mutually exclusive options, you either do a fixed amount of transfers of transfer over a fixed amount of time."
#endif


static bool roleIsSlave = false; 				// Flag to check if role is slave or master (based on PB0 being pressed or not during boot)

uint32_t time_elapsed;									// Variable to calculate time during which there was data tranmission
const uint8_t displayRefreshOn = 1;						// Turn ON display refresh on master side
const uint8_t displayRefreshOff = 0;					// Turn OFF display refresh on master side
uint8_t boot_to_dfu = 0; 								// Flag indicating if device should boot into DFU mode
uint16_t mtuSize = 0;  									// Variable to hold the MTU size once a new connection is formed
uint16_t pduSize = 0;									// Variable to hold the PDU size once a new connection is formed
uint16_t maxDataSizeIndications = 0;
uint16_t maxDataSizeNotifications = 0;					// Variable to calculate maximum data size for optimum throughput
uint8_t connection = 0; 								// Variable to hold the connection handle
uint16_t phyInUse = PHY_1M;								// Variable to hold the PHY in use
uint16_t phyToUse = 0;									// Variable to hold the next PHY to use when changing to and from LE Coded Phy
bool notifications_enabled = false; 					// Flag to check if notifications are enabled or not
bool indications_enabled = false; 						// Flag to check if indications are enabled or not

bool sendNotifications = false; 						// Flag to trigger sending of notifications
bool sendIndications = false; 							// Flag to trigger sending of indications
bool sendWriteNoResponse = false;						// Flag to trigger sending of write no response
bool notification_accepted = true;						// Flag to check if previous notification command was accepted and generate new data for the next one
uint8 throughput_array_notifications[DATA_SIZE] = {0}; 	// Array to hold data payload to be sent over notifications
uint8 throughput_array_indications[DATA_SIZE] = {0}; 	// Array to hold data payload to be sent over indications
uint32 bitsSent = 0; 									// Variable to increment the amount of data sent and received and display the throughput
uint32 throughput = 0;									// Variable to hold throughput calculation
uint32 operationCount = 0;								// Variable to count how many GATT operations have occurred from both sides
uint8_t enableNotificationsIndications = 0;				// Variable to control enabling notifications and indications in master mode
uint8_t invalidData = 0;								// Variable to register how many notifications were not received by the application
#ifdef SEND_FIXED_TRANSFER_COUNT
uint32_t transferCount = 0;
#endif
char throughputString[] = "TH:           \n";			// Char array to print the bitsSent variable on the display every second, so this will be throughput
char mtuSizeString[] = "MTU:     "; 				// Char array to print MTU size on the display
char connIntervalString[] = "INTRV:      ";		// Char array to print connection interval on the display
char pduSizeString[] = "PDU:     ";				// Char array to print PTU size on the display
char deviceNameString[] = "Throughput Tester";			// Char array to with device name to match against scan results
char phyInUseString[] = "PHY:    ";						// Char array to print PHY in use on the display
char maxDataSizeNotificationsString[] = "DATA SIZE:     ";
char invalidDataString[] = "INVALD:     ";
char operationCountString[] = "CNT:       \n";
const char roleSlaveString[] = {"ROLE: Slave\n"};
const char roleMasterString[] = {"ROLE: Master\n"};
char statusConnectedString[] = {"RSSI:     \n"};
const char statusDisconnectedString[] = {"STATUS: Discon\n"};
const char notifyEnabledString[] = {"NOTIFY: Yes\n"};
const char notifyDisabledString[] = {"NOTIFY: No\n"};
const char indicateEnabledString[] = {"INDICATE: Yes\n"};
const char indicateDisabledString[] = {"INDICATE: No\n"};
char* roleString;
char* statusString = (char*)statusDisconnectedString;
char* notifyString = (char*)notifyDisabledString;
char* indicateString = (char*)indicateDisabledString;
/* -------------------- */



// App booted flag
static bool appBooted = false;
static bool Scanning = false;
static bool Testing = false;
static uint32 updateCounter;
static uint32 SMState = 0;

/**************************************************************************//**
* @brief Routine to refresh the info on the display based on the Bluetooth link status
*****************************************************************************/
void displayRefresh()
{


	//GRAPHICS_Clear();

	if (Scanning==0)
	{
		printf("\e[2J");

		printf("%s\n",roleString);
		printf("%s\n",statusString);
		printf("%s\n",connIntervalString);
		printf("%s\n",pduSizeString);
		printf("%s\n",mtuSizeString);
		printf("%s\n",maxDataSizeNotificationsString);
		printf("%s\n",phyInUseString);
		printf("%s\n",notifyString);
		printf("%s\n",indicateString);
		sprintf(throughputString+4, "%07lu", (unsigned long)throughput);
			throughputString[11] = ' ';
			throughputString[12] = 'b';
			throughputString[13] = 'p';
			throughputString[14] = 's';
		printf("%s\n",throughputString);
			printf("Counter: %d\n", updateCounter++ );

		//sprintf(operationCountString+5, "%09lu", operationCount);
		printf("%s\n",operationCountString);
		//sprintf(invalidDataString+8, "%04u", invalidData);
		//GRAPHICS_AppendString(invalidDataString);

	//	GRAPHICS_Update();
	}


}

uint32_t RTCC_CounterGet(void)
{
  return 5;
}


/**************************************************************************//**
* @brief Function to generate circular data (0-255) in the data payload
*****************************************************************************/
void generate_data_notifications(void){

	throughput_array_notifications[0] = throughput_array_notifications[maxDataSizeNotifications-1] + 1;

	for(int i = 1; i<maxDataSizeNotifications; i++)
	{
		throughput_array_notifications[i] = throughput_array_notifications[i-1] + 1;
	}
}


/**************************************************************************//**
* @brief Function to generate circular data (0-255) in the data payload
*****************************************************************************/
void generate_data_indications(void){

	throughput_array_indications[0] = throughput_array_indications[maxDataSizeIndications-1] + 1;

	for(int i = 1; i<maxDataSizeIndications; i++)
	{
		throughput_array_indications[i] = throughput_array_indications[i-1] + 1;
	}
}

/**************************************************************************//**
* @brief Processes advertisement packets looking for "Throughput Tester" device name
*****************************************************************************/


#if 1
int process_scan_response(struct gecko_msg_le_gap_scan_response_evt_t *pResp)
{
	/* Decoding advertising packets is done here. The list of AD types can be found
	 * at: https://www.bluetooth.com/specifications/assigned-numbers/Generic-Access-Profile */

    int i = 0;
    int ad_match_found = 0;
	int ad_len;
    int ad_type;

    while (i < (pResp->data.len - 1))
    {
        ad_len  = pResp->data.data[i];
        ad_type = pResp->data.data[i+1];

        if (ad_type == 0x09)
        {
            /* type 0x09 = Complete Local Name */

        	/* Check if device name is Throughput Tester */
        	if(memcmp(pResp->data.data+i+2, deviceNameString, 17) == 0)
			{
        		ad_match_found = 1;
        		break;
			}
        }

        /* Jump to next AD record */
        i = i + ad_len + 1;
    }

    return(ad_match_found);
}
#else

int process_scan_response(struct gecko_msg_le_gap_scan_response_evt_t *pResp) {
	// decoding advertising packets is done here. The list of AD types can be found
	// at: https://www.bluetooth.com/specifications/assigned-numbers/Generic-Access-Profile

	uint8_t i = 0;
	uint8_t ad_match_found = 0;
	uint8_t ad_len;
	uint8_t ad_type;

	while (i < (pResp->data.len - 1)) {

		ad_len = pResp->data.data[i];
		ad_type = pResp->data.data[i + 1];

		if (ad_type == 0x07 || ad_type == 0x06) {
			// type 0x06 = More 128-bit UUIDs available
			// type 0x07 = Complete list of 128-bit UUIDs available

			// note: this check assumes that the service we are looking for is first
			// in the list. To be fixed so that there is no such limitation...
			if (!memcmp(pResp->data.data + i + 2, serviceUUID, 16)) {
				ad_match_found = 1;
				break;
			}
		}

		//jump to next AD record
		i = i + ad_len + 1;
	}
	return (ad_match_found);
}
#endif
/**************************************************************************//**
* @brief Does a few things before initiating data transmissions. Read RTCC, disable
* display refresh in master side and turn ON LED indicating data transmission
*****************************************************************************/
void dataTransmissionStart(void)
{
	bitsSent = 0;
	throughput = 0;
	time_elapsed = RTCC_CounterGet();

	/* Turn OFF Display refresh on master side */
	gecko_cmd_gatt_write_characteristic_value_without_response(connection, gattdb_display_refresh, 1, &displayRefreshOff);

	/* Stop display refresh */
//	gecko_cmd_hardware_set_soft_timer(0, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);

#ifdef USE_LED_FOR_DATA_SENDING_SIGNALING
	/* Turn ON data LED */
	GPIO_PinOutSet(BSP_LED1_PORT,BSP_LED1_PIN);
#endif
}

/**************************************************************************//**
* @brief Does a few after data transmissions ended. Calculate transmission time,
* enable display refresh in master side and turn OFF LED indicating data transmission
*****************************************************************************/
void dataTransmissionEnd(void)
{
	time_elapsed = RTCC_CounterGet() - time_elapsed;

	/* Turn ON Display on master side - stack is probably still busy pushing the last few notifications out so we need to check output */
	while(gecko_cmd_gatt_write_characteristic_value_without_response(connection, gattdb_display_refresh, 1, &displayRefreshOn)->result!=0);

	/* Resume display refresh - stack is probably still busy pushing the last few notifications out so we need to check output */
	while(gecko_cmd_hardware_set_soft_timer(32768, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0)->result != 0);

#ifdef USE_LED_FOR_DATA_SENDING_SIGNALING
	/* Turn ON data LED */
	GPIO_PinOutClear(BSP_LED1_PORT,BSP_LED1_PIN);
#endif
	/* Calculate throughput */
	throughput = (uint32_t)((float)bitsSent / (float)((float)time_elapsed / (float)32768));
}

void testStateMachine(void)
{
	static struct gecko_msg_system_get_counters_rsp_t *getCounters;
	static uint8_t SMCounter=0;

	if ((Scanning==0))
	{
		if ((SMState == 0) && (SMCounter==0))
		{
			SMState = NOTIFICATIONS_START;
			Testing = true;
			printf("Starting Notifications Test for %ds \n", NOTIFICATIONS_TEST_INTERVAL);
		}
		if ((SMState == NOTIFICATIONS_START) && (SMCounter==NOTIFICATIONS_TEST_INTERVAL)) SMState = NOTIFICATIONS_END;
		if ((SMState == NOTIFICATIONS_END) && (SMCounter==0)) SMState = NOTIFICATIONS_TEST_FINISHED;
	}

if (Testing)
	{
		switch (SMState)
	    	  {
	    	  	  case NOTIFICATIONS_START:

	    	  		 // dataTransmissionStart();
	    	  		  sendNotifications = true;
	    	  		  generate_data_notifications();
	#if defined(SEND_FIXED_TRANSFER_COUNT)
	    	  		  transferCount = 0;
	#elif defined(SEND_FIXED_TRANSFER_TIME)
	    	  		  gecko_cmd_hardware_set_soft_timer(SEND_FIXED_TRANSFER_TIME, SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE, 1);
	#endif
	    	  		  getCounters = gecko_cmd_system_get_counters(1);
								SMState = NOTIFICATIONS_TEST_STARTED;
	    	  		  break;

							case NOTIFICATIONS_TEST_STARTED:
								SMCounter++;
							break;

	    	  	  case NOTIFICATIONS_END:

	#if !defined(SEND_FIXED_TRANSFER_COUNT) && !defined(SEND_FIXED_TRANSFER_TIME)
	    	  		 // dataTransmissionEnd();
	    	  		  sendNotifications = false;
	    	  		  getCounters = gecko_cmd_system_get_counters(1);
								SMCounter=0;

	#endif
	    	  		  break;

								case NOTIFICATIONS_TEST_FINISHED:
									printf("Test Finished\n");
									Testing = false;
								break;


	    	  	  case WRITE_NO_RESPONSE_START:
	    	  		  //dataTransmissionStart();
	    	  		  sendWriteNoResponse = true;
	    	  		  generate_data_notifications();
	#if defined(SEND_FIXED_TRANSFER_COUNT)
	    	  		  transferCount = 0;
	#elif defined(SEND_FIXED_TRANSFER_TIME)
	    	  		  gecko_cmd_hardware_set_soft_timer(SEND_FIXED_TRANSFER_TIME, SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE, 1);
	#endif
	    	  		  break;

	    	  	  case WRITE_NO_RESPONSE_END:
	#if !defined(SEND_FIXED_TRANSFER_COUNT) && !defined(SEND_FIXED_TRANSFER_TIME)
	    	  		  //dataTransmissionEnd();
	    	  		  sendWriteNoResponse = false;

	#endif
					  break;

	    	  	  case INDICATIONS_START:

	    	  		  //dataTransmissionStart();
	    	  		  sendIndications = true;
	#if defined(SEND_FIXED_TRANSFER_COUNT)
	    	  		  transferCount = 0;
	#elif defined(SEND_FIXED_TRANSFER_TIME)
	    	  		  gecko_cmd_hardware_set_soft_timer(SEND_FIXED_TRANSFER_TIME, SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE, 1);
	#endif
	    	  		  if(indications_enabled)
	    	  		  {
	    	  			  generate_data_indications();
	        	  		  while(gecko_cmd_gatt_server_send_characteristic_notification(connection, gattdb_throughput_indications, maxDataSizeIndications, throughput_array_indications)->result != 0);
	    	  		  }
	    	  		  break;

	    	  	  case INDICATIONS_END:

	#if !defined(SEND_FIXED_TRANSFER_COUNT) && !defined(SEND_FIXED_TRANSFER_TIME)
	    	  		  //dataTransmissionEnd();
	    	  		  sendIndications = false;
	#endif
	    	  		  break;

	    	  	  case PHY_CHANGE:
	    	  		  switch(phyInUse) {
										    	  		  case PHY_1M:

										    	  			  /* We're on 1M PHY, go to 2M PHY - only supported by xG12 and xG13 */
										    	  			  phyToUse = PHY_2M;
										    	  			  /* Change connection parameters for 2MPHY */

										    	  			  break;

										    	  		  case PHY_2M:

										    	  			  /* We're on 2M PHY, go to 125kbit Coded PHY (S=8) - only supported by xG13 */
										    	  			  phyToUse = PHY_S8;
										    	  			  /* Change connection parameters according to set_phy command description
															   * in API Ref. Minimum connection interval for LE Coded PHY is 40ms */
															  gecko_cmd_le_connection_set_parameters(connection, CONN_INTERVAL_125KPHY_MIN, CONN_INTERVAL_125KPHY_MAX, SLAVE_LATENCY_125KPHY, SUPERVISION_TIMEOUT_125KPHY);
															  /* We're on 2MPHY but with xG12, go back to 1M PHY */
															  phyToUse = PHY_1M;
															  /* Change connection parameters back to the minimum */
															  gecko_cmd_le_connection_set_parameters(connection, CONN_INTERVAL_1MPHY_MIN, CONN_INTERVAL_1MPHY_MAX, SLAVE_LATENCY_1MPHY, SUPERVISION_TIMEOUT_1MPHY);
										    	  			  break;

										    	  		  case PHY_S8:

										    	  			  /* We're on S8 PHY, go back to 1M PHY */
										    	  			  phyToUse = PHY_1M;
										    	  			  /* Change connection parameters back to the minimum */
										    	  			  gecko_cmd_le_connection_set_parameters(connection, CONN_INTERVAL_1MPHY_MIN, CONN_INTERVAL_1MPHY_MAX, SLAVE_LATENCY_1MPHY, SUPERVISION_TIMEOUT_1MPHY);

										    	  			  break;

	    	  		  }
	    	  		  break;
								default:
									break;
					}
				}



}//testStateMachine



/***********************************************************************************************//**
 *  \brief  Event handler function.
 *  \param[in] evt Event pointer.
 **************************************************************************************************/
void appHandleEvents(struct gecko_cmd_packet *evt)
{

 static struct gecko_msg_system_get_counters_rsp_t *getCounters;

  if (NULL == evt) {
    return;
  }


#if 0
	// Do not handle any events until system is booted up properly.
  if ((BGLIB_MSG_ID(evt->header) != gecko_evt_system_boot_id)
      && !appBooted) {
#if defined(DEBUG)
    printf("Event: 0x%04x\n", BGLIB_MSG_ID(evt->header));
#endif
    usleep(50000);
    return;
  }

#endif

#if 1
  if(notifications_enabled && sendNotifications)
     {

     	if(gecko_cmd_gatt_server_send_characteristic_notification(connection, gattdb_throughput_notifications, maxDataSizeNotifications, throughput_array_notifications)->result == 0)
 		{
     		bitsSent += (maxDataSizeNotifications*8);
     		operationCount++;
     		generate_data_notifications();
 #ifdef SEND_FIXED_TRANSFER_COUNT
     		if(++transferCount == SEND_FIXED_TRANSFER_COUNT) {
     			dataTransmissionEnd();
     			/* Stop sending notifications */
     			sendNotifications = false;
     		}
 #endif
 		}

	} //if if(notifications_enabled && sendNotifications)

     else if(sendWriteNoResponse)
     {

     	if(gecko_cmd_gatt_write_characteristic_value_without_response(connection, gattdb_throughput_write_no_response, maxDataSizeNotifications, throughput_array_notifications)->result == 0)
 		{
     		bitsSent += (maxDataSizeNotifications*8);
     		operationCount++;
     		generate_data_notifications();
 #ifdef SEND_FIXED_TRANSFER_COUNT
     		if(++transferCount == SEND_FIXED_TRANSFER_COUNT) {
     			dataTransmissionEnd();
     			/* Stop sending notifications */
     			sendWriteNoResponse = false;
     		}
 #endif
 		}

	}// else if(sendWriteNoResponse)

	#endif



  /* Handle events */
  switch (BGLIB_MSG_ID(evt->header)) {
    case gecko_evt_system_boot_id:

      appBooted = true;

			if (roleIsSlave == 0)
			{
				printf("Role is Master\n");
				roleString = (char*)roleMasterString;
			}
			else
			{
				printf("Role is Slave\n");
				roleString = (char*)roleSlaveString;
			}

      printf("System booted\n");


  		sprintf(connIntervalString+7, "%04u", 0);
  		sprintf(phyInUseString+5, "%s", "1M");
  		sprintf(mtuSizeString+5, "%03u", mtuSize);
  		sprintf(pduSizeString+5, "%03u", pduSize);
  		sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);
  		sprintf(invalidDataString+9, "%03u", invalidData);

  		//gecko_cmd_gatt_server_write_attribute_value(gattdb_display_refresh, 0, 1, &displayRefreshOn);

  		gecko_cmd_gatt_set_max_mtu(250);

  		gecko_cmd_system_set_tx_power(TX_POWER);

  		if(roleIsSlave) {
				printf("Starting advertising... \n");
  			/* Set advertising parameters. 100ms advertisement interval. All channels used.
  			* The first two parameters are minimum and maximum advertising interval, both in
  			* units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
  			gecko_cmd_le_gap_set_adv_parameters(ADV_INTERVAL_MIN,ADV_INTERVAL_MAX,7);

  			/* Start general advertising and enable connections. */
  			gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
  		}
			else {
  			gecko_cmd_le_gap_set_conn_parameters(CONN_INTERVAL_1MPHY_MIN, CONN_INTERVAL_1MPHY_MAX, SLAVE_LATENCY_1MPHY, SUPERVISION_TIMEOUT_1MPHY);
  			/* Set scan parameters and start scanning */
  			gecko_cmd_le_gap_set_scan_parameters(SCAN_INTERVAL, SCAN_WINDOW, ACTIVE_SCANNING);

  			gecko_cmd_le_gap_discover(le_gap_discover_generic);
				Scanning = 1;
  		}

  		gecko_cmd_hardware_set_soft_timer(32768, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);


      break;

			case gecko_evt_le_gap_scan_response_id:

			#if 0
							printf("address ---> ");
							for(uint8_t i=0; i< 6; i++) {
								printf("0x%02x ", evt->data.evt_le_gap_scan_response.address.addr[i]);
							}
							printf(" ---- ");
							printf("advertisement packet --> ");
							for(uint8_t i=0; i< evt->data.evt_le_gap_scan_response.data.len; i++) {
								printf("0x%02x ", evt->data.evt_le_gap_scan_response.data.data[i]);
							}
							printf("\r\n");
			#endif
							// process scan responses: this function returns 1 if we found the service we are looking for
							if (process_scan_response(&(evt->data.evt_le_gap_scan_response)) > 0) {
								struct gecko_msg_le_gap_open_rsp_t *pResp;
								// match found -> stop discovery and try to connect
								gecko_cmd_le_gap_end_procedure();
								printf("OK --- >Device found, connecting.\r\n");
								Scanning = 0;
								pResp = gecko_cmd_le_gap_open(evt->data.evt_le_gap_scan_response.address, evt->data.evt_le_gap_scan_response.address_type);
								// make copy of connection handle for later use (for example, to cancel the connection attempt)
								//connHandle = pResp->connection;
							}
					break;

      case gecko_evt_le_connection_opened_id:

      printf("Connection Opened\n");

    	  break;

            case gecko_evt_le_connection_closed_id:

            printf("Connection Closed\n");

      			/* Clear all flags and relevant parameters */
      			connection = 0;
      			mtuSize = 0;
      			pduSize = 0;
      			maxDataSizeNotifications = 0;
      			invalidData = 0;
      			operationCount = 0;
      			indications_enabled = false;
      			notifications_enabled = false;
      			throughput = 0;
      			enableNotificationsIndications = 0;
      			phyInUse = PHY_1M;
      			phyToUse = 0;

      			sprintf(connIntervalString+7, "%04u", 0);
      			sprintf(phyInUseString+5, "%s", "1M");
      			sprintf(mtuSizeString+5, "%03u", mtuSize);
      			sprintf(pduSizeString+5, "%03u", pduSize);
      			sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);
      			sprintf(invalidDataString+9, "%03u", invalidData);

      			statusString = (char*)statusDisconnectedString;
      			notifyString = (char*)notifyDisabledString;
      			indicateString = (char*)indicateDisabledString;

      			/* Reset data */
      			memset(throughput_array_notifications, 0, DATA_SIZE);
      			memset(throughput_array_indications, 0, DATA_SIZE);

      			if(roleIsSlave) {
      				/* Check if need to boot to dfu mode */
      				if (boot_to_dfu) {
      					/* Enter to DFU OTA mode */
      					gecko_cmd_system_reset(2);
      				}
      				else {
      					/* Restart advertising after client has disconnected */
      					gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
      				}
      			} else {
      				/* Back to scanning */
      				gecko_cmd_le_gap_discover(le_gap_discover_generic);
      			}
              break;

            case gecko_evt_gatt_server_characteristic_status_id:

      		  if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_throughput_notifications)
      		  {
      			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
      				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_notification)
      			  {
      				  notifications_enabled = true;
      				  notifyString = (char*)notifyEnabledString;
      			  }

      			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
      				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_disable)
      			  {
      				  notifications_enabled = false;
      				  notifyString = (char*)notifyDisabledString;
      			  }

      		  }

      		  if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_throughput_indications)
      		  {
      			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
      				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_indication)
      			  {
      				  indications_enabled = true;
      				  indicateString = (char*)indicateEnabledString;
      			  }

      			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_client_config &&
      				 evt->data.evt_gatt_server_characteristic_status.client_config_flags == gatt_disable)
      			  {
      				  indications_enabled = false;
      				  indicateString = (char*)indicateDisabledString;
      			  }

      			  if(evt->data.evt_gatt_server_characteristic_status.status_flags == gatt_server_confirmation)
      			  {
      				  /* Last indicate operation was acknowledged, send more data */
      				  bitsSent += ((maxDataSizeIndications)*8);
      				  operationCount++;
      				  generate_data_indications();
      #ifdef SEND_FIXED_TRANSFER_COUNT
      				  if(++transferCount == SEND_FIXED_TRANSFER_COUNT) {
      					  dataTransmissionEnd();
      					  // Stop sending indications
      					  sendIndications = false;
      				  }
      #endif
      				  if(indications_enabled && sendIndications)
      				  {
      					  while(gecko_cmd_gatt_server_send_characteristic_notification(connection, gattdb_throughput_indications, maxDataSizeIndications, throughput_array_indications)->result != 0);
      				  }
      			  }
      		  }

          	  break;

            case gecko_evt_gatt_characteristic_value_id:

          	  /* Data received */
          	  if(evt->data.evt_gatt_characteristic_value.att_opcode == gatt_handle_value_indication) {
          		  gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
          	  }

          	  bitsSent += (evt->data.evt_gatt_characteristic_value.value.len*8);
          	  operationCount++;

          	  /* Validate the data */
          	  for(int i=1; i<evt->data.evt_gatt_characteristic_value.value.len; i++)
          	  {
          		  if(evt->data.evt_gatt_characteristic_value.value.data[i] != (uint8)((evt->data.evt_gatt_characteristic_value.value.data[i-1])+1))
          		  {
      				  /* Data is not what we expected */
      				  invalidData++;
          		  }
          	  }

          	  break;

            case gecko_evt_gatt_server_attribute_value_id:

          	  if(evt->data.evt_gatt_server_attribute_value.attribute == gattdb_display_refresh)
          	  {
      			  /* Display ON/OFF state changes */
      			  if(evt->data.evt_gatt_server_attribute_value.value.data[0] == 0)
      			  {
      				  bitsSent = 0;
      				  throughput = 0;
      				  time_elapsed = RTCC_CounterGet();
      				  /* Disable display refresh */
      				  gecko_cmd_hardware_set_soft_timer(0, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);
      				  /* Turn ON data LED */
                printf("Data On\n");

                //GPIO_PinOutSet(BSP_LED1_PORT,BSP_LED1_PIN);

      			  }
      			  else
      			  {
      				  time_elapsed = RTCC_CounterGet() - time_elapsed;
      				  /* Enable display refresh */
      				  gecko_cmd_hardware_set_soft_timer(32768, SOFT_TIMER_DISPLAY_REFRESH_HANDLE, 0);
      				  /* Turn OFF data LED */
      				  printf("Data Off\n");
      				  /* Calculate throughput */
      				  throughput = (uint32_t)((float)bitsSent / (float)((float)time_elapsed / (float)32768));
      				  //throughput = bitsSent / (time_elapsed / 32768);

      			  }
          	  }

          	  if(evt->data.evt_gatt_server_attribute_value.attribute == gattdb_throughput_write_no_response)
          	  {
              	  bitsSent += (evt->data.evt_gatt_server_attribute_value.value.len*8);
              	  operationCount++;

              	  /* Validate the data */
              	  for(int i=1; i<evt->data.evt_gatt_server_attribute_value.value.len; i++)
              	  {
              		  if(evt->data.evt_gatt_server_attribute_value.value.data[i] != (uint8)((evt->data.evt_gatt_server_attribute_value.value.data[i-1])+1))
              		  {
          				  /* Data is not what we expected */
          				  invalidData++;
              		  }
              	  }
          	  }
          	  break;

            case gecko_evt_hardware_soft_timer_id:



          	  switch(evt->data.evt_hardware_soft_timer.handle)
          	  {
      			  case SOFT_TIMER_DISPLAY_REFRESH_HANDLE:

      		    	  if(gecko_cmd_le_connection_get_rssi(connection)->result != 0) {
      		    		  // Command didn't go through, most likely out of memory error
      		    		  //sprintf(statusConnectedString+6, "ERR");
      		    	  }

      		    	  displayRefresh();
									testStateMachine();



      	    		  //bitsSent = 0;
      				  break;

      			  case SOFT_TIMER_FIXED_TRANSFER_TIME_HANDLE:
      				  dataTransmissionEnd();
      				  sendNotifications = false;
      				  sendIndications = false;
      				  sendWriteNoResponse = false;
      				  break;
      			  default:
      				  break;
          	  }
      	  break;

      	  case gecko_evt_le_connection_rssi_id:
      		  sprintf(statusConnectedString+6, "%03d", evt->data.evt_le_connection_rssi.rssi);
      		  break;

            case gecko_evt_le_connection_phy_status_id:
          	  	  phyToUse = 0;
          	  	  phyInUse = evt->data.evt_le_connection_phy_status.phy;
          		  switch(phyInUse) {
      				  case PHY_1M:
      					  sprintf(phyInUseString+5, "%s", "1M");
      					  break;
      				  case PHY_2M:
      					  sprintf(phyInUseString+5, "%s", "2M");
      					  break;
      				  case PHY_S8:
      					  sprintf(phyInUseString+5, "%s", "S8");
      					  break;
      				  case PHY_S2:
      					  sprintf(phyInUseString+5, "%s", "S2");
      					  break;
      				  default:
      					  break;
      			  }
          	  break;

            case gecko_evt_gatt_mtu_exchanged_id:

          	  mtuSize = evt->data.evt_gatt_mtu_exchanged.mtu;

          	  sprintf(mtuSizeString+5, "%03u", mtuSize);

          	  connection = evt->data.evt_gatt_mtu_exchanged.connection;

          	  if(DATA_TRANSFER_SIZE_INDICATIONS == 0 || DATA_TRANSFER_SIZE_INDICATIONS > (mtuSize-3))
          	  {
          		  maxDataSizeIndications = mtuSize-3;
          	  }
          	  else
          	  {
          		  maxDataSizeIndications = DATA_TRANSFER_SIZE_INDICATIONS;
          	  }

          	  if(DATA_TRANSFER_SIZE_NOTIFICATIONS == 0 || DATA_TRANSFER_SIZE_NOTIFICATIONS > (mtuSize-3))
          	  {
      			  if(pduSize!=0 && mtuSize!=0) {
      				  if(pduSize <= mtuSize)
      				  {
      					  maxDataSizeNotifications = (pduSize - 7) + ((mtuSize - 3 - pduSize + 7) / pduSize * pduSize);
      				  }
      				  else
      				  {

      					  if(pduSize-mtuSize<=4)
      					  {
      						  maxDataSizeNotifications = pduSize - 7;
      					  } else {
      						  maxDataSizeNotifications = mtuSize - 3;
      					  }
      				  }
      			  }
          	  }
          	  else
          	  {
          		  maxDataSizeNotifications = DATA_TRANSFER_SIZE_NOTIFICATIONS;
          	  }
          	  sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);

          	  if(!roleIsSlave) {
      			  /* For the sake of simplicity we'll just assume that the CCCD handle for the indication
      			   * and notification characteristics is the characteristic handle + 1
      			   */
      			  enableNotificationsIndications = 1;
      			  gecko_cmd_gatt_write_descriptor_value(connection, gattdb_throughput_notifications+1, 1, &enableNotificationsIndications);
          	  }
          	  break;

            case gecko_evt_gatt_procedure_completed_id:

          	  if(enableNotificationsIndications == 1) {
          		  notifications_enabled = 1;
          		  enableNotificationsIndications = 2;
          		  gecko_cmd_gatt_write_descriptor_value(connection, gattdb_throughput_indications+1, 1, &enableNotificationsIndications);
          	  }

          	  if(enableNotificationsIndications == 2) {
          		  indications_enabled = 2;
          	  }
          	  break;

            case gecko_evt_le_connection_parameters_id:

          	  pduSize = evt->data.evt_le_connection_parameters.txsize;
          	  sprintf(pduSizeString+5, "%03u", pduSize);
          	  sprintf(connIntervalString+7, "%04u", (unsigned int)((float)evt->data.evt_le_connection_parameters.interval*1.25));
          	  statusString = (char*)statusConnectedString;


          	  if(DATA_TRANSFER_SIZE_NOTIFICATIONS == 0 || DATA_TRANSFER_SIZE_NOTIFICATIONS > (mtuSize-3))
          	  {
      			  if(pduSize!=0 && mtuSize!=0) {
      				  if(pduSize <= mtuSize)
      				  {
      					  maxDataSizeNotifications = (pduSize - 7) + ((mtuSize - 3 - pduSize + 7) / pduSize * pduSize);
      				  }
      				  else
      				  {
      					  if(pduSize-mtuSize<=4)
      					  {
      						  maxDataSizeNotifications = pduSize - 7;
      					  } else {
      						  maxDataSizeNotifications = mtuSize - 3;
      					  }
      				  }
      			  }
          	  }
          	  else
      		  {
      			  maxDataSizeNotifications = DATA_TRANSFER_SIZE_NOTIFICATIONS;
      		  }
          	  sprintf(maxDataSizeNotificationsString+11, "%03u", maxDataSizeNotifications);

          	  /* Change phy if request */
          	  if(phyToUse) {
          		  gecko_cmd_le_connection_set_phy(connection, phyToUse);
          	  }
          	  break;
    default:
      break;
		}

}
