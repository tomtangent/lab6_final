
// Standard includes
#include <stdio.h>
#include <stdbool.h>
#include <string.h>


// Simplelink includes
#include "simplelink.h"


// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "timer.h"
#include "uart.h"
#include "utils.h"

// Common interface includes
#include "uart_if.h"
#include "timer_if.h"
#include "gpio_if.h"
#include "spi.h"
#include "common.h"

#include "pin_mux_config.h"

#include "test.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

#include "i2c_if.h"

#define ZERO "00010000000100001001000001101111"
#define ONE "00010000000100000000000011111111"
#define TWO "00010000000100001000000001111111"
#define THREE "00010000000100000100000010111111"
#define FOUR "00010000000100001100000000111111"
#define FIVE "00010000000100000010000011011111"
#define SIX "00010000000100001010000001011111"
#define SEVEN "00010000000100000110000010011111"
#define EIGHT "00010000000100001110000000011111"
#define NINE "00010000000100000001000011101111"
#define LAST "00010000000100001011100001000111"
#define MUTE "00010000000100000010100011010111"
#define CONFIRM "00010000000100001100100000110111"

//SIMPLELINK
#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Winter2017"
#define SERVER_NAME                "a3d8qk3265cvvw-ats.iot.us-east-1.amazonaws.com" //"A39526W7VT5KHJ.iot.us-east-1.amazonaws.com"
#define GOOGLE_DST_PORT             8443

#define SL_SSL_CA_CERT "/cert/rootCA.der"
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                25   /* Current Date */
#define MONTH               5     /* Month 1-12 */
#define YEAR                2023  /* Current year */
#define HOUR                23    /* Time - hours */
#define MINUTE              39    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/TT_CC3200_Board/shadow HTTP/1.1\n\r"
#define GETHEADER "GET /things/TT_CC3200_Board/shadow HTTP/1.1\n\r"

#define HOSTHEADER "Host: a3d8qk3265cvvw-ats.iot.us-east-1.amazonaws.com\r\n" //"Host: A39526W7VT5KHJ.iot.us-east-1.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

//#define DATA1 "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : \"Ho brah, dis ting stay cool!!!\"\n\r}}}\n\r\n\r"
#define PART1 "{\"state\": {\r\n\"desired\" : {\r\n\"messageagain\" : \""
#define PART2 "\"\n\r}}}\n\r\n\r"
//"\"\r\n}}}\r\n\r\n"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

char messageToBeSent[300];
int score = -1;
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
//#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
//#if defined(ccs)
//extern void (* const g_pfnVectors[])(void);
//#endif
//#if defined(ewarm)
//extern uVectorEntry __vector_table;
//#endif




////=================

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);
//static int http_get(int);
long printErrConvenience(char * msg, long retVal);

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);



    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}

//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP,uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }
    else {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    GPIO_IF_LedConfigure(LED1|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;
    char* DATA = "";

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

//    strcat(DATA, PART1);
//    strcat(DATA, messageToBeSent);
//    strcat(DATA, PART2);
    // sprintf(DATA, "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : \"%s\"\n\r}}}\n\r\n\r", messageToBeSent);
    sprintf(DATA, "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" : \"%s: %d\"\n\r}}}\n\r\n\r", messageToBeSent, score);
    Report(DATA);

    int dataLength = strlen(DATA);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA);
    pcBufHeaders += strlen(DATA);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

//========


extern void (* const g_pfnVectors[])(void);
// Globals used to save and restore interrupt settings.

unsigned long cycle = 0;
unsigned long cycle_tap = 0;
unsigned long tap_counter = 0;
char reset_var = 0;
char input[33] = "";
int counter = 0;
unsigned long temp = 0;
unsigned long temp_tap = 0;

//unsigned int color = RED;

//For UART

char messageReceived[300];
int drawNow;
int char_position = 0;

void SendMessage(void);
void ClearCharAtSender(int pos);
void PrintCharSender(int pos, char c);

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting IR_output = { .port = GPIOA2_BASE, .pin = 0x2}; // pin 08 is GPIO input

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

char* decoder () {
    if (strcmp(ONE, input) == 0){
        return "1";
    }
    else if (strcmp(TWO, input) == 0){
        return "2";
    }
    else if (strcmp(THREE, input) == 0){
        return "3";
    }
    else if (strcmp(FOUR, input) == 0){
        return "4";
    }
    else if (strcmp(FIVE, input) == 0) {
        return "5";
    }
    else if (strcmp(SIX, input) == 0){
        return "6";
    }
    else if (strcmp(SEVEN, input) == 0){
        return "7";
    }
    else if (strcmp(EIGHT, input) == 0){
        return "8";
    }
    else if (strcmp(NINE, input) == 0) {
        return "9";
    }
    else if (strcmp(MUTE, input) == 0) {
        return "M";
    }
    else if (strcmp(LAST, input) == 0) {
        return "L";
    }
    else if (strcmp(ZERO, input)== 0){
        return "0";
    }
    else if (strcmp(CONFIRM, input)== 0){
        return "X";
    }
    else
        return NULL;
}

void multi_tap(char button, char *output, unsigned long temp_counter) {
    unsigned long ulStatus_tap;
    ulStatus_tap = MAP_GPIOIntStatus (IR_output.port, true);
    MAP_GPIOIntClear(IR_output.port, ulStatus_tap);

    temp_tap = cycle;
    cycle = 0;
    //temp_tap = temp_counter;

    //printf("Temp tap counter: %d\n", temp_counter);

    static char last_button = '\0';
    static int count = 0;
    char *letters;

    //Button cycler
    switch (button) {
        case '2': letters = "abc"; break;
        case '3': letters = "def"; break;
        case '4': letters = "ghi"; break;
        case '5': letters = "jkl"; break;
        case '6': letters = "mno"; break;
        case '7': letters = "pwrs"; break;
        case '8': letters = "tuv"; break;
        case '9': letters = "wxyz"; break;
        case '0': letters = " "; break;
        case 'X': count = 0; break;
        default: return;
    }

    //Check if button is different from last time, if so reset counter
    if (button != last_button) {
        last_button = button;
        count = 0;
    }else if(button != NULL){
        --char_position;
    }

    //Cycle through the count and character, if it's greater than a certain amount reset to 0
    if (temp_tap <= 5) {
        count++;
        if (count > strlen(letters)) {
            count = 1;
        }
    } else {
        count = 0;
    }
    //set the character that was cycled through
    *output = letters[count - 1];
}



void display(unsigned long temp_counter)
{
    char output = '\0';
    printf("%s\n", input);
    char* input = decoder();

    char c = *input;
    printf("%c\n", c);

    //If the button pressed is LAST
    if (c == 'L'){
        messageToBeSent[char_position] = 'L';
        messageToBeSent[char_position+1] = '\0';
        printf("Message: %s\n", messageToBeSent);
        drawNow = 1;
        SendMessage();

    //If the button pressed is MUTE, go back 1 character position and clear screen
    }else if(c == 'M'){
        --char_position;
        ClearCharAtSender(char_position);
    }

    else{

        if (c == 'X'){
            --char_position;
        }

        multi_tap(c, &output, temp_counter);
        ClearCharAtSender(char_position);
        PrintCharSender(char_position, output);
        //printf("Message: %s\n", output);
        if (c != 'X'){
            ++char_position;
        }


        printf("%s\n", input);

        char temp = output;
        printf("%c\n", temp);

        Report("%s\n\r", input);
        Report("%c\n\r", temp);
    }


}

void TimerA0IntHandler(void) {

    Timer_IF_InterruptClear(TIMERA0_BASE);
    cycle++;
}

//*****************************************************************************
static void GPIOA1IntHandler(void) {
    unsigned long ulStatus;

      ulStatus = MAP_GPIOIntStatus (IR_output.port, true);
      MAP_GPIOIntClear(IR_output.port, ulStatus);

      temp = cycle;
      cycle = 0;


    //Entering the proper signal
    if (reset_var == 1) {
        if (temp >= 4) {
            strcat(input, "1");
        }

        else{
            strcat(input, "0");
        }
        counter++;
        if (counter == 32){
            display(temp);
        }
    }

    //RESET Signal
    if (temp >= 17) {
        input[0] = '\0';
        reset_var = 1;
        counter = 0;

    }

}

void SendMessage(void){
    fillRect(0,0,128,64,0);
    //PRINT THE MESSAGE THAT IS TO BE SENT
    //printf(messageToBeSent);
}

//void PrintCharReceiver(int pos, char c){
//    if (c == ' '){
//        return;
//    }
//    int x = (pos % 21) *6;
//    int y = (pos % 21) *8;
//    y += 70;
//}


void PrintMessage(void){

    int pos = 0;
    int len = strlen(messageReceived);
    while(pos < len)
    {
        //PrintCharReceiver(pos, messageReceived[pos]);
        ++pos;
    }
}

void PrintCharSender(int pos, char c)
{
    messageToBeSent[pos] = c;
    //printf(messageToBeSent);
    messageToBeSent[pos+1] = '\0';
    if(c == ' ')
        return;
    int x = (pos % 21) * 6;
    int y = (pos / 21) * 8;
    drawChar(x, y, c, WHITE, 0, 1);

}

void ClearCharAtSender(int pos)
{
    int x = (pos % 21) * 6;
    int y = (pos / 21) * 8;
    fillRect(x, y, 6, 8, 0);
    messageToBeSent[pos] = '\0';
}

int getRandomNumber(int min, int max){
    return (rand() % (max-min+1))+min;
}

bool checkCollision(int yPos) {
    // Define the boundaries of the rectangles
    unsigned char rectWidth = 10;
    unsigned char posX = width() - rectWidth;
    int rectHeight;

    rectHeight = getRandomNumber(62, 117);  // Generate random height between 62 and 116 (exclusive)

    // Check collision with current rectangle
    if (yPos >= (height() - rectHeight) / 2 && yPos <= (height() - rectHeight) / 2 + rectHeight && posX <= 64 && posX + rectWidth >= 64) {
        return true;  // Collision detected
    }

    //posX -= rectWidth;  // Move to the next rectangle


    return false;  // No collision
}

void testfillrects_ball(unsigned int color1, unsigned int color2) {
    unsigned char x;
    unsigned char rectWidth = 10;
    unsigned char rectHeight;
    unsigned char posX = width() - rectWidth;



        rectHeight = getRandomNumber(62, 117);  // Generate random height between 62 and 116 (exclusive)
        fillRect(posX, (height() - rectHeight) / 2, rectWidth, rectHeight, color1);
        fillRect(posX, (height() - rectHeight) / 2, rectWidth, rectHeight, BLACK);
        posX -= rectWidth;

}


void printGameOver() {
    int color = WHITE;
    int x = 8;
    int y = 8;
    int size = 1;
    int bg = BLACK;
    char hw[] = "GAME OVER";
    int i;

    for (i = 0; i < strlen(hw); i++) {
        drawChar(x, y, hw[i], color, bg, size);
        x += 8;
    }
}

void printSentence(char* hw) {
    int color = WHITE;
    int x = 8;
    int y = 8;
    int size = 1;
    int bg = BLACK;
    int i;

    for (i = 0; i < strlen(hw); i++) {
        drawChar(x, y, hw[i], color, bg, size);
        x += 8;
    }
}

void game(){
    unsigned char x;
    unsigned char y;
    unsigned char aAddress = 0x18;
    unsigned char xOff = 0x3;
    unsigned char yOff = 0x5;
    int xSpd = 0;
    int ySpd = 0;
    int xPos = 64;
    int yPos = 64;

    int orb_x_pos = rand() % 125;
    int orb_y_pos = rand() % 125;

    int current_size = 4;

    int gameOver = 0;

   while(gameOver == 0){

       fillCircle(xPos, yPos, current_size, BLACK);

       //read from registers
       //x
       I2C_IF_Write(aAddress, &xOff, 1, 0);
       I2C_IF_Read(aAddress, &x, 1);
       //y
       I2C_IF_Write(aAddress, &yOff, 1, 0);
       I2C_IF_Read(aAddress, &y, 1);

       //Convert till to speed
       xSpd = (int)((signed char) y);
       ySpd = (int)((signed char) x);

       //Speed to position
       xPos -= xSpd/5;
       yPos -= ySpd/5;

       //printf("%d", xPos);
       //printf("%d", yPos);

       //On Orb position
       if(abs(xPos - orb_x_pos) <= current_size && abs(yPos - orb_y_pos) <= current_size ) {
          fillCircle(orb_x_pos, orb_y_pos, 2, BLACK);
          orb_x_pos = rand() % (125 - current_size);//getRandomNumber(10, (240 - current_size));
          orb_y_pos = rand() % (125 - current_size);//getRandomNumber(10, (240 - current_size));
          current_size += 1;
       }

       //Spawn orb
       fillCircle(orb_x_pos, orb_y_pos, 2, YELLOW);

       //BoundsChecking
       if(xPos >= 124 - current_size){
           xPos = 124 - current_size;
           gameOver = 1;
       }
       if(xPos <= 4 - current_size){
           xPos = 4 - current_size;
           gameOver = 1;
       }

       if(yPos >= 124 - current_size){
           yPos = 124 - current_size;
           gameOver = 1;
       }
       if(yPos <= 4 - current_size){
           yPos = 4 - current_size;
           gameOver = 1;
       }

       fillCircle(xPos, yPos, current_size, RED);

  }
   printSentence("GAME OVER");
   delay(50);
   fillScreen(BLACK);
   score = current_size - 4;
   char numString[100];
   sprintf(numString, "%d", score);
   printSentence(numString);
   printf("%d", score);
   delay(100);
   printSentence("ENTER YOUR NAME: ");
   delay(50);
}

//
void MasterMain()
{

    //
    // Set Tx buffer index
    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);


    //UART
//    MAP_UARTConfigSetExpClk(UARTA1_BASE,
//                                MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
//                                UART_BAUD_RATE,
//                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
//                                );

    //Set up interrupts
//    UARTFIFOLevelSet(UARTA1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
//    //UARTIntRegister(UARTA1_BASE, ReceiveHandler);
//    UARTIntClear(UARTA1_BASE, UART_INT_RX|UART_INT_RT);
//    UARTIntEnable(UARTA1_BASE, UART_INT_RX|UART_INT_RT);

    //Enable communication
   // UARTEnable(UARTA1_BASE);

    Adafruit_Init();
    fillScreen(BLACK);
}



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void) {
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    //MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************
int main() {
    unsigned long ulStatus;
    long lRetVal = -1;
    //int gameOver = 0;

    BoardInit();

    PinMuxConfig();

    InitTerm();

    ClearTerm();
    UART_PRINT("Hello world!\n\r");


////    Connect the CC3200 to the local access point
     lRetVal = connectToAccessPoint();
     //Set time so that encryption can be used
     lRetVal = set_time();
     if(lRetVal < 0) {
         UART_PRINT("Unable to set time in the device");
         LOOP_FOREVER();
     }
     //Connect to the website with TLS encryption
     lRetVal = tls_connect();
     if(lRetVal < 0) {
         ERR_PRINT(lRetVal);
     }

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC_UP, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerA0IntHandler);
    Timer_IF_Stop(TIMERA0_BASE, TIMER_A);
    MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, 40000);  //0.1ms
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(IR_output.port, GPIOA1IntHandler);

    //
    // Configure rising edge interrupt on SW2 and SW3
    //
    MAP_GPIOIntTypeSet(IR_output.port, IR_output.pin, GPIO_FALLING_EDGE);

    ulStatus = MAP_GPIOIntStatus (IR_output.port, false);
    MAP_GPIOIntClear(IR_output.port, ulStatus);

    // Enable SW2 and SW3 Interrupt
    MAP_GPIOIntEnable(IR_output.port, IR_output.pin);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);

    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //http_post(lRetVal);

    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    MasterMain();
    //fillScreen(BLACK);

     while(1)
     {
         if (score == -1){
             game();
             printf("%d", score);
             fillScreen(BLACK);
         }

         if(drawNow == 1){
             UART_PRINT("HERE");

             http_post(lRetVal);
             PrintMessage();
             //SendMessage();
             messageReceived[0] = '\0';
             //messageToBeSent[0] = '\0';
             char_position = 0;
             drawNow = 0;
             score = -1;


         }

     }



}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
