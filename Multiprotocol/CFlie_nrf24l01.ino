/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

// Most of this code was ported from theseankelly's related DeviationTX work.

#if defined(CFLIE_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define CFLIE_BIND_COUNT 60
//#define CFLIE_USE_CRTP_RPYT

//=============================================================================
// CRTP (Crazy RealTime Protocol) Implementation
//=============================================================================

// Port IDs
enum {
    CRTP_PORT_CONSOLE = 0x00,
    CRTP_PORT_PARAM = 0x02,
    CRTP_PORT_SETPOINT = 0x03,
    CRTP_PORT_MEM = 0x04,
    CRTP_PORT_LOG = 0x05,
    CRTP_PORT_POSITION = 0x06,
    CRTP_PORT_SETPOINT_GENERIC = 0x07,
    CRTP_PORT_PLATFORM = 0x0D,
    CRTP_PORT_LINK = 0x0F,
};

// Channel definitions for the LOG port
enum {
    CRTP_LOG_CHAN_TOC = 0x00,
    CRTP_LOG_CHAN_SETTINGS = 0x01,
    CRTP_LOG_CHAN_LOGDATA = 0x02,
};

// Command definitions for the LOG port's TOC channel
enum {
    CRTP_LOG_TOC_CMD_ELEMENT = 0x00,
    CRTP_LOG_TOC_CMD_INFO = 0x01,
};

// Command definitions for the LOG port's CMD channel
enum {
    CRTP_LOG_SETTINGS_CMD_CREATE_BLOCK = 0x00,
    // CRTP_LOG_SETTINGS_CMD_CREATE_BLOCK = 0x06, //v2

    CRTP_LOG_SETTINGS_CMD_APPEND_BLOCK = 0x01,
    CRTP_LOG_SETTINGS_CMD_DELETE_BLOCK = 0x02,
    CRTP_LOG_SETTINGS_CMD_START_LOGGING = 0x03,
    // CRTP_LOG_SETTINGS_CMD_START_LOGGING = 0x08, //v2

    CRTP_LOG_SETTINGS_CMD_STOP_LOGGING = 0x04,
    CRTP_LOG_SETTINGS_CMD_RESET_LOGGING = 0x05,
};

// Log variables types
enum {
    LOG_UINT8 = 0x01,
    LOG_UINT16 = 0x02,
    LOG_UINT32 = 0x03,
    LOG_INT8 = 0x04,
    LOG_INT16 = 0x05,
    LOG_INT32 = 0x06,
    LOG_FLOAT = 0x07,
    LOG_FP16 = 0x08,
};

#define CFLIE_TELEM_LOG_BLOCK_ID            0x7F
#define CFLIE_TELEM_LOG_BLOCK_PERIOD_10MS   0x32 // 50*10 = 500ms

#define CRTP_GET_PORT(hdr)    (((hdr) >> 4) & 0x0F)
#define CRTP_GET_CHANNEL(hdr) ((hdr) & 0x03)

uint8_t STAV = 0;

extern uint8_t telemetry_link;


// Setpoint type definitions for the generic setpoint channel
enum {
    CRTP_SETPOINT_GENERIC_STOP_TYPE = 0x00,
    CRTP_SETPOINT_GENERIC_VELOCITY_WORLD_TYPE = 0x01,
    CRTP_SETPOINT_GENERIC_Z_DISTANCE_TYPE = 0x02,
    CRTP_SETPOINT_GENERIC_CPPM_EMU_TYPE = 0x03,
};

static inline uint8_t crtp_create_header(uint8_t port, uint8_t channel)
{
    return ((port)&0x0F)<<4 | (channel & 0x03);
}

//=============================================================================
// End CRTP implementation
//=============================================================================

// Address size
#define TX_ADDR_SIZE 5

// Timeout for callback in uSec, 10ms=10000us for Crazyflie
#define CFLIE_PACKET_PERIOD 10000

#define MAX_PACKET_SIZE 32  // CRTP is 32 bytes

// CPPM CRTP supports up to 10 aux channels but deviation only
// supports a total of 12 channels. R,P,Y,T leaves 8 aux channels left
#define MAX_CPPM_AUX_CHANNELS 8

static uint8_t tx_payload_len = 0; // Length of the packet stored in packet
static uint8_t rx_payload_len = 0; // Length of the packet stored in rx_packet
static uint8_t rx_packet[MAX_PACKET_SIZE]; // For reading in ACK payloads

static uint8_t data_rate;

enum {
    CFLIE_INIT_SEARCH = 0,
    CFLIE_INIT_CRTP_LOG,
    CFLIE_INIT_DATA,
    CFLIE_SEARCH,
    CFLIE_DATA
};

static uint8_t crtp_log_setup_state;
enum {
    CFLIE_CRTP_LOG_SETUP_STATE_INIT = 0,
    CFLIE_CRTP_LOG_RESET,
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_INFO,
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_ITEM,
    // It might be a good idea to add a state here
    // to send the command to reset the logging engine
    // to avoid log block ID conflicts. However, there
    // is not a conflict with the current defaults in
    // cfclient and I'd rather be able to log from the Tx
    // and cfclient simultaneously
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_CREATE_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_START_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_COMPLETE,
};

// State variables for the crtp_log_setup_state_machine
static uint8_t toc_size;             // Size of the TOC read from the crazyflie
static uint8_t next_toc_variable;    // State variable keeping track of the next var to read
static uint8_t vbat_var_id;          // ID of the vbatMV variable
static uint8_t extvbat_var_id;       // ID of the extVbatMV variable
static uint8_t rssi_var_id;          // ID of the RSSI variable
static uint8_t isPressed_var_id;    // ID of the isPressed variable

// Constants used for finding var IDs from the toc
static const char* pm_group_name = "pm";
static const char* vbat_var_name = "vbatMV";
static const uint8_t vbat_var_type = LOG_UINT16;
static const char* extvbat_var_name = "extVbatMV";
static const uint8_t extvbat_var_type = LOG_UINT16;
static const char* radio_group_name = "radio";
static const char* rssi_var_name = "rssi";
static const uint8_t rssi_var_type = LOG_UINT8;

static const char* flapper_group_name = "flapper";
static const char* isPressed_var_name = "isPressed";
static const uint8_t isPressed_var_type = LOG_INT8;

// Repurposing DSM Telemetry fields
#define TELEM_CFLIE_INTERNAL_VBAT   TELEM_DSM_FLOG_VOLT2    // Onboard voltage
#define TELEM_CFLIE_EXTERNAL_VBAT   TELEM_DSM_FLOG_VOLT1    // Voltage from external pin (BigQuad)
#define TELEM_CFLIE_RSSI            TELEM_DSM_FLOG_FADESA   // Repurpose FADESA for RSSI

enum {
    PROTOOPTS_TELEMETRY = 0,
    PROTOOPTS_CRTP_MODE = 1,
    LAST_PROTO_OPT,
};

#define TELEM_OFF 0
#define TELEM_ON_ACKPKT 1
#define TELEM_ON_CRTPLOG 2

#define CRTP_MODE_RPYT 0
#define CRTP_MODE_CPPM 1

// Bit vector from bit position
#define BV(bit) (1 << bit)

#define PACKET_CHKTIME 500      // time to wait if packet not yet acknowledged or timed out    

// Helper for sending a packet
// Assumes packet data has been put in packet
// and tx_payload_len has been set correctly
static void send_packet()
{
    // clear packet status bits and Tx/Rx FIFOs
    NRF24L01_WriteReg(NRF24L01_07_STATUS, (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT)));
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    // Transmit the payload
    NRF24L01_WritePayload(packet, tx_payload_len);

    // // Check and adjust transmission power.
    NRF24L01_SetPower();
}

static uint16_t dbg_cnt = 0;
static uint8_t packet_ack()
{
	if (++dbg_cnt > 50)
	{
		// debugln("S: %02x\n", NRF24L01_ReadReg(NRF24L01_07_STATUS));
		dbg_cnt = 0;
	}
	switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)))
	{
		case BV(NRF24L01_07_TX_DS):
			rx_payload_len = NRF24L01_GetDynamicPayloadSize();
			if (rx_payload_len > MAX_PACKET_SIZE)
				rx_payload_len = MAX_PACKET_SIZE;
			NRF24L01_ReadPayload(rx_packet, rx_payload_len);
			return PKT_ACKED;
		case BV(NRF24L01_07_MAX_RT):
			return PKT_TIMEOUT;
	}
	return PKT_PENDING;
}

static void set_rate_channel(uint8_t rate, uint8_t channel)
{
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);
	NRF24L01_SetBitrate(rate);
}

static void send_search_packet()
{
	uint8_t buf[1];
	buf[0] = 0xff;
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)));
	NRF24L01_FlushTx();

	// if (sub_protocol == CFLIE_AUTO)
  // {
	  // if (rf_ch_num++ > 125)
	  // {
	    // rf_ch_num = 0;
	    // switch(data_rate)
		  // {
		    // case NRF24L01_BR_250K:
			    // data_rate = NRF24L01_BR_1M;
			    // break;
			  // case NRF24L01_BR_1M:
			    // data_rate = NRF24L01_BR_2M;
			    // break;
			  // case NRF24L01_BR_2M:
			    // data_rate = NRF24L01_BR_250K;
			    // break;
		  // }
	  // }
  // }
  set_rate_channel(data_rate, rf_ch_num);

	NRF24L01_WritePayload(buf, sizeof(buf));

}

// Frac 16.16
#define FRAC_MANTISSA 16 // This means, not IEEE 754...
#define FRAC_SCALE (1 << FRAC_MANTISSA)

// Convert fractional 16.16 to float32
static void frac2float(int32_t n, float* res)
{
	if (n == 0)
	{
		*res = 0.0;
		return;
	}
	uint32_t m = n < 0 ? -n : n; // Figure out mantissa?
	int i;
	for (i = (31-FRAC_MANTISSA); (m & 0x80000000) == 0; i--, m <<= 1);
	m <<= 1; // Clear implicit leftmost 1
	m >>= 9;
	uint32_t e = 127 + i;
	if (n < 0) m |= 0x80000000;
	m |= e << 23;
	*((uint32_t *) res) = m;
}

static void send_crtp_rpyt_packet()
{
	int32_t f_roll;
	int32_t f_pitch;
	int32_t f_yaw;
	uint16_t thrust;

	uint16_t val;

	struct CommanderPacketRPYT
	{
		float roll;
		float pitch;
		float yaw;
		uint16_t thrust;
	}__attribute__((packed)) cpkt;

	// Channels in AETR order
	// Roll, aka aileron, float +- 50.0 in degrees
	// float roll  = -(float) Channels[0]*50.0/10000;
	val = convert_channel_16b_limit(AILERON, -10000, 10000);
	// f_roll = -Channels[0] * FRAC_SCALE / (10000 / 50);
	f_roll = val * FRAC_SCALE / (10000 / 50);

	frac2float(f_roll, &cpkt.roll); // TODO: Remove this and use the correct Mode switch below...
	// debugln("Roll: raw, converted:  %d, %d, %d, %0.2f", Channel_data[AILERON], val, f_roll, cpkt.roll);

	// Pitch, aka elevator, float +- 50.0 degrees
	//float pitch = -(float) Channels[1]*50.0/10000;
	val = convert_channel_16b_limit(ELEVATOR, -10000, 10000);
	// f_pitch = -Channels[1] * FRAC_SCALE / (10000 / 50);
	f_pitch = -val * FRAC_SCALE / (10000 / 50);

	frac2float(f_pitch, &cpkt.pitch); // TODO: Remove this and use the correct Mode switch below...
	// debugln("Pitch: raw, converted:  %d, %d, %d, %0.2f", Channel_data[ELEVATOR], val, f_pitch, cpkt.pitch);

	// Thrust, aka throttle 0..65535, working range 5535..65535
	// Android Crazyflie app puts out a throttle range of 0-80%: 0..52000
	thrust = convert_channel_16b_limit(THROTTLE, 0, 32767) * 2;

	// Crazyflie needs zero thrust to unlock
	if (thrust < 900)
		cpkt.thrust = 0;
	else
		cpkt.thrust = thrust;

	// debugln("Thrust: raw, converted:  %d, %u, %u", Channel_data[THROTTLE], thrust, cpkt.thrust);

	// Yaw, aka rudder, float +- 400.0 deg/s
	// float yaw   = -(float) Channels[3]*400.0/10000;
	val = convert_channel_16b_limit(RUDDER, -10000, 10000);
	// f_yaw = - Channels[3] * FRAC_SCALE / (10000 / 400);
	f_yaw = val * FRAC_SCALE / (10000 / 400);
	frac2float(f_yaw, &cpkt.yaw);

	// debugln("Yaw: raw, converted:  %d, %d, %d, %0.2f", Channel_data[RUDDER], val, f_yaw, cpkt.yaw);

	// Switch on/off?
	// TODO: Get X or + mode working again:
	// if (Channels[4] >= 0) {
	//     frac2float(f_roll, &cpkt.roll);
	//     frac2float(f_pitch, &cpkt.pitch);
	// } else {
	//     // Rotate 45 degrees going from X to + mode or opposite.
	//     // 181 / 256 = 0.70703125 ~= sqrt(2) / 2
	//     int32_t f_x_roll = (f_roll + f_pitch) * 181 / 256;
	//     frac2float(f_x_roll, &cpkt.roll);
	//     int32_t f_x_pitch = (f_pitch - f_roll) * 181 / 256;
	//     frac2float(f_x_pitch, &cpkt.pitch);
	// }

	// Construct and send packet
	packet[0] = crtp_create_header(CRTP_PORT_SETPOINT, 0); // Commander packet to channel 0
	memcpy(&packet[1], (char*) &cpkt, sizeof(cpkt));
	tx_payload_len = 1 + sizeof(cpkt);
	send_packet();
}

static void send_crtp_cppm_emu_packet()
{
    struct CommanderPacketCppmEmu {
        struct {
            uint8_t numAuxChannels : 4; // Set to 0 through MAX_AUX_RC_CHANNELS
            uint8_t reserved : 4;
        } hdr;
        uint16_t channelRoll;
        uint16_t channelPitch;
        uint16_t channelYaw;
        uint16_t channelThrust;
        uint16_t channelAux[10];
    } __attribute__((packed)) cpkt;

    // To emulate PWM RC signals, rescale channels from (-10000,10000) to (1000,2000)
    // This is done by dividing by 20 to get a total range of 1000 (-500,500)
    // and then adding 1500 to to rebase the offset
    #define RESCALE_RC_CHANNEL_TO_PWM(chan) ((chan / 20) + 1500)

    // Make sure the number of aux channels in use is capped to MAX_CPPM_AUX_CHANNELS
    // uint8_t numAuxChannels = Model.num_channels - 4;
    uint8_t numAuxChannels = 4;
    if(numAuxChannels > MAX_CPPM_AUX_CHANNELS)
    {
        numAuxChannels = MAX_CPPM_AUX_CHANNELS;
    }

    cpkt.hdr.numAuxChannels = numAuxChannels;

    // Remap AETR to AERT (RPYT)
    cpkt.channelRoll = convert_channel_16b_limit(AILERON,2000,1000);
    cpkt.channelPitch = convert_channel_16b_limit(ELEVATOR,1000,2000);
    // Note: T & R Swapped:
    cpkt.channelYaw = convert_channel_16b_limit(RUDDER, 1000, 2000);
    cpkt.channelThrust = convert_channel_16b_limit(THROTTLE, 1000, 2000);

    // Rescale the rest of the aux channels - RC channel 4 and up
    for (uint8_t i = 0; i < 10; i++)
    {
        cpkt.channelAux[i] = convert_channel_16b_limit(i+4, 2000, 1000);
    }

    // Total size of the commander packet is a 1-byte header, 4 2-byte channels and
    // a variable number of 2-byte auxiliary channels
    uint8_t commanderPacketSize = 1 + 8 + (2*numAuxChannels);

    // Construct and send packet
    packet[0] = crtp_create_header(CRTP_PORT_SETPOINT_GENERIC, 0); // Generic setpoint packet to channel 0
    packet[1] = CRTP_SETPOINT_GENERIC_CPPM_EMU_TYPE;

    // Copy the header (1) plus 4 2-byte channels (8) plus whatever number of 2-byte aux channels are in use
    memcpy(&packet[2], (char*)&cpkt, commanderPacketSize); // Why not use sizeof(cpkt) here??
    tx_payload_len = 2 + commanderPacketSize; // CRTP header, commander type, and packet
    send_packet();
}

static void send_cmd_packet()
{
    #if defined(CFLIE_USE_CRTP_RPYT)
    {
      send_crtp_rpyt_packet();
    }
    #else
    {
      send_crtp_cppm_emu_packet();
    }
    #endif
}





static uint8_t crtp_log_setup_state_machine()
{
    uint8_t state_machine_completed = 0;

    switch (crtp_log_setup_state) {
        case CFLIE_CRTP_LOG_SETUP_STATE_INIT:
        {
            toc_size = 0;
            next_toc_variable = 0;

            vbat_var_id = 0xFF;   
            extvbat_var_id = 0xFF; 
            rssi_var_id = 0xFF;  
            isPressed_var_id = 0xFF;  

            crtp_log_setup_state = CFLIE_CRTP_LOG_RESET;
            // fallthrough
            
        }

        case CFLIE_CRTP_LOG_RESET:
        {
            crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO;
            packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS);
            packet[1] = CRTP_LOG_SETTINGS_CMD_RESET_LOGGING;
            tx_payload_len = 2;
            STAV = 120;
            send_packet();
            break;
        }


        case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO:
        {
            crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_INFO;
            packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC);
            packet[1] = CRTP_LOG_TOC_CMD_INFO;
            tx_payload_len = 2;
            send_packet();
            STAV = 1;
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_INFO:
        {
            if (packet_ack() == PKT_ACKED) {
                if (rx_payload_len >= 3 &&
                    rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC) &&
                    rx_packet[1] == CRTP_LOG_TOC_CMD_INFO) {
                    
                    toc_size = rx_packet[2];
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM;
                    STAV = 2;
                    return state_machine_completed;
                } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                    // retry
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO;
                    STAV = 3;
                    return state_machine_completed;
                }
            }
            send_cmd_packet(); 
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM:
        {
            crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_ITEM;
            packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC);
            packet[1] = CRTP_LOG_TOC_CMD_ELEMENT;
            packet[2] = next_toc_variable;
            tx_payload_len = 3;
            send_packet();
            STAV = 4;
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_ITEM:
        {
            if (packet_ack() == PKT_ACKED) {
                if (rx_payload_len >= 3 &&
                    rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC) &&
                    rx_packet[1] == CRTP_LOG_TOC_CMD_ELEMENT &&
                    rx_packet[2] == next_toc_variable) {

                    
                    uint8_t var_type = rx_packet[3];
                    char *group = (char*)&rx_packet[4];
                    char *name  = group + strlen(group) + 1;

                    // Match vbat
                    if (var_type == vbat_var_type &&
                        strcmp(group, pm_group_name) == 0 &&
                        strcmp(name, vbat_var_name) == 0) {
                        vbat_var_id = next_toc_variable;
                    }

                    
            
                    
                    if (var_type == rssi_var_type &&
                        strcmp(group, radio_group_name) == 0 &&
                        strcmp(name, rssi_var_name) == 0) {
                        rssi_var_id = next_toc_variable;
                    }

                    

                 
                    next_toc_variable++;
                    if (next_toc_variable >= toc_size) {
                        crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK;
                        STAV = 5;
                    } else {
                        crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM;
                    }
                    return state_machine_completed;
                } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM;
                    return state_machine_completed;
                }
            }
            send_cmd_packet();
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK:
        {
            crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_CREATE_BLOCK;
            STAV = 6;
            packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS);
            packet[1] = CRTP_LOG_SETTINGS_CMD_CREATE_BLOCK;
            
            packet[2] = CFLIE_TELEM_LOG_BLOCK_ID; 

            packet[3] = LOG_UINT16;
            packet[4] = vbat_var_id;

           

            tx_payload_len = 5;


            send_packet();
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_CREATE_BLOCK:
        {
            if (packet_ack() == PKT_ACKED) {
                STAV = 7;
                if (rx_payload_len >= 2 &&
                    rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS) &&
                    rx_packet[1] == CRTP_LOG_SETTINGS_CMD_CREATE_BLOCK) {
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK;
                    STAV = 8;
                    return state_machine_completed;
                } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK;
                    return state_machine_completed;
                }
            }
            send_cmd_packet();
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK:
        {
            crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_START_BLOCK;
            packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS);
            packet[1] = CRTP_LOG_SETTINGS_CMD_START_LOGGING;
            packet[2] = CFLIE_TELEM_LOG_BLOCK_ID;
            packet[3] = CFLIE_TELEM_LOG_BLOCK_PERIOD_10MS; 
            tx_payload_len = 4;
            send_packet();
            STAV = 9;
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_START_BLOCK:
        {
            if (packet_ack() == PKT_ACKED) {
                if (rx_payload_len >= 2 &&
                    rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS) &&
                    rx_packet[1] == CRTP_LOG_SETTINGS_CMD_START_LOGGING) {
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_COMPLETE;
                    STAV = 10;
                    return state_machine_completed;
                } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK;
                    return state_machine_completed;
                }
            }
            send_cmd_packet();
            break;
        }

        case CFLIE_CRTP_LOG_SETUP_STATE_COMPLETE:
        {
            state_machine_completed = 1;
            STAV = 11;
            return state_machine_completed;
            break;
        }
    }

    return state_machine_completed;
}


static uint8_t getVbatV(uint8_t b1, uint8_t b2)
{   
    uint16_t vbatMV = (b2 << 8) | b1;
    uint8_t vbatV = (uint8_t) (vbatMV / 100);

    return vbatV;
}



static void cflie_process_logdata_ack(void)
{
   

    if (rx_payload_len == 0) return;



//     // CRTP header 
    if (rx_payload_len >= 3 && rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_LOGDATA) && rx_packet[1] == CFLIE_TELEM_LOG_BLOCK_ID) {
		
		v_lipo2 = getVbatV(rx_packet[5], rx_packet[6]);
        

        telemetry_link = 1; 
         
    }

    
    

      
}









static void CFLIE_RF_init()
{
    NRF24L01_Initialize();

    // CRC, radio on
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x01);              // Auto Acknowledgement for data pipe 0
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x13);         // 3 retransmits, 500us delay

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num);        // Defined in initialize_rx_tx_addr
    NRF24L01_SetBitrate(data_rate);                          // Defined in initialize_rx_tx_addr

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, TX_ADDR_SIZE);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, TX_ADDR_SIZE);

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x01);       // Enable Dynamic Payload Length on pipe 0
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x06);     // Enable Dynamic Payload Length, enable Payload with ACK

	NRF24L01_SetTxRxMode(TX_EN);						// Clear data ready, data sent, retransmit and enable CRC 16bits, ready for TX
}



static uint16_t CFLIE_callback()
{
    switch (phase) {
    case CFLIE_INIT_SEARCH:
        send_search_packet();
        phase = CFLIE_SEARCH;
        break;
    case CFLIE_INIT_CRTP_LOG:
        if (crtp_log_setup_state_machine()) {
            phase = CFLIE_INIT_DATA;
        }
        break;
    case CFLIE_INIT_DATA:
        send_cmd_packet();
        phase = CFLIE_DATA;
        break;
    case CFLIE_SEARCH:
        switch (packet_ack()) {
        case PKT_PENDING:
            return PACKET_CHKTIME;                 // packet send not yet complete
        case PKT_ACKED:
            phase = CFLIE_INIT_CRTP_LOG;
            // PROTOCOL_SetBindState(0);
            // MUSIC_Play(MUSIC_DONE_BINDING);
            BIND_DONE;
            break;
        case PKT_TIMEOUT:
            send_search_packet();
        }
        break;

    case CFLIE_DATA:

        if (packet_ack() == PKT_PENDING)
            return PACKET_CHKTIME;         // packet send not yet complete

        cflie_process_logdata_ack();


        send_cmd_packet();

 

        break;
    }
    return CFLIE_PACKET_PERIOD;                  // Packet at standard protocol interval
    
}

// Generate address to use from TX id and manufacturer id (STM32 unique id)
static uint8_t CFLIE_initialize_rx_tx_addr()
{
    rx_tx_addr[0] = 
    rx_tx_addr[1] = 
    rx_tx_addr[2] = 
    rx_tx_addr[3] = 0xE7;
	
	unsigned x10 = (RX_num / 10U) % 10;
	unsigned x1 = RX_num - x10*10;
    
    switch (sub_protocol) {
    case CFLIE_2Mbps:
      data_rate = NRF24L01_BR_2M;
      rf_ch_num = option; // "RF channel" in the transmitter <0, 125>
	  rx_tx_addr[4] = x10*16 + x1; // "Receiver" in the transmitter <0, 63>
      break;
    case CFLIE_1Mbps:
      data_rate = NRF24L01_BR_1M;
      rf_ch_num = option; // "RF channel" in the transmitter <0, 125>
	  rx_tx_addr[4] = x10*16 + x1; // "Receiver" in the transmitter <0, 63>
      break;  
    case CFLIE_250kbps:
      data_rate = NRF24L01_BR_250K;
      rf_ch_num = option; // "RF channel" in the transmitter <0, 125>
	  rx_tx_addr[4] = x10*16 + x1; // "Receiver" in the transmitter <0, 63>
      break;
    default:  
      data_rate = NRF24L01_BR_2M;
      rf_ch_num = 80;
	  rx_tx_addr[4] = 0xE7; // CFlie uses fixed address
    }
    
    return CFLIE_INIT_SEARCH;
}

void CFLIE_init(void)
{
	BIND_IN_PROGRESS;	// autobind protocol

  phase = CFLIE_initialize_rx_tx_addr();
  crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_INIT;
  packet_count=0;

  CFLIE_RF_init();
}

#endif


// Zaměřit se na zobrazení packetu, ktere mi napřimo přijdou z TOC - jedna proměnná, natvrdo, vyčítání přímo ID, ne hlednání v tabulce