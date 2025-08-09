
/*
DALA CAN Bridge rev. 2.5
3-bus transparent CAN bridge with Battery upgrade SW for Nissan env200
*/

#include "can-bridge-env200.h"

/* Optional functionality */
#define USB_SERIAL
#define ENABLE_CAN3 
uint8_t	output_can_to_serial = 1;	//specifies if the received CAN messages should be dumped out to serial. NOTE: Increases CPU load drastically!
#define ID_BMS_REQUEST_CELL_INFO 0x388
#define ID_BMS_STATUS 0x382
#define ID_BMS_VOLTAGE 0x383
#define ID_BMS_FAULT 0x385
#define ID_CSM_CURRENT 0x514
#define CSM_ERR_MEMORY 0x40
#define CSM_ERR_OVERCURRENT 0x41
#define CSM_ERR_NO_OSCILLATION 0x42
#define CSM_ERR_SUPPLY_RANGE 0x46
#define CSM_ERR_ADC_DEFAULT 0x47
#define CSM_ERR_NO_DATA 0x49
#define CSM_ERR_DAC_DEFAULT 0x4A
#define CSM_ERR_REF_DEFAULT 0x4B
#define EXTENDER_FULL_GIDS_DEFAULT 450      //36 kWh extender pack at 80 Wh per GID


//General variables
#ifdef EXTENDER_FULL_GIDS_OVERRIDE
uint16_t        extender_full_gids            = EXTENDER_FULL_GIDS_OVERRIDE;
#else
uint16_t        extender_full_gids            = EXTENDER_FULL_GIDS_DEFAULT;
#endif
volatile	uint16_t	main_battery_soc        = 0;
volatile        uint16_t        total_battery_soc       = 0; 
volatile        uint16_t        GIDS                            = 0;    //total GIDs (main + extender)
volatile        uint16_t        main_pack_gids          = 0; 
volatile	uint8_t		can_busy			= 0;	//Tracks whether the can_handler() subroutine is running
volatile	uint16_t	sec_timer			= 1;	//Counts down from 1000
//Battery measurement variables
volatile        int32_t         pack_current_mA                = 0;
volatile        uint16_t        pack_voltage_mV                = 0;
volatile        int64_t         coulomb_count_mAs              = 0;
volatile        uint8_t         csm_error                      = 0;
//Energy tracking
volatile        int64_t         extender_energy_mWh           = 0;
volatile        int32_t         Extender_gids                 = 0;
volatile        uint16_t        extender_soc                  = 0;

//CAN message templates
volatile	uint8_t		battery_can_bus			= 2; //keeps track on which CAN bus the battery talks
static	can_frame_t		instrumentCluster5E3	= {.can_id = 0x5E3, .can_dlc = 5, .data = {0x8E,0x00,0x00,0x00,0x80}};
static	can_frame_t		ZE1startupMessage603	= {.can_id = 0x603, .can_dlc = 1, .data = {0x00}};
static	can_frame_t		ZE1startupMessage605	= {.can_id = 0x605, .can_dlc = 1, .data = {0x00}};
static	can_frame_t		ZE1message355			= {.can_id = 0x355, .can_dlc = 8, .data = {0x14,0x0a,0x13,0x97,0x10,0x00,0x40,0x00}}; 
volatile	uint8_t		ticks10ms				= 0;
static		can_frame_t	ZE1message5C5			= {.can_id = 0x5C5, .can_dlc = 8, .data = {0x84,0x01,0x06,0x79,0x00,0x0C,0x00,0x00}}; 
static  can_frame_t     bms_request388            = {.can_id = ID_BMS_REQUEST_CELL_INFO, .can_dlc = 4, .data = {0x00,0x00,0x01,0x04}};

//Because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
can_frame_t tx0_buffer[TXBUFFER_SIZE];
uint8_t		tx0_buffer_pos		= 0;
uint8_t		tx0_buffer_end		= 0;

can_frame_t tx2_buffer[TXBUFFER_SIZE];
uint8_t		tx2_buffer_pos		= 0;
uint8_t		tx2_buffer_end		= 0;

can_frame_t tx3_buffer[5]; 
uint8_t		tx3_buffer_pos		= 0;
uint8_t		tx3_buffer_end		= 0;

#ifdef USB_SERIAL
#include "usb-hub-sensor.h"
uint8_t ReadCalibrationByte( uint8_t index );
void ProcessCDCCommand(void);

uint8_t		configSuccess				= false;		//tracks whether device successfully enumerated
static FILE USBSerialStream;							//fwrite target for CDC
uint8_t		signature[11];								//signature bytes
//print variables
volatile	uint8_t		print_char_limit		= 0;
#endif

void hw_init(void){
	uint8_t caninit;

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, 48000000);		
	
	//turn off everything we don' t use
	PR.PRGEN		= PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
	PR.PRPA			= PR_ADC_bm | PR_AC_bm;
	PR.PRPC			= PR_TWI_bm | PR_USART0_bm | PR_HIRES_bm;
	PR.PRPD			= PR_TWI_bm | PR_USART0_bm | PR_TC0_bm | PR_TC1_bm;
	PR.PRPE			= PR_TWI_bm | PR_USART0_bm;
	
	//blink output
	PORTB.DIRSET	= 3;
	
	//start 16MHz crystal and PLL it up to 48MHz
	OSC.XOSCCTRL	= OSC_FRQRANGE_12TO16_gc |		//16MHz crystal
	OSC_XOSCSEL_XTAL_16KCLK_gc;						//16kclk startup
	OSC.CTRL	   |= OSC_XOSCEN_bm;				//enable crystal
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));			//wait until ready
	OSC.PLLCTRL		= OSC_PLLSRC_XOSC_gc | 2;		//XTAL->PLL, 2x multiplier (32MHz)
	OSC.CTRL	   |= OSC_PLLEN_bm;					//start PLL
	while (!(OSC.STATUS & OSC_PLLRDY_bm));			//wait until ready
	CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
	CLK.CTRL		= CLK_SCLKSEL_PLL_gc;			//use PLL output as system clock	
	
	//output 16MHz clock to MCP25625 chips (PE0)
	//next iteration: put this on some other port, pin  4 or 7, so we can use the event system
	TCE0.CTRLA		= TC0_CLKSEL_DIV1_gc;						//clkper/1
	TCE0.CTRLB		= TC0_CCAEN_bm | TC0_WGMODE_SINGLESLOPE_bm;	//enable CCA, single-slope PWM
	TCE0.CCA		= 1;										//compare value
	TCE0.PER		= 1;										//period of 1, generates 24MHz output
	
	PORTE.DIRSET	= PIN0_bm;									//set CLKOUT pin to output
	
	//setup CAN pin interrupts
	PORTC.INTCTRL	= PORT_INT0LVL_HI_gc;
	PORTD.INTCTRL	= PORT_INT0LVL_HI_gc | PORT_INT1LVL_HI_gc;	
	
	PORTD.INT0MASK	= PIN0_bm;						//PORTD0 has can1 interrupt
	PORTD.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	PORTD.INT1MASK	= PIN5_bm;						//PORTD5 has can2 interrupt
	PORTD.PIN5CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	#ifdef ENABLE_CAN3
	PORTC.INT0MASK	= PIN2_bm;						//PORTC2 has can3 interrupt
	PORTC.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	#endif
	
	//buffer checking interrupt
	TCC1.CTRLA		= TC0_CLKSEL_DIV1_gc;			//32M/1/3200 ~ 100usec
	TCC1.PER		= 3200;
	TCC1.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//same priority as can interrupts
	
	//we want to optimize performance, so we're going to time stuff
	//48MHz/48=1us timer, which we just freerun and reset whenever we want to start timing something
	//frame time timer
	TCC0.CTRLA		= TC0_CLKSEL_DIV1_gc;
	TCC0.PER		= 32000;						//32MHz/32000=1ms
	TCC0.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//interrupt on overflow
	
	PORTB.OUTCLR	= (1 << 0);
	
	can_system_init:
			
	//Init SPI and CAN interface:
	if(RST.STATUS & RST_WDRF_bm){ //if we come from a watchdog reset, we don't need to setup CAN
		caninit = can_init(MCP_OPMOD_NORMAL, 1); //on second thought, we do
	} else {
		caninit = can_init(MCP_OPMOD_NORMAL, 1);
	}
	
	if(caninit){		
		//PORTB.OUTSET |= (1 << 0);					//green LED, uncommented to save power
	} else {		
		//PORTB.OUTSET |= (1 << 1);					//red LED, uncommented to save power
		_delay_ms(10);
		goto can_system_init;
	}
	
	//Set and enable interrupts with round-robin
	XMEGACLK_CCP_Write((void * ) &PMIC.CTRL, PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);//PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm| PMIC_HILVLEN_bm;
	
	USB_Init(USB_OPT_RC32MCLKSRC | USB_OPT_BUSEVENT_PRILOW);
	#ifdef USB_SERIAL
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	#endif
	
	wdt_enable(WDTO_15MS);
	
	sei();
}

int main(void){
	#ifdef USB_SERIAL
	char * str = "";
	#endif
	
	hw_init();

	while(1){
		//Setup complete, wait for CAN messages to trigger interrupts
		#ifdef USB_SERIAL
		//when USB is essentially unused, we output general status info
		if(!output_can_to_serial){
			if(sec_interrupt){
				sec_interrupt = 0;
			
				/*//current shifter state
				str = "Shift: 00\n";
				int_to_hex((char *) (str + 7), shifter_state);
				print(str, 10);*/
			}
		}
		#endif
	}
}
#ifdef USB_SERIAL
/* services commands received over the virtual serial port */
void ProcessCDCCommand(void)
{
	uint16_t	ReportStringLength = 0;
	char *		ReportString = "";
	int16_t cmd = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	
	if(cmd > -1){
		switch(cmd){
			case 48: //0
			break;
			
			case 0: //reset
			case 90: //'Z'
			_delay_ms(1000);
			CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
			RST.CTRL		= RST_SWRST_bm;
			break;
			
			case 255: //send identity
			ReportString = "DALA CAN bridge - v2.5 Leaf\n"; ReportStringLength = 28;
			break;
			
			default: //when all else fails
			ReportString = "Unrecognized Command:   \n"; ReportStringLength = 25;
			ReportString[22] = cmd;
			break;
		}
		
		if(ReportStringLength){
			print(ReportString, ReportStringLength);
		}
		
	}
}
#endif
// Event handler for the LUFA library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void){}

void EVENT_USB_Device_Connect(void){}
#ifdef USB_SERIAL
// Event handler for the LUFA library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void){ configSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface); }

// Event handler for the LUFA library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(void){	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface); }

//appends string to ring buffer and initiates transmission
void print(char * str, uint8_t len){
	if((print_char_limit + len) <= 120){
		fwrite(str, len, 1, &USBSerialStream);
		print_char_limit += len;
	} else {
		fwrite("X\n",2,1,&USBSerialStream);
	}
}
#ifdef USB_SERIAL
void print_measurements(void){
    char msg[] = "I+0000 V0000 G00000\n";
    int cur = pack_current_mA;
    if(cur < 0){ msg[1] = '-'; cur = -cur; } else { msg[1] = '+'; }
    if(cur > 9999) cur = 9999;
    int_to_4digit_nodec(cur, msg + 2);
    int_to_4digit_nodec(pack_voltage_mV, msg + 8);
    int_to_5digit(Extender_gids, msg + 14);
    if(output_can_to_serial) print(msg, 19);
}

static void print_csm_error(uint8_t err){
    if(!output_can_to_serial || err == 0) return;
    char msg[] = "CSM ERR 0x00\n";
    msg[8] = hex_to_ascii((err >> 4) & 0xF);
    msg[9] = hex_to_ascii(err & 0xF);
    print(msg, 11);
}
#endif
#endif

static void update_total_soc(void){
    uint16_t main_full_gids = main_battery_soc ? ((uint32_t)main_pack_gids * 100) / main_battery_soc : main_pack_gids;
    uint16_t total_full_gids = main_full_gids + extender_full_gids;
    total_battery_soc = total_full_gids ? ((uint32_t)GIDS * 100) / total_full_gids : 0;
}

static void coulomb_tick(void){
    coulomb_count_mAs += (int64_t)pack_current_mA * 10;
    extender_energy_mWh += ((int64_t)pack_current_mA * pack_voltage_mV * 10) / 3600000000LL;
    Extender_gids = extender_energy_mWh / 80000;
    GIDS = main_pack_gids + Extender_gids;
    extender_soc = extender_full_gids ? (Extender_gids * 100) / extender_full_gids : 0;
    update_total_soc();
    send_can(battery_can_bus, bms_request388);
}

static void update_qc_capacity(can_frame_t *frame){
    uint16_t main_full_gids = main_battery_soc ? ((uint32_t)main_pack_gids * 100) / main_battery_soc : main_pack_gids;
    uint16_t total_full_gids = main_full_gids + extender_full_gids;
    uint16_t temp = ((uint32_t)230 * GIDS) / total_full_gids;
    frame->data[3] = (frame->data[3] & 0xF0) | ((temp >> 5) & 0xF);
    frame->data[4] = ((temp & 0x1F) << 3) | (frame->data[4] & 0x07);
    calc_crc8(frame);
}



//fires every 1ms
ISR(TCC0_OVF_vect){
        wdt_reset(); //Reset the watchdog
        sec_timer--; //Increment the 1000ms timer

        static uint8_t ms10 = 0;
        if(++ms10 >= 10){
                ms10 = 0;
                coulomb_tick();
        }

	#ifdef USB_SERIAL
	if(!can_busy) ProcessCDCCommand();
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
	//handle second print buffer
	if(print_char_limit <= 64) { print_char_limit = 0; }
	else { print_char_limit -= 64; }
	#endif
	
	//fires every second (1000ms tasks go here)
	if(sec_timer == 0){
		PORTB.OUTCLR = (1 << 1);
	}
}

//fires approx. every 100us
ISR(TCC1_OVF_vect){
	check_can1();
	check_can2();
	check_can3();
}

//can1 interrupt
ISR(PORTD_INT0_vect){
	can_busy = 1;
	can_handler(1);
}

//can2 interrupt
ISR(PORTD_INT1_vect){
	can_busy = 1;
	can_handler(2);
}

//can3 receive interrupt
ISR(PORTC_INT0_vect){
	can_busy = 1;
	can_handler(3);
}

//CAN handler, manages reading data from received CAN messages 
void can_handler(uint8_t can_bus){
	can_frame_t frame;
	uint16_t temp; //Temporary variable
	
	char strbuf[] = "1|   |                \n";
	if(can_bus == 2){ strbuf[0] = 50; }
	if(can_bus == 3){ strbuf[0] = 51; }
	
	uint8_t flag = can_read(MCP_REG_CANINTF, can_bus);
		
	if (flag & (MCP_RX0IF | MCP_RX1IF)){

		if(flag & MCP_RX1IF){ //prioritize the rollover buffer
			can_read_rx_buf(MCP_RX_1, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX1IF, 0x00, can_bus);
			} else {
			can_read_rx_buf(MCP_RX_0, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX0IF, 0x00, can_bus);
		}
		
		switch(frame.can_id){
                       case 0x1F2:
                                //Upon reading VCM originating 0x1F2 every 10ms, send the missing message to battery every 40ms
                                ticks10ms++;
                                if(ticks10ms > 3)
                                {
                                        ticks10ms = 0;
                                        send_can(battery_can_bus, ZE1message355);
                                }

                       break;
                       case 0x55B:
                               main_battery_soc = (frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6); //Capture SOC% needed for QC_rescaling
                               main_battery_soc /= 10; //Remove decimals, 0-100 instead of 0-100.0
                               update_total_soc();

                                battery_can_bus = can_bus; //Check on what side of the CAN-bridge the battery is connected to

                                //Upon reading 0x55B coming from battery every 100ms, send missing messages towards battery
                                send_can(battery_can_bus, instrumentCluster5E3);
                                send_can(battery_can_bus, ZE1message5C5);

                        break;
			case 0x5BC:
                                if((frame.data[5] & 0x10) == 0x00)
                                { //LB_MAXGIDS is 0, capture pack GIDs
                                        main_pack_gids = ((frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6));
                                }

                               GIDS = main_pack_gids + Extender_gids; //report combined GID count
                               update_total_soc();

                                //Avoid blinking GOM by always writing remaining GIDS
                                frame.data[0] = GIDS >> 2;
                                frame.data[1] = (GIDS << 6) & 0xC0;
                                frame.data[5] &= 0x03; //Clear LB_Output_Power_Limit_Reason and LB_MaxGIDS
			break;
                       case 0x59E:   // QC capacity message
                                update_qc_capacity(&frame);
                        break;
                        case ID_BMS_STATUS:
                                pack_voltage_mV = 0;
                        break;
                        case ID_BMS_VOLTAGE:
                                temp = (frame.data[2] << 8) | frame.data[3];
                                pack_voltage_mV += ((uint32_t)temp * 5000) / 65535;
                                temp = (frame.data[4] << 8) | frame.data[5];
                                pack_voltage_mV += ((uint32_t)temp * 5000) / 65535;
                        break;
                        case ID_BMS_FAULT:
                                print_measurements();
                        break;
                        case ID_CSM_CURRENT:
                                pack_current_mA = (((int32_t)frame.data[0] << 24) | ((int32_t)frame.data[1] << 16) | ((int32_t)frame.data[2] << 8) | frame.data[3]) - 0x80000000;
                                if(frame.data[4] & 0x01){
                                        uint8_t err = frame.data[4] >> 1;
                                        if(err != csm_error){
                                                csm_error = err;
                                                print_csm_error(err);
                                        }
                                }
                                print_measurements();
                        break;
                        case 0x679: //Send missing 2018+ startup messages towards battery
                                send_can(battery_can_bus, ZE1startupMessage603);
                                send_can(battery_can_bus, ZE1startupMessage605);
                        break;
		default:
			break;
			}
			
			
		//block unwanted messages
			uint8_t blocked = 0;
			switch(frame.can_id){
				case 0xABC:	//Example on block
					blocked = 1;
					break;
					break;
				default:
					blocked = 0;
					break;
			}
			if(!blocked){
				if(can_bus == 1){send_can2(frame);} else {send_can1(frame);}
				
				if(output_can_to_serial){
					SID_to_str(strbuf + 2, frame.can_id);
					canframe_to_str(strbuf + 6, frame);
					#ifdef USB_SERIAL
					print(strbuf,23);
					#endif
				}
			}
		}
		
		
		
		if(flag & 0xA0){
			uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
			if(flag2 & 0xC0){
				can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors
				//ReportString = "CANX RX OVF\n";
				//ReportString[3] = 48 + can_bus;
				//print(ReportString,12);
			}
			if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
			if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);	}
		}
		can_busy = 0;
}


void send_can(uint8_t can_bus, can_frame_t frame){
	if(can_bus == 1) send_can1(frame);
	if(can_bus == 2) send_can2(frame);
	if(can_bus == 3) send_can3(frame);
}

void send_can1(can_frame_t frame){	
	//put in the buffer
	memcpy(&tx0_buffer[tx0_buffer_end++], &frame, sizeof(frame));
	
	if(tx0_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx0_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can1();
}



void check_can1(void){
	uint8_t reg;
	
	if(tx0_buffer_end != tx0_buffer_pos){
		//check if TXB0 is free use
		reg = can1_read(MCP_REG_TXB0CTRL);
	
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can1_load_txbuff(0, (can_frame_t *) &tx0_buffer[tx0_buffer_pos++]);
			can1_rts(0);
			if(tx0_buffer_pos == tx0_buffer_end){ //end of buffer, reset
				tx0_buffer_end = 0;
				tx0_buffer_pos = 0;
			}
		}
	}
}

void send_can2(can_frame_t frame){
	//put in the buffer
	memcpy(&tx2_buffer[tx2_buffer_end++], &frame, sizeof(frame));
	
	if(tx2_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx2_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can2();
}

void check_can2(void){
	uint8_t reg;
	
	if(tx2_buffer_end != tx2_buffer_pos){
		//check if TXB0 is free use
		reg = can2_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can2_load_txbuff(0, (can_frame_t *) &tx2_buffer[tx2_buffer_pos++]);
			can2_rts(0);
			if(tx2_buffer_pos == tx2_buffer_end){ //end of buffer, reset
				tx2_buffer_end = 0;
				tx2_buffer_pos = 0;
			}
		}
	}
}

void send_can3(can_frame_t frame){
	//put in the buffer
	memcpy(&tx3_buffer[tx3_buffer_end++], &frame, sizeof(frame));
	
	if(tx3_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx3_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can3();
}

void check_can3(void){
	uint8_t reg;
	
	if(tx3_buffer_end != tx3_buffer_pos){
		//check if TXB0 is free use
		reg = can3_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can3_load_txbuff(0, (can_frame_t *) &tx3_buffer[tx3_buffer_pos++]);
			can3_rts(0);
			if(tx3_buffer_pos == tx3_buffer_end){ //end of buffer, reset
				tx3_buffer_end = 0;
				tx3_buffer_pos = 0;
			}
		}
	}
}


