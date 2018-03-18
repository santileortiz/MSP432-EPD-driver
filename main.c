#include "msp.h"

#include "sample_image_2_7.xbm"

#define ARRAY(type, ...) ((type[]){__VA_ARGS__})
#define CU8(...) (ARRAY(uint8_t, __VA_ARGS__))

#define EPD_Pin_RESET(i) P6OUT=i?P6OUT|BIT4:P6OUT&~BIT4
#define EPD_Pin_PANEL_ON(i) P3OUT=i?P3OUT|BIT6:P3OUT&~BIT6
#define EPD_Pin_DISCHARGE(i) P5OUT=i?P5OUT|BIT2:P5OUT&~BIT2
#define EPD_Pin_BORDER(i) P5OUT=i?P5OUT|BIT0:P5OUT&~BIT0
#define EPD_Pin_EPD_CS(i) P2OUT=i?P2OUT|BIT5:P2OUT&~BIT5
#define EPD_Pin_BUSY P4IN&BIT6?1:0


void delay_ms (unsigned int n) {
	while (n>0) {
		TA0CCTL1 &= ~CCIFG;
		TA0CCR1 = TA0CCR1+3000;
		while (TA0CCTL1 & CCIFG);
		TA0CCTL1 &= ~CCIFG;
		n--;
	}
}

// NOTE: it won't be very accurate at 1.5MHz
void delay_us (char n) {
	switch (SystemCoreClock) {
	case 1500000:
		TA0CCR1 = n*3/2;
		break;
	case 3000000:
		TA0CCR1 = n*3;
		break;
	case 12000000:
		TA0CCR1 = n*12;
		break;
	case 24000000:
		TA0CCR1 = n*24;
		break;
	default: // 48MHz
		TA0CCR1 = n*48;
		break;
	}
	while (TA0CCTL1 & CCIFG);
	TA0CCTL1 &= ~CCIFG;
}

void SPI_put (uint8_t c) {
	UCB0TXBUF = c;
	while (UCB0STATW & UCBUSY);
}

uint8_t EPD_get_COG_ID (){
	// >80ns
	EPD_Pin_EPD_CS(0);
	SPI_put (0x71);
	SPI_put (0x00);
	EPD_Pin_EPD_CS(1);
	return UCB0RXBUF;
}

void EPD_power_off () {
	// turn of power and all signals
	EPD_Pin_BORDER(0);
	EPD_Pin_PANEL_ON(0);
	delay_ms(10);

	// ensure SPI MOSI and CLOCK are Low before CS Low
	SPI_put(0x00);
	SPI_put(0x00);
	EPD_Pin_EPD_CS(0);
	EPD_Pin_RESET(0);

	// pulse discharge pin
	EPD_Pin_DISCHARGE(1);
	delay_ms(150);
	EPD_Pin_DISCHARGE(0);
}

#define EPD_write_command(cmd, data) EPD_write_command_len(cmd,CU8(data),1)
void EPD_write_command_len (uint8_t cmd, uint8_t *data, uint8_t len) {
	// >80ns
	EPD_Pin_EPD_CS(0);
	SPI_put (0x70);
	SPI_put (cmd);
	EPD_Pin_EPD_CS(1);
	// >80ns
	EPD_Pin_EPD_CS(0);
	SPI_put (0x72);
	for (; 0 < len; len--){
		SPI_put (*data);
		data++;
	}
	EPD_Pin_EPD_CS(1);
}

uint8_t EPD_read_command (uint8_t cmd) {
	// >80ns
	EPD_Pin_EPD_CS(0);
	SPI_put (0x70);
	SPI_put (cmd);
	EPD_Pin_EPD_CS(1);
	// >80ns
	EPD_Pin_EPD_CS(0);
	SPI_put (0x73);
	SPI_put (0x00);
	EPD_Pin_EPD_CS(1);
	return UCB0RXBUF;
}

void EPD_startup(void) {
	// power up sequence
	EPD_Pin_RESET(0);
	EPD_Pin_PANEL_ON(0);
	EPD_Pin_DISCHARGE(0);
	EPD_Pin_BORDER(0);
	EPD_Pin_EPD_CS(0);

	SPI_put(0x00);
	SPI_put(0x00);

	delay_ms(5);
	EPD_Pin_PANEL_ON(1);
	delay_ms(10);

	EPD_Pin_EPD_CS(1);
	EPD_Pin_BORDER(1);
	EPD_Pin_RESET(1);

	delay_ms(5);

	EPD_Pin_RESET(0);
	delay_ms(5);

	EPD_Pin_RESET(1);
	delay_ms(5);

	// wait for COG to become ready
	while (EPD_Pin_BUSY);

	// read the COG ID
	int cog_id = EPD_get_COG_ID ();

	if (0x12 != cog_id) {
	//if (0xFF != cog_id) {
		// Error: EPD_UNSUPPORTED_COG;
		EPD_power_off ();
		return;
	}

	// Disable OE
	EPD_write_command (0x02, 0x40);

	// check breakage
	int broken_panel = EPD_read_command (0x0f);
	if (0x00 == (0x80 & broken_panel)) {
		// Error: EPD_PANEL_BROKEN;
		EPD_power_off();
		return;
	}

	// power saving mode
	EPD_write_command (0x0b, 0x02);

	// channel select
	EPD_write_command_len (0x01, CU8(0x00, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x00), 0x08);

	// high power mode osc
	EPD_write_command (0x07, 0xd1);

	// power setting
	EPD_write_command (0x08, 0x02);

	// Vcom level
	EPD_write_command (0x09, 0xc2);

	// power setting
	EPD_write_command (0x04, 0x03);

	// driver latch on
	EPD_write_command (0x03, 0x01);

	// driver latch off
	EPD_write_command (0x03, 0x00);

	delay_ms(5);

	uint8_t dc_ok = 0;

	int i;
	for (i = 0; i < 4; ++i) {
		// charge pump positive voltage on - VGH/VDL on
		EPD_write_command (0x05, 0x01);
		delay_ms(240);

		// charge pump negative voltage on - VGL/VDL on
		EPD_write_command (0x05, 0x03);
		delay_ms(40);

		// charge pump Vcom on - Vcom driver on
		EPD_write_command (0x05, 0x0f);
		delay_ms(40);

		// check DC/DC
		int dc_state = EPD_read_command (0x0f);
		if (0x40 == (0x40 & dc_state)) {
			dc_ok = 1;
			break;
		}
	}
	if (!dc_ok) {
		// Error: EPD_DC_FAILED;
		EPD_power_off();
		return;
	}
	// output enable to disable
	EPD_write_command (0x02, 0x40); //typo in original code?
	// EPD_write_command (0x02, 0x06);
	//SPI_off();
}

typedef enum {           // Image pixel -> Display pixel
	EPD_compensate,  // B -> W, W -> B (Current Image)
	EPD_white,       // B -> N, W -> W (Current Image)
	EPD_inverse,     // B -> N, W -> B (New Image)
	EPD_normal       // B -> B, W -> W (New Image)
} EPD_stage;

// output one line of scan and data bytes to the display,
// this is specific to 2.7" and has hardcoded values for it!
#define EPD_WIDTH 264
#define EPD_HEIGHT 176
void EPD_line(uint16_t line, const uint8_t *data, uint8_t fixed_value, EPD_stage stage) {

	//SPI_on();

	// send command index
	//delay_us(10);
	EPD_Pin_EPD_CS(0);
	SPI_put (0x70);
	SPI_put (0x0a);
	EPD_Pin_EPD_CS(1);
	//delay_us(10);

	// CS low
	EPD_Pin_EPD_CS (0);
	SPI_put(0x72);
	SPI_put(0x00); // Border byte

	// odd pixel data
	uint16_t b;
	for (b = EPD_WIDTH/8; b > 0; --b) {
		if (0 != data) {
			uint8_t pixels = data[b - 1] & 0x55;

			switch(stage) {
			case EPD_compensate:  // B -> W, W -> B (Current Image)
				pixels = 0xaa | (pixels ^ 0x55);
				break;
			case EPD_white:       // B -> N, W -> W (Current Image)
				pixels = 0x55 + (pixels ^ 0x55);
				break;
			case EPD_inverse:     // B -> N, W -> B (New Image)
				pixels = 0x55 | ((pixels ^ 0x55) << 1);
				break;
			case EPD_normal:       // B -> B, W -> W (New Image)
				pixels = 0xaa | pixels;
				break;
			}
			SPI_put(pixels);
		} else {
			SPI_put(fixed_value);
		}
	}

	// scan line bytes
	for (b = EPD_HEIGHT/4; b > 0; --b) {
		uint8_t n = 0x00;
		if (line / 4 == b - 1) {
			n = 0x03 << (2 * (line & 0x03));
		}
		SPI_put(n);
	}

	// even pixel data
	for (b = 0; b < EPD_WIDTH/8; ++b) {
		if (0 != data) {
			uint8_t pixels = data[b] & 0xaa;

			switch(stage) {
			case EPD_compensate:  // B -> W, W -> B (Current Image)
				pixels = 0xaa | ((pixels ^ 0xaa) >> 1);
				break;
			case EPD_white:       // B -> N, W -> W (Current Image)
				pixels = 0x55 + ((pixels ^ 0xaa) >> 1);
				break;
			case EPD_inverse:     // B -> N, W -> B (New Image)
				pixels = 0x55 | (pixels ^ 0xaa);
				break;
			case EPD_normal:       // B -> B, W -> W (New Image)
				pixels = 0xaa | (pixels >> 1);
				break;
			}
			uint8_t p1 = (pixels >> 6) & 0x03;
			uint8_t p2 = (pixels >> 4) & 0x03;
			uint8_t p3 = (pixels >> 2) & 0x03;
			uint8_t p4 = (pixels >> 0) & 0x03;
			pixels = (p1 << 0) | (p2 << 2) | (p3 << 4) | (p4 << 6);
			SPI_put(pixels);
		} else {
			SPI_put(fixed_value);
		}
	}

	// CS high
	EPD_Pin_EPD_CS (1);

	// output data to panel
	EPD_write_command (0x02, 0x07);

	//SPI_off();
}

void EPD_dummy_line() {
	EPD_line(0x7fffu, 0, 0x00, EPD_compensate);
}

void EPD_frame(const uint8_t *image, EPD_stage stage){
	uint8_t line;
	for (line = 0; line < EPD_HEIGHT ; ++line) {
		EPD_line(line, &image[line * EPD_WIDTH/8], 0, stage);
	}
}

void EPD_nothing_frame() {
	int line;
	for (line = 0; line < EPD_HEIGHT; ++line) {
		EPD_line(line, 0, 0x00, EPD_compensate);
	}
}

void EPD_image (const uint8_t *new_image) {
	//EPD_frame(image, EPD_compensate);
	//EPD_frame(image, EPD_white);
	EPD_frame(new_image, EPD_inverse);
	EPD_frame(new_image, EPD_normal);
}

void EPD_end(void) {

	EPD_nothing_frame();


	EPD_dummy_line();
	// only pulse border pin for 2.70" EPD
	delay_ms(25);
	EPD_Pin_BORDER(0);
	delay_ms(200);
	EPD_Pin_BORDER(0);

	//SPI_on();

	// ??? - not described in datasheet
	EPD_write_command (0x0b, 0x00);

	// latch reset turn on
	EPD_write_command (0x03, 0x01);

	// power off charge pump Vcom
	EPD_write_command (0x05, 0x03);

	// power off charge pump neg voltage
	EPD_write_command (0x05, 0x01);

	delay_ms(120);

	// discharge internal
	EPD_write_command (0x04, 0x80);

	// power off all charge pumps
	EPD_write_command (0x05, 0x00);

	// turn of osc
	EPD_write_command (0x07, 0x01);

	delay_ms(50);

	EPD_power_off();
}

void main(void)
{
	volatile uint32_t i;


	WDTCTL = WDTPW | WDTHOLD;                 // Stop watchdog timer

	////////////////////
	// EPD pinout setup

	P6DIR |= BIT4; // Pin_RESET
	P3DIR |= BIT6; // Pin_PANEL_ON
	P5DIR |= BIT2; // Pin_DISCHARGE
	P5DIR |= BIT0; // Pin_BORDER
	P2DIR |= BIT5; // Pin_EPD_CS

	P1SEL0 |= BIT5 | BIT6 | BIT7;            // set 3-SPI pin as second function

	//__enable_interrupt();
	//NVIC->ISER[0] = 1 << ((EUSCIB0_IRQn) & 31); // Enable eUSCIA3 interrupt in NVIC module

	///////////////////
	// Set up SPI:
	// 3-pin, 8-bit SPI master, Mode 0, MSB
	UCB0CTLW0 |= UCSWRST;
	UCB0CTLW0 |= UCMST | UCSYNC | UCCKPH | UCMSB;

	// Set up SPI Clock (Can't be higher that 16MHz)
	UCB0CTLW0 &= ~UCSSEL_M;
	UCB0CTLW0 |= UCSSEL__SMCLK;
	switch (SystemCoreClock) {
	case 1500000:
	case 3000000:
	case 12000000: // SPI_CLK == MCLK == __SYSTEM_CLOCK
		UCB0BR0 = 0x01;
		UCB0BR1 = 0;
		break;
	case 24000000: // SPI_CLK == 12MHz
		UCB0BR0 = 0x02;
		UCB0BR1 = 0;
		break;
	default: 	// SPI_CLK == 16MHz
		UCB0BR0 = 0x03;
		UCB0BR1 = 0;
		break;
	}
	UCB0CTLW0 &= ~UCSWRST; // Start SPI module

	//SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Wake up on exit from ISR
	//UCB0IE |= UCTXIE;                     // Enable TX interrupt

	///////////////////
	// Set up Timer A0:
	TA0CTL |= TASSEL__SMCLK|MC__CONTINUOUS;

	EPD_startup ();
	EPD_image (sample_image_2_7_bits);
	EPD_end ();
	while (1) {
		__sleep();
		//__no_operation();			   		// For debug,Remain in LPM0
	}
}

// SPI interrupt service routine
void EUSCIB0_IRQHandler(void)
{
	static uint8_t TXData;

    if (UCB0IFG & UCTXIFG)
    {
        UCB0TXBUF = TXData;                  // Transmit characters
        //UCB0IE &= ~UCTXIE;
		TXData++;                             // Increment transmit data
        //while (!(UCB0IFG&UCRXIFG));
        //RXData = UCB0RXBUF;
        UCB0IFG &= ~UCRXIFG;
    }
}
