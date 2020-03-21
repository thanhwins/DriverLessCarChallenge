#include "Hal.h"
#include "LCDI2C.h"

#define SW1_PIN	160
#define SW2_PIN	161
#define SW3_PIN	163
#define SW4_PIN	164

using namespace EmbeddedFramework;

int main(int argc, char *argv[]) {
	GPIO *gpio = new GPIO();
	I2C *i2c_device = new I2C();
	LCDI2C *lcd = new LCDI2C();
	
	int sw1_stat = 1;
	int sw2_stat = 1;
	int sw3_stat = 1;
	int sw4_stat = 1;
	
	// Setup input
	gpio->gpioExport(SW1_PIN);
	gpio->gpioExport(SW2_PIN);
	gpio->gpioExport(SW3_PIN);
	gpio->gpioExport(SW4_PIN);
	
	gpio->gpioSetDirection(SW1_PIN, INPUT);
	gpio->gpioSetDirection(SW2_PIN, INPUT);
	gpio->gpioSetDirection(SW3_PIN, INPUT);
	gpio->gpioSetDirection(SW4_PIN, INPUT);
	
	i2c_device->m_i2c_bus = 2;
	
	if (!i2c_device->HALOpen()) {
		printf("Cannot open I2C peripheral\n");
		exit(-1);
	} else printf("I2C peripheral is opened\n");
	
	unsigned char data;
	if (!i2c_device->HALRead(0x38, 0xFF, 1, &data, "")) {
		printf("LCD is not found!\n");
		exit(-1);
	} else printf ("LCD is connected\n");
	
	usleep(10000);
	
	lcd->LCDInit(i2c_device, 0x38, 20, 4);
	lcd->LCDBacklightOn();
	lcd->LCDCursorOn();
	
	lcd->LCDSetCursor(0,2);
	lcd->LCDPrintStr("Line 3");
	lcd->LCDSetCursor(0,3);
	lcd->LCDPrintStr("Line 4");
	lcd->LCDSetCursor(14,1);
	lcd->LCDPrintStr("Hehehehehe");
	
	while(true) {
		unsigned int bt_status = 0;
		gpio->gpioGetValue(SW1_PIN, &bt_status);
		if (!bt_status) {
			if (bt_status != sw1_stat) {
				lcd->LCDSetCursor(0,0);
				sw1_stat = bt_status;
			}
		} else sw1_stat = bt_status;
		
		gpio->gpioGetValue(SW2_PIN, &bt_status);
		if (!bt_status) {
			if (bt_status != sw2_stat) {
				lcd->LCDSetCursor(1,0);
				sw2_stat = bt_status;
			}
		} else sw2_stat = bt_status;
		
		gpio->gpioGetValue(SW3_PIN, &bt_status);
		if (!bt_status) {
			if (bt_status != sw3_stat) {
				lcd->LCDSetCursor(2,0);
				sw3_stat = bt_status;
			}
		} else sw3_stat = bt_status;
		
		gpio->gpioGetValue(SW4_PIN, &bt_status);
		if (!bt_status) {
			if (bt_status != sw4_stat) {
				lcd->LCDSetCursor(3,0);
				sw4_stat = bt_status;
			}
		} else sw4_stat = bt_status;
	}
}
