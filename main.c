#include "MKL46Z4.h"

#define LED_GREEN_POS (5)
#define LED_RED_POS (29)
#define SW1_POS (3)
#define SW2_POS (12)

int STATUS_SW1 = 0;
int STATUS_SW2 = 0;
int SYSTEM_STATUS = -1;

// Led's
void led_green_init() {
	SIM->COPC = 0;									// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;				// Conecta el reloj al puerto D
	PORTD->PCR[LED_GREEN_POS] = PORT_PCR_MUX(1);	// Configura los pines necesarios como GPIO
	GPIOD->PDDR |= (1 << LED_GREEN_POS);			// Se configura como pin de salida
	GPIOD->PSOR |= (1 << LED_GREEN_POS);			// Se pone a 1 el pin de salida
}

void led_green_on(void) {
	GPIOD->PCOR |= (1 << LED_GREEN_POS);
}

void led_green_off(void) {
	GPIOD->PSOR |= (1 << LED_GREEN_POS);
}

void led_red_init() {
	SIM->COPC = 0;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[LED_RED_POS] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1 << LED_RED_POS);
	GPIOE->PSOR |= (1 << LED_RED_POS);
}

void led_red_on(void) {
	GPIOE->PCOR |= (1 << LED_RED_POS);
}

void led_red_off(void) {
	GPIOE->PSOR |= (1 << LED_RED_POS);
}


// Switches
void sw1_init() {
	SIM->COPC = 0;							// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;		// Conecta el reloj al puerto C

	PORTC->PCR[SW1_POS] |= PORT_PCR_MUX(1);	// Activa el GPIO
	PORTC->PCR[SW1_POS] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
	PORTC->PCR[SW1_POS] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

	// IRQ
	PORTC->PCR[SW1_POS] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
	NVIC_SetPriority(31, 0);	// Prioridad de la interrupcion 31
	NVIC_EnableIRQ(31);			// Activa la interrupcion
}

void sw2_init() {
	SIM->COPC = 0;							// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;		// Conecta el reloj al puerto C

	PORTC->PCR[SW2_POS] |= PORT_PCR_MUX(1);	// Activa el GPIO
	PORTC->PCR[SW2_POS] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
	PORTC->PCR[SW2_POS] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

	// IRQ
	PORTC->PCR[SW2_POS] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
	NVIC_SetPriority(31, 0);	// Prioridad de la interrupcion 31
	NVIC_EnableIRQ(31);			// Activa la interrupcion
}

void PORTDIntHandler(void) {
	int pressed_switch = PORTC->ISFR;
	PORTC->ISFR = 0xFFFFFFFF;	// Clear IRQ

  	// SW1
	if(pressed_switch == (0x8)) {
		STATUS_SW1 += 1;
		if(STATUS_SW1 > 1) {
			STATUS_SW1 = 0;
		}
	}

	// SW2
	if(pressed_switch == (0x1000)) {
		STATUS_SW2 += 1;
		if(STATUS_SW2 > 1) {
			STATUS_SW2 = 0;
		}
	}

	SYSTEM_STATUS = STATUS_SW1 + (STATUS_SW2 * 2);
}


// Main
int main(void) {
	led_green_init();
	led_red_init();
	sw1_init();
	sw2_init();

	while(1) {
		/* System status
		 *
		 * Switch 1, switch 2, estado
		 * 00 = 0 Ambas puertas están cerradas
		 * 01 = 1 Puerta 2 abierta
		 * 10 = 2 Puerta 1 abierta
		 * 11 = 3 Ambas puertas están abiertas
		 */
		switch(SYSTEM_STATUS) {
			case 0:
				led_green_on();
				led_red_off();
				break;
			case 1:
				led_green_off();
				led_red_on();
				break;
			case 2:
				led_green_off();
				led_red_on();
				break;
			case 3:
				led_green_off();
				led_red_on();
				break;
		}
	}
	return 0;
}
