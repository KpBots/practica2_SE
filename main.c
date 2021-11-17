#include "MKL46Z4.h"

#define LED_GREEN_POS (5)
#define LED_RED_POS (29)
#define SW1_POS (3)
#define SW2_POS (12)


// Led's
void led_green_init()
{
	SIM->COPC = 0;									// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;				// Conecta el reloj al puerto D
	PORTD->PCR[LED_GREEN_POS] = PORT_PCR_MUX(1);	// Configura los pines necesarios como GPIO
	GPIOD->PDDR |= (1 << LED_GREEN_POS);			// Se configura como pin de salida
	GPIOD->PSOR |= (1 << LED_GREEN_POS);			// Se pone a 1 el pin de salida
}

void led_green_toggle()
{
	GPIOD->PTOR |= (1 << LED_GREEN_POS);
}

void led_green_on(void)
{
	GPIOD->PCOR |= (1 << LED_GREEN_POS);
}

void led_green_off(void)
{
	GPIOD->PSOR |= (1 << LED_GREEN_POS);
}

void led_red_init()
{
	SIM->COPC = 0;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[LED_RED_POS] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1 << LED_RED_POS);
	GPIOE->PSOR |= (1 << LED_RED_POS);
}

void led_red_toggle(void)
{
	GPIOE->PTOR |= (1 << LED_RED_POS);
}

void led_red_on(void)
{
	GPIOE->PCOR |= (1 << LED_RED_POS);
}

void led_red_off(void)
{
	GPIOE->PSOR |= (1 << LED_RED_POS);
}

// Switches
void sw1_init()
{
	SIM->COPC = 0;							// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;		// Conecta el reloj al puerto C

	PORTC->PCR[SW1_POS] |= PORT_PCR_MUX(1);	// Activa el GPIO
	PORTC->PCR[SW1_POS] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
	PORTC->PCR[SW1_POS] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown
}

void sw2_init()
{
	SIM->COPC = 0;							// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;		// Conecta el reloj al puerto C

	PORTC->PCR[SW2_POS] |= PORT_PCR_MUX(1);	// Activa el GPIO
	PORTC->PCR[SW2_POS] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
	PORTC->PCR[SW2_POS] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown
}

int sw1_pressed()
{
	return GPIOC->PDIR == (1<<SW2_POS);
}

int sw2_pressed()
{
	return GPIOC->PDIR == (1<<SW1_POS);
}

// Main
int main(void)
{
	int status_sw1, status_sw2, system_status;
	led_green_init();
	led_red_init();
	sw1_init();
	sw2_init();

	status_sw1 = 0;
	status_sw2 = 0;
	system_status = 0;

	while(1) {
		if(sw1_pressed()){
			status_sw1 += 1;
			while(sw1_pressed())
				continue;
		}
		if(status_sw1 > 1)
			status_sw1 = 0;

		if(sw2_pressed()){
			status_sw2 += 1;
			while(sw2_pressed())
				continue;
		}
		if(status_sw2 > 1)
			status_sw2 = 0;


		/* System status
		 *
		 * Switch 1, switch 2, estado
		 * 00 = 0 Ambas puertas están cerradas
		 * 01 = 1 Puerta 2 abierta
		 * 10 = 2 Puerta 1 abierta
		 * 11 = 3 Ambas puertas están abiertas
		 */
	 	system_status = status_sw1 + (status_sw2 * 2);
		
		switch(system_status) {
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
