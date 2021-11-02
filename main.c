#include "MKL46Z4.h"

#define LED_GREEN_POS (5)
#define LED_RED_POS (29)
#define SW1_POS (3)

#define MASK(x) (1UL << (x))


void delay(void)
{
	volatile int i;

	for (i = 0; i < 1000000; i++);
}

void led_green_init()
{
	SIM->COPC = 0;									// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;				// Conecta el reloj al puerto D
	PORTD->PCR[LED_GREEN_POS] = PORT_PCR_MUX(1);	// Configura los pines necesarios como GPIO
	GPIOD->PDDR |= (1 << LED_GREEN_POS);			// Se configura como pin de salida
	GPIOD->PSOR = (1 << LED_GREEN_POS);				// Se pone a 1 el pin de salida
}

void led_green_toggle()
{
	GPIOD->PTOR = (1 << LED_GREEN_POS);
}

void led_red_init()
{
	SIM->COPC = 0;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[LED_RED_POS] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= (1 << LED_RED_POS);
	GPIOE->PSOR = (1 << LED_RED_POS);
}

void led_red_toggle(void)
{
	GPIOE->PTOR = (1 << LED_RED_POS);
}

void sw1_init()
{
	SIM->COPC = 0;							// Desactiva el Watchdog
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;		// Conecta el reloj al puerto C

	PORTC->PCR[SW1_POS] |= PORT_PCR_MUX(1);	// Activa el GPIO
	PORTC->PCR[SW1_POS] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
	PORTC->PCR[SW1_POS] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown
}

long int sw1_pressed()
{
	return GPIOC->PDIR;
}

int main(void)
{
	led_green_init();
	led_red_init();
	sw1_init();

	led_red_toggle();

	while(1) {
		if(!(sw1_pressed())){
			led_red_toggle();
			led_green_toggle();
			delay();
		}
	}
	return 0;
}

