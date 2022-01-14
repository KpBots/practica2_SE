#include "MKL46Z4.h"

#define LED_GREEN_POS (5)
#define LED_RED_POS (29)
#define SW1_POS (3)
#define SW2_POS (12)
#define RIGHT_MOTOR_FRONT (5)
#define RIGHT_MOTOR_BACK (4)
#define LEFT_MOTOR_FRONT (8)
#define LEFT_MOTOR_BACK (9)
#define RIGHT_IR (3)
#define LEFT_IR (12)

const int CORE_CLOCK = 48000000;

int STATUS_SW1 = 0;
int STATUS_SW2 = 0;
int BUTTONS_STATUS = 0;


// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// RIGHT (SW1) = PTC3 (pin 73)
// LEFT (SW2) = PTC12 (pin 88)

/*** The following pins are named following the classic layout of an Arduino UNO ***/
// MOTORS
// RIGHT_MOTOR_FRONT = PTA4 (Digital pin 4)
// RIGHT_MOTOR_BACK = PTA5 (Digital pin 5)
// LEFT_MOTOR_FRONT = PTC8 (Digital pin 6)
// LEFT_MOTOR_BACK = PTC9 (Digital pin 7)

// IR SENSORS
// RIGHT_IR = PTD3 (Digital pin 2)
// LEFT_IR = PTA12 (Digital pin 3)


// Enable IRCLK (Internal Reference Clock)
// see Chapter 24 in MCU doc
void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

int delay(int duration)
{
  volatile int i;

  for (i = 0; i < duration; i++){}
  return i;
}


// Led's
void led_green_init()
{
  SIM->COPC = 0;                  // Desactiva el Watchdog
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;       // Conecta el reloj al puerto D
  PORTD->PCR[LED_GREEN_POS] = PORT_PCR_MUX(1);  // Configura los pines necesarios como GPIO
  GPIOD->PDDR |= (1 << LED_GREEN_POS);      // Se configura como pin de salida
  GPIOD->PSOR |= (1 << LED_GREEN_POS);      // Se pone a 1 el pin de salida
}

void led_green_on(void)
{
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


// Motors
void right_motor_init()
{
  SIM->COPC = 0;                            // Desactiva el Watchdog
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;       // Conecta el reloj al puerto A
  PORTA->PCR[RIGHT_MOTOR_FRONT] = PORT_PCR_MUX(1); // Configura los pines necesarios como GPIO
  PORTA->PCR[RIGHT_MOTOR_BACK] = PORT_PCR_MUX(1);  // Configura los pines necesarios como GPIO
  GPIOA->PDDR |= (1 << RIGHT_MOTOR_FRONT);  // Se configura como pin de salida
  GPIOA->PSOR |= (1 << RIGHT_MOTOR_FRONT);  // Se pone a 1 el pin de salida
  GPIOA->PDDR |= (1 << RIGHT_MOTOR_BACK);   // Se configura como pin de salida
  GPIOA->PSOR |= (1 << RIGHT_MOTOR_BACK);   // Se pone a 1 el pin de salida
}

void right_motor_on(void)
{
  GPIOA->PCOR |= (1 << RIGHT_MOTOR_FRONT);
  GPIOA->PSOR |= (1 << RIGHT_MOTOR_BACK);
}

void right_motor_off(void) {
  GPIOA->PSOR |= (1 << RIGHT_MOTOR_FRONT);
  GPIOA->PSOR |= (1 << RIGHT_MOTOR_BACK);
}

void left_motor_init()
{
  SIM->COPC = 0;                            // Desactiva el Watchdog
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;       // Conecta el reloj al puerto A
  PORTC->PCR[LEFT_MOTOR_FRONT] = PORT_PCR_MUX(1); // Configura los pines necesarios como GPIO
  PORTC->PCR[LEFT_MOTOR_BACK] = PORT_PCR_MUX(1);  // Configura los pines necesarios como GPIO
  GPIOC->PDDR |= (1 << LEFT_MOTOR_FRONT);  // Se configura como pin de salida
  GPIOC->PSOR |= (1 << LEFT_MOTOR_FRONT);  // Se pone a 1 el pin de salida
  GPIOC->PDDR |= (1 << LEFT_MOTOR_BACK);   // Se configura como pin de salida
  GPIOC->PSOR |= (1 << LEFT_MOTOR_BACK);   // Se pone a 1 el pin de salida
}

void left_motor_on(void)
{
  GPIOC->PCOR |= (1 << LEFT_MOTOR_FRONT);
  GPIOC->PSOR |= (1 << LEFT_MOTOR_BACK);
}

void left_motor_off(void) {
  GPIOC->PSOR |= (1 << LEFT_MOTOR_FRONT);
  GPIOC->PSOR |= (1 << LEFT_MOTOR_BACK);
}


// Switches
void sw1_init() {
  SIM->COPC = 0;              // Desactiva el Watchdog
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;   // Conecta el reloj al puerto C

  PORTC->PCR[SW1_POS] |= PORT_PCR_MUX(1); // Activa el GPIO
  PORTC->PCR[SW1_POS] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
  PORTC->PCR[SW1_POS] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

  // IRQ
  PORTC->PCR[SW1_POS] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
  NVIC_SetPriority(31, 0);  // Prioridad de la interrupcion 31
  NVIC_EnableIRQ(31);     // Activa la interrupcion
}

void sw2_init() {
  SIM->COPC = 0;              // Desactiva el Watchdog
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;   // Conecta el reloj al puerto C

  PORTC->PCR[SW2_POS] |= PORT_PCR_MUX(1); // Activa el GPIO
  PORTC->PCR[SW2_POS] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
  PORTC->PCR[SW2_POS] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown

  // IRQ
  PORTC->PCR[SW2_POS] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
  NVIC_SetPriority(31, 0);  // Prioridad de la interrupcion 31
  NVIC_EnableIRQ(31);     // Activa la interrupcion
}

// Switches IRQs
void PORTDIntHandler(void) {
  // Buttons
  int pressed_switch = PORTC->ISFR;
  PORTC->ISFR = 0xFFFFFFFF;

  // SW1
  if(pressed_switch == (0x8)) {
    ++STATUS_SW1;
  }

  // SW2
  if(pressed_switch == (0x1000)) {
    ++STATUS_SW2;
  }

  BUTTONS_STATUS = STATUS_SW1 + STATUS_SW2;
}


// IR SENSORS
// RIGHT_IR = PTD3 (Digital pin 2)
// LEFT_IR = PTA12 (Digital pin 3)
void right_ir_init() {
  SIM->COPC = 0;                        // Desactiva el Watchdog
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;   // Conecta el reloj al puerto D

  PORTD->PCR[RIGHT_IR] |= PORT_PCR_MUX(1); // Activa el GPIO
  PORTD->PCR[RIGHT_IR] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
  PORTD->PCR[RIGHT_IR] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown
  GPIOD->PDDR &= ~(1 << RIGHT_IR);
}

void left_ir_init() {
  SIM->COPC = 0;              // Desactiva el Watchdog
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;   // Conecta el reloj al puerto A

  PORTA->PCR[LEFT_IR] |= PORT_PCR_MUX(1); // Activa el GPIO
  PORTA->PCR[LEFT_IR] |= PORT_PCR_PE_MASK;// Pull enable, habilita la resistencia interna
  PORTA->PCR[LEFT_IR] |= PORT_PCR_PS_MASK;// Pull select, selecciona el modo de funcionamiento pullup/pulldown
  GPIOA->PDDR &= ~(1 << LEFT_IR);
}

int right_ir_active() {
  return( !(GPIOD->PDIR & (1 << RIGHT_IR)) );
}

int left_ir_active() {
  return( !(GPIOA->PDIR & (1 << LEFT_IR)) );
}


/* Main code */

// 1- Sleep until any switch is pushed (SW1 or SW3)
// 2- Line follow
int main(void)
{
  led_green_init();
  led_red_init();
  led_green_off();
  led_red_off();

  sw1_init();
  sw2_init();

  right_motor_init();
  left_motor_init();
  right_motor_off();
  left_motor_off();

  right_ir_init();
  left_ir_init();

  // The microcontroller awaits in sleep state
  asm("wfi");

  // Line follow algorythm
  while(1){
    if (right_ir_active())
    {
      left_motor_on();
      led_green_on();
    } else {
      left_motor_off();
      led_green_off();
    }

    if (left_ir_active())
    {
      right_motor_on();
      led_red_on();
    } else {
      right_motor_off();
      led_red_off();
    }
  }

  return 0;
}
