#include "MKL46Z4.h"
#include "lcd.h"

#define LED_GREEN_POS (5)
#define LED_RED_POS (29)
#define SW1_POS (3)
#define SW2_POS (12)

const int CORE_CLOCK = 48000000;
const int DURATION_SHORT = 1000000;
const int DURATION_LONG = DURATION_SHORT * 3;
const int WAITING = 0;
const int READING = 1;
int STATUS_SW1 = 0;
int STATUS_SW2 = 0;
int SYSTEM_STATUS = 0;
int DOTS = 0;
int DASHES = 0;


// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// RIGHT (SW1) = PTC3 (pin 73)
// LEFT (SW2) = PTC12 (pin 88)

// Enable IRCLK (Internal Reference Clock)
// see Chapter 24 in MCU doc
void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

int delay(int duration, int waiting)
{
  volatile int i;

  for (i = 0; i < duration; i++){
    if (!waiting && SYSTEM_STATUS)
      return i;
  }
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

void PORTDIntHandler(void) {
  int pressed_switch = PORTC->ISFR;
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ

  // SW1
  if(pressed_switch == (0x8)) {
    ++STATUS_SW1;
  }

  // SW2
  if(pressed_switch == (0x1000)) {
    ++STATUS_SW2;
  }

  SYSTEM_STATUS = STATUS_SW1 + STATUS_SW2;
}

// Hit condition: (else, it is a miss)
// - Left switch = Dash
// - Right switch = Dot

int main(void)
{
  led_green_init();
  led_red_init();
  led_green_off();
  led_red_off();

  sw1_init();
  sw2_init();

  irclk_ini(); // Enable internal ref clk to use by LCD
  lcd_ini();

  volatile unsigned int result = 0;
  volatile int elapsed_time = 0;
  while (result != 505) {

    // Bloqued until any button are pushed unless it waits too long
    while(!SYSTEM_STATUS) {
      if(elapsed_time)
        break;
    }

    // Update
    if(STATUS_SW1) {
      led_green_on();
      delay(DURATION_SHORT, READING);
      led_green_off();
      ++DOTS;
    } else if(STATUS_SW2) {
      led_green_on();
      delay(DURATION_LONG, READING);
      led_green_off();
      ++DASHES;
    }

    // S
    if (DOTS == 3 && DASHES == 0) {
      // First S of the sequence
      if(!result) {
        result += 5;
      // Last S of the sequence
      } else if(result == 50) {
        result *= 10.1;
      // If the sequence is broken
      } else {
        result = 0;
      }
      DOTS = 0;
    }

    // O
    if (DASHES == 3 && DOTS == 0) {
      if(result == 5) {
        result *= 10;
      // If the sequence is broken
      } else {
        result = 0;
      }
      DASHES = 0;
    }

    // Invalid character
    if (DOTS > 5 || DASHES > 5){
      DOTS = 0;
      DASHES = 0;
      result = 0;
    }

    // Update LCD
    lcd_display_n_dec(result);
    // Update system values
    STATUS_SW1 = 0;
    STATUS_SW2 = 0;
    SYSTEM_STATUS = 0;

    elapsed_time = delay(DURATION_LONG * 2, WAITING);

    // Waiting too long
    if (elapsed_time > DURATION_LONG){
      DOTS = 0;
      DASHES = 0;
      result = 0;
    }
    else {
      elapsed_time = 0;
    }
  }

  // Stop reading the input and show blinking SOS in the LCD
  while (1) {
    LCD->AR |= LCD_AR_BLINK(1);
    LCD->AR |= LCD_AR_BRATE(0xBB);
    led_red_on();
    lcd_display_n_dec(result);
  }

  return 0;
}
