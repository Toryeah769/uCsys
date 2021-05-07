/* Prosjektoppgave i Mikrokontrolelrsystemer for Torje Johansen og Karl Gran Grodås
  Vi har skapt systemet for en digital safe.
  Man bruker et potmeter og en knapp for å lese inn en kodesekvens, som matches mot den korrekte koden.
  Døra låses opp, og en timer gjør at en LED blinker så lenge døra er åpen.
  Knappen og timeren er styrt med interrupt.
  Programmer kommuniserer via UART.

  I dette programmer har vi dekt følgende elementer:
  - I/O
  - Interrupt
  - USART
  - Timere
  - ADC
*/

// --- Defines
#define F_CPU 16000000
#define BAUD 9600
#define A0 0
#define A1 1

// --- Includes
#include <util/setbaud.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"

// --- Function declarations
void initADC(void);
void initInterupts(void);
void initTimer(void);
uint8_t analogRead(int Pin);
void sendADCToSerial(void);
void checkSequence(void);
void print(unsigned char data[]);

// --- Variable declaration
// Arrays:
char correctSequence[4] = {1, 2, 3, 4}; //Rett kode for å låse opp
char inputSequence[4] = {}; //Koden man trykker inn
//Values
int pos = 0; //Viser til posisjon i innputSequence[]
int potValue; //Potmeterverdien, mappet fra 0 - 9
//Bools:
int interrupted = 0; //Flagges på interrupt, brukes til å teller opp pos-variabeln
int locked = 1; //Viser om døra låst eller ikke
int messageSent = 0; //Brukes for å påse at en beskjed skrives bare en gang
int alarm = 0; //Brukes for å vise om alarmen har gått

int main(void) {
  // Inits
  initUSART();
  initTimer();
  initInterupts();
  initADC();
  //IO
  DDRB = (1 << DDB5);

  //Main loop
  //Programmet har to tilstander: locked eller !locked. (Endre til switch?)
  while (1) {
    PORTB = (0 << DDB5); //Setter diodeutgangen lav. 50/50 for at den er høy etter man lukker døra igjen.
    while (locked) {  //... Så lenge døra er lukka (som den i utgangspunktet er)
      if (interrupted) { //Interrupted-variabelen flagges ved interrup av knappen
        pos++; //Inkrementerer pos-variabelen,for å kunne sende
        interrupted = 0; //Fjerner flagget
      }
      sendADCToSerial(); //Funksjonen konverterer ADC og sender til serial
      checkSequence(); //Funksjonen sjekker om den innleste sekvensen er korrekt
    }

    while (!locked) { //..Så lenge døra er åpen
      if (interrupted) { //Samme knapp og interrupt-subrutine også her.
        pos++;  //pos-variabelen bruker her for å låse døra igjen.
        interrupted = 0;
      }
      if (alarm) { //alarm flagges av timer-interrupt
        PORTB ^= (1 << DDB5); //Toggler LED
        alarm = 0; //Fjerner flagget
      }
      if (!messageSent) { //Denne beskjeden skal bare sendes en gang
        print("Press button to lock.\n");
        messageSent = 1;
      }
      if (pos) { //Dersom knappen har blitt trykt vil pos være ulik 0.
        print("Locking door..\n");
        _delay_ms(1000); //Lagt inn for kul effekt.
        print("Door locked.\n");
        locked = 1; //Endrer alle nødvendige flagg ved tilstandsskifte
        pos = 0;
        messageSent = 0;
      }
    }
  }
  return 0;
}


// --- Interrupts
ISR(INT0_vect)  // Interupt på knappen
  {
  interrupted = 1; //Setter flagg hvis knappen blir trykt
  _delay_ms(100); //Debounce
}

ISR (TIMER1_COMPA_vect)  // //Interrupt på timeren
{
  alarm = 1; //Setter flagg når tidsintervall er nådd
}

// ------ Funksjoner
// --- Funksjon som lineæriserer ADC fra 0 - 9 og printer verdi til serial.
void sendADCToSerial(void) { 
	unsigned char data[10];
	int in_min = 0;
	int in_max = 255; //8-bits oppløsning med left shifted adc gir maks 255.
	int out_min = 0;
	int out_max = 9;
	int x = analogRead(A0); //Egenlag funksjon for å lese ADC.
	potValue = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; //Lineæriserer verdien fra 0 - 9
	_delay_ms(20); //Stabilitet
	if (inputSequence[pos] != potValue) { //Dersom vi har en ny verdi (brukes for å ikke spamme serial; sender bare verdi ved endring)
		inputSequence[pos] = potValue; //Lagrer verdien ved endring. Låser seg når man trykker knappen og pos-variabelen inkrementerer
		print("sequence: "); //Printer foreløpige innlest sekvens. Merk at siste som vises ikke er låst.
		for (int i = 0; i <= pos; i++) {
			itoa(inputSequence[i], data, 10); //Konverterer int til char for printing.
			print(data);
		}
		print("\n");
	}
}

// --- Funksjon for å printe tekst
void print(unsigned char data[]) {
  int i = 0;
  while (data[i] != 0) { //Printer sekvensen
    while (!( UCSR0A & (1 << UDRE0)));
    UDR0 = data[i];
    i++;
  }
}

// --- Funksjon for å sjekke om innlest sekvens er rette
void checkSequence(void) {
  if (pos > 3) { //Kjøres bare hvis fire siffer er innlest. Linjden under sjekker verdi for verdi i arrayet (tungvindt?).
    if (inputSequence[0] == correctSequence[0] && inputSequence[1] == correctSequence[1] && inputSequence[2] == correctSequence[2] && inputSequence[3] == correctSequence[3]) {
      print("Door open!\n");
      locked = 0;
    }
    else {
      print("Wrong code.\n");
    }
    while (pos) { //Tømmer array og nullstiller pos-variabel.
      inputSequence[pos] = 0;
      pos--;
    }
  }
}

// --- Funksjon som etterligner den kjære "analogRead" fra Arduino. Leser av en analogpinne. Bare laget for A0 og A1.
uint8_t analogRead(int Pin) {
  if (Pin == A0) {
    ADMUX &= ~(1 << MUX0); //A0 for ADC
  }
  if (Pin == A1) {
    ADMUX |= (1 << MUX0); //A1 for ADC
  }
  ADCSRA |= (1 << ADSC); //Starter avlesning
  loop_until_bit_is_clear(ADCSRA, ADSC); //Venter til den er klart (13 (25) sykluser).
  return ADCH; //Returnerer kun 8 MSB (left shifted)
}

// ------ Initialiseringsfunksjoner
// --- Initialisere ADC. Aktiverer ADC-innganger, setter referanse og venstrejusterer, skalerer.
void initADC(void) {
  DDRD = 0xff; //utgang (Spør Rolf)
  PORTD = 0x00; //skrive 0
  ADMUX = (1 << REFS0) | (1 << ADLAR); // AVcc som ref(Uten kondensator) og left adjusted (8MSB i ADCH)
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADEN: enable ADC. ADPS2,1,0: Scaling, dividerer freq på 128.
}

// --- Initialiserer interrupts. Velger inngang og trigging.
void initInterupts(void) {
  EICRA = (1 << ISC01); //Negativ flanke
  EIMSK = (1 << INT0); // Enable INT0 (pinne 2).
  sei(); // enable global interrupts
}

// --- Initialiserer timere. Setter skaleringen
void initTimer(void) {  // (hentet fra eksempelkode)
  OCR1A = 0x3D08; // Ett sekund
  TCCR1B |= (1 << WGM12); // Mode 4, CTC on OCR1A
  TIMSK1 |= (1 << OCIE1A); //Set interrupt on compare match
  TCCR1B |= (1 << CS12) | (1 << CS10); //prescale 1024
}

// --- Initialiserer USART-kommunikasjon. (Krever setbaud.h)
void initUSART(void) { // (hentet fra eksempelkode)
  UBRR0H = UBRRH_VALUE; 
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
  /* Enable USART transmitter/receiver */
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 data bits, 1 stop bit */
  _delay_ms(10);
}
