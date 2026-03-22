//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include "defines.h"
#include <stdint.h>
#include <util/twi.h>

#include "lcd.c"

//#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include "display.c"

#include "text.h"



#define DISPLAY 0


#include "adc.c"
#define TW_STATUS   (TWSR & TW_STATUS_MASK)



//***********************************
						
uint8_t  LOK_ADRESSE = 0x7F; //	11001100	Trinär
uint8_t WEICHENCODE = 0;
// test
//uint8_t  LOK_ADRESSE = 0x0F; //   11001100   Trinär

//									
//***********************************

/*
 commands
 LO     0x0202  // 0000001000000010
 OPEN   0x02FE  // 0000001011111110
 HI     0xFEFE  // 1111111011111110
 */


//#define OUTPORT	PORTD		// Ausgang fuer Motor

//#define INPORT   PORTD  // Input signal auf INT0
#define INPIN   PIND  // Input signal

#define DATAPIN  2 


#define LOOPLEDPORT		PORTB
#define LOOPLEDDDR      DDRB
#define LOOPLED			0

#define INT0_RISING	   0
#define INT0_FALLING		1


volatile uint8_t   loopstatus=0x00;            


//void lcd_puts(const char *s);

// EADOGM
volatile uint16_t laufsekunde=0;
volatile uint8_t laufminute=0;
volatile uint8_t laufstunde=0;

volatile uint16_t motorsekunde=0;

volatile uint16_t motorminute=0;

volatile uint16_t stopsekunde=0;

volatile uint16_t stopminute=0;
volatile uint16_t batteriespannung =0;
volatile uint16_t updatecounter =0;

volatile uint8_t                 curr_screen=0; // aktueller screen
volatile uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
volatile uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
volatile uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
volatile uint8_t                 last_cursorspalte=0; // letzte colonne des cursors


volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor
volatile uint16_t                 blink_cursorpos=0xFFFF;

volatile uint8_t                 displaystatus = 0; // bits for sending display data
                                          // 0-3: sendposition
                                          // 4,5,6: 
                                          // 7: aktivbit


volatile uint8_t                 displaydata[8]  = {};



uint16_t spicounter=0;

//

volatile uint8_t	INT0status=0x00;				
volatile uint8_t	signalstatus=0x00; // status TRIT
volatile uint8_t  pausestatus=0x00;


volatile uint8_t   address=0x00; 
volatile uint8_t   data=0x00;   



volatile uint16_t MEMBuffer = 0;

//volatile uint16_t	HIimpulsdauer=0;			//	Dauer des LOimpulsdaueres Definitiv

volatile uint8_t	HIimpulsdauerPuffer=22;		//	Puffer fuer HIimpulsdauer
volatile uint8_t	HIimpulsdauerSpeicher=0;		//	Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint8_t   pausecounter = 0; //  neue ädaten detektieren
volatile uint8_t   abstandcounter = 0; // zweites Paket detektieren

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
volatile uint8_t   lokadresse = 0;

volatile uint8_t   lokadresseA = 0;
volatile uint8_t   lokadresseB = 0;

volatile uint8_t   deflokadresse = 0;
volatile uint8_t   lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   oldlokdata = 0;
volatile uint8_t   lokdata = 0;
volatile uint8_t   deflokdata = 0;
//volatile uint16_t   newlokdata = 0;

volatile uint8_t   rawdataA = 0;
volatile uint8_t   rawdataB = 0;

// ***
volatile uint8_t   rawfunktionA = 0;
volatile uint8_t   rawfunktionB = 0;

volatile uint8_t     oldspeed = 0;
volatile uint8_t     newspeed = 0;
volatile uint8_t     startspeed = 0; // Anlaufimpuls
volatile uint8_t     speedcode = 0;
volatile int8_t      speedintervall = 0;

volatile uint8_t   dimmcounter = 0; // LED dimmwertcounter
volatile uint8_t   ledpwm = 0x40; // LED PWM 50%
volatile uint8_t   ledstatus=0; // status LED


volatile uint8_t   ledonpin = WEICHEA_PIN; // Stirnlampe ON
volatile uint8_t   ledoffpin = WEICHEB_PIN; // Stirnlampe OFF

// ***
volatile uint8_t   speed = 0;

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls

volatile uint8_t     pwmpin = MOTORA_PIN;           // Motor PWM
volatile uint8_t     richtungpin = MOTORB_PIN;      // Motor Richtung


volatile uint16_t	taktimpuls=0;

volatile uint16_t   motorPWM=0;

volatile uint8_t weichenstatus = 0;
volatile uint16_t weichenimpulscounter = 0;

volatile uint8_t   taskcounter = 0;

volatile uint8_t   speedlookup[15] = {};
uint8_t speedlookuptable[10][15] =
{
   {0,18,36,54,72,90,108,126,144,162,180,198,216,234,252},
   {0,30,40,50,60,70,80,90,100,110,120,130,140,150,160},
   {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140},
   {0,7,14,21,28,35,42,50,57,64,71,78,85,92,100},
   {0,33,37,40,44,47,51,55,58,62,65,69,72,76,80},
   
   //{0,41,42,44,47,51,56,61,67,74,82,90,99,109,120},
   {0,11,15,19,26,34,43,55,68,82,98,116,136,157,180},
   {0,41,43,45,49,54,60,66,74,82,92,103,114,127,140},
   {0,41,44,48,53,59,67,77,87,99,113,128,144,161,180},
   {0,42,45,50,57,65,75,87,101,116,134,153,173,196,220},
   {0,42,45,51,58,68,79,93,108,125,144,165,188,213,240}
};
uint8_t speedcodelookuptable[16] = {0,0x3,0x0C,0x0F,0x30,0x33,0x3C,0x3F,0xC0,0xC3,0xCC,0xCF,0xF0,0xF3,0xFC,0xFF};

volatile uint8_t   lastDIR =  0;
uint8_t loopledtakt = 0x40;
uint8_t refreshtakt = 0x45;
uint16_t speedchangetakt = 0x350; // takt fuer beschleunigen/bremsen


volatile uint8_t loktyptable[4];

volatile uint8_t speedindex = 5;

volatile uint8_t   maxspeed =  252;//prov.
volatile uint8_t   minspeed =  0;
uint16_t displaycounter0;
uint16_t displaycounter1;

uint16_t displayfenstercounter = 0; // counter fuer abgelaufene Zeit im Display-Fenster




void displayfensterfunction(void);


void spi_init(void) // SPI-Pins aktivieren
{
   // https://embedds.com/serial-peripheral-interface-in-avr-microcontrollers/
   //set MOSI, SCK and SS as output
   DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2);
   //set SS to high
   PORTB |= (1<<PB2);
   //enable master SPI at clock rate Fck/2
   SPCR = (1<<SPE)|(1<<MSTR);
   SPSR |= (1<<SPI2X);
}


void slaveinit(void)
{
   
 	OSZIPORT |= (1<<OSZI_PULS_A);	//Pin 6 von PORT D als Ausgang fuer OSZI A
	OSZIDDR |= (1<<OSZI_PULS_A);	//Pin 7 von PORT D als Ausgang fuer SOSZI B
   OSZIPORT |= (1<<OSZI_PULS_B);   //Pin 6 von PORT D als Ausgang fuer OSZI A
   OSZIDDR |= (1<<OSZI_PULS_B);   //Pin 7 von PORT D als Ausgang fuer SOSZI B
   OSZIPORT |= (1<<SYNC);   //Pin 6 von PORT D als Ausgang fuer OSZI A
   OSZIDDR |= (1<<SYNC);   //Pin 7 von PORT D als Ausgang fuer SOSZI B

   //OSZIPORT |= (1<<INT_0);   //
   //OSZIDDR |= (1<<INT_0);   //Pin 7 von PORT D als Ausgang fuer SOSZI B
/*
   OSZIDDR |= (1<<PAKETA);
   OSZIPORT |= (1<<PAKETA);   //PAKETA
      //
   OSZIDDR |= (1<<PAKETB); 
   OSZIPORT |= (1<<PAKETB);   //PAKETB
 */  
   displaydata[0] = 3;
   displaydata[1] = 44;
   
   
	//LOOPLEDPORT |=(1<<LOOPLED);
   LOOPLEDDDR |= (1<<LOOPLED);
   
   WEICHEDIP_DDR &= ~(1<<WEICHEDIP0);
   WEICHEDIP_DDR &= ~(1<<WEICHEDIP1);
   WEICHEDIP_DDR &= ~(1<<WEICHEDIP2);

   WEICHEDIP_PORT |= (1<<WEICHEDIP0); // pullup
   WEICHEDIP_PORT |= (1<<WEICHEDIP1);
   WEICHEDIP_PORT |= (1<<WEICHEDIP2);

  

  
/*
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD
*/
   
   TESTDDR |= (1<<TEST0); // test0
   TESTPORT |= (1<<TEST0); // HI
   
   MOTORDDR |= (1<<MOTORA_PIN);  // Output Motor A 
   MOTORPORT |= (1<<MOTORA_PIN); // HI
   
   MOTORDDR |= (1<<MOTORB_PIN);  // Output Motor B 
   MOTORPORT |= (1<<MOTORB_PIN); // HI
   
   WEICHEDDR |= (1<<WEICHEA_PIN);  // Lampe A
   WEICHEPORT &= ~(1<<WEICHEA_PIN); // LO
    
   WEICHEDDR |= (1<<WEICHEB_PIN);  // Lampe B
   WEICHEPORT &= ~(1<<WEICHEB_PIN); // LO
   /*
   for(uint8_t i=0;i<2;i++)
   {
      LOOPLEDPORT |=(1<<LOOPLED);
      _delay_ms(100);
      LOOPLEDPORT &= ~(1<<LOOPLED);
      _delay_ms(100);

   }
   */
   

   //initADC(MEM);
   
   // default
   //LOOPLEDPORT |=(1<<LOOPLED);
   /*
   pwmpin = MOTORA_PIN;
   richtungpin = MOTORB_PIN;
   ledonpin = WEICHEA_PIN;
   ledoffpin = WEICHEB_PIN;
   */
   uint8_t i = 0;
   for (i=0;i<15;i++)
   {
      speedlookup[i] = speedlookuptable[speedindex][i];
   }

   maxspeed = speedlookup[14];
   minspeed = speedlookup[1];
   
   loopstatus |= (1<<FIRSTRUNBIT);
/*
   // TWI
   DDRC |= (1<<5);   //Pin 0 von PORT C als Ausgang (SCL)
   PORTC |= (1<<5);   //   ON
   DDRC |= (1<<4);   //Pin 1 von PORT C als Ausgang (SDA)
   PORTC |= (1<<4);   //   ON
*/
   
   
   
   
   //LOOPLEDPORT |=(1<<LOOPLED);
}


void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
//	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
   TCCR2B |= (1<<CS21); // no prescaler
	//OC2 akt


	TIMSK2 |= (1<<OCIE2A);			//CTC Interrupt aktivieren
   TIMSK2 |=(1<<TOIE2);        //interrupt on Compare Match A
	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2A = wert;					//Setzen des Compare Registers auf HI-impulsdauer
} 



void int0_init(void)
{
   EICRA |= (1 << ISC00) | (1 << ISC01);  // Trigger interrupt on rising edge
   EIMSK |= (1 << INT0);  // Enable external interrupt INT0

 //  INT0status |= (1<<INT0_RISING);
   INT0status = 0;
   INT0status |= (1<<INT0_WAIT);
}

// MARK:  INT0
ISR(INT0_vect) 
{
   //OSZI_A_LO();
   //if(displayfenstercounter%4 == 0)
   {
      //OSZI_B_LO();
      
      if (INT0status == 0) // neue Daten beginnen
      {
         displaystatus &= ~(1<<DISPLAY_GO); // displayfenster end
         
         //OSZI_A_HI(); 
         INT0status |= (1<<INT0_START);
         INT0status |= (1<<INT0_WAIT); // delay, um Wert des Eingangs zum richtigen Zeitpunkt zu messen
         
         INT0status |= (1<<INT0_PAKET_A); // erstes Paket lesen
         //OSZIPORT &= ~(1<<PAKETA); 
         
         pausecounter = 0; // pausen detektieren, reset fuer jedes HI
         abstandcounter = 0;// zweites Paket detektieren, 
         
         waitcounter = 0;
         tritposition = 0;
         funktion = 0;
         //OSZI_A_HI();
         
      } 
      
      else // Data im Gang, neuer Interrupt
      {
         INT0status |= (1<<INT0_WAIT);
         
         pausecounter = 0;
         abstandcounter = 0; 
         waitcounter = 0;
         //     OSZIALO;
      }
      //OSZI_A_HI();
      //OSZI_B_HI();
      //SYNC_HI();
   }
   /*
   else
   {
      INT0status = 0;
   }
   */
}


// MARK: ISR Timer2

ISR(TIMER2_COMPA_vect) // // Schaltet Impuls an MOTOROUT LO wenn speed
{
   

   // MARK: TIMER0 TIMER0_COMPA INT0
   if (INT0status & (1<<INT0_WAIT))
   {
      waitcounter++; 
      if (waitcounter >2)// Impulsdauer > minimum, nach einer gewissen Zeit den Stautus abfragen
      {
         
         //OSZI_A_LO();
         //OSZIAHI;
         INT0status &= ~(1<<INT0_WAIT);
         if (INT0status & (1<<INT0_PAKET_A))
         {
            //OSZI_B_LO();
            if (tritposition < 8) // Adresse)
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseA |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseA &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // Funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionA |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionA &= ~(1<<(tritposition-8)); // bit ist 0
               }
            }
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataA |= (1<<((tritposition-10))); // bit ist 1
               }
               else // 
               {
                  rawdataA &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
         }
         
         if (INT0status & (1<<INT0_PAKET_B))
         {
            if (tritposition < 8) // Adresse)
            {
               
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseB |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseB &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // bit 8,9: funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionB |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionB &= ~(1<<(tritposition-8)); // bit ist 0
               }
               
            }
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataB |= (1<<(tritposition-10)); // bit ist 1
               }
               else 
               {
                  rawdataB &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
            
            if (!(lokadresseB == LOK_ADRESSE))
            {
               
            }
         }

         
         if (tritposition < 17)
         {
            tritposition ++;
         }
         else // Paket gelesen
         {
            // Paket A?
            if (INT0status & (1<<INT0_PAKET_A)) // erstes Paket, Werte speichern
            {
               
               oldfunktion = funktion;
               
               INT0status &= ~(1<<INT0_PAKET_A); // Bit fuer erstes Paket weg
               INT0status |= (1<<INT0_PAKET_B); // Bit fuer zweites Paket setzen
               tritposition = 0;
            }
            else if (INT0status & (1<<INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               //SYNC_LO();
               //displaystatus |= (1<<DISPLAY_GO);
               // // Displayfenster begin
               if(displayfenstercounter++ > 4)
               {
                  displaystatus |= (1<<DISPLAY_GO);
                  displayfenstercounter=0;
               }
               //displayfenstercounter = MAXFENSTERCOUNT;
               
               // MARK: EQUAL
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {
                  OSZI_A_LO();
                  SYNC_LO();
                  if (lokadresseB == LOK_ADRESSE)
                  {
                     //OSZI_A_LO();
                     
                     OSZI_B_LO();
                     // Daten uebernehmen
                     
                     lokstatus |= (1<<ADDRESSBIT);
                     deflokadresse = lokadresseB;
                     //deffunktion = (rawdataB & 0x03); // bit 0,1 funktion als eigene var
                     deffunktion = rawfunktionB;
                     
                     
                     if (deffunktion)
                     {
                        lokstatus |= (1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // change setzen
                        
                     }
                     else
                     {
                        lokstatus &= ~(1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // led-change setzen
                        
                     }
                     // deflokdata aufbauen
                     for (uint8_t i=0;i<8;i++)
                     {
                        //if ((rawdataB & (1<<(2+i))))
                        if ((rawdataB & (1<<i)))
                        {
                           deflokdata |= (1<<i);
                        }
                        else 
                        {
                           deflokdata &= ~(1<<i);
                        }
                     }
                     
                     // Weichennummer checken
                     WEICHENCODE = 0xFF;
                     WEICHENCODE = WEICHEDIP_PIN & 0x38;
     
                     WEICHENCODE >>= 3;
                     WEICHENCODE = 7-WEICHENCODE; // dipschalter ist active LOW > invertieren
                     // Weiche stellen einleiten
                        if(deflokdata == speedcodelookuptable[WEICHENCODE])
                        {
                           WEICHEPORT |= (1<<WEICHEA_PIN); // Start Bewegung
                           weichenstatus |= (1<<WEICHESTART);
                           if(lokstatus & (1<<FUNKTIONBIT)) // Weiche auf Ablenkung stellen
                           {
                              WEICHEPORT |= (1<<WEICHEB_PIN); 
                              weichenstatus |= (1<<ABLENKUNG);
                           }
                           else // Weiche auf Gerade stellen
                           {
                              WEICHEPORT &= ~(1<<WEICHEB_PIN);
                              weichenstatus |= (1<<GERADE);
                           }
                        }
                        else
                        {
                           WEICHEPORT &= ~(1<<WEICHEA_PIN); 
                           //weichenstatus &= ~(1<<ABLENKUNG);
                        }


                     // end Weiche
                     OSZI_B_HI();
                  }
                  else 
                  {
                     // aussteigen
                     //deflokdata = 0xCA;
                     INT0status = 0;
                     return;
                  }
                  OSZI_A_HI();
                  SYNC_HI();
                  
               }// if (lokadresseA &&...
               else 
               {
                  lokstatus &= ~(1<<ADDRESSBIT);
                  // aussteigen
                  //deflokdata = 0xCA;
                  INT0status = 0;
                  return;
                  
               }
               
               INT0status |= (1<<INT0_END);
               //     OSZIPORT |= (1<<PAKETB);
               if (INT0status & (1<<INT0_PAKET_B))
               {
                  //               TESTPORT |= (1<<TEST2);
               }
               //SYNC_HI();
            } // End Paket B
         }
        // OSZI_B_HI();
      } // waitcounter > 2
   } // if INT0_WAIT
   
   if (INPIN & (1<<DATAPIN)) // Pin HI, input   im Gang
   {
      //      HIimpulsdauer++; // zaehlen
   }
   else  // LO, input fertig, Bilanz
   {
      if (abstandcounter < 20)
      {
         abstandcounter++;
      }
      else //if (abstandcounter ) // Paket 2
      {
         
         abstandcounter = 0;
         // OSZIAHI;
      }
      
      if (pausecounter < 120)
      {
         pausecounter ++; // pausencounter incrementieren
      }
      else 
      {
         
         //OSZIBHI; //pause detektiert
         pausecounter = 0;
         INT0status = 0; //Neue Daten abwarten
         return;
      }
      
   } // input LO
   //OSZI_B_HI();
}

void displayfensterfunction(void)
{
   
   displayfenstercounter = 0;
   _delay_ms(2);
}

//LiquidCrystal_I2C lcd(0x27, 16, 2); // Adresse anpassen


int main (void) 
{
   
   
	slaveinit();

	
	//uint16_t loopcount0=0;
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;

   uint16_t firstruncount0=0;
   uint16_t firstruncount1=0;

	
	_delay_ms(200);

   
   oldfunktion = 0x03; // 0x02
   oldlokdata = 0xCE;
   ledpwm = LEDPWM;
   
   
   sei();
   

   
   uint8_t counter = 0;
   uint16_t lcdcounter = 0;
    
   if (DISPLAY)
   {
     // setlogscreen();
   }
   
	while (1)
   {  
      //OSZI_B_LO();
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      
        
      // firstrun
      
      if(loopstatus & (1<<FIRSTRUNBIT))
      {
         firstruncount0++;
         if (firstruncount0>=0x0A)
         {

            firstruncount0=0;
            
            firstruncount1++;
            
            if (firstruncount1 >= 0xF0)
            {
              
                int0_init();
               
               _delay_ms(2);
                timer2(4);
               //sei();
               loopstatus &= ~(1<<FIRSTRUNBIT);
            }
         }
         
      }// end firstrun
      
   

      // dip lesen
      /*
     WEICHENCODE = 0xFF;
     WEICHENCODE = WEICHEDIP_PIN & 0x38;
     
      WEICHENCODE >>= 3;
      WEICHENCODE = 7-WEICHENCODE; // dipschalter ist active LOW > invertieren
      */      
      // {0,0x3,0x0C,0x0F,0x30,0x33,0x3C,0x3F,0xC0,0xC3,0xCC,0xCF,0xF0,0xF3,0xFC,0xFF};
      /*
      if(deflokdata == speedcodelookuptable[WEICHENCODE])
      {
         WEICHEPORT |= (1<<WEICHEA_PIN); // Start Bewegung
         weichenstatus |= (1<<WEICHESTART);
         if(lokstatus & (1<<FUNKTIONBIT)) // Weiche auf Ablenkung stellen
         {
            WEICHEPORT |= (1<<WEICHEB_PIN); 
            weichenstatus |= (1<<ABLENKUNG);
         }
         else // Weiche auf Gerade stellen
         {
            WEICHEPORT &= ~(1<<WEICHEB_PIN);
            weichenstatus |= (1<<GERADE);
         }
      }
      else
      {
         WEICHEPORT &= ~(1<<WEICHEA_PIN); 
         //weichenstatus &= ~(1<<ABLENKUNG);
      }
      */
  
      loopcount0++;
      if (loopcount0>=refreshtakt)
      {
         //OSZI_B_LO();
         loopcount0=0;
         
         loopcount0=0;            
         // Takt for display
         displaycounter1++;
         if (displaycounter1 > MAXLOOP1)
         {
            displaycounter1=0;
            LOOPLEDPORT ^= (1<<LOOPLED);
            counter++;
         }
         

         
         
         
         //OSZI_B_HI();
      }  // loopcount0>=refreshtakt
      OSZI_B_HI();
   
      
   }//while


 return 0;
}
