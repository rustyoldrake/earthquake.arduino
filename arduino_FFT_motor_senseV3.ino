#include <avr/pgmspace.h>
//**************************************************************************************************

#define FFT_SIZE     128
#define N           (2 * FFT_SIZE)
#define N_WAVE          256    /* full length of Sinewave[] */
#define LOG2_N_WAVE     8      /* log2(N_WAVE) */
#define VERTICAL        30

#ifndef prog_uint8_t
#define prog_uint8_t const uint8_t
#endif

const uint8_t Sinewave[N_WAVE-N_WAVE/4] PROGMEM = {
  +0,  +3,  +6,  +9, +12, +15, +18, +21, +24, +28, +31, +34, +37, +40, +43, +46,
 +48, +51, +54, +57, +60, +63, +65, +68, +71, +73, +76, +78, +81, +83, +85, +88,
 +90, +92, +94, +96, +98,+100,+102,+104,+106,+108,+109,+111,+112,+114,+115,+117,
+118,+119,+120,+121,+122,+123,+124,+124,+125,+126,+126,+127,+127,+127,+127,+127,
+128,+127,+127,+127,+127,+127,+126,+126,+125,+124,+124,+123,+122,+121,+120,+119,
+118,+117,+115,+114,+112,+111,+109,+108,+106,+104,+102,+100, +98, +96, +94, +92,
 +90, +88, +85, +83, +81, +78, +76, +73, +71, +68, +65, +63, +60, +57, +54, +51,
 +48, +46, +43, +40, +37, +34, +31, +28, +24, +21, +18, +15, +12,  +9,  +6,  +3,
  +0,  -3,  -6,  -9, -12, -15, -18, -21, -24, -28, -31, -34, -37, -40, -43, -46,
 -48, -51, -54, -57, -60, -63, -65, -68, -71, -73, -76, -78, -81, -83, -85, -88,
 -90, -92, -94, -96, -98,-100,-102,-104,-106,-108,-109,-111,-112,-114,-115,-117,
-118,-119,-120,-121,-122,-123,-124,-124,-125,-126,-126,-127,-127,-127,-127,-127
};

void rev_bin( int fr[], int fft_n )
{
    int m, mr, nn, l, tr;

    mr = 0;
    nn = fft_n - 1;        

    for (m=1; m<=nn; ++m) {
            l = fft_n;     
         do {
             l >>= 1;  
            } while (mr+l > nn); 

            mr = (mr & (l-1)) + l;

        if (mr <= m)
             continue;
            tr = fr[m];
            fr[m] = fr[mr];
            fr[mr] = tr;
    }
}

short FIX_MPY( int a, int b )
{
    long c = (long)a * (long)b;
        c >>= 15;
     return c;
}

void fix_fft( int fr[],  int fi[], int fft_n )
{
    int m, mr, nn, i, j, l, k, istep, shift;
    int qr, qi, tr, ti, wr, wi;

    mr = 0;
    nn = fft_n - 1;
    l  = 1;
    k  = LOG2_N_WAVE-1;

    while (l < fft_n) {
            istep = l << 1;
            for (m=0; m<l; ++m) {
                    j = m << k;
                    /* 0 <= j < N_WAVE/2 */
            wr =  pgm_read_word(&Sinewave[j+N_WAVE/4]);
            wi = -pgm_read_word(&Sinewave[j]);

    shift  = 1;     //////////////////////
                    if (shift) {
                            wr >>= 1;
                            wi >>= 1;
                       }
                    for (i=m; i<fft_n; i+=istep) { //istep = 2, 4, 8, 16 ......
                            j = i + l;
                            tr = FIX_MPY(wr,fr[j]) - FIX_MPY(wi,fi[j]);
                            ti = FIX_MPY(wr,fi[j]) + FIX_MPY(wi,fr[j]);
                            qr = fr[i];
                            qi = fi[i];

                     if (shift) {
                             qr >>= 1;
                             qi >>= 1;
                         }             
                            fr[j] = qr - tr;
                            fi[j] = qi - ti;
                            fr[i] = qr + tr;
                            fi[i] = qi + ti;
                    }
            }
            --k;
      l = istep;
    }
}

void fix_fftr( int f[], int fft_size )
{
  int  *fr=f, *fi=&f[fft_size];

  rev_bin( fr, fft_size );

  fix_fft( fi, fr, fft_size );
}

void print_charta( int data[], int dlina )
{
  int i, j, peak = 0, nbr;  // local ints, careful about duplciate globals or in main loop
  const char mark1 = '[', mark2 = ']', nomk = '-';

  for ( i = 0; i < N/dlina; i++)
   { if ( (data[i]) > peak ){
        peak = data[i];
        nbr = i;
       }
    }

  for ( j = VERTICAL -1; j >= 0; --j )
            { Serial.print('|');
           for ( i = 0; i < N/dlina; i++)
            { if ( (data[i]) > j* (peak/VERTICAL)) {
                 Serial.print( mark1);
                 delay(1);
                 Serial.print( mark2);
            }
              else{
                 Serial.print( nomk);
                 delay(1);
                 Serial.print( nomk);
              }
            }
           Serial.println('|');
        }
   Serial.print("\t Ampl: ");
   Serial.print(peak);
   Serial.print("\t N: ");
   Serial.println(nbr);
}


   




//**************************************************************************************************
//   IIIIIIIIIINTEGER DEFINE


int   x[N], fx[N];
int   i, incomingByte; 
int   RealTimeSpew, RealTimeCount; // RA: Spew if 0 means quiet, non active; non-zero is period ms trigger for count to send out N data

int   MENUSELECT;  // RA stores 1-4 keystroke to figure out where to put the 0-9 info that follows 
int   THRESHOLD = 5; // RA: Variables that are user input 0-9
int   DECAY = 5; // RA: Variables that are user input 0-9
int   THRESHOLDCALC, DECAYCALC;  // RA these are computed from inputs

char   ledPin = 9; // Pin 9 is PWM output  - see below
unsigned int   ALERTBULB = 0;  // thisis the integer for the PWM output pin VALUE that provides visuual indicate of "alert" and of the decay


//const int sdvig = 32767; //DC bias of the ADC, approxim +2.5V.

//const int sdvig = 15000; //DC bias of the ADC, approxim +1V????  THE BLUE SENSORS DC BIAS IS VERY DIFFERENT FROM GEOPHONE. (( WORKS with 2.5v dc bias in, 1v out)


const int sdvig = 00000; //DC bias of the ADC, approxim +1V????  THE BLUE SENSORS DC BIAS IS VERY DIFFERENT FROM GEOPHONE. (( WORKS)

// TEST changed this to zero to try non DC Bias
//const int sdvig = 0000; //DC bias of the ADC, approxim +0 (TESTING OK??)

void setup() {
  Serial.begin(115200);  
  RealTimeSpew = 0;
  RealTimeCount = 0;
}




void loop()
{
  ADCSRA = 0x87;
  ADMUX = 0x60;
  ADCSRA |= (1<<ADSC);
  while(!(ADCSRA & 0x10));

  for( i = 0; i < (N >> 1); i++ ) {
    ADCSRA |= (1<<ADSC);
    
      //// TEST: DELAY TO SEE IF WE CAN "DOWNSHIFT FREQUENCY" Range DOWN  - i.e. we'r emore interested in 3-300 hz rather than 70-4000hz for bump.

     ///delay (1);  ///// XXXXXXXXXXXXXXXXXX CAREFUL XXXXXXXXXXXXXXX  1 is approx 10X shift down - e.g. 100hz input up to 910hz displayed 
     /// delay (5);  ///// XXXXXXXXXXXXXXXXXX CAREFUL XXXXXXXXXXXXXXX  1 is approx 10X shift down - e.g. 100hz input up to 910hz displayed 
     /// TESTING delay(1) to see if we can shift range down to 3-300hz
      
      // works, but slowing sampling in this way is VERY Crude.  NOt long term option, but does provide lower frequency rez for seismic. 
      // to do: recalbrate entire algo for LF
      ///  END OF TEST

    while(!(ADCSRA & 0x10));
    x[i] = ADC - sdvig;
  }  

  ADCSRA = 0x00;

  for (i=0; i<N; i++){
    if ( i < (N >> 1) )  fx[i] = x[i];
    if( i >= (N >> 1) )  fx[i] =    0;
  }  

  fix_fftr( fx, FFT_SIZE );

// Ryan's Realtime Spews
// Runs until S hit again

  if (RealTimeSpew > 0){
    if (RealTimeCount = RealTimeSpew) {
      for ( i = 0; i < N/2; i++){
      fx[i] = sqrt((long)fx[i] * (long)fx[i] + (long)fx[i+N/2] * (long)fx[i+N/2]);
      }
     delay(10);
     // delay
 
 // pulled this external funciton into main loop as needed to interface globals     
 //       print_SPEW( fx, 4 );
//        void print_SPEW( int data[], int dlina )
        //*** RYAN RUNNING CODE on "S"
// ***************************** SSSSSSSSSSSSSSSSSSSSSSSSSS
      int ii, jj, MAXPEAK = 0, nn; // Pulled the peak calc global - changed variables, used in "S' Loop
      for ( ii = 0; ii < N/4; ii++)
       { if ( (fx[ii]) > MAXPEAK ){
        MAXPEAK = fx[ii];
        nn = ii;
       }
      }
                  
      if (MAXPEAK > (THRESHOLDCALC + (THRESHOLD * 10)))
      {
       //Serial.println("******************************************************************************************************");
       //Serial.println("************** A  L  E  R  T  ***    A  L  E  R  T  ***    A  L  E  R  T  ***    A  L  E  R  T  ******");
       //Serial.println("******************************************************************************************************");      
       Serial.print("- TRIGGER  -" );
       
       ALERTBULB = 255;  /// set this alert bulb to provide visual indication of "trigger" and then let it fade
      }
      else
      {
       Serial.print("-- NORMAL -- ");          
       if (ALERTBULB > 9) {ALERTBULB -= DECAY; }   // better ways to do this but OK for now - the 'bulb' is the alert and decay subtracted each cycle. decay is user prog
       else if (ALERTBULB) { ALERTBULB = 0;}
      }
      
      
       Serial.print("MAXPEAK " );
       Serial.print(MAXPEAK);
       Serial.print(" N(mp): ");
       Serial.print(nn);
       //Serial.print(" Freq(aprx): ");
       //Serial.print(nn*70);

       // Serial.print(" Confidence: ");
       // Serial.print(" Trigger = ");
       
       Serial.print(" //// TH, ");
       Serial.print(THRESHOLD);
       Serial.print(" ");
       Serial.print(THRESHOLDCALC + (THRESHOLD * 10));
       Serial.print(" THC, ");
       Serial.print(THRESHOLDCALC);
       Serial.print("         DECAY, ");
       Serial.print(DECAY);
       Serial.print(" ALERTBULB, ");  
       Serial.print(ALERTBULB);

       Serial.println("");
       //Serial.println("\t armed/active/safe?");
       //Serial.print(THRESHOLD,THRESHOLDCALC,DECAY,DECAYCALC);
    
        // cleanup now update THRESHOLD CALC trailing average
        THRESHOLDCALC *= (0.9);
        THRESHOLDCALC += (0.1 * MAXPEAK);
    
        // Then reset counter for real time "S"pew   
        RealTimeCount = 0; // Reset counter and let it start ramping up on next pass
        
        analogWrite(ledPin, ALERTBULB);    
    }
    else {
    RealTimeCount++;
    }
  }  // opens @ if realtime spew



// ===============
///Next section if there is commms input (still in main loop)
// ================
//Debugging monitor, allow to check processing data on each stage.
// x command - printout data received from ADC (input raw data).
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'x') {
      for ( i = 0; i < FFT_SIZE; i++){
        Serial.print("\t");
        Serial.print(x[i], DEC);
        if ((i+1)%10 == 0) Serial.print("\n");
      } 
      Serial.print("\n");
      delay(50);
    }
    
    
// f command - printout data after FFT. Clear view of each bin in the spectrum.
// ---->>>>>   Both format, as a table and chart representation   <<<<<-------
// Chart is AUTOSCALE mode, pay attention to the magnitude printed at the bottom.
  if (incomingByte == 'f') {
  for ( i = 0; i < N/2; i++){
      fx[i] = sqrt((long)fx[i] * (long)fx[i] + (long)fx[i+N/2] * (long)fx[i+N/2]);
      }
      for ( i=0; i < (FFT_SIZE >> 1); i++){
        Serial.print("\t");
        Serial.print(fx[i], DEC);     
        if ((i+1)%10 == 0) Serial.print("\n");
      } 
      delay(50);
      Serial.print("\n");
      
      print_charta( fx, 4 );
  }
  
  /// This is Spew ON and Spew Off
  if (incomingByte == 's') {
    if (RealTimeSpew == 0)  {
        RealTimeSpew = 10
        ;
        RealTimeCount = 0;  
    }
    else {
     //
     RealTimeSpew = 0; 
    }    
  }
  
  ///  MMMMMMMMMMMMMMMMMMMMMMM MENU LOOP ENTRY
  
  if (incomingByte == 'm') {
   Serial.println("");
   Serial.println("");
   Serial.println("\t [t] to set THRESHOLD (Amplitude above trail average that triggers)");
   Serial.println("\t [d] to set DECAY/FADE rate (how fast a trigger reverts i.e. halflife)");
   Serial.println("\t [y] to set YYY for ZZZ");
   Serial.println("\t Commmand? ");
   Serial.print("\n");

   MENUSELECT = 0;  // set this to 0 until keystroke made
      
   while ((incomingByte == 'm') || (incomingByte == 13))  {
     // FIRST LOBBY to determine what user wants to program 
     //Needed the 'm' to pull us into this loop, and the 13 to keep us in (regular COM rx on idle non input)
     // Serial.println("\t MADE IT TO WHILE LOOP WAIT ONCE");
     delay(100);  // no need to overdo the checking - small delay
     
     if (Serial.available() > 0) 
     incomingByte = Serial.read();
     // hang out here while we wait for keystroke WAIT STUCK IN LOOP     
    
     }  // end of m 13 loop
    // when we get out, the MENUSELECT should be set to keystroke
   MENUSELECT = incomingByte;
   Serial.println("CURRENT SETTINGS:");
   Serial.println("=========");
   Serial.print("THRESHOLD ");
   Serial.println(THRESHOLD);
   Serial.print("DECAY ");  
   Serial.println(DECAY);
   Serial.println("");
   
   Serial.print("Modifying ");
   Serial.write(MENUSELECT);
   Serial.println(" [0-9] = ? __");
   
   while ((incomingByte == MENUSELECT ) || (incomingByte == 13)) {
     // final step, we've hit M, then 1,2,3 etc, now need 0-9 to fill it
     // SECOND Lobby to pull in value 
     if (Serial.available() > 0) 
     incomingByte = Serial.read();
     delay(100);
     // hang out here while we wait for keystroke WAIT STUCK IN LOOP     
   }
   
   if (MENUSELECT == 116) {  // this is 't'
   THRESHOLD = incomingByte-48; // ascii to int (sub 48)
   }
   else if (MENUSELECT == 100) { //this is "d"
   DECAY = incomingByte-48;  // ascii to int
   }
   else {
   Serial.println("Nothing Modified ! (fat fingers) ");
   }

   //Serial.print("MENUSELECT ");
   //Serial.write(MENUSELECT);
   Serial.println("SETTINGS:");
   Serial.println("=========");
   Serial.print("THRESHOLD ");
   Serial.println(THRESHOLD);
   Serial.print("DECAY ");  
   Serial.println(DECAY);
   
   Serial.println("");
   Serial.println("update complete");    
  
  

  
  }  // End of the Menu Cul-de-sac "if "m"
  
  
  } // End of the NORMAL - ongoing/running "if serial available loop - from here, its every time
  
}  // bookends the primary void main loop 
  
// google fusion table APIs (push to table)


