//#DECLARATIONS

//##Sampling Galvanometer

//****** sample size sets the 'grain' of the detector
// a larger size will smooth over small variations
// a smaller size will excentuate small changes
const byte samplesize = 10; //set sample array size
const byte analysize = samplesize - 1;  //trim for analysis array,
volatile unsigned long microseconds; //sampling timer
volatile byte sampleIndex = 0;
volatile unsigned long samples[samplesize];
float threshold = 1.71; //threshold multiplier
word value = 0;
byte rawOutput = 1; // raw biodata stream via serial data
String rawOutputGalv = ""; // Global variable to store raw data
float rawAverage = 0;
float rawStdevi = 0;
float rawThreshDelta = 0;
float rawDelta = 0;
unsigned long rawOutputTime = 0;
int rawOutputDelay = 0;

//Timing and tracking
unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
void sample();
void analyzeSample();

//OLED
//int buttonC = 5;
//int buttonB = 6;
//int buttonA = 9;




//#FUNCTIONS

//## Galvanometer
void sample(){
  if(sampleIndex < samplesize) {
    samples[sampleIndex] = micros() - microseconds;
    microseconds = samples[sampleIndex] + microseconds; //rebuild micros() value w/o recalling
    //micros() is very slow
    //try a higher precision counter
    //samples[index] = ((timer0_overflow_count << 8) + TCNT0) - microseconds;
    sampleIndex += 1;
  }
}

void analyzeSample(){
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;
  byte change = 0;

  if (sampleIndex >= samplesize) { //array is full
    unsigned long sampanalysis[analysize]; //copy to new array - is this needed?
    int setnote=0;
    for (byte i=0; i<analysize; i++){
      //skip first element in the array
      sampanalysis[i] = samples[i+1];  //load analysis table (due to volitle)
      //manual calculation
      if(sampanalysis[i] > maxim) { maxim = sampanalysis[i]; }
      if(sampanalysis[i] < minim) { minim = sampanalysis[i]; }
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];  //prep stdevi
    }

    //calculation
    averg = averg/analysize;
    stdevi = sqrt(stdevi / analysize - averg * averg); //calculate stdevu
    if (stdevi < 1) { stdevi = 1.0; } //min stdevi of 1
    delta = maxim - minim;

    //**********perform change detection
    if (delta > (stdevi * threshold)){
      change = 1;
    }
    //*********

    //rawOutput data output
    //write a value ever rawOutputDelay milliseconds to slow down the data flow
    //    also write if a change is detected at any time!
    if (rawOutput && (change || (currentMillis > rawOutputTime + rawOutputDelay))) {
      rawOutputTime = currentMillis;
      rawAverage = map(averg,0,600,0,100);
      rawStdevi = stdevi;
      rawThreshDelta = map(constrain(threshold*stdevi,0,300),0,300,0,100);
      rawDelta = map(delta,0,300,0,100);

      /*
      //string approach
      rawOutputGalv = "";
      rawOutputGalv += String(map(averg, 0, 600, 0, 100)) + ","; // average
      rawOutputGalv += String(stdevi) + ",";  // standard deviatoin
      rawOutputGalv += String(map(constrain(threshold * stdevi, 0, 300), 0, 300, 0, 100)) + ","; //threshold compare against delta
      rawOutputGalv += String(map(delta, 0, 300, 0, 100)) + ","; //delta
      rawOutputGalv += String(change * 90); //change detected
      */
      
      /*
      //print to serial

      Serial.print(map(averg,0,600,0,100)); Serial.print(","); //Average
      Serial.print(stdevi); Serial.print(","); //standard deviation
      Serial.print(map(constrain(threshold*stdevi,0,300),0,300,0,100)); Serial.print(","); //threshold compare against delta
      Serial.print(map(delta,0,300,0,100)); Serial.print(","); //delta
      Serial.print(change*90); //Serial.print(","); //change detected
      Serial.println(); //end of raw data packet
      */
    }

        /*
    // Gil's approach
    if(change){
      value = stdevi;
    }
    */

    //reset array for next sample
    sampleIndex = 0;
  }
}
