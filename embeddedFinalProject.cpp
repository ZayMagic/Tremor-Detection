//Embedded Challenge Spring 2024

#include <Adafruit_CircuitPlayground.h>
#include <ArduinoFFT.h> // Ensure this library is installed via PlatformIO

// Constants for error handling
const double MIN_ACCELERATION = -10.0; // Minimum expected accelerometer reading
const double MAX_ACCELERATION = 10.0;  // Maximum expected accelerometer reading
const double DEFAULT_FREQUENCY = -1.0; // Default frequency in case of errors



// Constants for FFT
//const uint16_t samples = 128; // Total samples for FFT
const uint16_t samples = 128; // Total samples for FFT

//const double samplingFrequency = 25.0; 
const double samplingFrequency = 25; // Sampling frequency in Hz


// FFT variables
ArduinoFFT<double> FFT; // Specify the type for Arduino FFT template
double vReal[samples];
double vImag[samples];


unsigned long samplingPeriod;
int brightness = 128; // Initial brightness level, set to 50% of maximum
int count = 0; // counter for samples

// Function declarations
bool takeSamples();
double analyzeFrequency();
void displayResults(double frequency, int& count);


void setup() {
  Serial.begin(115200);
  CircuitPlayground.begin();
  samplingPeriod = round(1000000 * (1.0 / samplingFrequency));
}


void loop() {
    if (CircuitPlayground.leftButton()) {
    brightness = 50; // Decrease brightness
  }
  if (CircuitPlayground.rightButton()) {
    brightness = 255; // Increase brightness
  }
  if (takeSamples()) {
    Serial.print("Current Time: ");
    Serial.println(millis()/1000);
    double frequency = analyzeFrequency();
    displayResults(frequency,count);
  }
}


bool takeSamples() {
  static unsigned long lastTime = 0;
  static uint16_t index = 0;
  if (micros() - lastTime >= samplingPeriod) {
    lastTime = micros();
    vReal[index] = CircuitPlayground.motionX(); // Get X-axis acceleration
    //Serial.print(">val:");
    //Serial.println(CircuitPlayground.motionX());
    vImag[index] = 0; // Imaginary part is zero
    index++;
    if (index >= samples) {
      index = 0;
      return true; // Sampling complete
    }
  }
  return false; // Sampling not complete
  Serial.println("Fail taking sample");
}


double analyzeFrequency() {
  //process the collected data to handle discontinutouty 
  //windowing: reduce spectral leakage by tapering the beginning and end of the signal data.
  FFT.windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  //compute: compute FFT of the input signal
  FFT.compute(vReal, vImag, samples, FFT_FORWARD);
  //complexToMagnitude: Converts the compute FFT output into magnitude spectrum
  FFT.complexToMagnitude(vReal, vImag, samples);
  //peak: find the freq with the highest magnitude in the FFT output 
  int peak = FFT.majorPeak(vReal, samples, samplingFrequency);
  double amplitude = vReal[peak];
  //Serial print for debuging
  Serial.print(">Detected frequency:");
  Serial.println(peak);
  Serial.print(">Detected Amplitude:");
  Serial.println(vReal[peak]);
  //if the amplitude of the movement is > 1.5
  //validate it as a peak
  if(amplitude>=0.5){
    return peak;
  }
  // if the amplitude is smaller than 1.5, disregard
  else {
    return 0;
  }
}

//void displayResults(double frequency) {
void displayResults(double frequency, int& count) {
  //if the frequency we detected is within the range
  if (frequency >= 3.0 && frequency <= 6.0) {
    //increse the cuonter by 1
   count+=1;
  }
  Serial.print("counter: "); //testing line
  Serial.println(count);    //testing line
  static unsigned long lastTime = 0;
  static uint16_t index = 0;
  if (millis() - lastTime >= 180000) { //set a windows size to 3 mins
    lastTime = millis();
    if (count>= 24) {  //if more than 8 times that our detected frequency falls in the range of 3-6, we considered it as a tremour
      for (int i = 0; i < 10; i++) {
        CircuitPlayground.setPixelColor(i, brightness, 0, 0); // Red color for detected tremor
        Serial.print("Current Time: ");
        Serial.println(millis()/1000);
      }
    }
    else {
      for (int i = 0; i < 10; i++) {
        CircuitPlayground.setPixelColor(i, 0, brightness, 0); // Green color for no tremor
        Serial.print("Current Time: ");
        Serial.println(millis()/1000);
      }
    }
    CircuitPlayground.strip.show(); // Correct function to update the LEDs
    delay(1000);
    CircuitPlayground.clearPixels();
    Serial.println("count reset"); //reset the counter for the next window
    count = 0;
 }
}



