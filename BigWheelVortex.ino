// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       BigWheelVortex.ino
    Created:	9/23/2019 10:15:57 PM
    Author:     DESKTOP-62IO2EV\scott
*/

// Define User Types below here or use a .h file
//


// Define Function Prototypes that use User Types below here or use a .h file
//


// Define Functions below here or use other .ino or cpp files
//


//#include <vl53l0x_types.h>
//#include <vl53l0x_tuning.h>
//#include <vl53l0x_platform_log.h>
//#include <vl53l0x_platform.h>
//#include <vl53l0x_interrupt_threshold_settings.h>
//#include <vl53l0x_i2c_platform.h>
//#include <vl53l0x_device.h>
//#include <vl53l0x_def.h>
//#include <vl53l0x_api_strings.h>
//#include <vl53l0x_api_ranging.h>
//#include <vl53l0x_api_core.h>
//#include <vl53l0x_api_calibration.h>
//#include <vl53l0x_api.h>
#include <Adafruit_VL53L0X.h>
#include <FastLED.h>

#define LED_PIN     5
#define NUM_LEDS    30
#define STANDBY_BRIGHTNESS  64
#define NOT_MOVING_BRIGHTNESS  128
#define FULL_BRIGHTNESS  255
//#define BRIGHTNESS  128
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define RANGEFINDER_SAMPLECOUNT  2
#define DELTA_AVERAGE_SAMPLECOUNT  3
#define POT_SAMPLECOUNT  3
#define UPDATES_PER_SECOND 100
#define SLOW_GLOW_SPEED 20

// This example shows several ways to set up and use 'palettes' of colors
// with FastLED.
//
// These compact palettes provide an easy way to re-colorize your
// animation on the fly, quickly, easily, and with low overhead.
//
// USING palettes is MUCH simpler in practice than in theory, so first just
// run this sketch, and watch the pretty lights as you then read through
// the code.  Although this sketch has eight (or more) different color schemes,
// the entire sketch compiles down to about 6.5K on AVR.
//
// FastLED provides a few pre-configured color palettes, and makes it
// extremely easy to make up your own color schemes with palettes.
//
// Some notes on the more abstract 'theory and practice' of
// FastLED compact palettes are at the bottom of this file.




CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

//################################################################################################
// TIME OF FLIGHT DEMO
//Adafruit_VL53L0X lox = Adafruit_VL53L0X();
//
//void setup() {
//	Serial.begin(115200);
//
//	// wait until serial port opens for native USB devices
//	while (!Serial) {
//		delay(1);
//	}
//
//	Serial.println("Adafruit VL53L0X test");
//	if (!lox.begin()) {
//		Serial.println(F("Failed to boot VL53L0X"));
//		while (1);
//	}
//	// power 
//	Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
//}
//
//
//void loop() {
//	VL53L0X_RangingMeasurementData_t measure;
//
//	Serial.print("Reading a measurement... ");
//	lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
//
//	if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//		Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
//	}
//	else {
//		Serial.println(" out of range ");
//	}
//
//	delay(100);
//}

//################################################################################################
// FAST LED DEMO -
//
//void setup() {
//	delay(3000); // power-up safety delay
//	FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
//	FastLED.setBrightness(BRIGHTNESS);
//
//	currentPalette = RainbowColors_p;
//	currentBlending = LINEARBLEND;
//}
//
//
//void loop()
//{
//	ChangePalettePeriodically();
//
//	static uint8_t startIndex = 0;
//	startIndex = startIndex + 1; /* motion speed */
//
//	FillLEDsFromPaletteColors(startIndex);
//
//	FastLED.show();
//	FastLED.delay(1000 / UPDATES_PER_SECOND);
//}
//###################################################################################
//LED ADJUST WITH POT
//int potPin = A0;  //Declare potPin to be analog pin A0
//int LEDPin = 9;  // Declare LEDPin to be arduino pin 9
//int readValue;  // Use this variable to read Potentiometer
//int writeValue; // Use this variable for writing to LED
//
//void setup() {
//	pinMode(potPin, INPUT);  //set potPin to be an input
//	pinMode(LEDPin, OUTPUT); //set LEDPin to be an OUTPUT
//	Serial.begin(9600);      // turn on Serial Port
//}
//
//void loop() {
//
//	readValue = analogRead(potPin);  //Read the voltage on the Potentiometer
//	writeValue = (255. / 1023.) * readValue; //Calculate Write Value for LED
//	analogWrite(LEDPin, writeValue);      //Write to the LED
//	Serial.print("You are writing a value of ");  //for debugging print your values
//	Serial.println(writeValue);
//
//}
//################################################################################################
// FAST LED ADJUSTABLE -
//

//For pushbutton logic
	// if the input just went from LOW and HIGH and we've waited long enough
	// to ignore any noise on the circuit, toggle the output pin and remember
	// the time
	//if (reading == HIGH && previous == LOW && millis() - time > debounce) {
	//	if (state == HIGH)
	//		state = LOW;
	//	else
	//		state = HIGH;
	//	time = millis();
	//}

#define IS_LEDCONTROLLER  0

int potPin = A0;  //Declare potPin to be analog pin A0
int LEDPin = 9;  // Declare LEDPin to be arduino pin 9
int potReadValue;  // Use this variable to read Potentiometer
int speedValue; // Use this variable for writing to LED

int switchPin = 2;         // the number of the input pin
//int outPin = 13;       // the number of the output pin

int modeSwitchState = HIGH;      // the current state of the output pin
int previousModeSwitchPos = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 500;   // the debounce time, increase if the output flickers

bool DIRECTION_IS_TOWARDS = true;

int MAX_SPEED_DISTANCE_DELTA_MM = 300;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setup() {
	delay(3000); // power-up safety delay
	
	if (IS_LEDCONTROLLER) {
		FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
		FastLED.setBrightness(STANDBY_BRIGHTNESS);

		SetupBlackAndWhiteStripedPalette();
		currentBlending = LINEARBLEND;
		pinMode(switchPin, INPUT_PULLUP); //set the mode switch

		Wire.begin(9);
		// Attach a function to trigger when something is received.
		Wire.onReceive(receiveEvent);
		//Wire.onRequest(requestEvent);

	}
	else {
		lox.begin();
		Wire.begin();
		pinMode(potPin, INPUT);  //set potPin to be an input
	}
	//Serial.begin(9600);      // turn on Serial Port

}


void loop()
{
	if (!IS_LEDCONTROLLER) {
		//Get the speed rating and communicate to the LED Controller
		EnterSpeedSensorLoop();
	}
	else {
		//Handle the speed value and control the LEDs accordingly
		EnterLEDLoop();
	}
}

void EnterSpeedSensorLoop() {
	while (1) {
		//static int16_t direction = 1;
		static int16_t lastDeltaMeasure = 0;
		static int16_t speed = 0;
		static int16_t lastSpeed = 0;
		static bool manualMode = false;
		VL53L0X_RangingMeasurementData_t measure;

		long goodRangeSamples = 0;

		float currentDeltaMeasure = 0;
		long currentDeltaTotal = 0;
		long goodDeltaSamples = 0;

		static bool goodChangeDetected = false;

		goodChangeDetected = false;

		if (manualMode == false)
		{
			//############################################################
			//Using the TIME OF FLIGHT Sensor
			//============================================================
			//-----------------------------------------------
			//LOOP TO GET SAMPLES FROM RANGE FINDER
			int16_t lastMeasure = 0;
			for (size_t i = 0; i < DELTA_AVERAGE_SAMPLECOUNT; i++)
			{
				//reset our count of good range samples of each bigger loop
				uint16_t lastSample = 0;
				uint16_t thisSample = 0;
				long currentMeasure = 0;
				goodRangeSamples = 0;
				//-----------------------------------------------
				//LOOP TO GET SAMPLES FROM RANGE FINDER
				for (size_t x = 0; x < RANGEFINDER_SAMPLECOUNT; x++)
				{
					//need at least two consecutive in the same direction, 
					//then 

					lox.rangingTest(&measure, true);
					if (measure.RangeStatus != 4)
					{
						thisSample = measure.RangeMilliMeter;

						//Only measure getting closer....
						//also allow last sample to be zero.
						if (lastSample == 0)
						{
							if (thisSample != 0)
							{
								lastSample = thisSample;
								currentMeasure += thisSample;
								goodRangeSamples++;
							}
						}
						else
						{
							if ((DIRECTION_IS_TOWARDS && lastSample >= thisSample - 40) ||
								lastSample <= thisSample +40)
							{
								currentMeasure += thisSample;
								goodRangeSamples++;
							}
							else
							{
								//throw this loop away;
								goodRangeSamples = 0;
								break;
							}
						}

						lastSample = thisSample;
					}
				}
				//END OF RANGEFINDER_SAMPLECOUNT LOOP
				//-----------------------------------------------
				//Test the samples and average them
				if (goodRangeSamples > 0)
				{
					currentMeasure = currentMeasure / goodRangeSamples;

					//We need at least one pass to make a diff.
					if (lastMeasure != 0)
					{
						if (currentMeasure < 5)
						{
							//ignore small noise
						}
						else
						{
							if (DIRECTION_IS_TOWARDS)
							{
								currentDeltaTotal += lastMeasure - currentMeasure;
							}
							else
							{
								currentDeltaTotal += currentMeasure - lastMeasure;
							}
							goodDeltaSamples++;
						}
					}

					lastMeasure = currentMeasure;
				}
				//-----------------------------------------------
			}
			//END OF DELTA_AVERAGE_SAMPLECOUNT LOOP
			//============================================================
			if (goodDeltaSamples > 0)
			{
				currentDeltaMeasure = currentDeltaTotal / goodDeltaSamples;
				goodChangeDetected = true;

				//Since the TOWARDS direct gives negative deltas...
			}
			//############################################################
		}
		else
		{
			//--------------------------------------
			//For manually testing the speed using the dial
			//potReadValue = analogRead(potPin);  //Read the voltage on the Potentiometer
			//if (potReadValue > 2) {
			//	deltaMeasure = ((255. / 1023.) * potReadValue) + 1;
			//}

			long currentMeasure = 0;
			for (size_t i = 0; i < POT_SAMPLECOUNT; i++)
			{
				potReadValue = analogRead(potPin);  //Read the voltage on the Potentiometer

				currentMeasure += ((255. / 1023.) * potReadValue) + 1;
				goodRangeSamples++;
			}

			if (goodRangeSamples > 0) {
				currentMeasure = currentMeasure / goodRangeSamples;

				//Since we are manually setting the speed, just take the value.
				currentDeltaMeasure = currentMeasure;

				if (currentDeltaMeasure < 5) {
					//ignore small noise
					currentDeltaMeasure = 0;
				}
				//always when manual
				goodChangeDetected = true;
			}
			else {
				currentDeltaMeasure = 0;
			}

		}

		//--------------------------------------

		if (goodChangeDetected && lastDeltaMeasure != currentDeltaMeasure) {

			//ignore bacwards
			if (currentDeltaMeasure < 0)
				currentDeltaMeasure = 0;

			//send the message
			//Speed is 0-255
			//MAX Speed is determined is a closing distance of 300mm
			//speed = 254 * (deltaMeasure / MAX_SPEED_DISTANCE_DELTA_MM);
			speed = 254 * (currentDeltaMeasure / 256.);
			if (speed > 254) {
				speed = 254;
			}
			//send the speed value over I2C
			Wire.beginTransmission(9); // transmit to device #9
			Wire.write(speed);              // sends x 
			Wire.endTransmission();    // stop transmitting

			//only remember if we got a good change detection
			lastDeltaMeasure = currentDeltaMeasure;
		}

		delay(200);
	}
}

void EnterLEDLoop() {
	while (1) {
		static uint8_t startIndex = 0;
		static uint8_t localSpeedValue = 0;
		static uint8_t standbyGlowSpeed = SLOW_GLOW_SPEED;
		localSpeedValue = speedValue;

		modeSwitchState = digitalRead(switchPin);
		previousModeSwitchPos = modeSwitchState;
		startIndex = startIndex + 1; /* motion speed */

		if (modeSwitchState == HIGH) {
			//-------   SPIN!!!  -------------------
			if (localSpeedValue > 2) {
				FastLED.setBrightness(FULL_BRIGHTNESS);
				if (localSpeedValue > 200) {
					// SUPER FAST SPIN
					currentPalette = RainbowStripeColors_p;
					currentBlending = LINEARBLEND;
				}
				else {
					//REGULAR SPIN
					SetupBlackAndWhiteStripedPalette();
					currentBlending = LINEARBLEND;
				}
			}
			else {
				//NOT MOVING Glow
				FastLED.setBrightness(NOT_MOVING_BRIGHTNESS);
				currentPalette = PartyColors_p;
				currentBlending = LINEARBLEND;
				localSpeedValue = SLOW_GLOW_SPEED;
			}
		}
		else {
			//-------   STANDBY  -------------------
			FastLED.setBrightness(STANDBY_BRIGHTNESS);
			startIndex = startIndex + 1; /* motion speed */
			currentPalette = RainbowColors_p;
			currentBlending = LINEARBLEND;

			potReadValue = analogRead(potPin);  //Read the voltage on the Potentiometer
			if (potReadValue > 2) {
				standbyGlowSpeed = ((255. / 1023.) * potReadValue) + 1;
				if (standbyGlowSpeed > 255)
					standbyGlowSpeed = 255;
				else if (standbyGlowSpeed < 1)
					standbyGlowSpeed = 1;
			}
			else {
				standbyGlowSpeed = SLOW_GLOW_SPEED;
			}
			localSpeedValue = standbyGlowSpeed;
		}

		FillLEDsFromPaletteColors(startIndex);
		FastLED.show();
		FastLED.delay(1000 / localSpeedValue);
	}
}

void receiveEvent(int bytes) {
	static uint8_t localReadSpeedValue = 0; 
	// read one character from the I2C
	localReadSpeedValue = Wire.read();
	if (localReadSpeedValue == 0)
		localReadSpeedValue = 1;
	else {
		if (localReadSpeedValue > 254) {
			localReadSpeedValue = 254;
		}
	}
	speedValue = localReadSpeedValue;    
}

//###################################################################################

//Adafruit_VL53L0X lox = Adafruit_VL53L0X();
//
//void setup() {
//	delay(3000); // power-up safety delay
//	FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
//	FastLED.setBrightness(BRIGHTNESS);
//
//	currentPalette = RainbowColors_p;
//	currentBlending = LINEARBLEND;
//
//	lox.begin();
//}
//
//
//void loop()
//{
//	ChangePalettePeriodically();
//
//	static uint8_t startIndex = 0;
//	static uint8_t currentSpeed = 100;
//
//	startIndex = startIndex + 1; /* motion speed */
//
//	VL53L0X_RangingMeasurementData_t measure;
//	if(startIndex % 5 == 0)
//		lox.rangingTest(&measure, false);
//
//	FillLEDsFromPaletteColors(startIndex);
//
//	FastLED.show();
//	FastLED.delay(1000 / UPDATES_PER_SECOND);
//}

//################################################################################################

//Adafruit_VL53L0X lox = Adafruit_VL53L0X();
//
//void setup() {
//	delay(3000); // power-up safety delay
//	FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
//	FastLED.setBrightness(BRIGHTNESS);
//
//	currentPalette = RainbowColors_p;
//	currentBlending = LINEARBLEND;
//
//	lox.begin();
//
//		//Serial.begin(115200);
//	
//		//// wait until serial port opens for native USB devices
//		//while (!Serial) {
//		//	delay(1);
//		//}
//	
//		//Serial.println("Adafruit VL53L0X test");
//		//	Serial.println(F("Failed to boot VL53L0X"));
//		//	while (1);
//		//}
//		// power 
//		//if (!lox.begin()) {
//		//Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
//
//	SetupBlackAndWhiteStripedPalette();
//	currentBlending = LINEARBLEND;
//
//}
//
//
//void loop()
//{
//	//ChangePalettePeriodically();
//
//	static int16_t direction = 1;
//
//	static uint8_t startIndex = 100000;
//
//	static int16_t deltaMeasure = 0;
//	static int16_t lastMeasure = 0;
//	static int16_t currentMeasure = 0;
//	static int16_t goodSamples = 0;
//
//	VL53L0X_RangingMeasurementData_t measure;
//
//	if (deltaMeasure > 0) {
//		startIndex = startIndex + direction; /* motion speed */
//	}
//
//	currentMeasure = 0;
//	goodSamples = 0;
//
//	for (size_t i = 0; i < SAMPLECOUNT; i++)
//	{
//		lox.rangingTest(&measure, false);
//		if (measure.RangeStatus != 4) {
//			goodSamples++;
//			currentMeasure += measure.RangeMilliMeter;
//		}
//	}
//
//	if (goodSamples > 0) {
//		currentMeasure = currentMeasure / goodSamples;
//
//		deltaMeasure = currentMeasure - lastMeasure;
//
//		if (deltaMeasure < 0) {
//			//flip to positive
//			deltaMeasure = deltaMeasure * -1;
//			direction = -1;
//		}
//		else {
//			direction = 1;
//		}
//	}
//
//	lastMeasure = currentMeasure;
//
//	if (deltaMeasure > 0) {
//		FillLEDsFromPaletteColors(startIndex);
//		FastLED.show();
//		FastLED.delay(500 / deltaMeasure);
//	}
//}


//###################################################################################
void FillLEDsFromPaletteColors(uint8_t colorIndex)
{
	uint8_t brightness = 255;

	for (int i = 0; i < NUM_LEDS; i++) {
		leds[i] = ColorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
		colorIndex += 3;
	}
}


// There are several different palettes of colors demonstrated here.
//
// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
//
// Additionally, you can manually define your own color palettes, or you can write
// code that creates color palettes on the fly.  All are shown here.

void ChangePalettePeriodically()
{
	uint8_t secondHand = (millis() / 1000) % 60;
	static uint8_t lastSecond = 99;

	if (lastSecond != secondHand) {
		lastSecond = secondHand;
		if (secondHand == 0) { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
		if (secondHand == 10) { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND; }
		if (secondHand == 15) { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
		if (secondHand == 20) { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
		if (secondHand == 25) { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
		if (secondHand == 30) { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
		if (secondHand == 35) { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
		if (secondHand == 40) { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
		if (secondHand == 45) { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
		if (secondHand == 50) { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND; }
		if (secondHand == 55) { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
	}
}

// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette()
{
	for (int i = 0; i < 16; i++) {
		currentPalette[i] = CHSV(random8(), 255, random8());
	}
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette()
{
	// 'black out' all 16 palette entries...
	fill_solid(currentPalette, 16, CRGB::Black);
	// and set every fourth one to white.
	currentPalette[0] = CRGB::White;
	currentPalette[4] = CRGB::White;
	currentPalette[8] = CRGB::White;
	currentPalette[12] = CRGB::White;

}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
	CRGB purple = CHSV(HUE_PURPLE, 255, 255);
	CRGB green = CHSV(HUE_GREEN, 255, 255);
	CRGB black = CRGB::Black;

	currentPalette = CRGBPalette16(
		green, green, black, black,
		purple, purple, black, black,
		green, green, black, black,
		purple, purple, black, black);
}


// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
	CRGB::Red,
	CRGB::Gray, // 'white' is too bright compared to red and blue
	CRGB::Blue,
	CRGB::Black,

	CRGB::Red,
	CRGB::Gray,
	CRGB::Blue,
	CRGB::Black,

	CRGB::Red,
	CRGB::Red,
	CRGB::Gray,
	CRGB::Gray,
	CRGB::Blue,
	CRGB::Blue,
	CRGB::Black,
	CRGB::Black
};



// Additional notes on FastLED compact palettes:
//
// Normally, in computer graphics, the palette (or "color lookup table")
// has 256 entries, each containing a specific 24-bit RGB color.  You can then
// index into the color palette using a simple 8-bit (one byte) value.
// A 256-entry color palette takes up 768 bytes of RAM, which on Arduino
// is quite possibly "too many" bytes.
//
// FastLED does offer traditional 256-element palettes, for setups that
// can afford the 768-byte cost in RAM.
//
// However, FastLED also offers a compact alternative.  FastLED offers
// palettes that store 16 distinct entries, but can be accessed AS IF
// they actually have 256 entries; this is accomplished by interpolating
// between the 16 explicit entries to create fifteen intermediate palette
// entries between each pair.
//
// So for example, if you set the first two explicit entries of a compact 
// palette to Green (0,255,0) and Blue (0,0,255), and then retrieved 
// the first sixteen entries from the virtual palette (of 256), you'd get
// Green, followed by a smooth gradient from green-to-blue, and then Blue.
