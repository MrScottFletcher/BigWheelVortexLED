/*
    Name:       BigWheelVortex.ino
    Created:	9/23/2019 10:15:57 PM
    Author:     Scott Fletcher
	With FULL credit to the internet for various sample, tutorials, and the like.
*/

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
#define DELTA_AVERAGE_SAMPLECOUNT  2
#define POT_SAMPLECOUNT  3
#define UPDATES_PER_SECOND 100
#define SLOW_GLOW_SPEED 20
#define SPEED_SENSOR_LOOP_DELAY_MS 1

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

#define IS_LEDCONTROLLER  0

int potPin = A0;  //Declare potPin to be analog pin A0
int LEDPin = 9;  // Declare LEDPin to be arduino pin 9
int speedValue; // Use this variable for writing to LED

int modeSwitchPin = 2;  // the number of the mode input pin

bool DIRECTION_IS_TOWARDS = true;

float MAX_POSSIBLE_SPEED_DISTANCE_DELTA_MM = 1100;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setup() {
	delay(3000); // power-up safety delay
	
	if (IS_LEDCONTROLLER) {
		FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
		FastLED.setBrightness(STANDBY_BRIGHTNESS);

		SetupBlackAndWhiteStripedPalette();
		currentBlending = LINEARBLEND;
		pinMode(modeSwitchPin, INPUT_PULLUP); //set the mode switch

		Wire.begin(9);
		// Attach a function to trigger when something is received.
		Wire.onReceive(receiveEvent);
	}
	else {
		lox.begin();
		Wire.begin();
		pinMode(potPin, INPUT);  //set potPin to be an input
	}
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
		static bool potChangeDetected = false;

		goodChangeDetected = false;

		long potReadValue = 0;
		long currentPotMeasure = 0;
		long goodPotSamples = 0;
		for (size_t i = 0; i < POT_SAMPLECOUNT; i++)
		{
			potReadValue = analogRead(potPin);  //Read the voltage on the Potentiometer

			currentPotMeasure += ((255. / 1023.) * potReadValue) + 1;
			goodPotSamples++;
		}

		if (goodPotSamples > 0) {
			currentPotMeasure = currentPotMeasure / goodPotSamples;
		}

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
			currentDeltaMeasure = currentPotMeasure;

			if (currentDeltaMeasure < 5) {
				//ignore small noise
				currentDeltaMeasure = 0;
			}
			//always when manual
			goodChangeDetected = true;
		}

		//--------------------------------------

		if (goodChangeDetected && lastDeltaMeasure != currentDeltaMeasure) {

			//ignore bacwards
			if (currentDeltaMeasure < 0)
				currentDeltaMeasure = 0;

			//send the message
			//Speed is 0-255
			//MAX Speed is determined is a closing distance of something like 300mm
			//determined by the POT value
			//Do 1/2 of the potReadMax of 1024 to create a centerpoint of +/- adjustment
			long maxSpeedDistanceDelta = MAX_POSSIBLE_SPEED_DISTANCE_DELTA_MM * (potReadValue / 512.);
			speed = 254 * (currentDeltaMeasure / maxSpeedDistanceDelta);
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

		delay(SPEED_SENSOR_LOOP_DELAY_MS);
	}
}

void EnterLEDLoop() {
	int modeSwitchState = 0;
	int previousModeSwitchPos = 0;
	while (1) {
		static uint8_t startIndex = 0;
		static uint8_t localSpeedValue = 0;
		static uint8_t standbyGlowSpeed = SLOW_GLOW_SPEED;
		long potReadValue = 0;

		localSpeedValue = speedValue;
		modeSwitchState = digitalRead(modeSwitchPin);
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

