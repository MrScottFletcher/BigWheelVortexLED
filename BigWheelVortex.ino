/*
    Name:       BigWheelVortex.ino
    Created:	9/23/2019 10:15:57 PM
    Author:     Scott Fletcher
	With FULL credit to the internet for various sample, tutorials, and the like.
*/

#include <Adafruit_VL53L0X.h>
#include <FastLED.h>

#define LED_PIN     5
#define NUM_LEDS    300
#define STANDBY_BRIGHTNESS  64
#define NOT_MOVING_BRIGHTNESS  64
#define FULL_BRIGHTNESS  255
//#define BRIGHTNESS  128
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define POT_SAMPLECOUNT  3
#define UPDATES_PER_SECOND 100
#define SLOW_GLOW_SPEED 20

//ROTARY ENCODER
#define rotaryOutputA 2
#define rotaryOutputB 3
int rotaryCounter = 0;
int rotaryaState;
int rotaryaLastState;

//ROTARY POTS
volatile boolean potTurnDetected;
volatile boolean up;

const int potPinCLK = 2;                   // Used for generating interrupts using CLK signal
const int potPinDT = 3;                    // Used for reading DT signal
const int potPinSW = 4;                    // Used for the push button switch

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

#define IS_LEDCONTROLLER  0

int LEDPin = 9;  // Declare LEDPin to be arduino pin 9
int speedValue; // Use this variable for writing to LED

int modeSwitchPin = 7;  // the number of the mode input pin

bool DIRECTION_IS_TOWARDS = false;

float FULL_ROTATION_DELTA = 600;
float FULL_SPEED_DELTA_PER_SAMPLE = 300;
float SPEED_MULTIPLIER_RESET_DEFAULT = 74;
#define SPEED_SENSOR_SAMPLE_DELAY_MS 250

#include <RotaryEncoder.h>

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(A2, A3);


void setup() {
	delay(3000); // power-up safety delay
	
	//The same pins for both units
	pinMode(modeSwitchPin, INPUT_PULLUP); //set the mode switch

	pinMode(potPinCLK, INPUT);
	pinMode(potPinDT, INPUT);
	pinMode(potPinSW, INPUT);
	attachInterrupt(0, isr, FALLING);   // interrupt 0 is always connected to pin 2 on Arduino UNO

	//Serial.begin(57600);
	//Serial.println("SimplePollRotator example for the RotaryEncoder library.");


	if (IS_LEDCONTROLLER) {
		FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
		FastLED.setBrightness(STANDBY_BRIGHTNESS);

		SetupBlackAndWhiteStripedPalette();
		currentBlending = LINEARBLEND;

		Wire.begin(9);
		// Attach a function to trigger when something is received.
		Wire.onReceive(receiveEvent);
	}
	else {


		// You may have to modify the next 2 lines if using other pins than A2 and A3
		PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
		PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.

		Wire.begin();
	}
}

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
	encoder.tick(); // just call tick() to check the state.
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
		static int lastPos = 0;
		//VL53L0X_RangingMeasurementData_t measure;

		//long goodRangeSamples = 0;

		float currentDeltaMeasure = 0;
		long currentDeltaTotal = 0;
		long goodDeltaSamples = 0;

		static bool potChangeDetected = false;

		static long virtualPosition = SPEED_MULTIPLIER_RESET_DEFAULT;    // without STATIC it does not count correctly!!!

		if (!(digitalRead(potPinSW))) {      // check if pushbutton is pressed
			virtualPosition = SPEED_MULTIPLIER_RESET_DEFAULT;              // if YES, then reset counter to ZERO
			//Serial.print("Reset = ");      // Using the word RESET instead of COUNT here to find out a buggy encoder
			//Serial.println(virtualPosition);
		}

		//Actually up is down for some reason.  Reverse the incrementer
		if (potTurnDetected) {		    // do this only if rotation was detected
			if (up)
				virtualPosition--;
			else
				virtualPosition++;
			potTurnDetected = false;          // do NOT repeat IF loop until new rotation detected
			//Serial.print("Count = ");
			//Serial.println(virtualPosition);
		}
		if (virtualPosition > 255)
			virtualPosition = 255;
		if (virtualPosition < 0)
			virtualPosition = 0;

		float currentPotMeasure = virtualPosition;

		int modeSwitchState = digitalRead(modeSwitchPin);

		if (modeSwitchState == HIGH)
		{
			//############################################################
			//Using the rotary encoder
			//============================================================

			int newPos1 = encoder.getPosition();
			delay(SPEED_SENSOR_SAMPLE_DELAY_MS);
			int newPos2 = encoder.getPosition();
			if (newPos1 != newPos2) {
				//Serial.print(newPos1);
				//Serial.println();

				//END OF RANGEFINDER_SAMPLECOUNT LOOP
				//-----------------------------------------------
				//Test the samples and average them
				if (DIRECTION_IS_TOWARDS)
				{
					currentDeltaMeasure = newPos2 - newPos1;
				}
				else
				{
					currentDeltaMeasure = newPos1 - newPos2;
				}
				lastPos = newPos2;
			}
			else {
				//no diff.  Speed is zero.
				currentDeltaMeasure = 0;
			}
		}
		else
		{
			//--------------------------------------
			//For manually testing the speed using the dial
			currentDeltaMeasure = currentPotMeasure;
			if(currentDeltaMeasure < 0)
				currentDeltaMeasure = 0;

			//delay just like we were actually sampling
			delay(SPEED_SENSOR_SAMPLE_DELAY_MS);

			if (currentDeltaMeasure < 5) {
				//ignore small noise
				currentDeltaMeasure = 0;
			}
		}

		//--------------------------------------

		if (lastDeltaMeasure != currentDeltaMeasure) {

			//ignore bacwards
			if (currentDeltaMeasure < 0)
				currentDeltaMeasure = 0;

			//send the message
			//Speed is 0-255
			//MAX Speed is determined is a closing distance of something like 300mm
			//determined by the POT value
			//Do 1/2 of the potReadMax of 1024 to create a centerpoint of +/- adjustment
			//long speedMultiplier = FULL_SPEED_DELTA_PER_SAMPLE * (currentPotMeasure / 255.);

			if (modeSwitchState == HIGH)
			{
				//If in auto, Do the speed multiplier
				speed = currentDeltaMeasure * ((currentPotMeasure * 5) / 255.);
			}
			else {
				//manually set with the dial above
				speed = (currentDeltaMeasure - SPEED_MULTIPLIER_RESET_DEFAULT) * 30;
			}
			if (speed > 254) {
				speed = 254;
			}
			if (speed < 0) {
				speed = 0;
			}
			//send the speed value over I2C
			Wire.beginTransmission(9); // transmit to device #9
			Wire.write(speed);              // sends x 
			Wire.endTransmission();    // stop transmitting

			//only remember if we got a good change detection
			lastDeltaMeasure = currentDeltaMeasure;
		}
	}
}


void isr() {                    // Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
	if (digitalRead(potPinCLK))
		up = digitalRead(potPinDT);
	else
		up = !digitalRead(potPinDT);
	potTurnDetected = true;
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


			static bool potChangeDetected = false;
			static long virtualPosition = SPEED_MULTIPLIER_RESET_DEFAULT;    // without STATIC it does not count correctly!!!

			if (!(digitalRead(potPinSW))) {      // check if pushbutton is pressed
				virtualPosition = SPEED_MULTIPLIER_RESET_DEFAULT;              // if YES, then reset counter to ZERO
				//Serial.print("Reset = ");      // Using the word RESET instead of COUNT here to find out a buggy encoder
				//Serial.println(virtualPosition);
			}

			//Actually up is down for some reason.  Reverse the incrementer
			if (potTurnDetected) {		    // do this only if rotation was detected
				if (up)
					virtualPosition = virtualPosition - 4;
				else
					virtualPosition = virtualPosition + 4;

				if (virtualPosition > 255)
					virtualPosition = 255;
				if (virtualPosition < 0)
					virtualPosition = 0;

				potTurnDetected = false;          // do NOT repeat IF loop until new rotation detected
				//Serial.print("Count = ");
				//Serial.println(virtualPosition);
			}

			if (virtualPosition > 2) {
				standbyGlowSpeed = virtualPosition + 1;
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

