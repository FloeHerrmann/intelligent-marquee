#include <Streaming.h>
#include <Wire.h>
#include <Adafruit_HTU21DF.h>
#include <LCD03.h>

// - - - - - - - - - - - - - - - - - - - - - - - -

//#define TRACE_OUTPUT
//#define SIMULATION
#define SERIAL_OUTPUT
#define USE_LCD

// - - - - - - - - - - - - - - - - - - - - - - - -

// Pins for relays
#define RELAY_OUT 23
#define RELAY_IN 25
// States for Relay
#ifndef SIMULATION
	#define RELAY_ON LOW
	#define RELAY_OFF HIGH
#else
	#define RELAY_ON HIGH
	#define RELAY_OFF LOW
#endif
// Pins for buttons
#define BUTTON_OUT 27
#define BUTTON_IN 29
// States for Buttons
#define BUTTON_PRESSED LOW
// Cycle for measuring
#define MEASUREMENT_CYCLE 1000
// Duration for sunblind transition
#define SUNBLIND_DURATION 6500
// Sunblind states
#define SUNBLIND_STATE_IN 0
#define SUNBLIND_STATE_IN_RAIN 1
#define SUNBLIND_STATE_TRANSITION_IN 2
#define SUNBLIND_STATE_TRANSITION_OUT 3
#define SUNBLIND_STATE_OUT 4
// Input pin rain sensor
#define RAIN_SENSOR_INPUT A0
// Rain states
#define RAIN_REALLY_WET 	0	// SEHR NASS
#define RAIN_WET 			1	// NASS
#define RAIN_MOIST 			2	// FEUCHT
#define RAIN_DAMPISH 		3	// LEICHT FEUCHT
#define RAIN_DRY 			4	// TROCKEN
// Rain thresholds
#define RAIN_REALLY_WET_THRESHOLD 	204		// SEHR NASS
#define RAIN_WET_THRESHOLD 			409		// NASS
#define RAIN_MOIST_THRESHOLD 		614		// FEUCHT
#define RAIN_DAMPISH_THRESHOLD 		818		// LEICHT FEUCHT
#define RAIN_DRY_THRESHOLD 			1024	// TROCKEN
// Input pin wind speed sensor
#define WIND_SPEED_SENSOR_INPUT 2
#define WIND_SPEED_SENSOR_DEBOUNCE 10
// wind speed threshold
#define WIND_SPEED_THRESHOLD 10
// Control modes
#define CONTROL_MANUAL true
#define CONTROL_AUTOMATIC false

// - - - - - - - - - - - - - - - - - - - - - - - -

long sunblindPosition;
long sunblindState;
long sunblindPreviousState;

long sunblindTransitionInTime = 0;
long sunblindTransitionOutTime = 0;

bool sunblindTransitionInStart = false;
bool sunblindTransitionOutStart = false;

long measurementTime = 0;

bool controlHelper = false;

long rainSensorValue = 0;
long rainState = RAIN_DRY;
long rainPreviousState = RAIN_DRY;

float temperatureSensorValue = 0.0;
float humiditySensorValue = 0.0;

long windSensorMeasureTimer = 0;
long windSensorDebounceTimer = 0;
long windSensorValue = 0;
long windSensorCounter = 0;
float windSensorSpeed = 0.0;

// - - - - - - - - - - - - - - - - - - - - - - - -

byte degree[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

// - - - - - - - - - - - - - - - - - - - - - - - -

Adafruit_HTU21DF htu21df = Adafruit_HTU21DF();
LCD03 lcd;

// - - - - - - - - - - - - - - - - - - - - - - - -

void setup() {

	#ifdef SERIAL_OUTPUT
		Serial.begin(57600);
		Serial << "- - MARKISE - -" << endl;
	#endif

	pinMode( RELAY_OUT , OUTPUT );
	pinMode( RELAY_IN , OUTPUT );

	pinMode( BUTTON_IN , INPUT_PULLUP );
	pinMode( BUTTON_OUT , INPUT_PULLUP );

	digitalWrite( RELAY_OUT , RELAY_OFF );
	digitalWrite( RELAY_IN , RELAY_OFF );

	#ifdef USE_LCD
		// Initialise the LCD
		lcd.begin(20, 4);
		lcd.createChar( 0 , degree );
		lcd.backlight();

		// Write to the LCD
		lcd.setCursor( 0 , 0 );
		lcd.print( "********************");
		lcd.print( "* INTELL. MARKISE  *");
		lcd.print( "* KREATIVE KOEPFE  *");
		lcd.print( "********************");

		// Wait for 5 seconds
		delay(2500);

	#endif

	// Markise einfahren > Definierter Startzustand
	#ifdef SERIAL_OUTPUT
		Serial << "EINFAHREN (START)" << endl;
	#endif
	digitalWrite( RELAY_IN , RELAY_ON );
	delay( SUNBLIND_DURATION );
	digitalWrite( RELAY_IN , RELAY_OFF );
	sunblindState = SUNBLIND_STATE_IN;
	sunblindPosition = 0;
	#ifdef SERIAL_OUTPUT
		Serial << "EINGEFAHREN" << endl;
	#endif

	#ifndef SIMULATION

		if( !htu21df.begin() ) {
			#ifdef SERIAL_OUTPUT
				Serial.println( "ERROR: HTU21DF NOT FOUND!" );
			#endif
			#ifdef USE_LCD
				lcd.setCursor( 0 , 0 );
				lcd.print( "ERROR: " );lcd.newLine();
				lcd.print( "HTU21DF NOT FOUND" );
				delay( 2000 );
				lcd.clear();
			#endif
		}

		temperatureSensorValue = htu21df.readTemperature();
		humiditySensorValue = htu21df.readHumidity();

	#endif

	#ifdef USE_LCD

		lcd.clear();
		lcd.home();
		lcd.print( "TEMP: -" );

		lcd.setCursor( 0 , 1 );
		lcd.print( "LUFT: -" );

		lcd.setCursor( 0 , 2 );
		lcd.print( "FEUC: -" );

		lcd.setCursor( 0 , 3 );
		lcd.print( "WIND: -" );
	#endif

	pinMode( WIND_SPEED_SENSOR_INPUT , INPUT_PULLUP );
	attachInterrupt( digitalPinToInterrupt( WIND_SPEED_SENSOR_INPUT ) , WindsensorInterruptRoutine , CHANGE );
}

void loop() {

	// - - - - - - - - - - - - - - - - - - - - - - - -
	// EINFAHREN auf Knopfdruck
	if( digitalRead( BUTTON_IN ) == BUTTON_PRESSED ){
		if( sunblindPosition > 0 ) {
			controlHelper = CONTROL_MANUAL;
			if( sunblindTransitionInStart == false ) {
				sunblindTransitionInStart = true;
				sunblindTransitionInTime = millis();
				sunblindPreviousState = sunblindState;
				sunblindState = SUNBLIND_STATE_TRANSITION_IN;
				#ifdef SERIAL_OUTPUT
					Serial << "EINFAHREN (TASTER)" << endl;
				#endif
			} else {
				sunblindPosition -= millis() - sunblindTransitionInTime;
				#ifdef TRACE_OUTPUT
					Serial << "POSITION > " << sunblindState << "(" << sunblindPosition << ")" << endl;
				#endif
				if( sunblindPosition <= 0 ) {
					sunblindPreviousState = sunblindState;
					sunblindState = SUNBLIND_STATE_IN;
					#ifdef SERIAL_OUTPUT
						Serial << "EINGEFAHREN (TASTER)" << endl;
					#endif
				} else  {
					//sunblindPreviousState = sunblindState;
					sunblindState = SUNBLIND_STATE_TRANSITION_IN;
					sunblindTransitionInTime = millis();
				}
			}
			digitalWrite( RELAY_IN , RELAY_ON );
		} else {
			sunblindTransitionInStart = false;
			digitalWrite( RELAY_IN , RELAY_OFF );
		}
	} else {
		digitalWrite( RELAY_IN , RELAY_OFF );
		if( sunblindTransitionInStart == true ) {
			sunblindTransitionInStart = false;
			sunblindPosition -= millis() - sunblindTransitionInTime;
			#ifdef TRACE_OUTPUT
				Serial << "POSITION > " << sunblindState << "(" << sunblindPosition << ")" << endl;
			#endif
			if( sunblindPosition <= 0 ) {
				sunblindPreviousState = sunblindState;
				sunblindState = SUNBLIND_STATE_IN;
				#ifdef SERIAL_OUTPUT
					Serial << "EINGEFAHREN (TASTER)" << endl;
				#endif
			} else  {
				//sunblindPreviousState = sunblindState;
				sunblindState = SUNBLIND_STATE_TRANSITION_IN;
			}
		}
		controlHelper = CONTROL_AUTOMATIC;
	}

	// - - - - - - - - - - - - - - - - - - - - - - - -
	// Ausfahren auf Knopfdruck
	if( digitalRead( BUTTON_OUT ) == BUTTON_PRESSED ){
		if( sunblindPosition < SUNBLIND_DURATION ) {
			controlHelper = CONTROL_MANUAL;
			if( sunblindTransitionOutStart == false ) {
				sunblindTransitionOutStart = true;
				sunblindTransitionOutTime = millis();
				sunblindPreviousState = sunblindState;
				sunblindState = SUNBLIND_STATE_TRANSITION_OUT;
				#ifdef SERIAL_OUTPUT
					Serial << "AUSFAHREN (TASTER)" << endl;
				#endif
			} else {
				sunblindPosition += millis() - sunblindTransitionOutTime;
				#ifdef TRACE_OUTPUT
					Serial << "POSITION > " << sunblindState << "(" << sunblindPosition << ")" << endl;
				#endif
				if( sunblindPosition >= SUNBLIND_DURATION ) {
					sunblindPreviousState = sunblindState;
					sunblindState = SUNBLIND_STATE_OUT;
					#ifdef SERIAL_OUTPUT
						Serial << "AUSGEFAHREN (TASTER)" << endl;
					#endif
				} else  {
					//sunblindPreviousState = sunblindState;
					sunblindState = SUNBLIND_STATE_TRANSITION_OUT;
					sunblindTransitionOutTime = millis();
				}
			}
			digitalWrite( RELAY_OUT , RELAY_ON );
		} else {
			sunblindTransitionOutStart = false;
			digitalWrite( RELAY_OUT , RELAY_OFF );
		}
	} else {
		digitalWrite( RELAY_OUT , RELAY_OFF );
		if( sunblindTransitionOutStart == true ) {
			sunblindTransitionOutStart = false;
			sunblindPosition += millis() - sunblindTransitionOutTime;
			#ifdef TRACE_OUTPUT
				Serial << "POSITION > " << sunblindState << "(" << sunblindPosition << ")" << endl;
			#endif
			if( sunblindPosition >= SUNBLIND_DURATION ) {
				sunblindPreviousState = sunblindState;
				sunblindState = SUNBLIND_STATE_OUT;
				#ifdef SERIAL_OUTPUT
					Serial << "AUSGEFAHREN (TASTER)" << endl;
				#endif
			} else  {
				//sunblindPreviousState = sunblindState;
				sunblindState = SUNBLIND_STATE_TRANSITION_OUT;
				sunblindTransitionOutTime = millis();
			}
		}
		if( digitalRead( BUTTON_IN ) != BUTTON_PRESSED ) controlHelper = CONTROL_AUTOMATIC;
	}

	if( controlHelper == CONTROL_AUTOMATIC ){
		if( millis() - measurementTime >= MEASUREMENT_CYCLE ) {

			// Rain sensor
			rainSensorValue = analogRead( RAIN_SENSOR_INPUT );
			if( rainSensorValue >= 0 && rainSensorValue <= RAIN_REALLY_WET_THRESHOLD ) {
				if( rainState != RAIN_REALLY_WET ) rainPreviousState = rainState;
				rainState = RAIN_REALLY_WET;
			} else if( rainSensorValue > RAIN_REALLY_WET_THRESHOLD && rainSensorValue <= RAIN_WET_THRESHOLD ) {
				if( rainState != RAIN_WET ) rainPreviousState = rainState;
				rainState = RAIN_WET;
			} else if( rainSensorValue > RAIN_WET_THRESHOLD && rainSensorValue <= RAIN_MOIST_THRESHOLD ) {
				if( rainState != RAIN_MOIST ) rainPreviousState = rainState;
				rainState = RAIN_MOIST;
			} else if( rainSensorValue > RAIN_MOIST_THRESHOLD && rainSensorValue <= RAIN_DAMPISH_THRESHOLD ) {
				if( rainState != RAIN_DAMPISH ) rainPreviousState = rainState;
				rainState = RAIN_DAMPISH;
			} else if( rainSensorValue > RAIN_DAMPISH_THRESHOLD && rainSensorValue <= RAIN_DRY_THRESHOLD ){
				if( rainState != RAIN_DRY ) rainPreviousState = rainState;
				rainState = RAIN_DRY;
			}

			// Temperature sensor
			#ifdef SIMULATION
				temperatureSensorValue = random( 1000 , 2500 ) / 100.0f;
			#else
				htu21df.reset();
				temperatureSensorValue = htu21df.readTemperature();
			#endif

			// Humidity sensor
			#ifdef SIMULATION
				humiditySensorValue = random( 2000 , 10000 ) / 100.0f;
			#else
				humiditySensorValue = htu21df.readHumidity();
			#endif

			// Wind sensor speed
			if( windSensorValue == 0 ) windSensorSpeed = 0.0;
			else windSensorSpeed = ( (float)windSensorValue + 2.0 ) / 3.0;

			#ifdef USE_LCD
				lcd.home();
				lcd.print( "TEMP: " );
				lcd.print( temperatureSensorValue );
				lcd.write( 0 );
				lcd.print( "C" );

				lcd.setCursor( 0 , 1 );
				lcd.print( "LUFT: " );
				lcd.print( humiditySensorValue );
				lcd.print( "%   " );

				lcd.setCursor( 0 , 2 );
				lcd.print( "FEUC: " );
				switch( rainState ) {
					case RAIN_REALLY_WET:
						lcd.print( "SEHR NASS    " ); break;
					case RAIN_WET:
						lcd.print( "NASS         " ); break;
					case RAIN_MOIST:
						lcd.print( "FEUCHT       " ); break;
					case RAIN_DAMPISH:
						lcd.print( "LEICHT FEUCHT" ); break;
					case RAIN_DRY:
						lcd.print( "TROCKEN      " ); break;
				}

				lcd.setCursor( 0 , 3 );
				lcd.print( "WIND: " );
				lcd.print( windSensorSpeed );
				lcd.print( "m/s    " );
			#endif

			if( ( millis() - windSensorMeasureTimer ) > MEASUREMENT_CYCLE ) {
				windSensorValue = 0;
				windSensorSpeed = 0.0;
			}

			#ifdef SERIAL_OUTPUT
				printCurrentState();
			#endif

			checkState( sunblindState , sunblindPreviousState , rainState , rainPreviousState , windSensorValue );

			measurementTime = millis();
		}
	}
}

void checkState( long _sunblindState , long _sunblindPreviousState , long _rainState , long _rainPreviousState , long _windSensorValue ) {
	#ifdef SERIAL_OUTPUT
		Serial << _sunblindState << " / " << _sunblindPreviousState << " / " << _rainState << " / " << _rainPreviousState << " / " << _windSensorValue << endl;
	#endif

	if( _sunblindState > SUNBLIND_STATE_IN_RAIN && _windSensorValue >= WIND_SPEED_THRESHOLD ) {
		// Markise AUSGEFAHREN &&
		// Wind ÜBER GRENZWERT
		#ifdef SERIAL_OUTPUT
			Serial << "EINFAHREN (WIND)" << endl;
		#endif
		digitalWrite( RELAY_OUT , RELAY_OFF );
		digitalWrite( RELAY_IN , RELAY_ON );
		delay( sunblindPosition + 100 );
		digitalWrite( RELAY_IN , RELAY_OFF );
		sunblindPreviousState = sunblindState;
		sunblindState = SUNBLIND_STATE_IN;
		sunblindPosition = 0;
		#ifdef SERIAL_OUTPUT
			Serial << "EINGEFAHREN (WIND)" << endl;
		#endif
	} else {
		if( _sunblindState > SUNBLIND_STATE_IN_RAIN && _rainState <= RAIN_WET ) {
			// Markise AUSGEFAHREN &&
			// Regen-Status IST NASS oder SEHR NASS
			#ifdef SERIAL_OUTPUT
				Serial << "EINFAHREN (REGEN)" << endl;
			#endif
			digitalWrite( RELAY_OUT , RELAY_OFF );
			digitalWrite( RELAY_IN , RELAY_ON );
			delay( sunblindPosition + 100 );
			digitalWrite( RELAY_IN , RELAY_OFF );
			sunblindPreviousState = sunblindState;
			sunblindState = SUNBLIND_STATE_IN_RAIN;
			sunblindPosition = 0;
			#ifdef SERIAL_OUTPUT
				Serial << "EINGEFAHREN (REGEN)" << endl;
			#endif
		} else if( _sunblindState == SUNBLIND_STATE_IN_RAIN && _rainPreviousState <= RAIN_WET && _rainState >= RAIN_DAMPISH ) {
			// Markise IST EINGEFAHREN &&
			// Regen-Status WAR NASS oder SEHR NASS &&
			// Regen-Status IST TROCKEN oder LEICHT FEUCHT
			#ifdef SERIAL_OUTPUT
				Serial << "AUSFAHREN (TROCKEN)" << endl;
			#endif
			digitalWrite( RELAY_IN , RELAY_OFF );
			digitalWrite( RELAY_OUT , RELAY_ON );
			delay( SUNBLIND_DURATION + 100 );
			digitalWrite( RELAY_OUT , RELAY_OFF );
			sunblindPreviousState = sunblindState;
			sunblindState = SUNBLIND_STATE_OUT;
			sunblindPosition = SUNBLIND_DURATION;
			#ifdef SERIAL_OUTPUT
				Serial << "AUSGEFAHREN (TROCKEN)" << endl;
			#endif
		}
	}

	delay( 50 );
}

void WindsensorInterruptRoutine() {
	if( ( millis() - windSensorDebounceTimer ) > WIND_SPEED_SENSOR_DEBOUNCE ) {
		windSensorCounter++;

		if( ( millis() - windSensorMeasureTimer ) > MEASUREMENT_CYCLE ) {
			windSensorMeasureTimer = millis();
			windSensorValue = windSensorCounter;
			windSensorCounter = 0;
		}
	}
	windSensorDebounceTimer = millis();
}

#ifdef SERIAL_OUTPUT
	void printCurrentState(){
		// Aktueller Zustand
		switch( sunblindState ) {
			case SUNBLIND_STATE_IN:
				Serial << "EINGEFAHREN"; break;
			case SUNBLIND_STATE_IN_RAIN:
				Serial << "EINGEFAHREN (REGEN)"; break;
			case SUNBLIND_STATE_TRANSITION_IN:
				Serial << "EINFAHREN"; break;
			case SUNBLIND_STATE_TRANSITION_OUT:
				Serial << "AUSFAHREN"; break;
			case SUNBLIND_STATE_OUT:
				Serial << "AUSGEFAHREN"; break;
		}

		// Vorheriger Zustand
		switch( sunblindPreviousState ) {
			case SUNBLIND_STATE_IN:
				Serial << " (EINGEFAHREN)"; break;
			case SUNBLIND_STATE_IN_RAIN:
				Serial << " (EINGEFAHREN (REGEN))"; break;
			case SUNBLIND_STATE_TRANSITION_IN:
				Serial << " (EINFAHREN)"; break;
			case SUNBLIND_STATE_TRANSITION_OUT:
				Serial << " (AUSFAHREN)"; break;
			case SUNBLIND_STATE_OUT:
				Serial << " (AUSGEFAHREN)"; break;
		}

		Serial << " (POS: " << sunblindPosition << ") / ";

		// Regen Status
		switch( rainState ) {
			case RAIN_REALLY_WET:
				Serial << "SEHR NASS"; break;
			case RAIN_WET:
				Serial << "NASS"; break;
			case RAIN_MOIST:
				Serial << "FEUCHT"; break;
			case RAIN_DAMPISH:
				Serial << "LEICHT FEUCHT"; break;
			case RAIN_DRY:
				Serial << "TROCKEN"; break;
		}

		// Vorheriger Regen Status
		switch( rainPreviousState ) {
			case RAIN_REALLY_WET:
				Serial << " (SEHR NASS) / "; break;
			case RAIN_WET:
				Serial << " (NASS) / "; break;
			case RAIN_MOIST:
				Serial << " (FEUCHT) / "; break;
			case RAIN_DAMPISH:
				Serial << " (LEICHT FEUCHT) / "; break;
			case RAIN_DRY:
				Serial << " (TROCKEN) / "; break;
		}

		// Wind sensor value
		Serial << windSensorValue << " (" << windSensorSpeed << "m/s) / ";

		// Temperature value
		Serial << temperatureSensorValue << "°C / ";

		// Humidity value
		Serial << humiditySensorValue << "%";

		Serial << endl;
	}
#endif
