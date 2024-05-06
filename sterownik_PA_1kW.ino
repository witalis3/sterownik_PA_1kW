#include "Arduino.h"
#include <EEPROM.h>
#include <UTFT.h>
//#include <SoftwareSerial.h>
#include <UTouch.h>

#include "sterownik_PA_1kW.h"

/*
 Na bazie sterownika DC5ME:
 	 - wyświetlacz z buydisplay model ER-TFTM050-5 5 cali z kontrolerem SSD1963 i dotykiem rezystancyjnym (resistive touch);
 	 	 sterowanie na magistrali 16bit z Mega 2560
 	 	 	 - shield dla resistive:
 	 	 	 	 - J7, J8, J9, J10 otwarte
 	 	 	 	 - J11, J12, J13, J14 zwarte
 	 	 	 	 - J1, J2, J3 zwarte (software SPI)
 	 	 	 	 - J4, J5, J6 otwarte (hardware SPI)
 	 	 	 	 - J15, J16 zwarte (Due)
 	 	 	 	 - WAKE otwarty
 	 - biblioteki UTouch i UTFT ze strony sprzedawcy (buydisplay)
 	 - fonty ze strony rinkydinkelectronics
wersja dla SP3JDZ:
	- miernik częstotliwości na bazie :
	// Timer and Counter example for Mega2560
	// Author: Nick Gammon
	// Date: 24th April 2012
	 * // input on pin D47 (T5) Timer5 -> kolizja z BAND0 -> WE_FREQ

 wersja dla SP3OCC:
- pomiar temperatury tranzystorów jak w Wolfie, pomiar temperatury radiator a z płytki PA (LM35)
- całkowity pobór prądu z 5V (wyświetlacz, shield, arduino mega 2560) około 400mA; pobór prądu z 12V:?
- schemat sterownika: KICAD5 C:\KICAD\projekty\PA_500W\sterownik_PA_1kW_1_3_SP3OCC
- schemat zabezpieczenia: KICAD6 C:\KICAD\projekty\PA_500W\zabezpieczenie nadprądowe\protection_module_SP3OCC
- moduł PA A600 V3 Razvan (M0HZH)
- moduł LPF SP2FP wersja 600-800W
-----------------------------------------------------------------
 ToDo
 	 - ver. 1.11.0 wersja dla SP3JDZ
 	 	 - oblutowane! IDD
 	 	 - oblutowane! forward
 	 	 - reszta wejść na razie zasymulowana do testów miernika częstotliwości
 	 	 - zrobione! przełączanie pasm przenieść ze sterownik_FT810
 	 	 - wpleść pomiar freq ze zmiana pasma
 	 	 	 - graf!
 	 	 - implementacja miernika f
 	 	 	-  input on pin D47 (T5) Timer5 PL2 noga 37 -> kolizja z BAND0; złącze J11 pin 1 (masa pin 5 -> nietypowo) -> przełożyć BAND0 na inny pin
 	 	 	-
	- ver. 1.10.0 wersja dla SP3OCC
 	 	 - zrobione! pomiar prądu ACS713 30A
 	 	 - zrobione! pomiar temperatury tranzystorów: czujniki KTY81/110
 	 	 - zrobione! pomiar temperatury radiatora LM35
 	 	 - zrobione! we PTT
 	 	 - 400W pokazuje 225W
 	 	 - pokazuje 54,5V; uśrednianie

 	 	 - obsługa PTT
 	 	 	 - sprawdzenie, czy TS480 daje pik przy przejściu na nadawanie (ew. sekwencer)
 	 	 	 - przejście PTT przez sterownik
 	 	 	 	 - blokada PTT przy wystąpieniu błędu
 	 	 	 	 	 - komunikat
 	 	 	 	 	 - usunięcie komunikatu z panelu
 	 	 	 	 	 	 - usunięcie blokady
 	 	 - zrobione! wyjście PTT na przekaźniki i BIAS na PA (osobno)
 	 	 - zrobione! REF i FWD (SWR)
 	 	 - zrobione! pomiar 48V i 12V

 	 	 - sterowanie pracą wentylatora
 	 	 	 - na początek jeden stopień
 	 	 - alarm od przekroczenia SWR
 	 	 - informacje o błędach/zadziałaniu komparatorów z płytki blokady

	---------------------------------------------------------------------------------------------
	 - ver. 1.9.15 wybór trybu wyświetlania mocy na PINie: 2kW/500W (lub inne wybrane)
	 - ver. 1.9.14 poprawienie poprawki ;-)
	 - ver. 1.9.13 poprawienie błędu od oldBandIdx (nie przełączał LPFów)
 	 - ver. 1.9.12
 	 	 - poprawienie PEP; oldBandIdx; alternatywne we/wy
 	 - ver. 1.9.11
 	 	 - obsługa 3 tłumików
 	 - ver. 1.9.10
 	 	 - brak komunikatu podczas strojenia z ATU dla SWR powyżej Max (5.0)
 	 - ver. 1.9.9
 	 	 - brak reakcji na wartość ujemną prądu drenu
 	 - ver. 1.9.8
 	 	 - obsługa pomiaru prądu na ACS758 (prąd do wyboru np. 100A)
 	 	 - zamiana miejscami 50V z pomiarem prądu (50A)
 	 	 - na doPin_Czas_Petli (D2) wejście z ATU blokujące nadmierne SWRy na czas strojenia
 	 	 	 - stan wysoki blokuje
 	 - ver. 1.9.7
 	 	 - ;start "2 kW"
 	 	 - ;T1 i T2 w jednej linii na wyświetlaczu
 	 	 - ;T3 pod spodem i 14V
 	 - ver. 1.9.6 wersja z dwoma tranzystorami
 	 	 - termistor1 -> tranzystor1 -> Fan1
 	 	 - termistor2 -> tranzystor2 -> Fan1
 	 	 - termistor3 -> radiator -> Fan2
 	 - uwaga: doPin_SWR_ant daje sygnał przy niższym SWR niż 3 (tymczasowe piki)
 	 - ver. 1.9.5 trzeci termistor
 	 - ver. 1.9.4
 	 	 - zerowanie mocy szczytowej po zmianie pasma (ręcznej lub auto)
 	- spowolnienie przy spadku z dużego SWR?
 		- test
 			- wyłączyć 4% spadek przy value > valueMax (spadek wartości powyżej ValueMax natychmiastowy)
 			- max SWR = 5.0
 				- jest lepiej
 			- co gdy brak uśredniania
 				- jest nieźle
 	 - obsługa/brak obsługi kodu banddata z poza LPFów
 	 	 - i wyłączanie obsługi pasma 6m (#define JEST_6m)
 	 	 - blokada

 	- zrobione! D3 wejście: stan niski kolejny alarm -> od termostatu; obsługa jak inne

 	 - uwagi
 	 	 - linijka od mocy skacze
 	 	 	 - oscyloskop
 	 	 	 - może external Vref?
 	 	 	 	 - kupiony MCP1541 napięcie odniesienia 4,096V 1% -> Vref

	- gdy SWR na antenie > 3
		- info na wyjściu D4 -> stan niski
		- na razie nie:
			- przejście na STBY
				- komunikat na dole
 ----------------------------------------------------------------
- jakieś starocie do ewentualnego sprawdzenia:
 	 - zauważone usterki
 	 	 - nie pamiętam
			 - przy braku 50V (alarm od tego napięcia) działa PTT?
			 - po puszczeniu PTT linijka SWR leci do końca czasami i powoduje generowanie błądu
				 - może jest pomiar podczas odbioru jeszcze?
				 - czy nie ma za dużych pojemności na wyj couplera -> inne dla FWD i inne dla REF (REF > FWD)
			 - po przekroczeniu prądu brak kodu BCD?
 */

// zmienne na potrzeby miernika częstotliwosci:
// these are checked for in the main program
volatile unsigned long timerCounts;
volatile boolean counterReady;
// internal to counting routine
unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;


UTFT myGLCD(SSD1963_800480, 38, 39, 40, 41);

//URTouch myTouch(6, 5, 4, 3, 2);
UTouch myTouch(43, 42, 44, 45, 46);
// Declare which fonts we will be using
// Download http://www.rinkydinkelectronics.com/r_fonts.php
extern uint8_t SmallFont[];		// 8x12 -> 18 (8x16)
extern uint8_t Grotesk16x32[];	// ? 22 (17x20)
extern uint8_t GroteskBold16x32[];	// ? 23 (18x22) bold
extern uint8_t GroteskBold32x64[];	// ? 31 (37x49)
extern uint8_t nadianne[];			// 16x16 -> 21 (13x17) lub 22 (17x20)
extern uint8_t franklingothic_normal[];		// 16x16 ->  22 (17x20) lub 21 (13x17)

// Define some colors
// Help under http://www.barth-dev.de/online/rgb565-color-picker/
#define vgaBackgroundColor	0x10A2
#define vgaTitleUnitColor	0x632C
#define vgaValueColor		0x94B2
#define vgaBarColor			0xCE59

/*
 * define input and output Pins
 *
 * wejścia/wyjścia analogowe/cyfrowe A0-A15:
 *
 */
#define BL_ONOFF_PIN     	54	// A0
#define aiPin_pwrForward   	6	// A6
#define aiPin_pwrReturn    	7	// A7
#define aiPin_drainVoltage 	8	// pomiar 48V
#define aiPin_aux1Voltage  	9	// pomiar 12V
#define aiPin_pa1Amper     	15	// prąd drenu
#define aiPin_temperatura1	12	//  temperatura pierwszego tranzystora - blokada po przekroczeniu thresholdTemperaturTransistorMax
#define aiPin_temperatura2	13	// temperatura drugiego tranzystora - blokada po przekroczeniu thresholdTemperaturTransistorMax
#define aiPin_temperatura3	14	// temperatura radiatora

#define doPin_air1          13 	// wentylator = A14
#define doPin_air2         	59	// wentylator2 = A5

/*
 * wejścia/wyjścia cyfrowe:
 * D0, D1 zarezerowowane dla ew. serial debug na USB
 *
 */

#define doPin_P12PTT	 56	// A2 P12PTT wyjście na przekaźniki N/O
#ifdef CZAS_PETLI
#define doPin_CZAS_PETLI 49		// D49
#endif
// wyjścia sterujące LPFem:
#define doPin_40_60m	2
#define doPin_80m			3
#define doPin_160m		4
#define doPin_20_30m	14
#define doPin_15_17m	15
#define doPin_10_12m	16

#define doPin_blokada       5	// aktywny stan wysoki - blokada głównie od temperatury -> do sekwencera?
#define doPin_ResetAlarmu      	6	// reset alarmu sprzętowego na płytce zabezpieczeń
#define diPin_AlarmOdIDD	7	// alarm od przekroczenia IDD z płytki zabezpieczeń
#define diPin_We_PTT          	8		// wejście PTT z gniazda (z TRX)
#define doPin_BIAS   		9		// BIAS w PA pin 9
/*
 * na przyszłość: Fan1, Fan2, Fan3 sterowanie obrotami wentylatora
#define diPin_stby        	10	// ustawienie PA w tryb ominięcia; aktywny stan wysoki
#define diPin_Imax        	11	// przekroczony prąd drenu
#define diPin_Pmax        12	// przekroczenie mocy sterowania (na wejściu)
*/
#define doPin_FanOn   13	// włączenie wentylatora
#define dinAlaOdT1		21	// wejście alarmu od przekroczenia temperatury tranzystora 1
#define dinAlaOdT2		20	// wejście alarmu od przekroczenia temperatury tranzystora 2

#define BAND_A_PIN  	58	// band data A
#define BAND_B_PIN  	48	// band data B
#define BAND_C_PIN  	49	// band data C
#define BAND_D_PIN  	55	// band data D

//
// D20, D21 magistrala I2C -> nie do wykorzystania
// digital pin 22-46 used by UTFT (resistive touch)
// 47-53 wolne

// zapisywanie w EEPROM: Auto/Manual i pasmo
#define COLDSTART_REF      0x12   	// When started, the firmware examines this "Serial Number"
#define CZAS_REAKCJI 1000			// the time [ms] after which the writing into EEPROM takes place
boolean byla_zmiana = false;
unsigned long czas_zmiany;
boolean airBox1Manual = false;

// Define the analogValue variables
float pwrForwardValue;
float pwrReturnValue;
float drainVoltageValue;
float aux1VoltageValue;
float pa1AmperValue;
volatile float temperaturValue1 = 0.0f;
volatile float temperaturValue2 = 0.0f;
float temperaturValue3;
int forwardValue;		// odczyt napięcia padającego z direct couplera
int forwardValueAvg;	// średnia z napięcia padającego z direct couplera
int returnValue;		// odczyt napięcia odbitego z direct couplera
int returnValueAvg;		// średnia napięcia odbitego z direct couplera
int fwd_calc = 0;		// sumaryczny odczyt
int rev_calc = 0;
int p_curr = 0;			// licznik odczytów
float fwd_pwr;
float rev_pwr;
#define SWR_SAMPLES_CNT             1

// Define the boolValue variables
bool pttValue = false;

// zmienne powodujące przejście PA w tryb standby -> blokada nadawania (blokada PTT)
bool stbyValue = false;
bool ImaxValue;
bool PmaxValue;
bool SWRmaxValue;
bool SWR3Value;
bool SWRLPFmaxValue;
bool SWR_ster_max;
bool TemperaturaTranzystoraMaxValue;
bool TermostatValue;

bool errLedValue;

#define inputFactorVoltage (Vref/1023.0)
#define pwrForwardFactor (inputFactorVoltage * (222.0/5.0))
#define pwrReturnFactor (inputFactorVoltage * (222.0/5.0))
#ifdef SP2HYO
#define pwrForwardFactor (inputFactorVoltage * (320.0/5.0))
#define pwrReturnFactor (inputFactorVoltage * (320.0/5.0))
#endif
#ifdef SP3JDZ
#define pwrForwardFactor (inputFactorVoltage * (222.0/5.0))
#define pwrReturnFactor (inputFactorVoltage * (222.0/5.0))
#endif
#define drainVoltageFactor (inputFactorVoltage * (120.0/5.0))// 5V Input = 60V PA
#define aux1VoltageFactor (inputFactorVoltage * (19.35/5.0)) // 5V Input = 30V PA
#define aux2VoltageFactor (inputFactorVoltage * (15.0/5.0)) // 5V Input = 15V PA
#ifdef ACS713
#define pa1AmperFactor (inputFactorVoltage * (30/4.0))    // 133mV/A ACS713
#define pa1AmperOffset (1023/5.0 * 0.5)                     // 0.5V z czujnika Hallla -> zmierzyć i wstawić ewentualnie
#else
#define pa1AmperFactor (inputFactorVoltage * (65.0/5.0))    // 1k Ris w BTS50085
#define pa1AmperOffset (0.0)                     // 0.0V	- pomiar z BTS50085 - od zera
#endif
//#define pa2AmperFactor (inputFactorVoltage * (62.5/2.5))    // 40mV/A
//#define pa2AmperOffset (1023/5 * 2.505)                     // 2.5V
//#define temperaturFactor (inputFactorVoltage * (100.0/5.0)) // 5V = 100°C

// zmienne na potrzeby pomiaru temperatury
float Uref = 5.0;			// napięcie zasilające dzielnik pomiarowy temperatury
float Vref = 4.833;			// napięcie odniesienia dla ADC
//int beta = 3500;			// współczynnik beta termistora
int R25 = 1800;				// rezystancja termistora w temperaturze 25C
int Rf1 = 995;				// rezystancja rezystora szeregowego  nr 1
int Rf2 = 995;				// rezystancja rezystora szeregowego  nr 2

int Rf3 = 2700;				// rezystancja rezystora szeregowego z termistorem nr 3

#define modeManualName "MANUALLY"
#define modeAutoName   "AUTO"
enum
{
	MANUAL = 0,
	AUTO
};
byte mode = MANUAL;
enum
{
	BAND_160 = 0,
	BAND_80,
	BAND_60,
	BAND_40,
	BAND_30,
	BAND_20,
	BAND_17,
	BAND_15,
	BAND_12,
	BAND_10,
	BAND_6,
	BAND_NUM
};
enum
{
#ifdef ALTER
	ATT1 = 14,
	ATT2,
	ATT3
#else
	ATT1 = 18,
	ATT2,
	ATT3
#endif
};
byte ATT[BAND_NUM] = {ATT1, ATT1, ATT1, ATT1, ATT1, ATT2, ATT2, ATT2, ATT2, ATT3};
String BAND[BAND_NUM] = {"    160", "    80", "    60", "    40", "    30", "    20","    17", "    15","    12", "    10", "    6m"};
// przełączanie pasm ze sterownik_FT810:
enum
{
	LPF1_PIN = 2,
	LPF2_PIN,
	LPF3_PIN,
	LPF4_PIN = 14,
	LPF5_PIN,
	LPF6_PIN,
	LPF7_PIN
};
byte current_band = BAND_80;
byte prev_band = BAND_NUM;
byte Band_PIN[BAND_NUM] = {LPF3_PIN, LPF2_PIN, LPF1_PIN, LPF1_PIN, LPF4_PIN, LPF4_PIN, LPF5_PIN, LPF5_PIN, LPF6_PIN, LPF6_PIN, LPF7_PIN};

byte AutoBandIdx = 15;

#define thresholdCurrent           1.0
#define thresholdPower             5.0
#define thresholdSWR               3.5		// zmiana na 3.5 dla HYO
#define thresholdTemperaturAirOn1   55
#define thresholdTemperaturTransistorMax	80		// temperatura tranzystora (z termistora nr 1), przy której PA jest blokowane - STBY
#define thresholdTemperaturAirOn2   50

String infoString = "";
String warningString = "";		// nieużywany
String errorString = "";
bool genOutputEnable;

// Touch
int touchX = -1;
int touchY = -1;

unsigned long timeAtCycleStart, timeAtCycleEnd, timeStartMorseDownTime,
		actualCycleTime, timeToogle500ms = 0;
int drawWidgetIndex = 6;
bool toogle500ms;

#define cycleTime        30

float pwrForwardReturnRatio;
float swrValue;

class PushButton
{
	int _xPos, _yPos, _height, _width;
public:
	PushButton(int xPos, int yPos, int height, int width)
{
		_xPos = xPos;
		_yPos = yPos;
		_height = height;
		_width = width;
}
	bool isTouchInside(int x, int y)
	{
		// Check if touch is inside this widget
		return ((x > _xPos and x < _xPos + _width)
				and (y > _yPos and y < _yPos + _height));
	}
private:
};

class InfoBox
{
	// This class draws a info box with value or text, title and unit.
	// Class Variables
	String _title, _unit, _text;

	float _value, _minValue, _maxValue;

	int _xPos, _yPos, _height, _width;
	int _xPadding, _yPadding;
	int _colorValue, _colorBack, _font;

	bool _raisedError = false;
	bool _drawLater = false;

public:
	InfoBox(String title, String unit, int xPos, int yPos, int height,
			int width, float minValue, float maxValue, int colorValue,
			int colorBack, int font)
	{
		// Store parameter
		_title = title;
		_unit = unit;
		_value = 0;
		_text = "";

		_xPos = xPos;
		_yPos = yPos;
		_height = height;
		_width = width;
		_minValue = minValue;
		_maxValue = maxValue;
		_xPadding = 4;
		_yPadding = 1;

		_colorValue = colorValue;
		_colorBack = colorBack;
		_font = font;
	}

	void init()
	{
		// Called by main setup
		// Background
		myGLCD.setBackColor(_colorBack);
		myGLCD.setColor(_colorBack);
		myGLCD.fillRect(_xPos, _yPos, _xPos + _width, _yPos + _height);

		// Title
		myGLCD.setColor(vgaTitleUnitColor);
		myGLCD.setFont(SmallFont);
		int titleFontXsize = myGLCD.getFontXsize();
		int titleFontYsize = myGLCD.getFontYsize();
		int titleLength = _title.length();
		myGLCD.print(_title,
				_xPos + (_width - titleLength * titleFontXsize) - _xPadding,
				_yPos + _yPadding + 1);

		// Unit
		myGLCD.setFont(franklingothic_normal);
		int unitFontXsize = myGLCD.getFontXsize();
		int unitFontYsize = myGLCD.getFontYsize();
		int unitLength = _unit.length();
		myGLCD.print(_unit,
				_xPos + (_width - unitLength * unitFontXsize) - _xPadding,
				_yPos + _yPadding + titleFontYsize + 1);
	}

	void setColorValue(int color)
	{
		_colorValue = color;
	}

	void setColorBack(int color)
	{
		_colorBack = color;
	}

	bool isValueOk()
	{
		return not _raisedError;
	}

	void setFloat(float value, int dec, int length, bool show)
	{
		// dec ile po przecinku, length długość całkowita
		if ((value != _value) or _drawLater)
		{
			_value = value;
			myGLCD.setBackColor(_colorBack);

			if (value < _minValue or value > _maxValue)
			{
				myGLCD.setColor(VGA_RED);
				if (_raisedError == false and errorString == "")
				{
					_raisedError = true;
					errorString = "Error: " + _title + " " + _value + _unit
							+ " outside range of " + int(_minValue) + _unit
							+ "-" + int(_maxValue) + _unit;
				}
			}
			else
			{
				_raisedError = false;
				myGLCD.setColor(_colorValue);
			}

			if (show)
			{
				myGLCD.setFont(_font);
				myGLCD.printNumF(_value, dec, _xPos + _xPadding,
						_yPos + _yPadding, '.', length);
				_drawLater = false;
			}
			else
			{
				_drawLater = true;
			}
		}
	}

	void setInt(int value, int length, bool show)
	{
		if ((value != _value) or _drawLater)
		{
			_value = value;

			myGLCD.setBackColor(_colorBack);

			if (value < _minValue or value > _maxValue)
			{
				myGLCD.setColor(VGA_RED);
				if (_raisedError == false and errorString == "")
				{
					_raisedError = true;
					errorString = "Error: " + _title + " " + _value + _unit
							+ " outside range of " + int(_minValue) + _unit
							+ "-" + int(_maxValue) + _unit;
				}
			}
			else
			{
				_raisedError = false;
				myGLCD.setColor(_colorValue);
			}

			if (show)
			{
				myGLCD.setFont(_font);
				myGLCD.printNumI(_value, _xPos + _xPadding, _yPos + _yPadding,
						length);
				_drawLater = false;
			}
			else
			{
				_drawLater = true;
			}
		}
	}
	void setText(String text)
	{
		if (text != _text)
		{
			_text = text;
			init();

			myGLCD.setBackColor(_colorBack);
			//myGLCD.setBackColor(VGA_RED);
			myGLCD.setColor(_colorValue);
			//myGLCD.setColor(VGA_GREEN);
			myGLCD.setFont(_font);
			if (text.length() < 46)
			{
				myGLCD.setFont(_font);
			}
			else
			{
				myGLCD.setFont(SmallFont);
			}
			myGLCD.print(_text, _xPos + _xPadding, _yPos + _yPadding);
		}
	}

	float getValue()
	{
		return _value;
	}

	String getText()
	{
		return _text;
	}

	bool isTouchInside(int x, int y)
	{
		// Check if touch is inside this widget
		return ((x > _xPos and x < _xPos + _width)
				and (y > _yPos and y < _yPos + _height));
	}
};

class DisplayBar
{
	// This class draws a bar with scale, title, actual and maximum value.
	// Class Variables
	String _title, _unit;

	float _value, _valueMin, _valueOld, _valueMax;
	float _minValue, _maxValue, _rangeValue;
	float _warnValue1, _warnValue2;
	float _level, _levelOld, _delta;

	int _xPos, _yPos;
	int _xPosBar, _yPosBar, _heightBar, _widthBar;
	int _height, _width;
	int _xPadding, _yPadding;
	int _colorBar, _colorBack, _font;
	int _noOffHelplines;

	// filter
	int _holdMaxCycles;
	int _filterForValueRefresh;
	float _deltaMaxNeg;
	bool _showMax;

	InfoBox *ptrActBox;
	InfoBox *ptrMaxBox;

public:
	DisplayBar(String title, String unit, int xPos, int yPos, int height,
			int width, float minValue, float maxValue, float warnValue1,
			float warnValue2, int colorBar, int colorBack, int noOffHelplines)
	{
		// Store parameter
		_title = title;
		_unit = unit;

		_valueMin = minValue;
		_value = _valueMin;
		_valueOld = _value;
		_valueMax = 0;

		_minValue = minValue;
		_maxValue = maxValue;
		_rangeValue = _maxValue - _minValue;
		_warnValue1 = warnValue1;
		_warnValue2 = warnValue2;

		_xPos = xPos;
		_yPos = yPos;
		_height = height;
		_width = width;
		_xPadding = 4; // x padding inside the box
		_yPadding = 1; // y padding inside the box

		_colorBar = colorBar;
		_colorBack = colorBack;
		_font = 1;

		_noOffHelplines = noOffHelplines;

		// Filter
		_holdMaxCycles = 4;
		_deltaMaxNeg = maxValue / 100 * 4; // max decrement 4% of the max value
		_filterForValueRefresh = 0;
		_showMax = false;

	}


	void init()
	{
		// na potrzeby zmiany skali mocy:
		_deltaMaxNeg = _maxValue / 100 * 4; // max decrement 4% of the max value
		_rangeValue = _maxValue - _minValue;

		// Called by main setup
		// Background
		myGLCD.setBackColor(_colorBack);
		myGLCD.setColor(_colorBack);
		myGLCD.fillRect(_xPos, _yPos, _xPos + _width, _yPos + _height);

		// Title
		myGLCD.setFont(franklingothic_normal);
		myGLCD.setColor(vgaTitleUnitColor);
		myGLCD.print(_title, _xPos + _xPadding, _yPos + _yPadding);

		// Info boxes
		int xPosInfoBox = _xPos + _width - _xPadding - 125;
		int yPosInfoBox = _yPos + _height - _yPadding - 64;
		_xPosBar = _xPos + _xPadding;
		_yPosBar = _yPos + (_height / 2);
		_heightBar = (_yPos + _height) - _yPosBar - 4;
		_widthBar = xPosInfoBox - _xPos - 2 * _xPadding;

		//                               title    unit     xPos           yPos              height  width, minValue, maxValue,  colorValue       colorBack             font
		ptrActBox = new InfoBox("", _unit, xPosInfoBox, yPosInfoBox, 32, 125, 0,
				_maxValue, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
		ptrMaxBox = new InfoBox("PEP", _unit, xPosInfoBox, yPosInfoBox + 32, 32,
				125, 0, _maxValue, vgaValueColor, vgaBackgroundColor,
				GroteskBold16x32);

		ptrActBox->init();
		ptrMaxBox->init();

		//        xPos,     yPos,           height,  width
		drawScale(_xPosBar, _yPosBar - 18, 15, _widthBar);
		myGLCD.drawRect(_xPosBar, _yPosBar, _xPosBar + _widthBar,
				_yPosBar + _heightBar);
		_xPosBar = _xPosBar + 1;
		_yPosBar = _yPosBar + 1;
		_widthBar = _widthBar - 2;
		_heightBar = _heightBar - 2;

		myGLCD.setColor( VGA_BLACK);
		myGLCD.drawRect(_xPosBar, _yPosBar, _xPosBar + _widthBar,
				_yPosBar + _heightBar);
		_xPosBar = _xPosBar + 1;
		_yPosBar = _yPosBar + 1;
		_widthBar = _widthBar - 2;
		_heightBar = _heightBar - 2;
	}

	void drawScale(int xPos, int yPos, int height, int width)
	{
		// Draw the scale with value and warning levels
		myGLCD.setColor(vgaValueColor);

		// Horizontal base line
		myGLCD.fillRect(xPos, yPos + height - 2, xPos + width, yPos + height);

		// Draw warning level
		int warningLevel1 = (_warnValue1 - _minValue) / _rangeValue * _widthBar;
		int warningLevel2 = (_warnValue2 - _minValue) / _rangeValue * _widthBar;
		myGLCD.setColor(VGA_YELLOW);
		myGLCD.fillRect(xPos + warningLevel1, yPos + height - 1,
				xPos + warningLevel2, yPos + height);
		myGLCD.setColor(VGA_RED);
		myGLCD.fillRect(xPos + warningLevel2, yPos + height - 1, xPos + width,
				yPos + height);

		// Draw helplines and values
		myGLCD.setColor(vgaValueColor);
		myGLCD.setFont(SmallFont);

		float xPosHelpline;
		float helpValue;
		float helpValueIncrement = _rangeValue / _noOffHelplines;

		for (float helpline = 0; helpline <= _noOffHelplines; helpline++)
		{
			helpValue = _minValue + (helpline * helpValueIncrement);
			xPosHelpline = xPos + (helpline / _noOffHelplines * _widthBar);
			myGLCD.drawLine(xPosHelpline, yPos, xPosHelpline,
					yPos + height - 2);
			if (helpline != _noOffHelplines)
			{
				//if (helpValue <= 10 & helpValue > 0)
				if (helpValue <= 10 && helpValue > 0)
				{
					// Small values as float with 1 dec
					myGLCD.printNumF(helpValue, 1, xPosHelpline + 3, yPos);
				}
				else
				{
					// Larg values as int
					myGLCD.printNumI(helpValue, xPosHelpline + 3, yPos);
				}
			}
		}
	}

	void setValue(float value, bool show)
	{
		// Set value and draw bar and info box
		// Refresh the info box only all 4 updates
		/*
		 * spowolnione zmniejszanie tylko dla wartości z zakresu poniżej _valueMax; "wyskoki" są brane bez zmian
		 * -> dodatkowy warunek value < _maxValue
		 */

		if (value < _valueOld and value < _maxValue)
		{
			_delta = _valueOld - value;
			if (_delta > _deltaMaxNeg)
			{
				value = _valueOld - _deltaMaxNeg;
			}
		}

		// Set the actual value info box
		if (value < 100)
		{
			ptrActBox->setFloat(value, 1, 4, show);
		}
		else
		{
			ptrActBox->setInt(value, 4, show);
		}

		// Update the bar
		_value = value;
		if (_showMax)
		{
			setValueMax(_value);
		}

		if (show)
		{
			_showMax = true;
		}
		else
		{
			_showMax = false;
		}

		_level = (value - _minValue) / _rangeValue * _widthBar;

		if (_level > _widthBar)
		{
			_level = _widthBar;
		}
		if (_level > _levelOld)
		{
			myGLCD.setColor(_colorBar);
			myGLCD.fillRect(_xPosBar + _levelOld, _yPosBar, _xPosBar + _level,
					_yPosBar + _heightBar);
		}
		else
		{
			myGLCD.setColor(_colorBack);
			myGLCD.fillRect(_xPosBar + _level, _yPosBar, _xPosBar + _levelOld,
					_yPosBar + _heightBar);
		}

		_levelOld = _level;
		_valueOld = value;
	}

	void setValueMax(float value)
	{
		// Set the maximum value
		if (value > _valueMax)
		{
			_valueMax = value;

			// Set the maximum value info box
			if (value < 100)
			{
				ptrMaxBox->setFloat(value, 1, 4, true);
			}
			else
			{
				ptrMaxBox->setInt(value, 4, true);
			}
		}
	}

	void resetValueMax()
	{
		_valueMax = -1;
	}

	float getValue()
	{
		// Return the actual value
		return _value;
	}

	bool isTouchInside(int x, int y)
	{
		// Check if touch is inside this widget
		return ((x > _xPos and x < _xPos + _width)
				and (y > _yPos and y < _yPos + _height));
	}
	void setMinValue(float value)
	{
		_minValue = value;
	}
	void setMaxValue(float value)
	{
		_maxValue = value;
	}
	void setWarnValue1(float value)
	{
		_warnValue1 = value;
	}
	void setWarnValue2(float value)
	{
		_warnValue2 = value;
	}
};
// SETUP the grafic objects
//                        title         unit    xPos  yPos  height  width, _minValue,  _maxValue,  colorValue       colorBack             font
InfoBox modeBox("MODE", "", 395, 60, 32, 200, 0, 0, vgaValueColor, vgaBackgroundColor, Grotesk16x32);
//InfoBox frequencyBox("Frequency", "MHZ", 395, 20, 32, 200, 1.7, 55.0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

InfoBox bandBox("LPF", "m", 20, 20, 72, 350, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold32x64);

InfoBox pa1AmperBox("PA 1", "A", 20, 340, 32, 125, 0, 62.0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox drainVoltageBox("DRAIN", "V", 20, 380, 32, 125, 45, 56, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

InfoBox aux1VoltageBox("AUX ", "V", 170, 380, 32, 125, 11, 15, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
//InfoBox aux1VoltageBox("AUX R", "V", 170, 340, 32, 125, 11, 15, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

// InfoBox aux2VoltageBox("AUX B", "V", 320, 340, 32, 125, 11, 14, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox airBox1("AIR1", "", 470, 340, 32, 125, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox airBox2("AIR2", "", 470, 380, 32, 125, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

//InfoBox pa2AmperBox("PA 2", "A", 170, 380, 32, 125, 0, 24.9, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

// temperatury 10 do 70 stopni
InfoBox temperaturBox1("", "`C", 170, 340, 32, 125, 10, 70, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox temperaturBox2("", "`C", 320, 340, 32, 125, 10, 70, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

//InfoBox temperaturBox2("", "`C", 320, 380, 32, 125, 10, 60, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox temperaturBox3("", "`C", 320, 380, 32, 125, 10, 65, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

// wypełniacz pustego boksu
//InfoBox emptyBox("", "", 170, 380, 32, 125, 0.0, 0.0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

InfoBox msgBox("", "", 20, 420, 32, 760, 0, 0, vgaValueColor, vgaBackgroundColor, Grotesk16x32);
InfoBox txRxBox("", "", 645, 340, 72, 135, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold32x64);

DisplayBar swrBar("SWR", "", 20, 226, 80, 760, 1, 5, 3, 4, vgaBarColor, vgaBackgroundColor, 16);
// title, unit, xPos, yPos, height, width, minValue, maxValue, warnValue1, warnValue2, colorBar, colorBack, noOffHelplines
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 2500, 750, 1750, vgaBarColor, vgaBackgroundColor, 10);

#ifdef SP2HYOi
	DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 2000, 600, 1400, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 2,0kw
	float minValue = 0.0;
	float maxValue = 650.0;
	float warnValue1 = 195.0;
	float warnValue2 = 455.0;
#endif

//pwrBar.DisplayBar("PWR", "W", 20, 126, 80, 760, 0, 650, 195, 455, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 0,65kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 3000, 900, 2100, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 3,0kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 2500, 750, 1750, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 2,5kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 1800, 540, 1260, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 1,8kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 1500, 450, 1050, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 1,5kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 1250, 375, 875, vgaBarColor, vgaBackgroundColor, 10);     // // Wybor wskaznika PWR_skali 1,25kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 1000, 300, 700, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 1,0kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 750, 225, 525, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 0,75kw
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 650, 195, 455, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 0,65kw
DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 500, 150, 350, vgaBarColor, vgaBackgroundColor, 10);      // // Wybor wskaznika PWR_skali 0,5kw

#ifdef SP3JDZ
DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 500, 150, 350, vgaBarColor, vgaBackgroundColor, 10);
#endif

PushButton Down(20, 20, 72, 165);
PushButton Up(185, 20, 72, 165);

ISR (TIMER5_OVF_vect)
{
  ++overflowCount;               // count number of Counter1 overflows
}  // end of TIMER5_OVF_vect


//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR (TIMER2_COMPA_vect)
{
  // grab counter value before it changes any more
  unsigned int timer5CounterValue;
  timer5CounterValue = TCNT5;  // see datasheet, (accessing 16-bit registers)

  // see if we have reached timing period
  if (++timerTicks < timerPeriod)
    return;  // not yet

  // end of gate time, measurement ready

  TCCR5A = 0;    // stop timer 5
  TCCR5B = 0;

  TCCR2A = 0;    // stop timer 2
  TCCR2B = 0;

  TIMSK2 = 0;    // disable Timer2 Interrupt
  TIMSK5 = 0;    // disable Timer5 Interrupt

  // calculate total count
  timerCounts = (overflowCount << 16) + timer5CounterValue;  // each overflow is 65536 more
  counterReady = true;              // set global flag for end count period
#ifdef CZAS_PETLI
  //PORTL = PORTL ^ (1 << PL1);			// nr portu na sztywno! = D48; 2 noga J11 (band data); chyba nie działa
#endif
}  // end of TIMER2_COMPA_vect

void setup()
{
	pinMode(BL_ONOFF_PIN, OUTPUT);  	//backlight
	digitalWrite(BL_ONOFF_PIN, LOW);	//off
	Serial1.begin(115200);	// RS na złączu J1
	if (eeprom_read_byte(0) != COLDSTART_REF)
	{
		EEPROM.write(1, current_band);
		EEPROM.write(2, mode);
		EEPROM.write(0, COLDSTART_REF); // COLDSTART_REF in first byte indicates all initialized
#ifdef DEBUG
		Serial1.println("writing initial values into memory");
#endif
	}
	else  // EEPROM contains stored data, retrieve the data
	{
		// read the current band
		current_band = EEPROM.read(1);
		// read mode
		mode = EEPROM.read(2);
#ifdef DEBUGi
		Serial1.println("reading current_band from memory: ");
		Serial1.println(current_band);
#endif
	}
	switch_bands();
	myGLCD.InitLCD();
	myGLCD.clrScr();

	myTouch.InitTouch(LANDSCAPE);
	myTouch.setPrecision(PREC_MEDIUM);

	//pinMode(diPin_blok_Alarm_SWR, INPUT_PULLUP);	// stan aktywny wysoki -> musi być coś podpięte lub rezystor do masy
	//pinMode(diPin_Termostat, INPUT_PULLUP);
	//pinMode(doPin_SWR_ant, OUTPUT);
	//digitalWrite(doPin_SWR_ant, HIGH);
	pinMode(aiPin_aux1Voltage, INPUT);
	pinMode(aiPin_drainVoltage, INPUT);

	pinMode(doPin_blokada, OUTPUT);		// aktywny stan wysoki
	digitalWrite(doPin_blokada, LOW);

	//pinMode(doPin_errLED, OUTPUT);
	//pinMode(diPin_SWR_ster_max, INPUT_PULLUP);

	pinMode(BAND_A_PIN, INPUT_PULLUP);
	pinMode(BAND_B_PIN, INPUT_PULLUP);
	pinMode(BAND_C_PIN, INPUT_PULLUP);
	pinMode(BAND_D_PIN, INPUT_PULLUP);
#ifdef CZAS_PETLI
	pinMode(doPin_CZAS_PETLI, OUTPUT);
	pinMode(48, OUTPUT);
#else
#endif

	pinMode(doPin_air1, OUTPUT);
	pinMode(doPin_air2, OUTPUT);

	pinMode(diPin_We_PTT, INPUT_PULLUP);
	pinMode(doPin_BIAS, OUTPUT);
	pinMode(doPin_P12PTT, OUTPUT);
	//pinMode(diPin_SWR_LPF_max, INPUT_PULLUP); 	// aktywny stan niski
	//pinMode(diPin_stby, INPUT_PULLUP);
	//pinMode(diPin_Imax, INPUT_PULLUP);
	//pinMode(diPin_Pmax, INPUT_PULLUP);
	//pinMode(diPin_SWRmax, INPUT_PULLUP);
	/*
	pinMode(doPin_ATT1, OUTPUT);
	digitalWrite(doPin_ATT1, HIGH);	// stan aktywny niski
	pinMode(doPin_ATT2, OUTPUT);
	digitalWrite(doPin_ATT2, HIGH);	// stan aktywny niski
	pinMode(doPin_ATT3, OUTPUT);
	digitalWrite(doPin_ATT3, HIGH);	// stan aktywny niski
*/
	myGLCD.setFont(GroteskBold32x64);
	myGLCD.setColor(vgaValueColor);
	myGLCD.print("Sterownik PA", CENTER, 50);
	myGLCD.setFont(Grotesk16x32);
	myGLCD.print("160m - 6m", CENTER, 150);
	digitalWrite(BL_ONOFF_PIN, HIGH);	//backlight on
	delay(1000);
	myGLCD.clrScr();

	// Set call sign and version
	myGLCD.setFont(nadianne);
	myGLCD.setColor(vgaTitleUnitColor);
	//myGLCD.print("DJ8QP ", RIGHT, 20);
	//myGLCD.print("DC5ME ", RIGHT, 40);
	myGLCD.setFont(SmallFont);
	myGLCD.print("V1.11.0  ", RIGHT, 60);

	// Init the grafic objects
	modeBox.init();
	if (mode == MANUAL)
	{
		modeBox.setText(modeManualName);
	}
	else
	{
		modeBox.setText(modeAutoName);
	}
	drainVoltageBox.init();
	pa1AmperBox.init();
	aux1VoltageBox.init();
	temperaturBox1.init();
	temperaturBox2.init();
	temperaturBox3.init();
	msgBox.init();
	txRxBox.init();
	pwrBar.init();
	swrBar.init();
	bandBox.init();
	bandBox.setText(BAND[current_band]);
	airBox1.init();
	airBox1.setText("OFF");
	airBox2.init();
	airBox2.setText("OFF");
	//emptyBox.init();
	// Start init test
	// przeniosłem do setupu
	read_inputs();
	pwrBar.setValue(pwrForwardValue, false);
	//swrValue = calc_SWR(pwrForwardValue, pwrReturnValue);
	//swrBar.setValue(swrValue, false);
	drainVoltageBox.setFloat(drainVoltageValue, 1, 4, false);
	aux1VoltageBox.setFloat(aux1VoltageValue, 1, 4, false);
	pa1AmperBox.setFloat(pa1AmperValue, 1, 4, false);
	temperaturBox1.setFloat(temperaturValue1, 1, 5, false);
	temperaturBox2.setFloat(temperaturValue2, 1, 5, false);
	temperaturBox3.setFloat(temperaturValue3, 1, 5, false);
	if (temperaturValue1 > thresholdTemperaturTransistorMax)
	{
		TemperaturaTranzystoraMaxValue = true;
	}
	if (temperaturValue2 > thresholdTemperaturTransistorMax)
	{
		TemperaturaTranzystoraMaxValue = true;
	}

	if (not drainVoltageBox.isValueOk())
	{
		errorString = "Startup error: Awaria zasilania";
	}
	else if (not aux1VoltageBox.isValueOk())
	{
		errorString = "Startup error: Auxilarry 1 voltage not Ok";
	}
	else if (not temperaturBox1.isValueOk())
	{
		errorString = "Startup error: Temperature 1 not Ok";
	}
	else if (not temperaturBox2.isValueOk())
	{
		errorString = "Startup error: Temperature 2 not Ok";
	}
	else if (not temperaturBox3.isValueOk())
	{
		errorString = "Startup error: Temperature 3 not Ok";
	}
	else if (pa1AmperBox.getValue() > thresholdCurrent)
	{
		errorString = "Startup error: PA 1 Current not 0A";
	}
	else if (pwrBar.getValue() > thresholdPower)
	{
		errorString = "Startup error: PWR detected";
	}
	else if (pttValue == true)
	{
		errorString = "Startup error: PTT detected";
	}
	else if (ImaxValue == true)
	{
		//errorString = "Startup error: Przekroczony IDD";
	}
	else if (PmaxValue == true)
	{
		//errorString = "Startup error: Przekroczony power input";
	}
	else if (SWRLPFmaxValue == true)
	{
		errorString = "Startup error: SWR LPF max";
	}
	else if (TemperaturaTranzystoraMaxValue == true)
	{
		errorString = "Startup error: Przekroczona temperatura tranzystora";
	}
	else if (TermostatValue == true)
	{
		errorString = "Startup error: Termostat on";
	}
	else
	{
		genOutputEnable = true;
		infoString = "Startup completed.";
	}
}

void loop()
{
	// licznik begin
	static bool liczy;
	  // wait if any serial is going on
	if ( not liczy)
	{
		startCounting (1);  // how many mS to count for
		 liczy = true;
	}
	if (counterReady)
	{
		  // adjust counts by counting interval to give frequency in kHz
		  //float frq = (timerCounts *  1.0) / timerPeriod;
		 // liczy = false;
		 // Serial.print ("Frequency: ");
		 // Serial.println ((unsigned long) frq);
		 //Serial.println (timerCounts);
	}
	// licznik end
	timeAtCycleStart = millis();
	if ((timeAtCycleStart - timeToogle500ms) > 500)
	{
		toogle500ms = not toogle500ms;
		timeToogle500ms = timeAtCycleStart;
	}

	// Read touch X and Y values
	if (myTouch.dataAvailable())
	{
		myTouch.read();
		touchX = myTouch.getX();
		touchY = myTouch.getY();
#ifdef DEBUG
		myGLCD.drawPixel(touchX, touchY);
		myGLCD.fillRect(700, 80, 780, 100);
		myGLCD.setColor(vgaTitleUnitColor);
		myGLCD.setFont(SmallFont);
		myGLCD.printNumI(touchX, 710, 85);
		myGLCD.printNumI(touchY, 750, 85);
#endif
	}
	read_inputs();
#ifdef DEBUGi
	Serial1.println("---------------------");
	Serial1.print("ptt: ");
	Serial1.println(pttValue);
	Serial1.print("stby: ");
	Serial1.println(stbyValue);
	Serial1.print("pwrForward: ");
	Serial1.println(pwrForwardValue);
	Serial1.print("pwrReturn: ");
	Serial1.println(pwrReturnValue);
	Serial1.print("drainVoltage: ");
	Serial1.println(drainVoltageValue);
	Serial1.print("aux1Voltage: ");
	Serial1.println(aux1VoltageValue);
	Serial1.print("pa1Amper: ");
	Serial1.println(pa1AmperValue);
	Serial1.print("temperatura1: ");
	Serial1.println(temperaturValue1);
	Serial1.print("input:      ");
	Serial1.println(analogRead(aiPin_temperatura2));
	Serial1.print("temperatura2: ");
	Serial1.println(temperaturValue2);
#endif

	//-----------------------------------------------------------------------------
	// Set display values. The widgets monitors the values and output an errorString
	pwrBar.setValue(pwrForwardValue, drawWidgetIndex == 1);
	//if (UpdatePowerAndVSWR())
	/*
	if (true)
	{
		swrValue = calc_SWR(forwardValue, returnValue);

		bool blok_Alarm_SWR = digitalRead(diPin_blok_Alarm_SWR);
		if (swrValue > thresholdSWR and not blok_Alarm_SWR)
		{
			digitalWrite(doPin_SWR_ant, LOW);
			SWR3Value = true;
		}
		else
		{
			digitalWrite(doPin_SWR_ant, HIGH);
			SWR3Value = false;
		}
		if (blok_Alarm_SWR and swrValue >= 5.0)
			swrValue = 4.9;		// sztuczne obniżenie wartości SWR podczas strojenia ATU

		swrBar.setValue(swrValue, drawWidgetIndex == 2);
	}
*/
	drainVoltageBox.setFloat(drainVoltageValue, 1, 4, drawWidgetIndex == 3);
	aux1VoltageBox.setFloat(aux1VoltageValue, 1, 4, drawWidgetIndex == 4);
	pa1AmperBox.setFloat(pa1AmperValue, 1, 4, drawWidgetIndex == 5);
	temperaturBox1.setInt(temperaturValue1, 3, drawWidgetIndex == 6);
	temperaturBox2.setInt(temperaturValue2, 3, drawWidgetIndex == 7);
	temperaturBox3.setInt(temperaturValue3, 3, drawWidgetIndex == 8);
	// temperatura1 i temperatura2 i wentylator1
	if (temperaturValue1 >= thresholdTemperaturAirOn1 or temperaturValue2 >= thresholdTemperaturAirOn1)
	{
		airBox1.setText("ON");
	}
	else if ((temperaturValue1 <= thresholdTemperaturAirOn1 - 2) and (temperaturValue2 <= thresholdTemperaturAirOn1 - 2))
	{
		airBox1.setText("OFF");
	}

	if (temperaturValue1 > thresholdTemperaturTransistorMax or temperaturValue2 > thresholdTemperaturTransistorMax)	// przekroczenie temperatury granicznej obudowy jednego z tranzystorów
	{
		TemperaturaTranzystoraMaxValue = true;
	}
	else if ((temperaturValue1 < thresholdTemperaturTransistorMax - 5) and (temperaturValue2 < thresholdTemperaturTransistorMax - 5))
	{
		TemperaturaTranzystoraMaxValue = false;
	}

	if (temperaturValue3 >= thresholdTemperaturAirOn2)
	{
		airBox2.setText("ON");
	}
	else if (temperaturValue3 <= thresholdTemperaturAirOn2 - 2)
	{
		airBox2.setText("OFF");
	}
	// Draw index defines the infoBox that can draw new values on the utft.
	// If all infoBoxes would draw together, the cycletime is to long and not constant for the morse output.
	/*
	if (drawWidgetIndex == 8)
	{
		drawWidgetIndex = 1;
	}
	else
	{
		drawWidgetIndex++;
	}
*/
	// Monitor additional inputs and set errorString
	if (ImaxValue == true)
	{
		//errorString = "Error: Przekroczony IDD";
	}
	if (TermostatValue == true)
	{
		errorString = "Error: Termostat on";
	}
	if (PmaxValue == true)
	{
		//errorString = "Error: Przekroczony power input";
	}
	if (SWRmaxValue == true)
	{
		errorString = "Error: Przekroczony SWR anteny";
	}

	if (SWRLPFmaxValue == true)
	{
		errorString = "Error: SWR LPF max";
	}
	if (SWR_ster_max == true)
	{
		errorString = "Error: Power input SWR";
	}

	if (TemperaturaTranzystoraMaxValue == true)
	{
		errorString = "Error: Przekroczenie temperatury tranzystora";
	}
	/*
	if (SWR3Value == true)
	{
		errorString = "Error: SWR anteny sterownik";
	}
	else if (errorString == "")
	{
		digitalWrite(doPin_SWR_ant, HIGH);
	}
*/
	//-----------------------------------------------------------------------------
	// Touch events
	if (touchX != -1 and touchY != -1)
	{
		if (modeBox.isTouchInside(touchX, touchY))
		{
			if (modeBox.getText() == modeManualName)
			{
				modeBox.setText(modeAutoName);
			}
			else
			{
				modeBox.setText(modeManualName);
			}
			byla_zmiana = true;
			czas_zmiany = millis();
		}
		  // przejście w tryb standby i powrót
		else if (txRxBox.isTouchInside(touchX, touchY))
		{
			if (stbyValue)
			{
				stbyValue = false;
			}
			else
			{
				stbyValue = true;
			}
		}
		else if (pttValue == false and modeBox.getText() == modeManualName
				and Up.isTouchInside(touchX, touchY))
		{
			if (current_band >= BAND_NUM - 1)
			{
				current_band = 0;
			}
			else
			{
				current_band++;
			}
			byla_zmiana = true;
			czas_zmiany = millis();
			bandBox.setText(BAND[current_band]);
			pwrBar.resetValueMax();
			swrBar.resetValueMax();
		}
		else if (pttValue == false and modeBox.getText() == modeManualName
				and Down.isTouchInside(touchX, touchY))
		{
			if (current_band <= 0)
			{
				current_band = BAND_NUM - 1;
			}
			else
			{
				current_band--;
			}
			byla_zmiana = true;
			czas_zmiany = millis();
			bandBox.setText(BAND[current_band]);
			pwrBar.resetValueMax();
			swrBar.resetValueMax();
		}
		else if (pwrBar.isTouchInside(touchX, touchY))
		{
			pwrBar.resetValueMax();
		}
		else if (swrBar.isTouchInside(touchX, touchY))
		{
			swrBar.resetValueMax();
		}
		else if (msgBox.isTouchInside(touchX, touchY))
		{
			if (msgBox.getText() == "")
			{
				infoString = "no more messages";
			}
			else
			{
				msgBox.setText("");
				errorString = "";
				infoString = "";
			}
		}
		else if (airBox1.isTouchInside(touchX, touchY))
		{
			if (airBox1Manual)
			{
				airBox1.setText("OFF");
				airBox1Manual = false;
			}
			else
			{
				airBox1.setText("ON");
				airBox1Manual = true;
			}
		}
		else if (airBox2.isTouchInside(touchX, touchY))
		{
			airBox2.setText("ON");	// j.w.
		}

		// Reset touch values
		touchX = -1;
		touchY = -1;
	}

	//-----------------------------------------------------------------------------
	// Reset genOutputEnable on any errorString
	if (errorString != "")
	{
		genOutputEnable = false;
	}
	else
	{
		genOutputEnable = true;
	}

	//-----------------------------------------------------------------------------
	// Signal evaluation
	if (stbyValue or TemperaturaTranzystoraMaxValue or PmaxValue or SWRmaxValue or SWRLPFmaxValue or SWR_ster_max or ImaxValue or TermostatValue or not genOutputEnable)
	{
		txRxBox.setColorValue(vgaBackgroundColor);
		txRxBox.setColorBack(VGA_YELLOW);
		txRxBox.setText("STBY");
		if (not stbyValue)
		{
			digitalWrite(doPin_blokada, HIGH);
		}
		if (not (TemperaturaTranzystoraMaxValue or PmaxValue or SWRmaxValue or SWRLPFmaxValue or SWR_ster_max or ImaxValue or TermostatValue or not genOutputEnable))
		{
			digitalWrite(doPin_blokada, LOW);
		}
	}
	else
	{
		if (pttValue and genOutputEnable)
		{
			swrValue = calc_SWR(forwardValue, returnValue);
			swrBar.setValue(swrValue, drawWidgetIndex == 2);
			digitalWrite(doPin_BIAS, HIGH);
			digitalWrite(doPin_P12PTT, HIGH);
			txRxBox.setColorValue(vgaBackgroundColor);
			txRxBox.setColorBack(VGA_RED);
			txRxBox.setText(" TX");
		}
		else
		{
			txRxBox.setColorValue(vgaBackgroundColor);
			txRxBox.setColorBack(VGA_GREEN);
			txRxBox.setText("OPR");
			digitalWrite(doPin_BIAS, LOW);
			digitalWrite(doPin_P12PTT, LOW);
		}
		digitalWrite(doPin_blokada, LOW);
	}

	if (modeBox.getText() == modeAutoName)
	{
		/*
		    DataPort Codes
			Band 	Code
			 		DCBA
			160m 	0001
			80m 	0010
			40m 	0011
			30m 	0100
			20m 	0101
			17m 	0110
			15m 	0111
			12m 	1000
			10m 	1001
			6m 		1010
		 */
		AutoBandIdx = 0;
		//AutoBandIdx = digitalRead(BAND_D_PIN) << 3 | digitalRead(BAND_C_PIN) << 2 | digitalRead(BAND_B_PIN) << 1 | digitalRead(BAND_A_PIN);
		AutoBandIdx = AutoBandIdx -1;
		if (AutoBandIdx >= 0 and AutoBandIdx <= 9)
		{
			current_band = AutoBandIdx;
			if (current_band != prev_band)
			{
				bandBox.setText(BAND[current_band]);
				pwrBar.resetValueMax();
				swrBar.resetValueMax();
				//prev_band = current_band;	-> zapamiętanie oldBnadIdx dopiero po przełączaniu LPFa
			}
		}
		// pasmo z pomiaru częstotliwości
		if (counterReady)
		{

		}

	}

	//-----------------------------------------------------------------------------
	// Write to outputs
	if (airBox1Manual)
	{
		digitalWrite(doPin_air1, true);
	}
	else
	{
		if (airBox1.getText() == "OFF")
		{
			digitalWrite(doPin_air1, false);
		}
		else
		{
			digitalWrite(doPin_air1, true);
		}
	}

	if (airBox2.getText() == "OFF")
	{
		digitalWrite(doPin_air2, false);
	}
	else
	{
		digitalWrite(doPin_air2, true);
	}
	if (pttValue == false and (pa1AmperBox.getValue() < thresholdCurrent))
	{
		if (current_band != prev_band)
		{
			switch_bands();
			prev_band = current_band;
		}
	}
/*
	if ((ImaxValue or TermostatValue or PmaxValue or SWRmaxValue or SWRLPFmaxValue or SWR_ster_max or TemperaturaTranzystoraMaxValue or not genOutputEnable)  and toogle500ms)
	{
		digitalWrite(doPin_errLED, LOW);
	}
	else
	{
		digitalWrite(doPin_errLED, HIGH);
	}
*/
	//-----------------------------------------------------------------------------
	// Display error messages
	if (errorString != "")
	{
		msgBox.setColorValue(VGA_RED);

		msgBox.setText(errorString);
	}
	else if (warningString != "")
	{
		msgBox.setColorValue(VGA_YELLOW);
		msgBox.setText(warningString);
	}
	else if (infoString != "")
	{
		msgBox.setColorValue(vgaValueColor);
		msgBox.setText(infoString);
	}

	if (byla_zmiana && (millis() - czas_zmiany > CZAS_REAKCJI))
	{
	    EEPROM.write(1, current_band);           // writing current band into eeprom
	    if (modeBox.getText() == modeManualName)
	    {
		    EEPROM.write(2, MANUAL);
	    }
	    else
	    {
	    	EEPROM.write(2, AUTO);
	    }
		byla_zmiana = false;
#ifdef DEBUG
		Serial.println("writing current settings to EEPROM: ");
		Serial.print("current_band: ");
		Serial.println(current_band);
		Serial.print("mode: ");
		Serial.println(modeBox.getText());
#endif
	}

#ifdef CZAS_PETLI
	PORTL = PORTL ^ (1 << PL0);			// nr portu na sztywno! = D49; 3 noga J11 (band data)
	/*
	 * o dziwo: bez alarmów pętla 3ms
	 *  teraz czas zróżnicowany: od 5ms do 30ms
	 * bez indeksu (indeks = 0 bez odświeżania wyświetlania ) - 4,8ms -> niby minimum
	 * indeks 1 (pwrBar): 30ms
	 * drawWidgetIndex 2 (swrBar): 5ms! -> tylko przy nadawaniu będzie więcej!
	 * 3 -> 30ms float
	 * 6 -> 5ms integer
	 */

#else
	// Keep the cycle time constant
	timeAtCycleEnd = millis();
	actualCycleTime = timeAtCycleEnd - timeAtCycleStart;

	if (actualCycleTime < cycleTime)
	{
		delay(cycleTime - actualCycleTime);
	}
#endif
}	// koniec głównej pętli

void getTemperatura1(uint8_t pin, int Rf)
{
	int u = analogRead(pin);
	float U = Vref*u/1023;
	float therm_resistance = (-(float)Rf) * U / (-Vref+ U);
#ifdef DEBUGt
	Serial1.print("therm_resistance1: ");
	Serial1.println(therm_resistance);
#endif
	uint_fast8_t point_left = 0;
	uint_fast8_t point_right = SENS_TABLE_COUNT - 1;
	for (uint_fast8_t i = 0; i < SENS_TABLE_COUNT; i++) {
		if (KTY81_120_sensTable[i][1] < therm_resistance)
		{
			point_left = i;
		}
	}
	for (uint_fast8_t i = (SENS_TABLE_COUNT - 1); i > 0; i--)
	{
		if (KTY81_120_sensTable[i][1] >= therm_resistance)
		{
			point_right = i;
		}
	}
	float power_left = (float)KTY81_120_sensTable[point_left][0];
	float power_right = (float)KTY81_120_sensTable[point_right][0];
	float part_point_left = therm_resistance - KTY81_120_sensTable[point_left][1];
	float part_point_right = KTY81_120_sensTable[point_right][1] - therm_resistance;
	float part_point = part_point_left / (part_point_left + part_point_right);
	float TRX_RF_Temperature_measured = (power_left * (1.0f - part_point)) + (power_right * (part_point));

	if (TRX_RF_Temperature_measured < -100.0f) {
		TRX_RF_Temperature_measured = 75.0f;
	}
	if (TRX_RF_Temperature_measured < 0.0f) {
		TRX_RF_Temperature_measured = 0.0f;
	}

	static float TRX_RF_Temperature1_averaged = 10.0f;
	TRX_RF_Temperature1_averaged = TRX_RF_Temperature1_averaged * 0.995f + TRX_RF_Temperature_measured * 0.005f;

	if (fabsf(TRX_RF_Temperature1_averaged - temperaturValue1) >= 1.0f)
	{ // hysteresis
		temperaturValue1 = TRX_RF_Temperature1_averaged;
		temperaturValue1 = 24.0;
	}
#ifdef DEBUGt
	Serial1.print("analogRead1: ");
	Serial1.println(u);
	Serial1.print("U1: ");
	Serial1.println(U);
	/*
	Serial.print("R: ");
	Serial.println(R);
	*/
	Serial1.print("T1: ");
	Serial1.println(temperaturValue1);
#endif
}
void getTemperatura2(uint8_t pin, int Rf)
{
	int u = analogRead(pin);
	float U = Vref*u/1023;
	float therm_resistance = (-(float)Rf) * U / (-Vref + U);
#ifdef DEBUGt
	Serial1.print("therm_resistance2: ");
	Serial1.println(therm_resistance);
#endif
	uint_fast8_t point_left = 0;
	uint_fast8_t point_right = SENS_TABLE_COUNT - 1;
	for (uint_fast8_t i = 0; i < SENS_TABLE_COUNT; i++) {
		if (KTY81_120_sensTable[i][1] < therm_resistance)
		{
			point_left = i;
		}
	}
	for (uint_fast8_t i = (SENS_TABLE_COUNT - 1); i > 0; i--)
	{
		if (KTY81_120_sensTable[i][1] >= therm_resistance)
		{
			point_right = i;
		}
	}
	float power_left = (float)KTY81_120_sensTable[point_left][0];
	float power_right = (float)KTY81_120_sensTable[point_right][0];
	float part_point_left = therm_resistance - KTY81_120_sensTable[point_left][1];
	float part_point_right = KTY81_120_sensTable[point_right][1] - therm_resistance;
	float part_point = part_point_left / (part_point_left + part_point_right);
	float TRX_RF_Temperature_measured = (power_left * (1.0f - part_point)) + (power_right * (part_point));

	if (TRX_RF_Temperature_measured < -100.0f) {
		TRX_RF_Temperature_measured = 75.0f;
	}
	if (TRX_RF_Temperature_measured < 0.0f) {
		TRX_RF_Temperature_measured = 0.0f;
	}

	static float TRX_RF_Temperature2_averaged = 10.0f;
	TRX_RF_Temperature2_averaged = TRX_RF_Temperature2_averaged * 0.995f + TRX_RF_Temperature_measured * 0.005f;

	if (fabsf(TRX_RF_Temperature2_averaged - temperaturValue2) >= 1.0f)
	{ // hysteresis
		temperaturValue2 = TRX_RF_Temperature2_averaged;
		temperaturValue2 = 25.0;
	}
#ifdef DEBUGt
	Serial1.print("analogRead2: ");
	Serial1.println(u);
	Serial1.print("U2: ");
	Serial1.println(U);
	Serial1.print("T2: ");
	Serial1.println(temperaturValue2);
#endif
}
void getTemperatura3(uint8_t pin)
{
	int reading = analogRead(pin);
	float temperaturaZmierzona = 100 * reading * (Vref / 1023);
	static float temperatura3srednia = 10.0f;
	temperatura3srednia = temperatura3srednia * 0.995f + temperaturaZmierzona * 0.005f;

	if (fabsf(temperatura3srednia - temperaturValue3) >= 1.0f)
	{ // hysteresis
		temperaturValue3 = temperatura3srednia;
		temperaturValue3 = 26.0;
	}

#ifdef DEBUGi
	Serial1.print("reading temp3: ");
	Serial1.println(reading);
	// Print the temperature in the Serial Monitor:
	Serial1.print("temp3: ");
	Serial1.println(temperaturaZmierzona);
	//Serial.print(" \xC2\xB0"); // shows degree symbol
	//Serial1.println("C");
#endif
}

void read_inputs()
{
	//-----------------------------------------------------------------------------
	// Read all inputs
	forwardValue = analogRead(aiPin_pwrForward);
	pwrForwardValue = sq(forwardValue * pwrForwardFactor) / 50;
	returnValue = analogRead(aiPin_pwrReturn);
	pwrReturnValue = sq(returnValue * pwrReturnFactor) / 50;
	//drainVoltageValue = analogRead(aiPin_drainVoltage) * drainVoltageFactor;
	drainVoltageValue = 50.0;
	//aux1VoltageValue = analogRead(aiPin_aux1Voltage) * aux1VoltageFactor;
	aux1VoltageValue = 12.0;
	pa1AmperValue = (analogRead(aiPin_pa1Amper) - pa1AmperOffset)*pa1AmperFactor;
	if (pa1AmperValue < 0)
	{
		pa1AmperValue = 0;
	}
	getTemperatura1(aiPin_temperatura1, Rf1);
	getTemperatura2(aiPin_temperatura2, Rf2);
	getTemperatura3(aiPin_temperatura3);

	pttValue = not digitalRead(diPin_We_PTT);					// aktywny stan niski
/*
	stbyValue = digitalRead(diPin_stby);					// aktywny stan wysoki
	ImaxValue = not digitalRead(diPin_Imax);				// aktywny stan niski
	PmaxValue = not digitalRead(diPin_Pmax);				// aktywny stan niski
	SWRmaxValue = not (digitalRead(diPin_SWRmax) or digitalRead(diPin_blok_Alarm_SWR));			// aktywny stan niski (dla diPin_SWRmax)
	// aktywny stan niski (dla diPin_SWR_LPF_max); aktywny stan wysoki dla blokady
	SWRLPFmaxValue = (not digitalRead(diPin_SWR_LPF_max)) and (not digitalRead(diPin_blok_Alarm_SWR));
	SWR_ster_max = not digitalRead(diPin_SWR_ster_max);		// aktywny stan niski
	TermostatValue = not digitalRead(diPin_Termostat);		// aktywny stan niski
	*/
}
float calc_SWR(int forward, int ref)
{
#define MAX_SWR	9.9
	float swr;
	if (forward > 0)
	{
		if (forward <= ref)
		{
			swr = MAX_SWR;
		}
		else
		{
			swr = (float)(forward + ref)/(forward - ref);
			if (swr > MAX_SWR)
			{
				swr = MAX_SWR;
			}
		}
	}
	else
	{
		swr = 1;
	}
	return swr;
}
/*
float calc_SWR(float forward, float ref)
{
	float swr;
	float stosunek;
	if (forward > 0)
	{
		stosunek = ref/forward;
		swr = fabs((1.0 + sqrtf(stosunek)) / (1.0 - sqrtf(stosunek)));
	}
	else
	{
		swr = 1;
	}
	return swr;
}
*/
bool UpdatePowerAndVSWR()
{
	bool retval = false;

	// Collect samples
	if (p_curr < SWR_SAMPLES_CNT)
	{
		fwd_calc += forwardValue;
		rev_calc += returnValue;
		p_curr++;
	}
	else
	{
		// Compute average values
		forwardValueAvg = fwd_calc / SWR_SAMPLES_CNT;
		//fwd_pwr = sq((forwardValueAvg) * pwrForwardFactor) / 50; NA RAZIE niepotrzebne
		returnValueAvg = rev_calc / SWR_SAMPLES_CNT;
		//rev_pwr = sq((returnValueAvg) * pwrReturnFactor) / 50;

		/*
		PowerFromADCValue(swrm.fwd_calc / SWR_SAMPLES_CNT, sensor_null,
				coupling_calc, &swrm.fwd_pwr, &swrm.fwd_dbm);
		PowerFromADCValue(swrm.rev_calc / SWR_SAMPLES_CNT, sensor_null,
				coupling_calc, &swrm.rev_pwr, &swrm.rev_dbm);
*/
		// Reset accumulators and variables for power measurements
		p_curr = 0;
		fwd_calc = 0;
		rev_calc = 0;
		// Calculate VSWR from power readings
/*
		swrm.vswr = (1 + sqrtf(swrm.rev_pwr / swrm.fwd_pwr))
				/ (1 - sqrtf(swrm.rev_pwr / swrm.fwd_pwr));
*/
		/*
		 // Perform VSWR protection iff threshold is > 1 AND enough forward power exists for a valid calculation
		 if ( ts.vswr_protection_threshold > 1 && swrm.fwd_pwr >= SWR_MIN_CALC_POWER)
		 {
		 if ( swrm.vswr > ts.vswr_protection_threshold )
		 {
		 RadioManagement_DisablePaBias ( );
		 swrm.high_vswr_detected = true;

		 // change output power to "PA_LEVEL_0_5W" when VSWR protection is active
		 RadioManagement_SetPowerLevel ( RadioManagement_GetBand ( df.tune_new), PA_LEVEL_MINIMAL );
		 }
		 }
		 */
		retval = true;
	}
	return retval;
}
void startCounting (unsigned int ms)
  {
	  counterReady = false;         // time not up yet
	  timerPeriod = ms;             // how many 1 mS counts to do
	  timerTicks = 0;               // reset interrupt counter
	  overflowCount = 0;            // no overflows yet

	  // reset Timer 2 and Timer 5
	  TCCR2A = 0;
	  TCCR2B = 0;
	  TCCR5A = 0;
	  TCCR5B = 0;

	  // Timer 5 - counts events on pin D47
	  TIMSK5 = _BV (TOIE1);   // interrupt on Timer 5 overflow

	  // Timer 2 - gives us our 1 mS counting interval
	  // 16 MHz clock (62.5 nS per tick) - prescaled by 128
	  //  counter increments every 8 uS.
	  // So we count 125 of them, giving exactly 1000 uS (1 mS)
	  TCCR2A = _BV (WGM21) ;   // CTC mode
	  OCR2A  = 124;            // count up to 125  (zero relative!!!!)

	  // Timer 2 - interrupt on match (ie. every 1 mS)
	  TIMSK2 = _BV (OCIE2A);   // enable Timer2 Interrupt

	  TCNT2 = 0;
	  TCNT5 = 0;      // Both counters to zero

	  // Reset prescalers
	  GTCCR = _BV (PSRASY);        // reset prescaler now
	  // start Timer 2
	  TCCR2B =  _BV (CS20) | _BV (CS22) ;  // prescaler of 128
	  // start Timer 5
	  // External clock source on T4 pin (D47). Clock on rising edge.
	  TCCR5B =  _BV (CS50) | _BV (CS51) | _BV (CS52);
}  // end of startCounting
void switch_bands()
{
#ifdef DEBUG
	Serial1.print("prev_band: ");
	Serial1.println(Band_PIN[prev_band]);
	Serial1.print("current_band: ");
	Serial1.println(Band_PIN[current_band]);
#endif
	if (Band_PIN[current_band] != Band_PIN[prev_band])
	{
#ifdef SP2FP
		digitalWrite(Band_PIN[prev_band], LOW);
		mcp.digitalWrite(BPF_PIN[prev_band], LOW);
		digitalWrite(Band_PIN[current_band], HIGH);
		mcp.digitalWrite(BPF_PIN[current_band], HIGH);
#else
		if (prev_band != BAND_160)	// tylko dla LPFów wg DJ0ABR 160m przy wyłączonych wszystkich przekaźnikach
		{
			digitalWrite(Band_PIN[prev_band], LOW);
		}
		if (current_band != BAND_160)
		{
			digitalWrite(Band_PIN[current_band], HIGH);
		}
#endif
	}
	prev_band = current_band;
}

