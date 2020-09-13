#include "Arduino.h"
#include <EEPROM.h>
#include <UTFT.h>
#include <SoftwareSerial.h>
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

 Całkowity pobór prądu z 5V (wyświetlacz, shield, arduino mega 2560) około 400mA.

 ToDo
 	 - co z mocą w setupie (może olać)?
 	 - bargraf od SWRa lewa strona??
 	 - obsługa/brak obsługi kodu banddata z poza LPFów
 	 	 - i wyłączanie obsługi pasma 6m (#define JEST_6m)
 	 	 - blokada

 	- zrobione! D3 wejście: stan niski kolejny alarm -> od termostatu
 		- stby nie
 		- blokada LDMOS aktywna (niski)
 		- dioda awaria
 		- komunikat o alarmie na wyświetlaczu

 	 - uwagi
 	 	 - linijka od mocy skacze
 	 	 	 - oscyloskop
 	 	 	 - może external Vref?
 	- spowolnienie przy spadku z dużego SWR?
 		- analiza

	- gdy SWR na antenie > 3
		- info na wyjściu D4 -> stan niski

		- na razie nie:
			- przejście na STBY
				- komunikat na dole
 */

//#define CZAS_PETLI
//#define DEBUG

UTFT myGLCD(SSD1963_800480, 38, 39, 40, 41);

//URTouch myTouch(6, 5, 4, 3, 2);
UTouch myTouch(43, 42, 44, 45, 46);
// Declare which fonts we will be using
// Download http://www.rinkydinkelectronics.com/r_fonts.php
extern uint8_t SmallFont[];
extern uint8_t Grotesk16x32[];
extern uint8_t GroteskBold16x32[];
extern uint8_t GroteskBold32x64[];
extern uint8_t nadianne[];
extern uint8_t franklingothic_normal[];

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

//					     	0	// wolne
#define aiPin_pwrForward   	1
#define aiPin_pwrReturn    	2
#define aiPin_drainVoltage 	3
#define aiPin_aux1Voltage  	4	// 12V
//						  	5	// wolne
#define aiPin_pa1Amper     	6	// prąd drenu
#define aiPin_temperatura1	7	// temperatura tranzystora - blokada po przekroczeniu thresholdTemperaturTransistorMax
#define aiPin_temperatura2	8	// temperatura np. radiatora
//							9	// wolne
#define doPin_Band_A   64	// doPin band A; 	A10
#define doPin_Band_B   65	// doPin band B; 	A11
#define doPin_Band_C   66	// doPin band C; 	A12
#define doPin_Band_D   67	// doPin band D; 	A13

#define doPin_air1         	68	// wentylator = A14
#define doPin_air2         	69	// wentylator = A15

/*
 * wejścia/wyjścia cyfrowe:
 * D0, D1 zarezerowowane dla serial debug
 *
 */
#define doPin_Czas_Petli	2	// pomiar czasu pętli głównej -> PE3 na sztywno
#define diPin_Termostat		3	// wejście alarmowe z termostatu
#define doPin_SWR_ant		4	// informacja o przekroczeniu SWR (według wartości pwrForwardValue i pwrReturnValue) na wyjściu antenowym
								// aktywny stan niski
#define doPin_blokada       5	// aktywny stan wysoki - blokada głównie od temperatury
#define doPin_errLED      	6	// dioda wystąpienia jakiegoś błędu - aktywny stan wysoki - jak jest błąd - stan wysoki i mruga
#define diPin_SWR_ster_max	7	// przekroczony SWR na wejściu
#define diPin_ptt          	8	// wejście PTT
#define diPin_SWR_LPF_max   9	// przekroczony SWR max od LPF
#define diPin_stby        	10	// ustawienie PA w tryb ominięcia
#define diPin_Imax        	11	// przekroczony prąd drenu
#define diPin_Pmax        	12	// przekroczenie mocy sterowania (na wejściu)
#define diPin_SWRmax      	13	// przekroczenie SWR na wyjściu antenowym

#define diPin_bandData_A  	14	// band data A
#define diPin_bandData_B  	15	// band data B
#define diPin_bandData_C  	16	// band data C
#define diPin_bandData_D  	17	// band data D
// D18, D19 wolne
// D20, D21 magistrala I2C (aktualnie niewykorzystywana)
// digital pin 22-46 used by UTFT (resistive touch)
// 47-53 wolne

// zapisywanie w EEPROM: Auto/Manual i pasmo
#define COLDSTART_REF      0x12   // When started, the firmware examines this "Serial Number"
#define CZAS_REAKCJI 1000		// the time [ms] after which the writing into EEPROM takes place
boolean byla_zmiana = false;
unsigned long czas_zmiany;

// Define the analogValue variables
float pwrForwardValue;
float pwrReturnValue;
float drainVoltageValue;
float aux1VoltageValue;
float pa1AmperValue;
float temperaturValue1;
float temperaturValue2;
int forwardValue;	// odczyt napięcia padającego z direct couplera
int returnValue;	// odczyt napięcia odbitego z direct couplera

// Define the boolValue variables
bool pttValue = false;

// zmienne powodujące przejście PA w tryb standby -> blokada nadawania (blokada PTT)
bool stbyValue = true;
bool ImaxValue;
bool PmaxValue;
bool SWRmaxValue;
bool SWR3Value;
bool SWRLPFmaxValue;
bool SWR_ster_max;
bool TemperaturaTranzystoraMaxValue;
bool TermostatValue;

bool errLedValue;

#define inputFactorVoltage (5.0/1023.0)
#ifdef SP2HYO
#define pwrForwardFactor (inputFactorVoltage * (320.0/5.0))
#define pwrReturnFactor (inputFactorVoltage * (320.0/5.0))
#endif
#ifdef SP3JDZ
#define pwrForwardFactor (inputFactorVoltage * (222.0/5.0))
#define pwrReturnFactor (inputFactorVoltage * (222.0/5.0))
#endif
#define drainVoltageFactor (inputFactorVoltage * (60.0/5.0))// 5V Input = 60V PA
#define aux1VoltageFactor (inputFactorVoltage * (30.0/5.0)) // 5V Input = 30V PA
#define aux2VoltageFactor (inputFactorVoltage * (15.0/5.0)) // 5V Input = 15V PA
#define pa1AmperFactor (inputFactorVoltage * (65.0/5.0))    // 1k Ris w BTS50085
// zmienne na potrzeby pomiaru temperatury
float Uref = 5.0;			// napięcie zasilające dzielnik pomiarowy temperatury
float Vref = 5.0;			// napięcie odniesienia dla ADC
int beta = 3500;			// współczynnik beta termistora
int R25 = 1800;				// rezystancja termistora w temperaturze 25C
int Rf1 = 2677;				// rezystancja rezystora szeregowego z termistorem nr 1
int Rf2 = 2685;				// rezystancja rezystora szeregowego z termistorem nr 2

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
String BAND[BAND_NUM] = {"    160", "    80", "    40", "    30", "    20","    17", "    15","    12", "    10", "     6"};
byte bandIdx = 1;
byte AutoBandIdx = 15;

#define thresholdCurrent           1.0
#define thresholdPower             5.0
#define thresholdSWR               3.0
#define thresholdTemperaturAirOn1   30
#define thresholdTemperaturTransistorMax	60		// temperatura tranzystora (z termistora nr 1), przy której PA jest blokowane - STBY
#define thresholdTemperaturAirOn2   30

String infoString = "";
String warningString = "";		// nieużywany
String errorString = "";
bool genOutputEnable;

// Touch
int touchX = -1;
int touchY = -1;

unsigned long timeAtCycleStart, timeAtCycleEnd, timeStartMorseDownTime,
		actualCycleTime, timeToogle500ms = 0;
int drawWidgetIndex;
bool toogle500ms;

#define cycleTime        30
// SWR:
float pwrForwardReturnRatio;
float swrValue = 1.0;
// SWR by UHSDR
// SWR and RF power meter public
typedef struct SWRMeter
{
    float fwd_calc;         // forward power readings in A/D units
    float rev_calc;         // reverse power readings in A/D units
    float fwd_pwr;          // forward power in watts current measurement
    float rev_pwr;          // reverse power in watts current measurement
    float fwd_pwr_avg;      // forward power in watts averaged
    float rev_pwr_avg;      // reverse power in watts averaged

    float fwd_dbm;          // forward power in dBm
    float rev_dbm;          // reverse power in dBm
    float vswr;             // vswr
    float vswr_dampened;        // dampened VSWR reading
    bool  pwr_meter_disp;       // TRUE if numerical FWD/REV power metering (in milliwatts) is to be displayed
    bool  pwr_meter_was_disp;   // TRUE if numerical FWD/REV power metering WAS displayed (used to clear it)
    byte   p_curr;         // count used to update power meter
    byte   sensor_null;        // used to null out the sensor offset voltage

    uint8_t coupling_calc[BAND_NUM];

    bool high_vswr_detected;

} SWRMeter;
#define SWR_SAMPLES_CNT             5//10
#define SWR_ADC_VOLT_REFERENCE          3.3     // NOMINAL A/D reference voltage.
//
// coefficients for very low power (<75 milliwatt) power levels.  Do NOT use this above approx. 0.07 volts input!
//
#define LOW_RF_PWR_COEFF_A          -0.0338205168744131     // constant (offset)
#define LOW_RF_PWR_COEFF_B          5.02584652062682        // "b" coefficient (for x)
#define LOW_RF_PWR_COEFF_C          -106.610490958242       // "c" coefficient (for x^2)
#define LOW_RF_PWR_COEFF_D          853.156505329744        // "d" coefficient (for x^3)
//
// coefficients for higher power levels (>50 milliwatts).  This is actually good down to 25 milliwatts or so.
//
#define HIGH_RF_PWR_COEFF_A         0.01209 //0.0120972709513557        // constant (offset)
#define HIGH_RF_PWR_COEFF_B         0.8334  //0.833438917330908     // "b" coefficient (for x)
#define HIGH_RF_PWR_COEFF_C             1.569   //1.56930042559198      // "c" coefficient (for x^2)

#define LOW_POWER_CALC_THRESHOLD        0.05    // voltage from sensor below which we use the "low power" calculations, above
#define SWR_CAL_MIN             75
#define SWR_CAL_MAX             150
#define SWR_CAL_DEFAULT             100
//
#define SENSOR_NULL_MIN             75
#define SENSOR_NULL_MAX             125
#define SENSOR_NULL_DEFAULT         100
#define	SWR_COUPLING_MIN					50
#define	SWR_COUPLING_MAX					150
#define	SWR_COUPLING_DEFAULT				100

SWRMeter                    swrm;

// end SWR by UHSDR
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

		if (value < _valueOld)
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
};
// SETUP the grafic objects
//                        title         unit    xPos  yPos  height  width, _minValue,  _maxValue,  colorValue       colorBack             font
InfoBox modeBox("MODE", "", 395, 60, 32, 200, 0, 0, vgaValueColor, vgaBackgroundColor, Grotesk16x32);
//InfoBox frequencyBox("Frequency", "MHZ", 395, 20, 32, 200, 1.7, 55.0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

InfoBox bandBox("LPF", "m", 20, 20, 72, 350, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold32x64);

InfoBox drainVoltageBox("DRAIN", "V", 20, 340, 32, 125, 48, 54, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox aux1VoltageBox("AUX R", "V", 170, 340, 32, 125, 11, 15, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
// InfoBox aux2VoltageBox("AUX B", "V", 320, 340, 32, 125, 11, 14, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox airBox1("AIR1", "", 470, 340, 32, 125, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox airBox2("AIR2", "", 470, 380, 32, 125, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

InfoBox pa1AmperBox("PA 1", "A", 20, 380, 32, 125, 0, 32.0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
//InfoBox pa2AmperBox("PA 2", "A", 170, 380, 32, 125, 0, 24.9, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

InfoBox temperaturBox1("", "`C", 320, 340, 32, 125, 10, 60, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);
InfoBox temperaturBox2("", "`C", 320, 380, 32, 125, 10, 60, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);


InfoBox emptyBox("", "", 170, 380, 32, 125, 0.0, 0.0, vgaValueColor, vgaBackgroundColor, GroteskBold16x32);

InfoBox msgBox("", "", 20, 420, 32, 760, 0, 0, vgaValueColor, vgaBackgroundColor, Grotesk16x32);
InfoBox txRxBox("", "", 645, 340, 72, 135, 0, 0, vgaValueColor, vgaBackgroundColor, GroteskBold32x64);

// title, unit, xPos, yPos, height, width, minValue, maxValue, warnValue1, warnValue2, colorBar, colorBack, noOffHelplines
//DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 2500, 750, 1750, vgaBarColor, vgaBackgroundColor, 10);
#ifdef SP2HYO
DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 1250, 375, 875, vgaBarColor, vgaBackgroundColor, 10);
#endif
#ifdef SP3JDZ
DisplayBar pwrBar("PWR", "W", 20, 126, 80, 760, 0, 500, 150, 350, vgaBarColor, vgaBackgroundColor, 10);
#endif

DisplayBar swrBar("SWR", "", 20, 226, 80, 760, 1, 5, 3, 4, vgaBarColor, vgaBackgroundColor, 16);

PushButton Down(20, 20, 72, 165);
PushButton Up(185, 20, 72, 165);


float getTemperatura(uint8_t pin, int Rf);
void read_inputs();
float calc_SWR(int forward, int ref);
// SWR by UHSDR
bool UpdatePowerAndVSWR();

void setup()
{
	// Run the setup and init everything
#ifdef DEBUG
		Serial.begin(115200);
#endif
	if (eeprom_read_byte(0) != COLDSTART_REF)
	{
		EEPROM.write(1, bandIdx);
		EEPROM.write(2, mode);
		EEPROM.write(0, COLDSTART_REF); // COLDSTART_REF in first byte indicates all initialized
#ifdef DEBUG
		Serial.println("writing initial values into memory");
#endif
	}
	else                       // EEPROM contains stored data, retrieve the data
	{
		// read the current band
		bandIdx = EEPROM.read(1);
		// read mode
		mode = EEPROM.read(2);
#ifdef DEBUG
		Serial.println("reading bandIdx from memory: ");
		Serial.println(bandIdx);
#endif

	}

	myGLCD.InitLCD();
	// The following two lines are needed for the  display
	// module to enable the backlight. If you are using any other
	// display module these lines should be commented out.
	// -------------------------------------------------------------
	// pinMode(8, OUTPUT);  	//backlight
	// digitalWrite(8, HIGH);	//on
	// -------------------------------------------------------------
	// - stan wysoki jest wymuszony rezystorem na nóżce 39 wyświetlacza - pin 8 jest zwolniony
	myGLCD.clrScr();

	myTouch.InitTouch(LANDSCAPE);
	myTouch.setPrecision(PREC_MEDIUM);

#ifdef CZAS_PETLI
	pinMode(doPin_Czas_Petli, OUTPUT);
#endif
	pinMode(diPin_Termostat, INPUT_PULLUP);
	pinMode(doPin_SWR_ant, OUTPUT);
	digitalWrite(doPin_SWR_ant, HIGH);
	pinMode(doPin_blokada, OUTPUT);
	pinMode(doPin_errLED, OUTPUT);
	pinMode(diPin_SWR_ster_max, INPUT_PULLUP);
	pinMode(doPin_Band_A, OUTPUT);
	pinMode(doPin_Band_B, OUTPUT);
	pinMode(doPin_Band_C, OUTPUT);
	pinMode(doPin_Band_D, OUTPUT);
	pinMode(diPin_bandData_A, INPUT_PULLUP);
	pinMode(diPin_bandData_B, INPUT_PULLUP);
	pinMode(diPin_bandData_C, INPUT_PULLUP);
	pinMode(diPin_bandData_D, INPUT_PULLUP);

	pinMode(doPin_air1, OUTPUT);
	pinMode(doPin_air2, OUTPUT);

	pinMode(diPin_ptt, INPUT_PULLUP);
	pinMode(diPin_SWR_LPF_max, INPUT_PULLUP);
	pinMode(diPin_stby, INPUT_PULLUP);
	pinMode(diPin_Imax, INPUT_PULLUP);
	pinMode(diPin_Pmax, INPUT_PULLUP);
	pinMode(diPin_SWRmax, INPUT_PULLUP);

	myGLCD.setFont(GroteskBold32x64);
	myGLCD.setColor(vgaValueColor);
	myGLCD.print("LDMOS-PA  1kW", CENTER, 50);
	myGLCD.setFont(Grotesk16x32);
	myGLCD.print("160m - 6m", CENTER, 150);
	//myGLCD.print("BLF 188XR", CENTER, 200);

/*  myGLCD.setFont(franklingothic_normal);
	myGLCD.print("TNX TO", CENTER, 290);
	myGLCD.print("DC5ME DF1AI DG2MEL DL3MBG DL6MFK", CENTER, 325);
	myGLCD.print("DL9MBI DO1FKP DO5HT K8FOD ON7PQ", CENTER, 350);
	myGLCD.print("DE DJ8QP", CENTER, 375);
*/
	delay(2000);
	myGLCD.clrScr();

	// Set call sign and version
	myGLCD.setFont(nadianne);
	myGLCD.setColor(vgaTitleUnitColor);
	//myGLCD.print("DJ8QP ", RIGHT, 20);
	//myGLCD.print("DC5ME ", RIGHT, 40);
	myGLCD.setFont(SmallFont);
	myGLCD.print("V1.9.1  ", RIGHT, 60);

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
	msgBox.init();
	txRxBox.init();
	pwrBar.init();
	swrBar.init();
	bandBox.init();
	bandBox.setText(BAND[bandIdx]);
	airBox1.init();
	airBox1.setText("OFF");
	airBox2.init();
	airBox2.setText("OFF");
	emptyBox.init();
	// Start init test
	// przeniosłem do setupu
	read_inputs();
	pwrBar.setValue(pwrForwardValue, false);
	/*
	 * nie sprawdzam SWR po włączeniu - wystarczy sprawdzić, czy jest moc
	swrValue = calc_SWR(forwardValue, returnValue);
	swrBar.setValue(swrValue, false);
	 */
	drainVoltageBox.setFloat(drainVoltageValue, 1, 4, false);
	aux1VoltageBox.setFloat(aux1VoltageValue, 1, 4, false);
	pa1AmperBox.setFloat(pa1AmperValue, 1, 4, false);
	temperaturBox1.setFloat(temperaturValue1, 1, 5, false);
	temperaturBox2.setFloat(temperaturValue2, 1, 5, false);
	if (temperaturValue1 > thresholdTemperaturTransistorMax)
	{
		TemperaturaTranzystoraMaxValue = true;
	}

	if (not drainVoltageBox.isValueOk())
	{
		errorString = "Startup error: Drain voltage not Ok";
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
		errorString = "Startup error: Protector Imax detected";
	}
	else if (PmaxValue == true)
	{
		errorString = "Startup error: Protector Pmax detected";
	}
	else if (SWRmaxValue == true)
	{
		errorString = "Startup error: Protector SWR max detected";
	}
	else if (SWRLPFmaxValue == true)
	{
		errorString = "Startup error: Protector SWR LPF max detected";
	}
	else if (TemperaturaTranzystoraMaxValue == true)
	{
		errorString = "Startup error: Protector transistor temp max detected";
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
	// SWR by UHSDR
	// SWR meter init
	swrm.p_curr				= 0;
	swrm.fwd_calc			= 0;
	swrm.rev_calc			= 0;
	swrm.fwd_pwr			= 0;
	swrm.rev_pwr			= 0;
	swrm.fwd_dbm			= 0;
	swrm.rev_dbm			= 0;
	swrm.vswr			 	= 0;
	swrm.sensor_null		= SENSOR_NULL_DEFAULT;
	{
		int idx;
		for (idx = 0; idx < BAND_NUM; idx++)
		{
			swrm.coupling_calc[idx]    = SWR_COUPLING_DEFAULT;
		}
	}
	swrm.pwr_meter_disp		= 0;	// Display of numerical FWD/REV power metering off by default
	swrm.pwr_meter_was_disp = 0;	// Used to indicate if FWD/REV numerical power metering WAS displayed
}

void loop()
{
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
	Serial.println("---------------------");
	Serial.print("ptt: ");
	Serial.println(pttValue);
	Serial.print("stby: ");
	Serial.println(stbyValue);
	Serial.print("pwrForward: ");
	Serial.println(pwrForwardValue);
	Serial.print("pwrReturn: ");
	Serial.println(pwrReturnValue);
	Serial.print("drainVoltage: ");
	Serial.println(drainVoltageValue);
	Serial.print("aux1Voltage: ");
	Serial.println(aux1VoltageValue);
	Serial.print("pa1Amper: ");
	Serial.println(pa1AmperValue);
	Serial.print("temperatura1: ");
	Serial.println(temperaturValue1);
	Serial.print("input:      ");
	Serial.println(analogRead(aiPin_temperatura2));
	Serial.print("temperatura2: ");
	Serial.println(temperaturValue2);
#endif

	//-----------------------------------------------------------------------------
	// Set display values. The widgets monitors the values and output an errorString
	// SWR by UHSDR
	if (UpdatePowerAndVSWR())
	{
		pwrForwardValue = sq(forwardValue * pwrForwardFactor) / 50;
		pwrReturnValue = sq(returnValue * pwrReturnFactor)/50;
		pwrBar.setValue(pwrForwardValue, drawWidgetIndex == 1);

		swrValue = swrm.vswr;
		swrBar.setValue(swrValue, drawWidgetIndex == 3);
		if (swrValue > thresholdSWR)
		{
			// blokada od wysokiego SWR anteny na razie wyłączona
			 digitalWrite(doPin_SWR_ant, LOW);
			 SWR3Value = true;
		}
		else
		{
			//digitalWrite(doPin_SWR_ant, LOW);
			SWR3Value = false;
		}
	}

	drainVoltageBox.setFloat(drainVoltageValue, 1, 4, drawWidgetIndex == 5);
	aux1VoltageBox.setFloat(aux1VoltageValue, 1, 4, drawWidgetIndex == 6);
	pa1AmperBox.setFloat(pa1AmperValue, 1, 4, drawWidgetIndex == 8);
	temperaturBox1.setFloat(temperaturValue1, 1, 5, drawWidgetIndex == 9);
	temperaturBox2.setFloat(temperaturValue2, 1, 5, drawWidgetIndex == 10);
	// temperatura1 i wentylator1
	if (temperaturValue1 >= thresholdTemperaturAirOn1)
	{
		airBox1.setText("ON");
	}
	else if (temperaturValue1 <= thresholdTemperaturAirOn1 - 2)
	{
		airBox1.setText("OFF");
	}
	if (temperaturValue1 > thresholdTemperaturTransistorMax)	// przekroczenie temperatury granicznej obudowy tranzystora
	{
		TemperaturaTranzystoraMaxValue = true;
	}
	else if (temperaturValue1 < thresholdTemperaturTransistorMax - 5)
	{
		TemperaturaTranzystoraMaxValue = false;
	}

	if (temperaturValue2 >= thresholdTemperaturAirOn2)
	{
		airBox2.setText("ON");
	}
	else if (temperaturValue2 <= thresholdTemperaturAirOn2 - 2)
	{
		airBox2.setText("OFF");
	}

	// Draw index defines the infoBox that can draw new values on the utft.
	// If all infoBoxes would draw together, the cycletime is to long and not constant for the morse output.
	if (drawWidgetIndex == 10)
	{
		drawWidgetIndex = 0;
	}
	else
	{
		drawWidgetIndex++;
	}

	// Monitor additional inputs and set errorString
	if (ImaxValue == true)
	{
		errorString = "Error: Protector Imax detected";
	}
	if (TermostatValue == true)
	{
		errorString = "Error: Termostat on";
	}
	if (PmaxValue == true)
	{
		errorString = "Error: Protector Pmax detected";
	}
	if (SWRmaxValue == true)
	{
		errorString = "Error: Protector SWR max detected";
	}

	if (SWRLPFmaxValue == true)
	{
		errorString = "Error: Protector SWR LPF max detected";
	}
	if (SWR_ster_max == true)
	{
		errorString = "Error: Protector input SWR max detected";
	}
	if (TemperaturaTranzystoraMaxValue == true)
	{
		errorString = "Error: Protector transistor temp max detected";
	}
	if (SWR3Value == true)
	{
		errorString = "Error: dangerous SWR detected";
	}
	else if (errorString == "")
	{
		digitalWrite(doPin_SWR_ant, HIGH);
	}

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
		else if (pttValue == false and modeBox.getText() == modeManualName
				and Up.isTouchInside(touchX, touchY))
		{
			if (bandIdx >= BAND_NUM - 1)
			{
				bandIdx = 0;
			}
			else
			{
				bandIdx++;
			}
			byla_zmiana = true;
			czas_zmiany = millis();
			bandBox.setText(BAND[bandIdx]);
		}
		else if (pttValue == false and modeBox.getText() == modeManualName
				and Down.isTouchInside(touchX, touchY))
		{
			if (bandIdx <= 0)
			{
				bandIdx = BAND_NUM - 1;
			}
			else
			{
				bandIdx--;
			}
			byla_zmiana = true;
			czas_zmiany = millis();
			bandBox.setText(BAND[bandIdx]);
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
			airBox1.setText("ON");	// nie zadziała -> samo się wyłączy po sprawdzeniu temperatury
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
			txRxBox.setColorValue(vgaBackgroundColor);
			txRxBox.setColorBack(VGA_RED);
			txRxBox.setText(" TX");
		}
		else
		{
			txRxBox.setColorValue(vgaBackgroundColor);
			txRxBox.setColorBack(VGA_GREEN);
			txRxBox.setText("OPR");
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
		AutoBandIdx = digitalRead(diPin_bandData_D) << 3 | digitalRead(diPin_bandData_C) << 2 | digitalRead(diPin_bandData_B) << 1 | digitalRead(diPin_bandData_A);
		AutoBandIdx = AutoBandIdx -1;
		if (AutoBandIdx >= 0 and AutoBandIdx <= 9)
		{
			bandIdx = AutoBandIdx;
			bandBox.setText(BAND[bandIdx]);
		}
	}

	//-----------------------------------------------------------------------------
	// Write to outputs
	if (airBox1.getText() == "OFF")
	{
		digitalWrite(doPin_air1, false);
	}
	else
	{
		digitalWrite(doPin_air1, true);
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
		switch (bandIdx)
		{
		case 0:		// 160m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 0);
			digitalWrite(doPin_Band_B, 0);
			digitalWrite(doPin_Band_A, 0);
			break;
		case 1:		// 80m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 0);
			digitalWrite(doPin_Band_B, 0);
			digitalWrite(doPin_Band_A, 1);
			break;
		case 2:		// 40m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 0);
			digitalWrite(doPin_Band_B, 1);
			digitalWrite(doPin_Band_A, 0);
			break;
		case 3:		// 30m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 0);
			digitalWrite(doPin_Band_B, 1);
			digitalWrite(doPin_Band_A, 1);
			break;
		case 4:		// 20m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 1);
			digitalWrite(doPin_Band_B, 0);
			digitalWrite(doPin_Band_A, 0);
			break;
		case 5:		// 17m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 1);
			digitalWrite(doPin_Band_B, 0);
			digitalWrite(doPin_Band_A, 1);
			break;
		case 6:		// 15m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 1);
			digitalWrite(doPin_Band_B, 1);
			digitalWrite(doPin_Band_A, 0);
			break;
		case 7:		// 12m
			digitalWrite(doPin_Band_D, 0);
			digitalWrite(doPin_Band_C, 1);
			digitalWrite(doPin_Band_B, 1);
			digitalWrite(doPin_Band_A, 1);
			break;
		case 8:		// 10m
			digitalWrite(doPin_Band_D, 1);
			digitalWrite(doPin_Band_C, 0);
			digitalWrite(doPin_Band_B, 0);
			digitalWrite(doPin_Band_A, 0);
			break;
		case 9:		// 6m
			digitalWrite(doPin_Band_D, 1);
			digitalWrite(doPin_Band_C, 0);
			digitalWrite(doPin_Band_B, 0);
			digitalWrite(doPin_Band_A, 1);
			break;
		default:
			break;
		}
	}

	if ((ImaxValue or TermostatValue or PmaxValue or SWRmaxValue or SWRLPFmaxValue or SWR_ster_max or TemperaturaTranzystoraMaxValue or not genOutputEnable)  and toogle500ms)
	{
		digitalWrite(doPin_errLED, LOW);
	}
	else
	{
		digitalWrite(doPin_errLED, HIGH);
	}

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
	    EEPROM.write(1, bandIdx);           // writing current band into eeprom
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
		Serial.print("bandIdx: ");
		Serial.println(bandIdx);
		Serial.print("mode: ");
		Serial.println(modeBox.getText());
#endif
	}

#ifdef CZAS_PETLI
	PORTE ^= (1<<PE4);		// nr portu na sztywno! = D2
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

float getTemperatura(uint8_t pin, int Rf)
{
	int u = analogRead(pin);
	float U = Vref*u/1023;
	float R = Rf*U/(Uref - U);
	float T = 1/(log(R/R25)/beta + 1/298.15);
#ifdef DEBUGi
	Serial.print("analogRead: ");
	Serial.println(u);
	Serial.print("U: ");
	Serial.println(U);
	Serial.print("R: ");
	Serial.println(R);
	Serial.print("T: ");
	Serial.println(T);
#endif
	return T - 273.15;
}

void read_inputs()
{
	//-----------------------------------------------------------------------------
	// Read all inputs
	forwardValue = analogRead(aiPin_pwrForward);
	returnValue = analogRead(aiPin_pwrReturn);
	drainVoltageValue = analogRead(aiPin_drainVoltage) * drainVoltageFactor;
	aux1VoltageValue = analogRead(aiPin_aux1Voltage) * aux1VoltageFactor;
	pa1AmperValue = analogRead(aiPin_pa1Amper)*pa1AmperFactor;
	temperaturValue1 = getTemperatura(aiPin_temperatura1, Rf1);
	temperaturValue2 = getTemperatura(aiPin_temperatura2, Rf2);

	pttValue = not digitalRead(diPin_ptt);					// aktywny stan niski
	stbyValue = digitalRead(diPin_stby);					// aktywny stan wysoki
	ImaxValue = not digitalRead(diPin_Imax);				// aktywny stan niski
	PmaxValue = not digitalRead(diPin_Pmax);				// aktywny stan niski
	SWRmaxValue = not digitalRead(diPin_SWRmax);			// aktywny stan niski
	SWRLPFmaxValue = not digitalRead(diPin_SWR_LPF_max);	// aktywny stan niski
	SWR_ster_max = not digitalRead(diPin_SWR_ster_max);		// aktywny stan niski
	TermostatValue = not digitalRead(diPin_Termostat);		// aktywny stan niski
}

float calc_SWR(int forward, int ref)
{
	float swr;
	if (ref > 0)
	{
		if (forward > ref)
		{
			swr = (float) (forward + ref) / (forward - ref);
			if (swr > 9.9)
			{
				swr = 9.9;
			}
		}
		else
		{
			swr = 9.9;
		}
	}
	else
	{
		swr = 1;
	}
	return swr;
}

// SWR by UHSDR
bool UpdatePowerAndVSWR()
{
	//uint16_t val_p, val_s = 0;
	bool retval = false;

	// Collect samples
	if (swrm.p_curr < SWR_SAMPLES_CNT)
	{
		// Add to accumulator to average A/D values
		swrm.fwd_calc += forwardValue;
		swrm.rev_calc += returnValue;
		swrm.p_curr++;
	}
	else
	{
		// Compute average values
		swrm.fwd_pwr = sq((swrm.fwd_calc / SWR_SAMPLES_CNT) * pwrForwardFactor) / 50;
		swrm.rev_pwr = sq((swrm.rev_calc / SWR_SAMPLES_CNT) * pwrReturnFactor) / 50;

		/*
		PowerFromADCValue(swrm.fwd_calc / SWR_SAMPLES_CNT, sensor_null,
				coupling_calc, &swrm.fwd_pwr, &swrm.fwd_dbm);
		PowerFromADCValue(swrm.rev_calc / SWR_SAMPLES_CNT, sensor_null,
				coupling_calc, &swrm.rev_pwr, &swrm.rev_dbm);
*/
		// Reset accumulators and variables for power measurements
		swrm.p_curr = 0;
		swrm.fwd_calc = 0;
		swrm.rev_calc = 0;
		// Calculate VSWR from power readings

		swrm.vswr = (1 + sqrtf(swrm.rev_pwr / swrm.fwd_pwr))
				/ (1 - sqrtf(swrm.rev_pwr / swrm.fwd_pwr));

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
