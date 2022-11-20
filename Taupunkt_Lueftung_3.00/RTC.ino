//////////////////////////////////////////////////////////////////////////////
// Das Taupunkt-Lüftungssystem
// mit Datenlogging
//
// Routinen zur Erzeugung eines Zeitstempels
//
// veröffentlicht in der MAKE 2/2022
//
// Ulrich Schmerold
// 3/2022
//////////////////////////////////////////////////////////////////////////////
#include <TimeLib.h>
#include <DS1307RTC.h>

#if IS_RTC_ENABLED
static tmElements_t tm; //time element

const char *createTimeStamp ()
{
  RTC.read(tm);
	static char timeStr[sizeof "YYYY-MM-DD hh:mm:ss"];

	snprintf(
		timeStr, sizeof timeStr,
		"%04d-%02d-%02d %02d:%02d:%02d",
		tm.Year + 1970,
		tm.Month,
		tm.Day,
		tm.Hour,
		tm.Minute,
		tm.Second
	);

	return timeStr;
}


void startRTC()
{
	//rtc.init();
  
  if (RTC.read(tm)) {

	  lcd.clear();
	  lcd.setCursor(0, 0);
	  lcd.print(F("RTC initialized."));

	  #if IS_USB_DEBUG_ENABLED
  		Serial.println(F("RTC OK"));
  	#endif

  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
  }

	delay(1000);

	wdt_reset();
	lcd.clear();
}

static uint8_t today = 0;

static bool _hasDayChanged = true;
static bool _isFirstDaySinceStart = true;

void getNow() {
	RTC.read(tm);

	_hasDayChanged = (today != tm.Day) ? true : false;

	// No day change, if it is not initialized.
	if (_hasDayChanged && today != 0)
	{
		_isFirstDaySinceStart = false;
	}

	today = tm.Day;
}

bool hasDayChanged() {
	return _hasDayChanged;
}

unsigned int getMinutesOfDay() {
	return tm.Hour * 60 + tm.Minute;
}

unsigned int getSecondsOfDay() {
	return tm.Hour * 3600 + tm.Minute * 60 + tm.Second;
}

bool isFirstDaySinceStart() {
	return _isFirstDaySinceStart;
}

#else

const char *createTimeStamp () { return F("No time available"); }
void startRTC() {}
void getNow() {}
bool hasDayChanged() { return false; }

#endif // IS_RTC_ENABLED
