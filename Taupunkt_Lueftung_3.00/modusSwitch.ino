#include "modeSwitch.h"

int checkModusSwitch() {
  int sensorValue = analogRead(MODUS_PIN);
  if (sensorValue < 450) {
    #if IS_USB_DEBUG_ENABLED
      Serial.println(F("Modus: ON"));
    #endif
    return MODUS_ON;
  } else if (sensorValue < 850) {
    #if IS_USB_DEBUG_ENABLED
      Serial.println(F("Modus: AUTO"));
    #endif
    return MODUS_AUTO;
  } else {
    #if IS_USB_DEBUG_ENABLED
      Serial.println(F("Modus: OFF"));
    #endif
    return MODUS_OFF;
  }
}
