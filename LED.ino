#include <stdint.h>
#include "FastLED.h"

#define NUM_LEDS 2

#define DATA_PIN 10

// Define the array of leds
CRGB leds[NUM_LEDS];

void LED_setup() { 
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  // Turn the LED on
  leds[0]  = CRGB::Green;
  //Turn the other LED Blue
  leds[1] = CRGB::Green;
  FastLED.show();
}

void setColor(uint8_t rgb[6]) {
  leds[0] = CRGB(rgb[0], rgb[1], rgb[2]);
  leds[1] = CRGB(rgb[3], rgb[4], rgb[5]);
  FastLED.show();
}

void LEDLoop() { 

}
