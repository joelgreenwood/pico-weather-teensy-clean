This is code for a weather alarm clock for the NT Core mini-course at Yale.  Weather is fetched by a raspberry pi pico2W and displayed on a paperwhite 2.1 display.  The pico sends the current outdoor temp to the teensy 4.0 and the teensy does closed loop temperature control with a peltier, thermosistor and h-bridge.  A touch sensor on the teensy activates temperature control and a plate is brough to the current outdoor temperature.

I based the pico code on this site: https://github.com/outofcoffee/pico-weather

A good how-to for wiring the pico is here: https://peppe8o.com/raspberry-pi-pico-weather-monitor/
  This is also a good source of alternative weather presentation.

I used a capacitive touch library for the teensy 4.0: https://github.com/adrianfreed/FastTouch

This is the lookup table for the tempature sensor: https://cdn-shop.adafruit.com/datasheets/103_3950_lookuptable.pdf

You will need your own API key from: https://home.openweathermap.org/
  In order to do the "one call" API call we are doing you need to give them a credit card.  You will not be charged if you use fewer than 1000 API calls in 24 hours.  However, if you make a programming error and you go over that you will be charged in batchs of 100 calls.  So yes, you will need to sign up but it's free as long as you're sane about how many times a day you ask for the weather.

