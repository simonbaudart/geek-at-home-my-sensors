/*
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2019 Sensnology AB
   Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik Ekblad
   Version 1.1 - GizMoCuz

   DESCRIPTION
   Use this sensor to measure volume and flow of your house water meter.
   You need to set the correct pulsefactor of your meter (pulses per m3).
   The sensor starts by fetching current volume reading from gateway (VAR 1).
   Reports both volume and flow back to gateway.

   Unfortunately millis() won't increment when the Arduino is in
   sleepmode. So we cannot make this sensor sleep if we also want
   to calculate/report flow.
   http://www.mysensors.org/build/pulse_water
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

// Enable signature
//#define MY_SIGNING_SOFT
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN 7
//#define MY_SIGNING_REQUEST_SIGNATURES

#define MY_NODE_ID 1

#include <MySensors.h>
#include <Debouncer.h>

#define DIGITAL_INPUT_SENSOR1 3
#define DIGITAL_INPUT_SENSOR2 4

#define DIGITAL_INPUT_SENSOR1A 14.
#define DIGITAL_INPUT_SENSOR1R 15.
#define DIGITAL_INPUT_SENSOR2A 16.
#define DIGITAL_INPUT_SENSOR2R 17.

#define DEBOUNCE 50

#define CHILD_ID1 1
#define CHILD_ID2 2

uint32_t SEND_FREQUENCY = 30000;                 // Minimum time between send (in milliseconds). We don't want to spam the gateway.

MyMessage volumeMsg1(CHILD_ID1, V_VOLUME);

MyMessage volumeMsg2(CHILD_ID2, V_VOLUME);

uint32_t lastSend = 0;
bool warmup = true;

uint32_t pulseCount1 = 0;
uint32_t oldPulseCount1 = 0;
bool pcReceived1 = false;
Debouncer debouncer1(DIGITAL_INPUT_SENSOR1, DEBOUNCE);

uint32_t pulseCount2 = 0;
uint32_t oldPulseCount2 = 0;
bool pcReceived2 = false;
Debouncer debouncer2(DIGITAL_INPUT_SENSOR2, DEBOUNCE);

Debouncer debouncer3(DIGITAL_INPUT_SENSOR1A, DEBOUNCE);
Debouncer debouncer4(DIGITAL_INPUT_SENSOR1R, DEBOUNCE);
Debouncer debouncer5(DIGITAL_INPUT_SENSOR2A, DEBOUNCE);
Debouncer debouncer6(DIGITAL_INPUT_SENSOR2R, DEBOUNCE);

void onPulse1()
{
  if (warmup)
  {
    Serial.println("pulse1 startup : ignored");
    warmup = false;
    return;
  }

  Serial.println("pulse1 counted");
  pulseCount1++;
}

void onPulse2()
{
  if (warmup)
  {
    Serial.println("pulse2 startup : ignored");
    warmup = false;
    return;
  }

  Serial.println("pulse2 counted");
  pulseCount2++;
}

void setup()
{
  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
  pinMode(DIGITAL_INPUT_SENSOR1, INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_SENSOR2, INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_SENSOR1A, INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_SENSOR1R, INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_SENSOR2A, INPUT_PULLUP);
  pinMode(DIGITAL_INPUT_SENSOR2R, INPUT_PULLUP);

  pulseCount1 = 0;
  pulseCount2 = 0;

  // Fetch last known pulse count value from gw
  request(CHILD_ID1, V_VOLUME);
  request(CHILD_ID2, V_VOLUME);

  debouncer1.subscribe(Debouncer::Edge::FALL, [](const int state) 
  {
    onPulse1();
  });

  debouncer2.subscribe(Debouncer::Edge::FALL, [](const int state) {
    onPulse2();
  });

  debouncer3.subscribe(Debouncer::Edge::FALL, [](const int state) {
    Serial.println("pulse1 inc");
    pulseCount1++;
  });

  debouncer4.subscribe(Debouncer::Edge::FALL, [](const int state) {
    Serial.println("pulse1 dec");
    pulseCount1--;
  });

  debouncer5.subscribe(Debouncer::Edge::FALL, [](const int state) {
    Serial.println("pulse2 inc");
    pulseCount2++;
  });

  debouncer6.subscribe(Debouncer::Edge::FALL, [](const int state) {
    Serial.println("pulse2 dec");
    pulseCount2--;
  });
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Water Meter", "2.0");

  // Register this device as Water flow sensor
  present(CHILD_ID1, S_WATER);
  present(CHILD_ID2, S_WATER);
}

void loop()
{
  debouncer1.update();
  debouncer2.update();
  debouncer3.update();
  debouncer4.update();
  debouncer5.update();
  debouncer6.update();

  uint32_t currentTime = millis();

  // Only send values at a maximum frequency or woken up from sleep
  if (currentTime - lastSend > SEND_FREQUENCY) {
    lastSend = currentTime;

    warmup = false;

    if (!pcReceived1) {
      //Last Pulsecount1 not yet received from controller, request it again
      request(CHILD_ID1, V_VOLUME);
      return;
    }

    if (!pcReceived2) {
      //Last Pulsecount2 not yet received from controller, request it again
      request(CHILD_ID2, V_VOLUME);
      return;
    }

    // Pulse count has changed
    if (pulseCount1 != oldPulseCount1) {
      oldPulseCount1 = pulseCount1;

      Serial.print("pulsecount1:");
      Serial.println(pulseCount1);

      send(volumeMsg1.set(pulseCount1));               // Send volume value to gw
    }

    if (pulseCount2 != oldPulseCount2) {
      oldPulseCount2 = pulseCount2;

      Serial.print("pulsecount2:");
      Serial.println(pulseCount2);

      send(volumeMsg2.set(pulseCount2));               // Send volume value to gw
    }
  }
}

void receive(const MyMessage &message)
{
  if (message.getSensor() == CHILD_ID1 && message.getType() == V_VOLUME) {
    uint32_t gwPulseCount = message.getULong();
    pulseCount1 += gwPulseCount;
    Serial.print("Received last pulse count 1 from gw:");
    Serial.println(pulseCount1);
    pcReceived1 = true;
  }

  if (message.getSensor() == CHILD_ID2 && message.getType() == V_VOLUME) {
    uint32_t gwPulseCount = message.getULong();
    pulseCount2 += gwPulseCount;
    Serial.print("Received last pulse count 2 from gw:");
    Serial.println(pulseCount2);
    pcReceived2 = true;
  }
}
