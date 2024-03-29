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

#define DIGITAL_INPUT_SENSOR1 3         // The digital input you attached your sensor 1.
#define DIGITAL_INPUT_SENSOR2 4         // The digital input you attached your sensor 2.

#define DIGITAL_INPUT_SENSOR1A 14       // The digital input you attached your sensor 2.
#define DIGITAL_INPUT_SENSOR1R 15       // The digital input you attached your sensor 2.
#define DIGITAL_INPUT_SENSOR2A 16       // The digital input you attached your sensor 2.
#define DIGITAL_INPUT_SENSOR2R 17       // The digital input you attached your sensor 2.

#define PULSE_FACTOR 1000                       // Number of blinks per m3 of your meter (One rotation/liter)

#define MAX_FLOW 40                             // Max flow (l/min) value to report. This filters outliers.

#define DEBOUNCE 50 							              // Sometimes we get interrupt on RISING,  50ms debounce ( max 2000 l/min at 1ppl)

#define CHILD_ID1 1                              // Id of the sensor child
#define CHILD_ID2 2                              // Id of the sensor child

uint32_t SEND_FREQUENCY =
  30000;           // Minimum time between send (in milliseconds). We don't want to spam the gateway.

MyMessage flowMsg1(CHILD_ID1, V_FLOW);
MyMessage volumeMsg1(CHILD_ID1, V_VOLUME);
MyMessage lastCounterMsg1(CHILD_ID1, V_VAR1);

MyMessage flowMsg2(CHILD_ID2, V_FLOW);
MyMessage volumeMsg2(CHILD_ID2, V_VOLUME);
MyMessage lastCounterMsg2(CHILD_ID2, V_VAR1);

double ppl = ((double)PULSE_FACTOR) / 1000;      // Pulses per liter

uint32_t lastSend = 0;

volatile uint32_t pulseCount1 = 0;
volatile uint32_t lastBlink1 = 0;
volatile double flow1 = 0;
bool pcReceived1 = false;
bool startup1 = true;
uint32_t oldPulseCount1 = 0;
double oldflow1 = 0;
double oldvolume1 = 0;
uint32_t lastPulse1 = 0;
Debouncer debouncer1(DIGITAL_INPUT_SENSOR1, DEBOUNCE);

volatile uint32_t pulseCount2 = 0;
volatile uint32_t lastBlink2 = 0;
volatile double flow2 = 0;
bool pcReceived2 = false;
bool startup2 = true;
uint32_t oldPulseCount2 = 0;
double oldflow2 = 0;
double oldvolume2 = 0;
uint32_t lastPulse2 = 0;
Debouncer debouncer2(DIGITAL_INPUT_SENSOR2, DEBOUNCE);

Debouncer debouncer3(DIGITAL_INPUT_SENSOR1A, DEBOUNCE);
Debouncer debouncer4(DIGITAL_INPUT_SENSOR1R, DEBOUNCE);
Debouncer debouncer5(DIGITAL_INPUT_SENSOR2A, DEBOUNCE);
Debouncer debouncer6(DIGITAL_INPUT_SENSOR2R, DEBOUNCE);

void onPulse1()
{
  if (startup1)
  {
    Serial.println("pulse1 startup : ignored");
    flow1=0;
    startup1 = false;
    return;
  }

  Serial.println("pulse1");

  uint32_t newBlink = micros();
  uint32_t interval = newBlink - lastBlink1;

  if (interval != 0) {
    lastPulse1 = millis();
    flow1 = (60000000.0 / interval) / ppl;
  }
  lastBlink1 = newBlink;

  Serial.println("pulse1 counted");
  pulseCount1++;
}

void onPulse2()
{
  if (startup2)
  {
    Serial.println("pulse2 startup : ignored");
    flow2=0;
    startup2 = false;
    return;
  }

  Serial.println("pulse2");

  uint32_t newBlink = micros();
  uint32_t interval = newBlink - lastBlink2;

  if (interval != 0) {
    lastPulse2 = millis();
    flow2 = (60000000.0 / interval) / ppl;
  }
  lastBlink2 = newBlink;

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

  pulseCount1 = oldPulseCount1 = 0;
  pulseCount2 = oldPulseCount2 = 0;

  // Fetch last known pulse count value from gw
  request(CHILD_ID1, V_VAR1);
  request(CHILD_ID2, V_VAR1);

  lastSend = lastPulse1 = lastPulse2 = millis();

  debouncer1.subscribe(Debouncer::Edge::FALL, [](const int state) {
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
  sendSketchInfo("Water Meter", "1.1");

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

    if (!pcReceived1) {
      //Last Pulsecount1 not yet received from controller, request it again
      request(CHILD_ID1, V_VAR1);
      return;
    }

    if (!pcReceived2) {
      //Last Pulsecount2 not yet received from controller, request it again
      request(CHILD_ID2, V_VAR1);
      return;
    }

    if (flow1 != oldflow1) {
      oldflow1 = flow1;

      Serial.print("l/min (1):");
      Serial.println(flow1);

      // Check that we don't get unreasonable large flow value.
      // could happen when long wraps or false interrupt triggered
      if (flow1 < ((uint32_t)MAX_FLOW)) {
        send(flowMsg1.set(flow1, 2));                   // Send flow value to gw
      }
    }

    if (flow2 != oldflow2) {
      oldflow2 = flow2;

      Serial.print("l/min (2):");
      Serial.println(flow2);

      // Check that we don't get unreasonable large flow value.
      // could happen when long wraps or false interrupt triggered
      if (flow2 < ((uint32_t)MAX_FLOW)) {
        send(flowMsg2.set(flow2, 2));                   // Send flow value to gw
      }
    }

    // No Pulse count received in 2min
    if (currentTime - lastPulse1 > 120000) {
      flow1 = 0;
    }

    if (currentTime - lastPulse2 > 120000) {
      flow2 = 0;
    }

    // Pulse count has changed
    if (pulseCount1 != oldPulseCount1) {
      oldPulseCount1 = pulseCount1;

      Serial.print("pulsecount1:");
      Serial.println(pulseCount1);

      send(lastCounterMsg1.set(pulseCount1));                  // Send  pulsecount value to gw in VAR1

      double volume1 = ((double)pulseCount1 / ((double)PULSE_FACTOR));
      if (volume1 != oldvolume1) {
        oldvolume1 = volume1;

        Serial.print("volume1:");
        Serial.println(volume1, 3);

        send(volumeMsg1.set(volume1, 3));               // Send volume value to gw
      }
    }

    if (pulseCount2 != oldPulseCount2) {
      oldPulseCount2 = pulseCount2;

      Serial.print("pulsecount2:");
      Serial.println(pulseCount2);

      send(lastCounterMsg2.set(pulseCount2));                  // Send  pulsecount value to gw in VAR1

      double volume2 = ((double)pulseCount2 / ((double)PULSE_FACTOR));
      if (volume2 != oldvolume2) {
        oldvolume2 = volume2;

        Serial.print("volume2:");
        Serial.println(volume2, 3);

        send(volumeMsg2.set(volume2, 3));               // Send volume value to gw
      }
    }
  }
}

void receive(const MyMessage &message)
{
  if (message.getSensor() == CHILD_ID1 && message.getType() == V_VAR1) {
    uint32_t gwPulseCount = message.getULong();
    pulseCount1 += gwPulseCount;
    flow1 = oldflow1 = 0;
    Serial.print("Received last pulse count 1 from gw:");
    Serial.println(pulseCount1);
    pcReceived1 = true;
  }

  if (message.getSensor() == CHILD_ID2 && message.getType() == V_VAR1) {
    uint32_t gwPulseCount = message.getULong();
    pulseCount2 += gwPulseCount;
    flow2 = oldflow2 = 0;
    Serial.print("Received last pulse count 2 from gw:");
    Serial.println(pulseCount2);
    pcReceived2 = true;
  }
}
