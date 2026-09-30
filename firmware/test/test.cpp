// Host-side test of boat_motor_controller.ino using stub Arduino headers.
// Build and run:  g++ -std=c++17 -I. test.cpp -o /tmp/fw_test && /tmp/fw_test
#include "Arduino.h"
void stopAllMotors(bool); void readSerialCommands(); void checkWatchdog();
void maybeSendStatus(); void maybeSendImu(); void handleLine(const char*);
int clampThrust(int); void applyThrust(int,int);
#include "../boat_motor_controller/boat_motor_controller.ino"
int main(){
 setup(); assert(pins[2]==0 && pins[4]==0 && pins[7]==0 && pins[8]==0);
 Serial.input="L:0 R:40\n";loop();
 assert(pins[9]==102 && pins[10]==0 && pins[7]==1 && pins[8]==1);
 assert(pins[2]==0 && pins[4]==0);
 events.clear();handleLine("L:0 R:0");
 assert(pins[7]==0 && pins[8]==0 && pins[9]==0 && pins[10]==0);
 assert(events[4]==7 && events[5]==8);
 handleLine("L:0 R:-40");assert(pins[9]==0 && pins[10]==102);
 events.clear();stopAllMotors(true);
 assert(events[0]==2 && events[1]==4 && events[2]==7 && events[3]==8);
 handleLine("L:0 R:40");nowMs=501;loop();
 assert(watchdogTripped && pins[7]==0 && pins[8]==0);
 Serial.room=512; nowMs=1002;loop();
 assert(Serial.output.find("WDT:1 L:0 R:0")!=std::string::npos);
 assert(Serial.output.find("AZ:1.0000")!=std::string::npos);

 puts("PASS: command, zero, reverse, stop ordering, watchdog, telemetry");
}
