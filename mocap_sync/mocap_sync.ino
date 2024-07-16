/*
  Hardware syncing with mocap and force plate
  Create by Fred Wang, March 2023
  Modified by Tim Han, July 2024
*/

#define syncInterruptPin 2
int syncingState = 0;
unsigned long timeElapsedContinous = 0;
unsigned long lastSyncTime = 0; 

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // setup ADC
  analogReference(INTERNAL);
  delay(1);

  // setting up interrupt for sync signal
  pinMode(syncInterruptPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(syncInterruptPin), detectSyncInput, FALLING);
}

void loop() {
  // detect approximately at 1000hz
  delay(1);

  // reset if received character "r"
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      Serial.println("Reset Successful");
      syncingState = 0;
    }
  }

  // if the sync signal drops out
  timeElapsedContinous = millis() - lastSyncTime;
  if ((timeElapsedContinous > 100)&(syncingState == 2)){  // minimal sync frequency of 10HZ
    Serial.println("End Recording");  // end of 
    syncingState = 3; // waiting for reset
  }

  // Serial.println(syncingState);
}

void detectSyncInput(){
  // wait for syncing signal
  unsigned long timeElapsed = millis() - lastSyncTime;
  lastSyncTime = millis();
  switch (syncingState){
    case 0: // waiting for the sync signal when preparing
      if (timeElapsed < 100) syncingState = 1;//minimal sync frequency of 10HZ
      break;
    case 1: // sync signal received, now wait for the pause when starting
      if (timeElapsed > 500) {
        syncingState = 2;
        Serial.println("Start Recording");
      }
      break;
    case 2: // the pause means that the mocap is starting capture, thus start sending data.
      // Serial.println("Start Recording");
      break;
  }
}

