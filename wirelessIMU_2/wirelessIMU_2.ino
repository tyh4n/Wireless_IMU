/*
  code for PURE wireless IMU
  use seeed nrf52 boards v1.1.1 for board opition in arduino IDE
  TODO: FIX Crash when connecting and disconnecting mutiple times in short period

  xiao nrf52840 sense:
    Notes: 
    Transmitting in BLE with xiao nrf52840 sense, this board doesn't support bluetooth classic
    set the board to seeed XIAO nrf52840 sense every time when uploading BUT WHY??????
    services and characteristics defined for BLE:
      imus: imu service
        imux: eular x in degree 
        imuy: eular y in degree
        imuz: eular z in degree
      blebas: battery service
        blebas:battery level in %
      TODO: Update this list

  BNO055 IMU
    Notes:
    Modified the BNO055 library to add the function to the orientation to vertical.
    Modified the BNO055 library to apply SIC matrix.
    Calibration data for this bno055:
      Accelerometer: 78 -20 34 
      Gyro: 0 0 8 
      Mag: -366 26 -126 
      Accel Radius: 1000
      Mag Radius: 883MAGX: 9.56 MAGY: 28.25 MAGZ: 45.56	
      SIC MATRIX: 
        1.0000   -0.0157    0.0042
        -0.0157   1.0292    0.0037
        0.0042    0.0037    0.9831	
    FROM BNO055 library:
      This driver reads raw data from the BNO055
      Connections
      ===========
      Connect SCL to analog 5
      Connect SDA to analog 4
      Connect VDD to 3.3V DC
      Connect GROUND to common ground

      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2

*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <bluefruit.h>
#include "IEEE_11073_float/IEEE11073float.h"

int DebugMode = 0;
int calibrationMode = 0;//only output serial message when calibrated, good for reading calibration data, need to enable debug mode first
#define DEVICE_NAME "PURE Wireless IMU 2"

//==================BNO055 config==================
/* Set the delay between fresh samples */
//#define BNO055_SAMPLERATE_DELAY_MS 8 //delay-based frequency control
#define BNO055_PERIOD 20 //ms, 50Hz
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);


// ================== BLE config==================
//defining service and characteristic for data
//UUIDs
#define IMU_SERVICE_UUID    "efb9635201f011eebe560242ac120002"

#define IMU_EULAR_X_UUID    "efb966b801f011eebe560242ac120002"
#define IMU_EULAR_Y_UUID    "efb9684801f011eebe560242ac120002"
#define IMU_EULAR_Z_UUID    "efb9697401f011eebe560242ac120002"
#define IMU_EULAR_ALL_UUID  "efb9687401f011eebe560242ac120002"

#define IMU_DEBUG_UUID      "efb9635301f011eebe560242ac120002"
#define IMU_MAG_ALL_UUID    "efb966a801f011eebe560242ac120002"
#define IMU_DEBUG_MSG_UUID  "efb266a801f011eebe560242ac120002"

#define IMU_CALIB_STAT_UUID "efb96a8201f011eebe560242ac120002"

//(MTU, GAP event length,queued number of Handle Value Notifications,queued number of Write without Response)
#define BLE_CONN_PARAMS BLE_GATT_ATT_MTU_DEFAULT, 100, 10, 2 //slow for computer
//#define BLE_CONN_PARAMS BLE_GATT_ATT_MTU_DEFAULT, 100, 5, 1 // fast for another nrf52


//ALL THE SERVICES
BLEService        imus = BLEService(IMU_SERVICE_UUID);

BLECharacteristic imux = BLECharacteristic(IMU_EULAR_X_UUID);
BLECharacteristic imuy = BLECharacteristic(IMU_EULAR_Y_UUID);
BLECharacteristic imuz = BLECharacteristic(IMU_EULAR_Z_UUID);
BLECharacteristic imuall = BLECharacteristic(IMU_EULAR_ALL_UUID);

BLECharacteristic imucal = BLECharacteristic(IMU_CALIB_STAT_UUID);

BLEService        debugs = BLEService(IMU_DEBUG_UUID);
BLECharacteristic magall = BLECharacteristic(IMU_MAG_ALL_UUID);
BLECharacteristic debugmsg = BLECharacteristic(IMU_DEBUG_MSG_UUID);


BLEService        htms = BLEService(UUID16_SVC_HEALTH_THERMOMETER);
BLECharacteristic htmc = BLECharacteristic(UUID16_CHR_TEMPERATURE_MEASUREMENT);

BLEBas blebas;    // BAS (Battery Service) helper class instance

//===================SLEEP CONFIG================
#define WAKE_LOW_PIN  PIN_A0
#define WAKE_HIGH_PIN PIN_A1
#define SLEEPING_DELAY 100000 //unit of BNO055_PERIOD
bool sleeping = true;//if the system is sleeping
int sleep_timeout = SLEEPING_DELAY;
//================flash config, needed for low-power mode==========
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#if defined(CUSTOM_CS) && defined(CUSTOM_SPI)
  Adafruit_FlashTransport_SPI flashTransport(CUSTOM_CS, CUSTOM_SPI);

#elif defined(ARDUINO_ARCH_ESP32)
  Adafruit_FlashTransport_ESP32 flashTransport;
#else
  #if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;
  #elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);
  #else
    #error No QSPI/SPI flash are defined on your board variant.h !
  #endif
#endif
Adafruit_SPIFlash flash(&flashTransport);

//digging into freeRTOS
static TaskHandle_t  _updateDataHandle;
#define UPDATE_STACK_SZ       (256*6)


void setup(void)
{
  //=================setup usb serial//================== 
  if(DebugMode) Serial.begin(115200);
  // wait for serial port to open!
  //if(DebugMode) while (!Serial) delay(10);

  //==================setup BLE//==================
  Bluefruit.begin(2,0);
  //set device name 
  Bluefruit.setName(DEVICE_NAME);
  //Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.autoConnLed(false);
  //extremely important for high notification frequency
  Bluefruit.Periph.setConnInterval (6, 6);
  //increasing queued number of Handle Value Notifications greatly improves notification frequency
  //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);//not fast enough
  Bluefruit.configPrphConn(BLE_CONN_PARAMS); 
  Bluefruit.setTxPower(8);
  //
  //===================battery thingy=========================
  setupBatteryMeasurement();
  //temperature
  setupHTM();

  //===================imu=========================
  // Setup and suspend the imu, waiting to be restarted
  if (DebugMode) Serial.println("Initialise the Bluefruit nRF52 module");
  setupIMUService();
  setupIMUDebugService();
  setupIMUsensor();

  //start
  startAdv();
  //sleep everything
  for (int i = 1; i < 5; i ++ ){
     bno.enterSuspendMode();
     delay(100);
  }
  
  postDebugMessage(String("debug msg"));
  sleeping = true;//indicate that the device is sleeping
  suspendLoop();//we dont use loop here due to instability when suspending and resuming
  
}

//measure sending freq for debug
int sendingInterval = 0;
int lastSending = 0;

void loop(void){
  //do nothing
}

void updateData(void* arg){//main loop
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    if (DebugMode) Serial.println("/////////////////////////////");
    //only read data if the BLE is connected
    if(Bluefruit.connected()){
        if (DebugMode){
          sendingInterval = millis() - lastSending;
          lastSending = millis();
          Serial.print("Interval: ");
          Serial.println(sendingInterval);
        }
        postIMUEular();
        postIMUCalibrationStat();
        postIMUDebug();
        postSensorTemperature();
        postBatteryPercentage();
        sleep_timeout = SLEEPING_DELAY;
      }else{
          sleep_timeout --;
          if (sleep_timeout == 0) goSleep();
          if (DebugMode){
            Serial.print("sleep in: ");
              Serial.println(sleep_timeout);
          }
      }
    vTaskDelayUntil(&xLastWakeTime, BNO055_PERIOD);//use task-based delay for a stable frequency, may change to timer in the future
    //delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

void setupIMUsensor(){
  //need to strat to config mode in order to set orientation for sensor
  if(DebugMode) Serial.print("starting sensor ");
  while(!bno.begin(OPERATION_MODE_CONFIG)){
    if(DebugMode) Serial.print("no BNO055 detected, reconnecting");
    delay(250);
  }
  if(DebugMode) Serial.println("sensor started ");
  delay(50);
  //remap the sensor orientation: x:-sensorY, y:sensorZ, z:-sensorx
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P8);
  //apply sensor offset to calibrated value, not used when in cailbration mode 
  if(!calibrationMode){
    applySensorOffsets();
    //bno.applySICmatrix( 1.0000,    0.0006,    0.0057,    0.0006,    1.0366,    0.0050,    0.0057,    0.0050,    0.9867);
  }
  //set sensor back to normal NDOF mode
  bno.setMode(OPERATION_MODE_NDOF);
  delay(50);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  if(DebugMode){
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  }
  //use external oscalliator
  bno.setExtCrystalUse(true);
}
/* IMU report services */
void setupIMUService(){
  //==================setup BNO055//==================
  /* Initialise the sensor */
  imus.begin();
  //attach BLECharacteristics to imus service
  imux.setProperties(CHR_PROPS_NOTIFY);
  imux.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imux.setFixedLen(4);
  imux.setUserDescriptor("imux");
  imux.setPresentationFormatDescriptor(0x14,0,0x2763);//type,exponent,unit, see https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned_Numbers.pdf
  imux.begin();

  imuy.setProperties(CHR_PROPS_NOTIFY);
  imuy.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuy.setFixedLen(4);
  imuy.setUserDescriptor("imuy");
  imuy.setPresentationFormatDescriptor(0x14,0,0x2763);
  imuy.begin();

  imuz.setProperties(CHR_PROPS_NOTIFY);
  imuz.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuz.setFixedLen(4);
  imuz.setUserDescriptor("imuz");
  imuz.setPresentationFormatDescriptor(0x14,0,0x2763);
  imuz.begin();

  imuall.setProperties(CHR_PROPS_NOTIFY);
  imuall.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuall.setFixedLen(12);
  imuall.setUserDescriptor("imuall-4byteXYZ");
  imuall.begin();
  
  imucal.setProperties(CHR_PROPS_NOTIFY);
  imucal.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imucal.setFixedLen(2);
  imucal.setUserDescriptor("imucal");
  imucal.setPresentationFormatDescriptor(0x0e,0,0);
  imucal.begin();
}

void postIMUEular(){
  //sending imu data to BLE
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);//NOT USING EULAR DUE TO BUG IN BNO055
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> euler = quat.toEuler();
  float eularSend[3];
  eularSend[0] = float(euler.x()*RAD_TO_DEG);
  eularSend[1] = float(euler.y()*RAD_TO_DEG);
  eularSend[2] = float(euler.z()*RAD_TO_DEG) ;
  imux.notify32(eularSend[0]);
  imuy.notify32(eularSend[1]);
  imuz.notify32(eularSend[2]);
  imuall.notify(&eularSend,12);
  
  if (DebugMode){
      /* Display the floating point data */
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.println("\t\t");
  }
}
void postIMUCalibrationStat(){
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  imucal.notify16(system*1000 + gyro *100 + accel*10 + mag + 1111);
  if (DebugMode){
      Serial.print("CALIBRATION: Sys=");
      Serial.print(system, DEC);
      Serial.print(" Gyro=");
      Serial.print(gyro, DEC);
      Serial.print(" Accel=");
      Serial.print(accel, DEC);
      Serial.print(" Mag=");
      Serial.println(mag, DEC);
  }
  if(calibrationMode){
    if (bno.isFullyCalibrated()){
      DebugMode = 1;
      adafruit_bno055_offsets_t newCalib;
      bno.getSensorOffsets(newCalib);
      displaySensorOffsets(newCalib);
    }else{
      DebugMode = 0;
    }
  }
}
/* IMU manual calibration*/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");
  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");
  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
  Serial.println();
  postDebugMessage(String(String(calibData.mag_offset_x) + " " + String(calibData.mag_offset_y) + " " +  String(calibData.mag_offset_z) + " " + String(calibData.mag_radius))); 

}

void applySensorOffsets()
{
  //apply offset data
  adafruit_bno055_offsets_t  calibData;
  calibData.accel_offset_x = -27;
  calibData.accel_offset_y = -51;
  calibData.accel_offset_z = 48;

  calibData.gyro_offset_x = 1;
  calibData.gyro_offset_y = 2;
  calibData.gyro_offset_z = 1;

  calibData.mag_offset_x = 486;
  calibData.mag_offset_y = -607;
  calibData.mag_offset_z = -102;

  calibData.accel_radius = 1000;

  calibData.mag_radius = 721;
  bno.setSensorOffsets(calibData);
}

/* temperature report services*/
void setupHTM(void)
{
  htms.begin();
  htmc.setProperties(CHR_PROPS_READ);
  htmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  htmc.setFixedLen(6);
  htmc.begin();
  uint8_t htmdata[6] = { 0b00000100, 0, 0 ,0 ,0, 2 }; // Set the characteristic to use Fahrenheit, with type (body) but no timestamp field
  htmc.write(htmdata, sizeof(htmdata));                    // Use .write for init data
}
void postSensorTemperature(){
  int8_t temp = bno.getTemp();
  double tempf = double(temp);
  uint8_t htmdata[6] = { 0b00000100, 0, 0 ,0 ,0, 2 };
  float2IEEE11073(tempf, &htmdata[1]);
  htmc.write(htmdata, sizeof(htmdata));  
}

/* IMU Debug services*/
void setupIMUDebugService(){
  //==================setup debug==================
  //start debug service
  debugs.begin();
  //set magnotometer debug data
  magall.setProperties(CHR_PROPS_NOTIFY);
  magall.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  magall.setFixedLen(12);
  magall.setUserDescriptor("magall-4byte_Float_XYZ");
  magall.begin();

  //set message based debug data
  debugmsg.setProperties(CHR_PROPS_READ);
  debugmsg.setPresentationFormatDescriptor(0x19,0,0);
  debugmsg.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  debugmsg.setUserDescriptor("debug code in 4 bytes");
  debugmsg.begin();
}

void postIMUDebug(){
  //sending magnotometer data to BLE
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  float magSend[3];
  magSend[0] = float(mag.x());
  magSend[1] = float(mag.y());
  magSend[2] = float(mag.z());
  magall.notify(&magSend,12);
  if (DebugMode){
    /* Display the floating point data */
    Serial.print("MAGX: ");
    Serial.print(mag.x());
    Serial.print(" MAGY: ");
    Serial.print(mag.y());
    Serial.print(" MAGZ: ");
    Serial.print(mag.z());
    Serial.println("\t\t");
  }
}

void postDebugMessage(String msg){
  int msg_len = msg.length() + 1;
  char msgArray[msg_len];
  msg.toCharArray(msgArray,msg_len);
  debugmsg.write(msgArray, msg_len);
}

/* called when a device is connected*/
void connect_callback(uint16_t conn_handle)
{
  Bluefruit.Advertising.start(0);  //restart the advertising for mutiple connection
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  if (DebugMode){
    Serial.print("Connected to ");
    Serial.println(central_name);
    postDebugMessage(String("connected"));
  }
  wakeup();
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  if (DebugMode){
    Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
    Serial.println("Advertising!");
  }
}

void startAdv(void){
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include IMU Service UUID
  Bluefruit.Advertising.addService(imus);

  // Include Name in scan response since no room for name in advertising
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */

  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(32, 1364);    // unit:0.625s
  Bluefruit.Advertising.setFastTimeout(10);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}



/* BLE power related functions*/
void deepSleep(){// shutting down the system for max power saving
  // shutdown when time reaches SLEEPING_DELAY ms
    delay(SLEEPING_DELAY); 
    // to reduce power consumption when sleeping, turn off all your LEDs (and other power hungry devices)
    digitalWrite(LED_BUILTIN, HIGH);                     
    // setup your wake-up pins.
    pinMode(WAKE_LOW_PIN,  INPUT_PULLUP_SENSE);    // this pin (WAKE_LOW_PIN) is pulled up and wakes up the feather when externally connected to ground.
    pinMode(WAKE_HIGH_PIN, INPUT_PULLDOWN_SENSE);  // this pin (WAKE_HIGH_PIN) is pulled down and wakes up the feather when externally connected to 3.3v.
    // power down onboard flash
    flash.begin();
    flashTransport.runCommand(0xB9);
    flash.end();
    // power down nrf52.
    sd_power_system_off();                             // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
}
void goSleep(){
  if (!sleeping){
    Bluefruit.Advertising.stop();  
    if (DebugMode) postDebugMessage(String("waiting for connection"));
    //suspend BNO055 and loop when disconnected
    for (int i = 1; i < 10; i ++ ){
      bno.enterSuspendMode();
      delay(40);
    }
    startAdv();
    sleeping = true;
    if (DebugMode) Serial.println("LOOP stopped");
    delay(1000);
    vTaskDelete(_updateDataHandle);
  }
}
void wakeup(){
  if (sleeping){
    //start and reset BNO055 when connected
    setupIMUsensor();
    //create the main loop task
    if (DebugMode) Serial.println("STARTING LOOP");
    if (DebugMode) postDebugMessage(String("STARTING LOOP"));
    xTaskCreate(updateData, "updateData", UPDATE_STACK_SZ, NULL, TASK_PRIO_LOW, &_updateDataHandle);

    if (DebugMode) Serial.println("LOOP started");
    if (DebugMode) postDebugMessage(String("LOOP started"));
    sleeping = false;
  }
}

//read voltage from battery
const double vRef = 3.3; // Assumes 3.3V regulator output is ADC reference voltage
const unsigned int numReadings = 1024; // 10-bit ADC readings 0-1023, so the factor is 1024
#define MIN_BATT_VOLTAGE 3.3
#define MAX_BATT_VOLTAGE 4.2
#define PIN_BATT_MEASURE_ENABLE 14
#define BAT_HIGH_CHARGE 22  // HIGH for 50mA, LOW for 100mA
void setupBatteryMeasurement(){
  //enable battery measurement
  pinMode(PIN_BATT_MEASURE_ENABLE, OUTPUT);
  digitalWrite(PIN_BATT_MEASURE_ENABLE, LOW);
  //set charging current ot high
  pinMode(BAT_HIGH_CHARGE, OUTPUT);
  digitalWrite(BAT_HIGH_CHARGE, LOW);
  blebas.begin();
}

double readBatteryVoltage(){
  unsigned int adcCount = analogRead(PIN_VBAT);
  double adcVoltage = (adcCount * vRef) / numReadings;
  double vBat = adcVoltage * 3.3338; // Voltage divider from Vbat to ADC
  if (DebugMode) Serial.println(vBat);
  return(vBat);
}

int readBatteryPercentage(){
  int pBat = (readBatteryVoltage() - MIN_BATT_VOLTAGE) / (MAX_BATT_VOLTAGE - MIN_BATT_VOLTAGE) * 100;
  if (pBat > 100) pBat = 100;
  if (pBat < 0) pBat = 0;
  return(pBat);
}

void postBatteryPercentage(){
  int pBat = readBatteryPercentage();
  blebas.write(pBat);
  if (DebugMode){
      Serial.print("BAT: ");
      Serial.println(pBat);
  }
}