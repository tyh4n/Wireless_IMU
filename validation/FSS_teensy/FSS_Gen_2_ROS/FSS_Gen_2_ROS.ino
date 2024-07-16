#define HWSERIAL Serial7 //5 7
#define RBSERIAL Serial6 //4 6
IntervalTimer sendTimer;

float g = 9.81;

//////////// Sampling Time ////////////
#define Timer_freq   300

//////////onboard switch button///////////
int shorted_reboot_pin = 5;
int zero_loadcell_pin = 6;
int zero_imu_pin = 7;
int over_write_pin = 8;
int sw1,sw2,sw3,sw4;
union {int i; byte b[2];} sw;

//////////// FSS computations ////////////
float loadcell_raw_0, loadcell_raw_1, loadcell_raw_2, loadcell_raw_3, loadcell_raw_4, loadcell_raw_5 = 0;
float loadcell_true_0_offset, loadcell_true_1_offset, loadcell_true_2_offset, loadcell_true_3_offset, loadcell_true_4_offset, loadcell_true_5_offset = 0;
float loadcell_true_0, loadcell_true_1, loadcell_true_2, loadcell_true_3, loadcell_true_4, loadcell_true_5 = 0;
float loadcell_true_0_p, loadcell_true_1_p, loadcell_true_2_p, loadcell_true_3_p, loadcell_true_4_p, loadcell_true_5_p = 0;
float Fx_raw, Fy_raw, Fz_raw, Mx_raw, My_raw, Mz_raw, COPx_raw, COPy_raw = 0;
float Fx_est, Fy_est, Fz_est, Mx_est, My_est, Mz_est, COPx_est, COPy_est = 0;
float conversion = 0.454; // from lbs to kg
float z_off_fss = -0.3601;
float Fz_thresh = -30;

//////////// Read from IMU VN100 //////////////
// Function declarations
void read_vn_imu_data(void);
void check_sync_byte(void);
unsigned short calculate_imu_crc(byte data[], unsigned int length);
// Attitude data
union {float f; byte b[4];} vn_yaw;
//union {float f; byte b[4];} vn_pitch;
//union {float f; byte b[4];} vn_roll;
// Angular rates
//union {float f; byte b[4];} vn_w_x;
//union {float f; byte b[4];} vn_w_y;
union {float f; byte b[4];} vn_w_z;
// Checksum
union {unsigned short s; byte b[2];} checksum;
// Parameters
bool imu_sync_detected = false;  // check if the sync byte (0xFA) is detected
byte imu_input[100];  // array to save data send from the IMU
float* ypr;

///////roboRIO Feedback/////////
byte incomingByte[50];
union {float f; byte b[4];} px_fb;
union {float f; byte b[4];} py_fb;
union {float f; byte b[4];} p_yaw_fb;
union {float f; byte b[4];} vx_fb;
union {float f; byte b[4];} vy_fb;
union {float f; byte b[4];} v_yaw_fb;
void check_start_byte(void);
bool start_byte_detected = false;
void read_roboRIO_data(void);

///////Send to roboRIO/////////
union {float f; byte b[4];} COPX;
union {float f; byte b[4];} COPY;
union {float f; byte b[4];} FZ;
union {float f; byte b[4];} MX;
union {float f; byte b[4];} MY;

///////LED/////////
int led = LED_BUILTIN;
int counter_led = 0;

///////Reset Buttons////////
int reset_status = 1;
int n_reset = 1000;

//////////// ROS Stuff ////////////
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
ros::NodeHandle  nh;
std_msgs::Float32MultiArray out_msg;
ros::Publisher pub_robo_states("/roboRIO/stateFeedback", &out_msg);
union {float f; byte b[4];} vx_ref;
union {float f; byte b[4];} vy_ref;
union {float f; byte b[4];} v_yaw_ref;
void rosCallback(const geometry_msgs::Twist& in_msg);
ros::Subscriber <geometry_msgs::Twist> sub_vel_cmd("/roboRIO/vel_cmd", rosCallback);

///////Interrupt function////////
void sendMessage(){ // sending a total of 44 bytes
  RBSERIAL.write(0xFA);
 
  RBSERIAL.write (vn_yaw.b[3]);
  RBSERIAL.write (vn_yaw.b[2]);
  RBSERIAL.write (vn_yaw.b[1]);
  RBSERIAL.write (vn_yaw.b[0]); 

  RBSERIAL.write (vn_w_z.b[3]);
  RBSERIAL.write (vn_w_z.b[2]);
  RBSERIAL.write (vn_w_z.b[1]);
  RBSERIAL.write (vn_w_z.b[0]);

  RBSERIAL.write (COPX.b[3]);
  RBSERIAL.write (COPX.b[2]);
  RBSERIAL.write (COPX.b[1]);
  RBSERIAL.write (COPX.b[0]);

  RBSERIAL.write (COPY.b[3]);
  RBSERIAL.write (COPY.b[2]);
  RBSERIAL.write (COPY.b[1]);
  RBSERIAL.write (COPY.b[0]);

  RBSERIAL.write (FZ.b[3]);
  RBSERIAL.write (FZ.b[2]);
  RBSERIAL.write (FZ.b[1]);
  RBSERIAL.write (FZ.b[0]);

  RBSERIAL.write (MX.b[3]);
  RBSERIAL.write (MX.b[2]);
  RBSERIAL.write (MX.b[1]);
  RBSERIAL.write (MX.b[0]);

  RBSERIAL.write (MY.b[3]);
  RBSERIAL.write (MY.b[2]);
  RBSERIAL.write (MY.b[1]);
  RBSERIAL.write (MY.b[0]);

  RBSERIAL.write (vx_ref.b[3]);
  RBSERIAL.write (vx_ref.b[2]);
  RBSERIAL.write (vx_ref.b[1]);
  RBSERIAL.write (vx_ref.b[0]);

  RBSERIAL.write (vy_ref.b[3]);
  RBSERIAL.write (vy_ref.b[2]);
  RBSERIAL.write (vy_ref.b[1]);
  RBSERIAL.write (vy_ref.b[0]);

  RBSERIAL.write (v_yaw_ref.b[3]);
  RBSERIAL.write (v_yaw_ref.b[2]);
  RBSERIAL.write (v_yaw_ref.b[1]);
  RBSERIAL.write (v_yaw_ref.b[0]);

  RBSERIAL.write (sw.b[1]);
  RBSERIAL.write (sw.b[0]);

  RBSERIAL.write(0xFF);

  counter_led++;
  if (counter_led >= 50){ // Blink LED at 1/100 the communication rate
  digitalWrite(led, !digitalRead(led)); 
  counter_led = 0;}
}

void setup() {
  HWSERIAL.begin(115200); // Start Serial1 for IMU communication
  RBSERIAL.begin(230400); // Start Serial1 for roboRIO communication
  pinMode(led,OUTPUT);
  pinMode(shorted_reboot_pin, INPUT_PULLUP); //pin5
  pinMode(zero_loadcell_pin, INPUT_PULLUP); //pin6
  pinMode(zero_imu_pin, INPUT_PULLUP); //pin7
  pinMode(over_write_pin, INPUT_PULLUP); //pin8
  
  sendTimer.begin(sendMessage, int((1.0/Timer_freq)*1000000));

  //Serial.begin(115200);
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  out_msg.data_length = 11;
  out_msg.data = (float *)malloc((sizeof(float))*out_msg.data_length * 2);  
  nh.advertise(pub_robo_states);
  nh.subscribe(sub_vel_cmd);
  delay(1000);
}

void loop() {
  // Check if new IMU data is available
  imu_sync_detected = false;
  if (HWSERIAL.available() > 4) check_sync_byte();
  // If sync byte is detected, read the rest of the data
  if (imu_sync_detected){
    noInterrupts();
    read_vn_imu_data();
    interrupts();
  };

  // Check if new roboRIO data is available
  start_byte_detected = false;
  if (RBSERIAL.available() > 0) check_start_byte();
  // If sync byte is detected, read the rest of the data
  if (start_byte_detected){
    noInterrupts();
    read_roboRIO_data();
    interrupts();
  }
  
  loadcell_raw_0 = analogRead(A9);
  loadcell_raw_1 = analogRead(A3);
  loadcell_raw_2 = analogRead(A2);
  loadcell_raw_3 = analogRead(A6);
  loadcell_raw_4 = analogRead(A7);
  loadcell_raw_5 = analogRead(A8);

  loadcell_true_0 = (-0.4641 * (loadcell_raw_0) * 0.66 + 154.38) * conversion * g; // in (kg)  149.93+5.9-1.45
  loadcell_true_1 = (-0.4667 * (loadcell_raw_1) * 0.66 + 156.78) * conversion * g; // 151.55+5.9-0.67
  loadcell_true_2 = (-0.4653 * (loadcell_raw_2) * 0.66 + 157.42) * conversion * g; //151.52 + 5.9
  loadcell_true_3 = (-0.4674 * (loadcell_raw_3) * 0.66 + 158.14) * conversion * g; //152.24 + 5.9
  loadcell_true_4 = (-0.4662 * (loadcell_raw_4) * 0.66 + 157.51) * conversion * g; //151.61 +5.9
  loadcell_true_5 = (-0.4660 * (loadcell_raw_5) * 0.66 + 158.00) * conversion * g; //152.1 + 5.9

  loadcell_true_0_p = loadcell_true_0 - loadcell_true_0_offset;
  loadcell_true_1_p = loadcell_true_1 - loadcell_true_1_offset;
  loadcell_true_2_p = loadcell_true_2 - loadcell_true_2_offset;
  loadcell_true_3_p = loadcell_true_3 - loadcell_true_3_offset;
  loadcell_true_4_p = loadcell_true_4 - loadcell_true_4_offset;
  loadcell_true_5_p = loadcell_true_5 - loadcell_true_5_offset;
      
  Fx_raw = 0.3389*loadcell_true_2_p - 0.3389*loadcell_true_3_p - 0.3389*loadcell_true_4_p + 0.3389*loadcell_true_5_p;
  Fy_raw = 0.3914*loadcell_true_0_p - 0.3914*loadcell_true_1_p - 0.1957*loadcell_true_2_p + 0.1957*loadcell_true_3_p - 0.1957*loadcell_true_4_p + 0.1957*loadcell_true_5_p;
  Fz_raw = -1 * (-0.9202*loadcell_true_0_p - 0.9202*loadcell_true_1_p - 0.9202*loadcell_true_2_p - 0.9202*loadcell_true_3_p - 0.9202*loadcell_true_4_p - 0.9202*loadcell_true_5_p);
  Mx_raw = 0.05754*loadcell_true_1_p - 0.05754*loadcell_true_0_p + 0.1383*loadcell_true_2_p + 0.0808*loadcell_true_3_p - 0.0808*loadcell_true_4_p - 0.1383*loadcell_true_5_p;
  My_raw = 0.1265*loadcell_true_0_p + 0.1265*loadcell_true_1_p - 0.01343*loadcell_true_2_p - 0.1131*loadcell_true_3_p - 0.1131*loadcell_true_4_p - 0.01343*loadcell_true_5_p;
  Mz_raw = -1 * (0.05381*loadcell_true_0_p - 0.05381*loadcell_true_1_p + 0.05381*loadcell_true_2_p - 0.05381*loadcell_true_3_p + 0.05381*loadcell_true_4_p - 0.05381*loadcell_true_5_p);
  COPx_raw = (My_raw + z_off_fss * Fx_raw) / Fz_raw * -1;
  COPy_raw = (Mx_raw + z_off_fss * Fy_raw)  / Fz_raw;
      
  Fx_est = (-0.8507) + (1.1525) * Fx_raw + (0.0177) * Fy_raw + (0.0113) * Fz_raw + (-0.0359) * Mx_raw + (0.0395) * My_raw + (0.0497) * Mz_raw;
  Fy_est = (-1.8632)  + (-0.0459) * Fx_raw + (0.9165) * Fy_raw + (-0.0229)* Fz_raw + (-0.3864) * Mx_raw + (0.0916) * My_raw + (0.0036) * Mz_raw;
  Fz_est = (-7.8490)+ (-0.0621) * Fx_raw + (0.0988) * Fy_raw + (1.0129) * Fz_raw + (0.1265) * Mx_raw + (0.0793) * My_raw + (-0.1873) * Mz_raw;
  
  Mx_est = (0.7041) + (0.0202) * Fx_raw +  (-0.3526) * Fy_raw + (-0.0014) * Fz_raw + (1.1131) * Mx_raw + (-0.0282) * My_raw + (0.0399) * Mz_raw;
  My_est = (-0.1653) + (0.4350) * Fx_raw + (0.0167) * Fy_raw + (-0.0010) * Fz_raw + (-0.0213) * Mx_raw + (0.9918) * My_raw + (-0.0486) * Mz_raw;
  Mz_est = (0.2298) + (0.0160) * Fx_raw + (0.0462) * Fy_raw + (-0.0046) * Fz_raw + (-0.1457) * Mx_raw + (0.0361) * My_raw + (0.9815) * Mz_raw;

  if (Fz_est <= Fz_thresh){
    COPx_est = (My_est + z_off_fss * Fx_est) / Fz_est * 1000.000; // in mm
    COPy_est = (Mx_est + z_off_fss * Fy_est)  / Fz_est * 1000.000; // in mm
  }
  else{
    COPx_est = 0.0;
    COPy_est = 0.0;
  }
  
  // Update the outgoing variables
  noInterrupts();
  //FSS 
  COPX.f = COPx_est;
  COPY.f = COPy_est;
  FZ.f = Fz_est;
  MX.f = Mx_est; 
  MY.f = My_est; 
  //switch
  sw1 = digitalRead(shorted_reboot_pin);   //pin 5    
  sw2 = digitalRead(zero_loadcell_pin);    // pin6  
  sw3 = digitalRead(zero_imu_pin);         // pin7
  sw4 = digitalRead(over_write_pin);       // pin8
  sw.i = sw1*1 + sw2*2 + sw3*4 + sw4*8;                           
  //For ROS
  out_msg.data[0] = px_fb.f;
  out_msg.data[1] = py_fb.f;
  out_msg.data[2] = p_yaw_fb.f;
  out_msg.data[3] = vx_fb.f;
  out_msg.data[4] = vy_fb.f;
  out_msg.data[5] = v_yaw_fb.f;
  out_msg.data[6] = MX.f;
  out_msg.data[7] = MY.f;
  out_msg.data[8] = vn_yaw.f;
  out_msg.data[9] = vn_w_z.f;
  out_msg.data[10] = sw.i;
  interrupts();
  pub_robo_states.publish(&out_msg);
  nh.spinOnce();
  delay(2); // Wait for ROS spinOnce to finish
  
  //reset_status = digitalRead(zero_loadcell_pin);
  reset_status = sw2;
  if (reset_status == 0){
    loadcell_true_0_offset = 0;
    loadcell_true_1_offset = 0;
    loadcell_true_2_offset = 0;
    loadcell_true_3_offset = 0;
    loadcell_true_4_offset = 0;
    loadcell_true_5_offset = 0;
    int i = 0;
    while (i < n_reset){
      loadcell_true_0_offset += loadcell_true_0;
      loadcell_true_1_offset += loadcell_true_1;
      loadcell_true_2_offset += loadcell_true_2;
      loadcell_true_3_offset += loadcell_true_3;
      loadcell_true_4_offset += loadcell_true_4;
      loadcell_true_5_offset += loadcell_true_5;
      i++;
    }
    loadcell_true_0_offset /= n_reset;
    loadcell_true_1_offset /= n_reset;
    loadcell_true_2_offset /= n_reset;
    loadcell_true_3_offset /= n_reset;
    loadcell_true_4_offset /= n_reset;
    loadcell_true_5_offset /= n_reset;
  }
}

////////////// Helper functions ////////////////
// ROS subcriber callback function
void rosCallback(const geometry_msgs::Twist& in_msg){
  noInterrupts();
  vx_ref.f = in_msg.linear.x;
  vy_ref.f = in_msg.linear.y;
  v_yaw_ref.f = in_msg.angular.z;
  interrupts();
}

// Check the start bytes from roboRIO
void check_start_byte(void){
  for (int i = 0; i < 6; i++) {
    RBSERIAL.readBytes(incomingByte, 2);
    if ((incomingByte[0] == 0xFA)&&(incomingByte[1] == 0xFE)) {
      start_byte_detected = true;
      break;
    }
  }
}
// Read the msg from roboRIO
void read_roboRIO_data(void){
  int len = RBSERIAL.readBytes(incomingByte, 25);
  if ((len == 25)&&(incomingByte[24] == 0xFF)){
    for (int i = 3; i >-1; i--) {
      px_fb.b[i] = incomingByte[3 - i];
      py_fb.b[i] = incomingByte[7 - i];
      p_yaw_fb.b[i] = incomingByte[11 - i];
      vx_fb.b[i] = incomingByte[15 - i];
      vy_fb.b[i] = incomingByte[19 - i];
      v_yaw_fb.b[i] = incomingByte[23 - i];
    }
  }
}

// Check for the sync byte (0xFA)
void check_sync_byte(void){
  for (int i = 0; i < 6; i++) {
    HWSERIAL.readBytes(imu_input, 1);
    if (imu_input[0] == 0xFA) {
      imu_sync_detected = true;
      break;
    }
  }
}

// Read the IMU bytes
void read_vn_imu_data(void){
  int n = 41;
  HWSERIAL.readBytes(imu_input, n);
  checksum.b[0] = imu_input[n-1];
  checksum.b[1] = imu_input[n-2];
  if (calculate_imu_crc(imu_input, n-2) == checksum.s) {
    for (int i = 0; i < 4; i++) {
      vn_yaw.b[i] = imu_input[3 + i];
//      vn_pitch.b[i] = imu_input[7 + i];
//      vn_roll.b[i] = imu_input[11 + i];
//      vn_w_x.b[i] = imu_input[15 + i];
//      vn_w_y.b[i] = imu_input[19 + i];
      vn_w_z.b[i] = imu_input[23 + i];
    }
  }
}

// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short calculate_imu_crc(byte data[], unsigned int length){
  unsigned int i;
  unsigned short crc = 0;
  for(i=0; i<length; i++){
    crc = (byte)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (byte)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}
