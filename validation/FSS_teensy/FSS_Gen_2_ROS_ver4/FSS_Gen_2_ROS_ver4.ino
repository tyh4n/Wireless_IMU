#define HWSERIAL Serial7
#define RBSERIAL Serial6
IntervalTimer sendTimer;

float g = 9.81;

//////////// Sampling Time ////////////
#define Timer_freq   400

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
float Fx_raw, Fy_raw, Fz_raw, Mx_raw, My_raw, Mz_raw;
float Fz_est, Mx_est, My_est;
float conversion = 0.454; // from lbs to kg
float chassis_yaw, chassis_yaw_offset = 0;
float concat_yaw, concat_yaw_old, concat_yaw_offset = 0;
float spin, HRI_yaw, HRI_yaw_offset, HRI_yaw_p, HRI_pitch_offset, HRI_roll_offset, HRI_pitch_p, HRI_roll_p = 0;

//////////// Read from IMU VN100 //////////////
// Function declarations
void read_vn_imu_data(void);
void check_sync_byte(void);
unsigned short calculate_imu_crc(byte data[], unsigned int length);
// Data
union {float f; byte b[4];} vn_yaw;
union {float f; byte b[4];} vn_pitch;
union {float f; byte b[4];} vn_roll;
union {float f; byte b[4];} vn_w_z;
// Checksum
union {unsigned short s; byte b[2];} checksum;
// Parameters
bool imu_sync_detected = false;  // check if the sync byte (0xFA) is detected
byte imu_input[100];  // array to save data send from the IMU

///////roboRIO Feedback/////////
byte incomingByte[100];
union {float f; byte b[4];} px_fb;
union {float f; byte b[4];} py_fb;
union {float f; byte b[4];} p_yaw_fb;
union {float f; byte b[4];} vx_fb;
union {float f; byte b[4];} vy_fb;
union {float f; byte b[4];} v_yaw_fb;
union {float f; byte b[4];} chassis_pitch;
union {float f; byte b[4];} chassis_roll;
union {float f; byte b[4];} chassis_dpitch;
union {float f; byte b[4];} chassis_droll;
void check_start_byte(void);
bool start_byte_detected = false;
void read_roboRIO_data(void);

///////Send to roboRIO/////////
union {float f; byte b[4];} PITCH_HRI;
union {float f; byte b[4];} ROLL_HRI;
union {float f; byte b[4];} FZ;
union {float f; byte b[4];} MX;
union {float f; byte b[4];} MY;

///////LED/////////
int led = LED_BUILTIN;
int counter_led = 0;

///////Reset Buttons////////
int reset_status = 0;
int n_reset = 1000;

//////////// ROS Stuff ////////////
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
ros::NodeHandle  nh;
std_msgs::Float32MultiArray out_msg;
ros::Publisher pub_robo_states("/roboRIO/stateFeedback", &out_msg);
union {float f; byte b[4];} vx_ref;
union {float f; byte b[4];} vy_ref;
union {float f; byte b[4];} v_yaw_ref;
float vx_ref_old, vy_ref_old, v_yaw_ref_old;
void rosCallback(const geometry_msgs::TwistStamped& in_msg);
ros::Subscriber <geometry_msgs::TwistStamped> sub_vel_cmd("/roboRIO/vel_cmd", rosCallback);
float signalCleaning(float v_ref, float *v_ref_old);

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

  RBSERIAL.write (PITCH_HRI.b[3]);
  RBSERIAL.write (PITCH_HRI.b[2]);
  RBSERIAL.write (PITCH_HRI.b[1]);
  RBSERIAL.write (PITCH_HRI.b[0]);

  RBSERIAL.write (ROLL_HRI.b[3]);
  RBSERIAL.write (ROLL_HRI.b[2]);
  RBSERIAL.write (ROLL_HRI.b[1]);
  RBSERIAL.write (ROLL_HRI.b[0]);

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


/////// LPF ////////
template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
LowPass<1> lpFSS_x(1,600,true);
LowPass<1> lpFSS_y(1,600,true);


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
  nh.getHardware()->setBaud(230400);
  nh.initNode();
  out_msg.data_length = 16;
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
    read_vn_imu_data();
  };

  // Check if new roboRIO data is available
  start_byte_detected = false;
  if (RBSERIAL.available() > 4) check_start_byte();
  // If sync byte is detected, read the rest of the data
  if (start_byte_detected){
    read_roboRIO_data();
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
  
  Fz_est = (-7.8490)+ (-0.0621) * Fx_raw + (0.0988) * Fy_raw + (1.0129) * Fz_raw + (0.1265) * Mx_raw + (0.0793) * My_raw + (-0.1873) * Mz_raw;
  Mx_est = (0.7041) + (0.0202) * Fx_raw +  (-0.3526) * Fy_raw + (-0.0014) * Fz_raw + (1.1131) * Mx_raw + (-0.0282) * My_raw + (0.0399) * Mz_raw;
  My_est = (-0.1653) + (0.4350) * Fx_raw + (0.0167) * Fy_raw + (-0.0010) * Fz_raw + (-0.0213) * Mx_raw + (0.9918) * My_raw + (-0.0486) * Mz_raw;
  
  concat_yaw_old = concat_yaw;
  concat_yaw = vn_yaw.f;
  if((concat_yaw_old > 150)&&(concat_yaw < -150)){
    concat_yaw_offset = concat_yaw_offset - 360;
  }else if((concat_yaw > 150)&&(concat_yaw_old < -150)){
    concat_yaw_offset = concat_yaw_offset + 360;
  }
  HRI_yaw = concat_yaw - concat_yaw_offset;
  HRI_yaw_p = HRI_yaw - HRI_yaw_offset;
  HRI_pitch_p = vn_pitch.f - HRI_pitch_offset;
  HRI_roll_p = vn_roll.f - HRI_roll_offset;
  chassis_yaw = p_yaw_fb.f/(-0.01745329222);
  spin = HRI_yaw_p-(chassis_yaw - chassis_yaw_offset);

  // Update the outgoing variables
  noInterrupts();
  //FSS 
  PITCH_HRI.f = HRI_pitch_p;
  ROLL_HRI.f = HRI_roll_p;
  FZ.f = Fz_est;
  MX.f = lpFSS_x.filt(Mx_est);
  MY.f = lpFSS_y.filt(My_est);

//  MX.f = Mx_est; 
//  MY.f = My_est; 
  
//  MX.f = My_est;
//  MY.f = lpFSS_y.filt(My_est);
//  MX.f = My_est;
//  MY.f = vy_ref.f;

  //switch
  sw1 = digitalRead(shorted_reboot_pin);    //pin 5    
  sw2 = digitalRead(zero_loadcell_pin);    // pin6  
  sw3 = digitalRead(zero_imu_pin);         // pin7 ///////////////////////////////////
  sw4 = digitalRead(over_write_pin);       // pin8
  sw.i = sw1*1 + sw2*2 + sw3*4 + sw4*8;
  interrupts();
                          
  //For ROS
  nh.spinOnce();

  if (sw2 == 0 || reset_status == 1){
    loadcell_true_0_offset = 0;
    loadcell_true_1_offset = 0;
    loadcell_true_2_offset = 0;
    loadcell_true_3_offset = 0;
    loadcell_true_4_offset = 0;
    loadcell_true_5_offset = 0;
    HRI_yaw_offset = 0;
    chassis_yaw_offset = 0;
    HRI_pitch_offset = 0;
    HRI_roll_offset = 0;
    int i = 0;
    while (i < n_reset){
      loadcell_true_0_offset += loadcell_true_0;
      loadcell_true_1_offset += loadcell_true_1;
      loadcell_true_2_offset += loadcell_true_2;
      loadcell_true_3_offset += loadcell_true_3;
      loadcell_true_4_offset += loadcell_true_4;
      loadcell_true_5_offset += loadcell_true_5;
      chassis_yaw_offset += chassis_yaw;
      HRI_yaw_offset += HRI_yaw;
      HRI_pitch_offset += vn_pitch.f;
      HRI_roll_offset += vn_roll.f;
      i++;
    }
    loadcell_true_0_offset /= n_reset;
    loadcell_true_1_offset /= n_reset;
    loadcell_true_2_offset /= n_reset;
    loadcell_true_3_offset /= n_reset;
    loadcell_true_4_offset /= n_reset;
    loadcell_true_5_offset /= n_reset;
    chassis_yaw_offset /= n_reset;
    HRI_yaw_offset /= n_reset;
    HRI_pitch_offset /= n_reset;
    HRI_roll_offset /= n_reset;
  }
}

////////////// Helper functions ////////////////
float signalCleaning(float v_ref, float *v_ref_old){
  if(abs(v_ref)>17.5){
    v_ref = *v_ref_old;
  }else{
    *v_ref_old = v_ref;
  }
  return v_ref;
}

// ROS subcriber callback function
void rosCallback(const geometry_msgs::TwistStamped& in_msg){
  noInterrupts();
//  vx_ref.f = in_msg.twist.linear.x; 
//  vy_ref.f = in_msg.twist.linear.y;
//  v_yaw_ref.f = in_msg.twist.angular.z;
  vx_ref.f = signalCleaning(in_msg.twist.linear.x, &vx_ref_old);
  vy_ref.f = signalCleaning(in_msg.twist.linear.y, &vy_ref_old);
  v_yaw_ref.f = signalCleaning(in_msg.twist.angular.z, &v_yaw_ref_old);
  reset_status = in_msg.twist.linear.z>50?1:0;
  
  // Pack out messages for ROS
  out_msg.data[0] = px_fb.f;
  out_msg.data[1] = py_fb.f;
  out_msg.data[2] = p_yaw_fb.f;
  out_msg.data[3] = vx_fb.f;
  out_msg.data[4] = vy_fb.f;
  out_msg.data[5] = v_yaw_fb.f;
  out_msg.data[6] = MX.f;
  out_msg.data[7] = MY.f;
  out_msg.data[8] = HRI_pitch_p;
  out_msg.data[9] = HRI_roll_p;
  out_msg.data[10] = spin;
  out_msg.data[11] = chassis_pitch.f;
  out_msg.data[12] = chassis_roll.f;
  out_msg.data[13] = chassis_dpitch.f;
  out_msg.data[14] = chassis_droll.f;
  out_msg.data[15] = sw.i;
  interrupts();
  pub_robo_states.publish(&out_msg);
  
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
  int len = RBSERIAL.readBytes(incomingByte, 41);
  if ((len == 41)&&(incomingByte[40] == 0xFF)){
    for (int i = 3; i >-1; i--) {
      px_fb.b[i] = incomingByte[3 - i];
      py_fb.b[i] = incomingByte[7 - i];
      p_yaw_fb.b[i] = incomingByte[11 - i];
      vx_fb.b[i] = incomingByte[15 - i];
      vy_fb.b[i] = incomingByte[19 - i];
      v_yaw_fb.b[i] = incomingByte[23 - i];
      chassis_pitch.b[i] = incomingByte[27 - i];
      chassis_roll.b[i] = incomingByte[31 - i];
      chassis_dpitch.b[i] = incomingByte[35 - i];
      chassis_droll.b[i] = incomingByte[39 - i];
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
      vn_pitch.b[i] = imu_input[7 + i];
      vn_roll.b[i] = imu_input[11 + i];
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
