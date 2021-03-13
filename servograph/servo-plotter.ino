#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 150 // minimum pulse length for servos (out of 4096)
#define SERVOMAX 600 // maximum pulse length for servos (out of 4096)
#define NUM_SERVOS 20 // total number of servos
#define SERVOS_CONTROLLER 16 // defines the number of servos per controller
#define SERVOLEN 36 // length of the servos in cm

// array to store pwm controllers
Adafruit_PWMServoDriver ctrls[] = {Adafruit_PWMServoDriver(0x40),
                                   Adafruit_PWMServoDriver(0x41)
                                   };

String cmd_pc;
String param;

uint8_t servo_num = 0; // number of servo to issue commands
uint8_t ctrl_num; // number of the controller to use
uint16_t servo_rng_min = SERVOMIN; // defines the lower end of pulse duration for the range to use
uint16_t servo_rng_max = SERVOMAX; // defines the upper end of pulse duration for the range to use
uint16_t zero_val = (SERVOMAX-SERVOMIN)/2; // defines the x-axis of the pulse duration
float fn_min; // minimum value of the function being shown
float fn_max; // maximum value of the function being shown
float x_min = 0; // minimum value for x (x of first servo)
float x_max = float(SERVOLEN)/100.0; // maximum value for x (x of last servo)

float x_val; // x-value used for servo postion
float y_val; // calculated y_value for servo

int offsets[] = 
{
  -11, 16, -6, -15, // 0, 1, 2, 3
  2, 4, 5, -10, // 4, 5, 6, 7
  -12, -30, -5, 0, // 8, 9, 10, 11
  -10, -10, -10, 0, // 12, 13, 14, 15
  20, 5, -5, 0, // 16, 17, 18, 19
};

//float y_val; // calculated y-value to plot
uint16_t pulsedur; // pulse duration for servo to match

void setup() {
  
  Serial.begin(9600);
  
  ctrls[0].begin();
  ctrls[0].setPWMFreq(60);
  yield();
  ctrls[1].begin();
  ctrls[1].setPWMFreq(60);
  yield();
  
  reset();
  //demo();
}

void loop() {
  while(!Serial.available()) {cmd_pc="";}
  while(Serial.available()) {
    cmd_pc = Serial.readString();
    int idx = cmd_pc.indexOf(';');
    if(idx != -1) {
      param = cmd_pc.substring(idx+1);
      cmd_pc = cmd_pc.substring(0,idx);
    }
    if(cmd_pc=="x") { go_to_x_axis(); }
    if(cmd_pc=="y") { go_to_y_axis(); }
    if(cmd_pc=="reset") { reset(); }
    if(cmd_pc=="sin") { sin_fn(); }
    if(cmd_pc=="bell") { guassian_fn(); }
    if(cmd_pc=="atan") { arctan(); }
    if(cmd_pc=="step_fn") { step_fn(); }
    if(cmd_pc=="parabola") { parabola(); }
    if(cmd_pc=="cubic") { cubic(); }
    if(cmd_pc=="ln") { ln(); }
    if(cmd_pc=="cosine") { cosine(); }
    if(cmd_pc=="cant_mode") { cant_mode(param.toInt()); }
    if(cmd_pc=="cant_motion") { cant_motion(param.toInt()); }
    if(cmd_pc=="cant_multi") { cant_multi(); }
    
  }
}

void set_servo_position(uint8_t servo, float y_val) {
  /* This function will set a servo to its required value.
   *  servo is the number of the servo in the physical line. This function will determine 
   *    the correct servo number and controller.
   */
  float tmp;
  
  uint8_t servo_slct = servo%16; // selected servo
  uint8_t controller = servo/16; // controller
  pulsedur = map_floats(y_val, fn_min, fn_max, servo_rng_min, servo_rng_max); // find pulse duration based on function range
  pulsedur += offsets[servo];
  ctrls[controller].setPWM(servo_slct, 0, pulsedur); // set position
  
  //if(servo==0) {Serial.println(pulsedur);}
}

void reset() {
  go_to_y_axis();
  delay(500);
  go_to_x_axis();
}

void demo() {
  int delay_tm = 2000;
  reset();
  delay(delay_tm);
  cosine();
  delay(delay_tm);
  arctan();
  delay(delay_tm);
  guassian_fn();
  delay(delay_tm);
  step_fn();
  delay(delay_tm);
  parabola();
  delay(delay_tm);
  cubic();
  delay(delay_tm);
  ln();
  delay(delay_tm);
  sin_fn();
}

void all_to_position(float y_val) {
  /* Set all the servos to a specified value.
   * y_val with be enforced to be between -1 and 1
   */
  y_val = constrain(y_val, -1, 1);
  fn_min = -1;
  fn_max = 1;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    set_servo_position(servo_cnt, y_val);
  }
}

void set_transverse_limits(int factor) {
  uint16_t y_rng_half = (SERVOMAX-SERVOMIN)/factor;
  zero_val = (SERVOMAX+SERVOMIN)/2;
  servo_rng_min = zero_val - y_rng_half;
  servo_rng_max = zero_val + y_rng_half;
}

void set_longitudinal_limits() {
  
}

void go_to_x_axis() {
  fn_min = -1;
  fn_max = 1;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    //set_servo_position(servo_cnt, 0.75);
    //delay(125);
    set_servo_position(servo_cnt, 0);
    //delay(125);
  }
}

void go_to_y_axis() {
  set_transverse_limits(7);
  fn_min = -1;
  fn_max = 1;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    //set_servo_position(servo_cnt, 0);
    //delay(125);
    set_servo_position(servo_cnt, 0.75);
    //delay(125);
  }
  
}

void sin_fn() {
  /* Plots the sine function.
   */
  set_transverse_limits(6);

  fn_min = -1;
  fn_max = 1;
  
  float omega = 0.002; // frequency [radians/second]
  float k = 15; // wavenumber [radians/meter]

  unsigned long tm_nxt = millis();

  while(true) {
    if(Serial.available()) {
      cmd_pc=Serial.readString();
      if(cmd_pc=="stop") { break; }
    }
    tm_nxt = millis();
    for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
      x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
      y_val = sin(-omega*tm_nxt + k*x_val);
      set_servo_position(servo_cnt, y_val);
    }

  }
}

void cosine() {
  set_transverse_limits(4);

  fn_min = -1;
  fn_max = 1;
  x_min = 0;
  x_max = 6.5;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_val = cos(x_val);
    set_servo_position(servo_cnt, y_val);
  }
}

void guassian_fn() {
  set_transverse_limits(4);
  
  fn_min = -1;
  fn_max = 1;

  x_min = -2.5;
  x_max = 2.5;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_val = exp(-x_val*x_val);
    set_servo_position(servo_cnt, y_val);
  }
}

void arctan() {
  set_transverse_limits(4);
  
  fn_min = -1.5;
  fn_max = 1.5;
  x_min = -2.5;
  x_max = 2.5;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_val = atan(3*x_val);
    set_servo_position(servo_cnt, y_val);
  }
}

void step_fn() {
  set_transverse_limits(4);

  fn_min = -1;
  fn_max = 1;
  x_min = -1;
  x_max = 1;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    if(x_val<0) {y_val=0;}
    else {y_val=1;}
    set_servo_position(servo_cnt, y_val);
  }
}

void parabola() {
  set_transverse_limits(4);

  fn_min = 0;
  fn_max = 1;
  x_min = -1;
  x_max = 1;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_val = pow(x_val, 2);
    set_servo_position(servo_cnt, y_val);
  }
}

void cubic() {
  set_transverse_limits(4);

  //fn_min = -1;
  //fn_max = 1;
  x_min = -2;
  x_max = 2;

  float a = 2;
  float b = 1;
  float c = -5;
  float d = 0;

  float y_vals[NUM_SERVOS];

  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_val = a*pow(x_val, 3) + b*pow(x_val, 2) + c*pow(x_val, 1) + d;
    y_vals[servo_cnt] = y_val;
  }
  
  fn_min = getMin(y_vals, NUM_SERVOS);
  fn_max = getMax(y_vals, NUM_SERVOS);
  
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    //x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    //y_val = pow(x_val, 3);
    //set_servo_position(servo_cnt, y_val);
    set_servo_position(servo_cnt, y_vals[servo_cnt]);
  }
}

void ln() {
  set_transverse_limits(4);

  fn_min = 0;
  fn_max = 2.5;
  x_min = 1;
  x_max = 10;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_val = log(x_val);
    set_servo_position(servo_cnt, y_val);
  }
}

float cant_phi(int mode, float x) {
  float c_n[] = {1.875104, 4.69409, 7.8547574, 10.99554};
  float phi = 0.5*(cos(c_n[mode-1]*x)-cosh(c_n[mode-1]*x))
                + 0.5*(sinh(c_n[mode-1]*x)-sin(c_n[mode-1]*x)) * (cos(c_n[mode-1])+cosh(c_n[mode-1])) / (sin(c_n[mode-1])+sinh(c_n[mode-1]));
  return -phi;
}

void cant_mode(int mode) {
  set_transverse_limits(7);

  fn_min = -1;
  fn_max = 1;
  x_min = 0;
  x_max = 1;
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_val = cant_phi(mode, x_val);
    set_servo_position(servo_cnt, y_val);
  }
}

void cant_motion(int mode) {
  set_transverse_limits(7);

  fn_min = -1;
  fn_max = 1;
  x_min = 0;
  x_max = 1;
  
  float omega = 0.0075; // frequency [radians/second]
  //omega = 0.015;

  float y_amps[NUM_SERVOS];
  
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
    y_amps[servo_cnt] = cant_phi(mode, x_val);
  }
  
  for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
    y_val = y_amps[servo_cnt];
    set_servo_position(servo_cnt, y_val);
  }

  delay(2500);

  while(true) {
    if(Serial.available()) {
      cmd_pc=Serial.readString();
      if(cmd_pc=="stop") { break; }
    }
    for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
      y_val = y_amps[servo_cnt]*sin(omega*millis());
      set_servo_position(servo_cnt, y_val);
    }
  }
}

void cant_multi() {
  set_transverse_limits(7);

  fn_min = -8;
  fn_max = 8;
  x_min = 0;
  x_max = 1;
  float sin_val;
  unsigned long tm;

  float modes[4][NUM_SERVOS];

  uint8_t num_freqs = 10;
  float freqs[num_freqs];
  float phases[4][num_freqs];
  
  for(uint8_t freq=0; freq<num_freqs; freq++) {
    freqs[freq] = map_floats(float(random(0, 2^16)), float(0), float(2^16), 0.01*(1-freq/num_freqs), 0.005*(1-freq/num_freqs));
    for(uint8_t mode=0; mode<4; mode++) {
      phases[mode][freq] = map_floats(float(random(0, 2^16)), float(0), float(2^16), float(0), 6.28);
    }
  }

  for(uint8_t mode=0; mode<4; mode++) {
    for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
      x_val = map_floats(float(servo_cnt), float(0), float(NUM_SERVOS), x_min, x_max);
      modes[mode][servo_cnt] = cant_phi(mode+1, x_val)/float(mode+1);
    }
  }

  while(true) {
    if(Serial.available()) {
      cmd_pc=Serial.readString();
      if(cmd_pc=="stop") { break; }
    }
    tm = millis();
    for(uint8_t servo_cnt=0; servo_cnt<NUM_SERVOS; servo_cnt++) {
      y_val = 0;
      for(uint8_t freq=0; freq<num_freqs; freq++) {
        for(uint8_t mode=0; mode<4; mode++) {
          y_val += sin(float(tm)*freqs[freq]+phases[mode][freq]) * modes[mode][servo_cnt];
        }
      }
      set_servo_position(servo_cnt, y_val);
    }
  }
}

void oscilloscope() {
  // This function is designed to plot oscilloscope results based on analog voltage readings from pin A0.
  // Once running, send the command "stop" to exit the function.
  // This has not been tested yet and may contain errors.
  
  set_transverse_limits(6); // sets angle range to plot over

  fn_max = 1024; // maximum possible analog reading
  fn_min = 0; // minimum possible analog value
  x_min = 0; // default
  x_max = 1; // default

  int volt;
  int results[NUM_SERVOS];
  int meas_delay = 33; // delay between each measurement
  unsigned long tm = millis();

  while(true) {
    if(Serial.available()) {
      cmd_pc = Serial.readString();
      if(cmd_pc=="stop") { break; }
    }
    if(tm>=millis()) {
      volt = analogRead(A0);
      for(int ele=0; ele<NUM_SERVOS-1; ele++) {
        results[ele] = results[ele+1];
        set_servo_position(ele, results[ele]);
      }
      results[NUM_SERVOS-1] = volt;
      set_servo_position(NUM_SERVOS-1, results[NUM_SERVOS-1]);
    }
  }
}

float map_floats(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int getMin(float* array, int sz)
{
  int minimum = array[0];
  for (int i = 0; i < sz; i++)
  {
    if (array[i] < minimum) minimum = array[i];
  }
  return minimum;
}

int getMax(float* array, int sz)
{
  int maximum = array[0];
  for (int i = 0; i < sz; i++)
  {
    if (array[i] > maximum) maximum = array[i];
  }
  return maximum;
}
