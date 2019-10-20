#include <avr/sleep.h>
#include <avr/wdt.h>
//#include <LowPower.h>
#include <SoftwareSerial.h>
SoftwareSerial esp(9, 10);  //RX,TX
#include <dht.h>
dht DHT;

#define comms_rx_pin 0
#define comms_tx_pin 1
#define temp_meas_pin 2
#define w_spd_itr_pin 3
#define w_spd_pwr_pin 6
#define w_dir_pwr_pin 7
#define status_led_pin 8
#define comms_pwr_pin 13

#define light_sensor_meas_pin A0
#define w_dir_meas_pin A1
#define bat_meas_pin A5

#define KMPHPP 2.4    //kmph per pulse of the wind speed meter
#define BOUNCE_GUARD 500   //ignore wind speed pulses shorter than this in u-secs
#define VREF 5.1    ///vref should be = 5v = Vcc


//basic control
bool testing = false;   //set to true for testing to see messages on COM3
bool wind_test = false;   //set to true for testing and show simulated data
bool startup_wifi_flag = false;    //set to true for the first time in a new ssid to save those params to false
int i;    //global counter var, do not use in functions
int reset_cycles = 10;   //rseet the MCU after this many dump cycles

//for wifi
String server_ip = "xxx.xxx.xxx.xxx";   //fill this in with the target ip
int server_port = 1234;       //fill this in with the target port
String post_uri = "/data";    //any extra address data here
String ssid = "xxxxx";   //fill with the target wifi ap ssid
String pwd = "xxxxx";    //fill with the target wifi ap password

//timing vars
//NOTE: one dump cycle = (sleep_reps * 8 + meas_period_s) * meas_reps
#define meas_reps 5  //5   //in reps how many measurement cycles to run for each dump
int meas_period_s = 6;      //in secs how long to activate the wind sensors for every measurement cycle
int sleep_reps = 6;  //6;    //in reps of 8s (7 = 56s), how many 8s reps to sleep the board each measurement cycle, 
int sleep_reps_min = 6;  //6;    //in reps of 8s
int sleep_reps_max = 20;  //20;    //in reps of 8s
int sleep_reps_delta = 100;    //in reps of 8s
int dump_rep_cnt = 0;
int meas_rep_cnt = 0;             //counts how many measurement reps have taken place

//wdt vars
int wdt_timeout_s = meas_period_s*meas_reps + 200;//300;   // set this to the sensor read time and the max acceptable connect time
int wdt_timeout_reps = wdt_timeout_s/8;
volatile int wdt_timeout_rep_cnt = 0;

//sleep vars
bool sleep_flag = false;

String result;
bool trouble_flag = false;      //used to detect bad connectivity and wait it out
bool connected_flag = false;
bool http_ok_flag = false;

//light sensor vars
double light_sensor_thd = 0.5;   //0.5;   //in volts
double light_sensor_v = 0.0;   //in volts

//battery meter vars
double bat_v = 0.0; //in volts
bool low_bat_flag = false;
double low_bat_thd = 3.3;
double high_bat_thd = 3.8;

//temperature and humidity vars, temp in deg.C
double temp = 0.0;
double hum = 0.0;

//vars for use in the wind speed interupt routine
volatile unsigned long t_prev = 0.0;  //time of the previous pulse
volatile int pulse_cnt = 0;   //how many pulses
volatile double w_spd[meas_reps];
double w_spd_mean = 0.0;

//wind direction vars
double w_dir_mean = 0.0;
double w_dir_x[meas_reps];
double w_dir_y[meas_reps];

//These numbers are for a 5v Vref and a 5V Vcc and a 10K pullup resistor
//the problem is that some of these meas values are very close, so mapping from the measured voltage to the azimuth will be tricky
//basically you are going to have to have a window of around 4 either side of the meas as the accept window
//int w_dir_az_range[] = {66,84,93,126,184,244,287,406,461,599,630,702,785,827,886,944};    //nat version
int w_dir_raw_range[] = {66,84,92,127,184,244,287,406,461,600,631,702,786,827,889,946};
int w_dir_az[] = {1125,675,900,1575,1350,2025,1800,225,450,2475,2250,3375,0,2925,3150,2700};


//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//########################################################################################################
//ISR
//these need volatile: w_spd[], pulse_cnt, t_prev
void on_wind_speed_pulse_interupt(){
  //this processes the wind speed pulse from the sensor, the pulses occur once per sensor rotation
  unsigned long t_now = micros();
  //check if we are starting the meas cycle again
  if (t_prev == 0){
    t_prev = t_now;      
  } 
  else{
    //get the duration since last function call
    unsigned long duration = t_now - t_prev;
    //only process if the duration is longer than the bounce protect, needed to filter out sensor switch bounce
    if (duration > BOUNCE_GUARD) {
      t_prev = t_now;   //only update t_prev if we accept the point a legit pulse
      pulse_cnt++;
      w_spd[meas_rep_cnt] += KMPHPP/((double)duration/1000000.0);
    }
  }
}


///*
//###########################################################################
//###########################################################################
//ISR for wdt and for sleep
ISR (WDT_vect) {
  if (!sleep_flag) {
    //this is for the watchdog timer case
    wdt_reset();   //this should reset the curretn timer with all settings, not sure if it resets the settings too
    wdt_disable();
    wdt_timeout_rep_cnt++;
    //Serial.println("Interrupt: " + (String)wdt_timeout_rep_cnt + " of " + (String)wdt_timeout_reps + " Result = " + (String)(wdt_timeout_rep_cnt > wdt_timeout_reps));
    if (wdt_timeout_rep_cnt > wdt_timeout_reps){
      if (testing) Serial.println("TIMEOUT RESTARTING!!!");
      digitalWrite(comms_pwr_pin, LOW);
      wdt_enable(4); //we have timedout, so we need to reset
    }
    else{
      wdt_enable(9);   //set stadnard 8s wdt
      set_int_8s_no_reset();   //modify the 8s wdt
    }
  }
}


void set_int_8s_no_reset() {
  //you need to run this after you have enabled wdt to ensure you disable reset, if you do not, you will reset on timer expiry
  //WDIE = 1: Interrupt Enable
  //WDE = 1 :Reset Enable
  //WDP3 = 0 :For 1000ms Time-out
  //WDP2 = 1 :For 1000ms Time-out
  //WDP1 = 1 :For 1000ms Time-out
  //WDP0 = 0 :For 1000ms Time-out
  //WDCE=4,WDE=3,WDIE=6,WDP3=5,WDP2=2,WDP1=1,WDP0=0
  // same settings for wdt and sleep => 8s, interrupts and no reset
  cli();
  wdt_reset(); 
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);  // Set WDCE and WDE to enable changes.
  WDTCSR = (1<<WDIE) | (0<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
  sei();
}


//sleep stuff
//###################################################################
void sleep_mcu(int reps){
  if (testing) Serial.println("starting sleep");
  int j=0;
  //sleep_settings_activate();
  sleep_flag = true;
  for (j=0;j<reps;j++){
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode.
    cli();
    sleep_enable();
    set_int_8s_no_reset();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    //######################################
    // After waking from watchdog interrupt the code continues to execute from this point.
    //######################################
    sleep_disable();
  }
  sleep_flag = false;
  if (testing) Serial.println("sleep finished");
}
//*/

//###########################################################################
void reset_vars_dump_cycle(){
  int k;
  meas_rep_cnt=0;
  for (k=0;k<meas_reps;k++){
    w_spd[k] = 0.0;
  }
  for (k=0;k<meas_reps;k++){
    w_dir_x[k] = 0.0;
    w_dir_y[k] = 0.0;
  }
  temp = 0.0;
  hum = 0.0;
  bat_v = 0.0;
}


double correct_deg(double deg){
  if (deg < 0){
    return deg + 360.0;   
  }
  if (deg >= 360){
    return deg - 360.0;   
  }
  return deg;
}

double rad2deg(double rad){
  return correct_deg(rad*(180.0/M_PI));  
}

double deg2rad(double deg){
  return M_PI*deg/180.0;  
}


//###########################################################################
void read_wind(){
  if (testing) Serial.print("Doing Wind...");
  double az_deg = -1.0;
  int k = 0;
  
  //start measurements
  //activate wind speed sensor
  pinMode(w_spd_pwr_pin, INPUT_PULLUP);  
  delay(100); 
  attachInterrupt(digitalPinToInterrupt(w_spd_itr_pin), on_wind_speed_pulse_interupt, FALLING);
  pulse_cnt = 0;
  t_prev = 0;

  //turn on the wind direction sensor
  digitalWrite(w_dir_pwr_pin, HIGH);
  delay(500);
  int tmp_wait = 200;    //ms between each successive wind dir meas   
  int tmp_cnt = 0;
  double w_dir_raw = 0;
  w_dir_raw=0.0;
  for(k=0; k<1000*meas_period_s/tmp_wait; k++){
    w_dir_raw += (double)analogRead(w_dir_meas_pin);
    delay(tmp_wait);
    tmp_cnt++;
  }
  //turn off the wind direction sensor
  digitalWrite(w_dir_pwr_pin, LOW);

  //deactivate the wind speed sensor
  detachInterrupt(digitalPinToInterrupt(w_spd_itr_pin));
  pinMode(w_spd_pwr_pin, OUTPUT);   
  digitalWrite(w_spd_pwr_pin, LOW);

  //process the wind speed results
  //###############################################
  if (pulse_cnt == 0) w_spd[meas_rep_cnt] = 0;  //test if wind speed is too low to accurately measure
  else w_spd[meas_rep_cnt] = w_spd[meas_rep_cnt]/(double)pulse_cnt;  //if ok

  //process the wind direction measurements
  //###############################################
  w_dir_raw = w_dir_raw/(double)tmp_cnt;
  
  //this looks for the closest match, if it finds one within 4 points of the nominal, it goes for it straight away,
  //otherwise, it just finds the closest and returns that
  int min_delta=1024;
  int min_index=8;
  for (k=0;k<16;k++){
    int delta = min(abs(w_dir_raw - w_dir_raw_range[k]), abs(w_dir_raw+1023 - w_dir_raw_range[k]));
    if (delta < 4){
       az_deg = (double)w_dir_az[k]/10.0;
       break;
    }
    if (delta < min_delta){
      min_delta = delta;
      min_index = k;
    }
  }
  if (az_deg == -1.0){
    az_deg = (double)w_dir_az[min_index]/10.0;
  }

  //process the w_dir results
  w_dir_x[meas_rep_cnt] = sin(deg2rad(az_deg));
  w_dir_y[meas_rep_cnt] = cos(deg2rad(az_deg));

  if (testing) Serial.println((String)az_deg + ", " + (String)w_spd[meas_rep_cnt]);
}


//###########################################################################
void pre_dump_processing(){
  int k;

  //this does the wind speed
  w_spd_mean=0;
  for (k=0;k<meas_reps;k++){
    w_spd_mean += w_spd[k]; 
  }
  w_spd_mean = w_spd_mean/(double)meas_reps;

  double w_dir_x_mean = 0;
  double w_dir_y_mean = 0;
  w_dir_mean=0;
  //this does the wind dir, mean weighted by w_spd
  for (k=0;k<meas_reps;k++){
    if (w_spd_mean != 0.0) {
      //do an weighted mean
      w_dir_x_mean += w_dir_x[k]*w_spd[k]; 
      w_dir_y_mean += w_dir_y[k]*w_spd[k]; 
    }
    else{
      //do an unweighted mean
      w_dir_x_mean += w_dir_x[k]; 
      w_dir_y_mean += w_dir_y[k]; 
    }
  }
  w_dir_x_mean = w_dir_x_mean/(double)meas_reps; 
  w_dir_y_mean = w_dir_y_mean/(double)meas_reps; 
  w_dir_mean = rad2deg(atan2(w_dir_x_mean, w_dir_y_mean));

  //this does the temp/hum/batt
  temp = temp/(double)meas_reps;
  hum = hum/(double)meas_reps;
  bat_v = bat_v/(double)meas_reps;
}


//###########################################################################
void adjust_sleep_cycle_length(){
  //handles low battery states
  if (bat_v < low_bat_thd) low_bat_flag = true;
  else if (bat_v > (low_bat_thd + 0.2)) low_bat_flag = false;

  // read the value from the sensor, returns int from 0-1023 maps from 0 to vref(5v):
  int light_sensor_meas = analogRead(light_sensor_meas_pin);
  
  if (!trouble_flag & !low_bat_flag){
    //adjust the sleep time based on the abient light, longer for lower light, shorter for more light
    light_sensor_v = (double)VREF*((double)light_sensor_meas + 1.0)/(double)1024;
    if (light_sensor_v <= light_sensor_thd){
      //sleep_reps = min(sleep_reps_max, sleep_reps + sleep_reps_delta);
      sleep_reps = sleep_reps_max;
    }
    if (light_sensor_v > light_sensor_thd){
      //sleep_reps = max(sleep_reps_min, sleep_reps - sleep_reps_delta);
      sleep_reps = sleep_reps_min;
    }  
    if (bat_v < 4.0){
      sleep_reps = sleep_reps*2;
    }
  }
  else {
    if (trouble_flag & !low_bat_flag){
      int mult_factor = 2;
      //we increase the sleep to max quickly
      //sleep_reps = min(sleep_reps_max, sleep_reps + mult_factor*sleep_reps_delta);
      sleep_reps = sleep_reps_max;
      if (bat_v < high_bat_thd){
        sleep_reps = sleep_reps*2;
      }
    }
    else{
      sleep_reps = 3600/8/meas_reps;   //1 hour sleep as low_bat_flag is active
      for (int k=0;k<4;k++){
        digitalWrite(status_led_pin, HIGH);
        delay(1000);
        digitalWrite(status_led_pin, LOW);
        delay(1000);
      }
    }
  }
}


//###########################################################################
void read_bat_lev(){
  int bat_meas = 0;
  bat_meas = analogRead(bat_meas_pin);
  bat_v += (double)VREF*((double)bat_meas+1.0)/1024.0;  
}


void read_temp_hum(){
  //read the raw value
  DHT.read11(temp_meas_pin);
  temp += DHT.temperature;
  hum += DHT.humidity;
}



//##########################################################################
//##########################################################################
//##########################################################################
//WIFI Functions
//##########################################################################
void reset_wifi() {
  if (testing) Serial.println("resetting wifi board");
  esp.println("AT+RST");
  get_result_fast(5000);
}

int setup_wifi_full(){
  //test the cipmux setting
  if (testing) Serial.println("setting up wifi board");
  esp.println("AT+CIPCLOSE");
  get_result_fast(500);
  esp.println("AT+CIPMUX=0");   // 0 for single TCP connection mode, 1 for multi TCP connection mode
  get_result_fast(500);
  esp.println("AT+CIPMODE=0");   // 0 for normal transfer mode
  get_result_fast(500);

  //these commands are stored in memory, so you only need once
  esp.println("AT+CWMODE_DEF=1");   // 1 for wifi client mode
  get_result_fast(500);
  esp.println("AT+CWAUTOCONN=1");   //set to 0 to not auto connect to ap on power on 
  get_result_fast(500);

  //use this to set the baud rate
  /*
  digitalWrite(comms_pwr_pin, HIGH);
  esp.begin(115200);  //this assumes default baud rate is used by the module
  delay(1000);
  esp.println("AT+UART_DEF=9600,8,1,0,0");
  delay(1000);
  esp.end();
  delay(1000);
  esp.begin(9600);  
  delay(3000);
  digitalWrite(comms_pwr_pin, LOW);
  */

  //only need this once to set params, then do not need
  // board will automatically connect to the stored hotspot after this
  connect_wifi();

  //all ok
  return 1;
}

int setup_wifi_fast(){
  //test the cipmux setting
  if (testing) Serial.println("setting up wifi board");
  esp.println("AT+CIPCLOSE");
  get_result_fast(100);
  esp.println("AT+CIPMUX=0");   // 0 for single TCP connection mode, 1 for multi TCP connection mode
  get_result_fast(100);
  esp.println("AT+CIPMODE=0");   // 0 for normal transfer mode
  get_result_fast(100);
  //all ok
  return 1;
}


int connect_wifi() {
  /*list ap signals, for testing
  esp.println("AT+CWLAPOPT=1,22");
  get_result_fast(2000);
  esp.println("AT+CWLAP");
  get_result_fast(5000);
  Serial.println(result);
  
  esp.println("AT+CIPSTATUS");
  get_result_fast(2000);
  if (testing) Serial.println("Getting connection status...");
  if (testing) Serial.println(result);
  if (result.indexOf("STATUS:2")!=-1 | result.indexOf("STATUS:3")!=-1 | result.indexOf("STATUS:4")!=-1 | result.indexOf("WIFI CONNECT")!=-1){
    if (testing) Serial.println("Disconnecting from current AP");
    esp.println("AT+CWQAP");
    get_result_fast(3000);
  }
  */

  if (testing) Serial.println("Disconnecting from current AP");
  esp.println("AT+CWQAP");
  get_result_fast(3000);

  //connect to the desired ssid
  if (testing) Serial.println("Connecting to target AP");
  esp.println("AT+CWJAP_DEF=\"" + ssid + "\",\"" + pwd + "\"");
  //get_result_fast(5000);
  get_result_slow(4000, 3);
  if (result.indexOf("OK")!=-1 | result.indexOf("WIFI CONNECTED")!=-1){
    if (testing) Serial.println("Connect OK");
    if (testing) Serial.println("result: " + result);
    return 1;    //connect ok
  }
    
  //could not connect error
  if (testing) Serial.println("Could Not Connect Error");
  if (testing) Serial.println("result: " + result);
  return 0;
}




int http_post(){
  //testing
  /*esp.println("AT+CWLAPOPT=1,127");
  delay(1000);
  esp.println("AT+CWLAP");
  esp.println("AT+CIFSR");
  get_result_fast(5000);
  Serial.println(result);*/
  
  //set the data vals to send
  //get the RSSI
  String rssi = "-1";
  esp.println("AT+CWJAP?");
  get_result_fast(1000);
  if (result.indexOf(",-")!=-1){
    rssi = result.substring(result.indexOf(",-")+1, min(result.length(),result.indexOf(",-")+5));
    rssi.trim();
  }
  String data_vals = "wspd_mean=" + (String)w_spd_mean + 
              "&wdir_mean=" + (String)w_dir_mean + 
              "&temp=" + (String)temp +
              "&hum=" + (String)hum + 
              "&rep_cnt=" + (String)dump_rep_cnt + 
              "&light_lev=" + (String)light_sensor_v +
              "&batt_lev=" + (String)bat_v + 
              "&rssi=" + rssi;
  //if (testing) Serial.println(data_vals);
  //curl --data "wspd_mean=11.2&wdir_mean=145.2&temp=24.6&hum=0.87&rep_cnt=42&light_lev=3.3&batt_lev=3.7&rssi=-65" http://192.168.20.4:5000/data
  
  //start a TCP connection.
  if (testing) Serial.println("Establish TCP");
  esp.println("AT+CIPSTART=\"TCP\",\"" + server_ip + "\"," + server_port + ",0");  //last number is the TCP keep alive counter in s, set to 0 to disable
  get_result_fast(3000);   //5000;
  if (testing) Serial.println(result);
  if (result.indexOf("ERROR") != -1){   // && result.indexOf("no ip") != -1){
    return 0;
  }
  
  //test that we activated the TCP or it is already activated, then we are good to proceed
  if (result.indexOf("OK")!=-1 || result.indexOf("ALREADY")!=-1){ 
    String cmd1 = "POST " + post_uri + " HTTP/1.1";
    String cmd2 = "Host: " + server_ip + ":" + (String)server_port;
    String cmd3 = "Content-Type: application/x-www-form-urlencoded";
    String cmd4 = "Content-Length: " + (String)data_vals.length();
    String cmd5 = "";

    //need to add the carriage return and new line (2 chars onto each line)
    int len = cmd1.length() + 2 + 
              cmd2.length() + 2 + 
              cmd3.length() + 2 + 
              cmd4.length() + 2 + 
              cmd5.length() + 2 + 
              data_vals.length() + 2;
          
    //cmd = "POST /data HTTP/1.0\r\nHost: 192.168.20.4:5000\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: 115\r\n\r\nwspd_mean=10.00&wspd_sd=4.00&wdir_mean=320.00&wdir_sd=9.00&temp_mean=9.00&temp_sd=7.00&light_lev=1.42&batt_lev=4.44";
    //cmd = "GET /db/11 HTTP/1.1";
    //len = cmd.length()+2;
    
    //determine the number of caracters to be sent.
    if (testing) Serial.println("Sending HTTP Msg");
    esp.print("AT+CIPSEND=");
    esp.println(len);
    //flush rx buffer
    get_result_fast(1000);   //2000;
    //send request
    esp.println(cmd1);
    delay(300);    
    esp.println(cmd2);    
    delay(300);    
    esp.println(cmd3);    
    delay(300);    
    esp.println(cmd4);    
    delay(300);    
    esp.println(cmd5);    
    delay(300);    
    esp.println(data_vals);    
    delay(3000);    
    //close the connection
    esp.println("AT+CIPCLOSE");
    get_result_fast(1000);   //5000;
    if (result.indexOf("ERROR")==-1){       //if (result.indexOf("OK")!=-1 | result.indexOf("Recv")!=-1 | result.indexOf((String)len)!=-1){
      if (testing) Serial.println(result);
      return 1;    //was send ok
    }
  }

  //send not successfull
  return 0;    //bad sending result error
}


//gets the result from the serial rx
String get_result_slow(int del, int cnt){
  int k;
  String res_new;
  
  result = "";
  delay(del);
  if (trouble_flag) delay(15000);
  for (k=0;k<20;k++){
    res_new = esp.readString();
    if (res_new == "") {
      cnt = cnt - 1;
      if (cnt == 0) break;
    }
    result += res_new;
    delay(1000);
  }
}

//gets the result from the serial rx
void get_result_fast(int start){
  result = "";
  delay(start);
  while(esp.available()){
    result = esp.readString();
    delay(500);
  }
}
//##########################################################################
//##########################################################################


//###########################################################################
//###########################################################################
//###########################################################################
//###########################################################################
//###########################################################################
//###########################################################################
//###########################################################################
//###########################################################################
void setup()
{
  analogReference(VREF);

  if (testing) Serial.begin(9600);
  if (testing) Serial.println("###########START############");
  pinMode(status_led_pin, OUTPUT);
  for (i=0;i<10;i++){
    digitalWrite(status_led_pin, HIGH);
    delay(100);
    digitalWrite(status_led_pin, LOW);
    delay(100);
  }

  //temperature setup
  DHT.read11(temp_meas_pin);
  delay(2000); 
  
  //wind sensor setup
  pinMode(w_spd_pwr_pin, OUTPUT);
  digitalWrite(w_spd_pwr_pin, LOW);    //off the sensor initally
  pinMode(w_dir_pwr_pin, OUTPUT);
  digitalWrite(w_dir_pwr_pin, LOW);    //off the sensor initally

  ///*
  //setup wdt
  wdt_enable(9);
  set_int_8s_no_reset();
  wdt_timeout_rep_cnt = 0;    //start the watchdog timer
  //*/
  
  //wifi setup
  pinMode(comms_pwr_pin, OUTPUT);
  digitalWrite(comms_pwr_pin, LOW);
  esp.begin(9600);
  if (startup_wifi_flag){
    digitalWrite(comms_pwr_pin, HIGH);
    delay(1000);
    setup_wifi_full();
    digitalWrite(comms_pwr_pin, LOW);
    delay(1000);
  }
  wdt_timeout_rep_cnt = 0;    //start the watchdog timer
}


//###########################################################################
//###########################################################################
//###########################################################################
//###########################################################################
//###########################################################################
void loop() {
  //periodic reset function
  if (dump_rep_cnt > reset_cycles) wdt_enable(0);

  //test for a low battery situation
  bat_v = 0;
  for(i=0;i<3;i++){
    read_bat_lev();
    delay(200);
  }
  bat_v = bat_v/3.0;
  if (testing) Serial.println(bat_v);
  adjust_sleep_cycle_length(); //set the sleep reps based on the ambient light level and battery level  
  if (!low_bat_flag){
    //we have good battery voltage, so carry on
    reset_vars_dump_cycle();
    if (testing) Serial.println("Starting Measurements");
    for (i=0; i<meas_reps; i++){
      //#####DO MEASUREMENTS###################
      digitalWrite(status_led_pin, HIGH);
      delay(50);
      digitalWrite(status_led_pin, LOW);
  
      if (!wind_test) read_wind();
      else delay(2000);
      read_temp_hum();
      read_bat_lev();
      meas_rep_cnt++;

      /*
      //for the case distributed sleep
      //#####SLEEP THE BOARD##################################
      if (testing) Serial.println("Sleeping the Board");
      wdt_disable();   //disable the wtc function
      delay(300);
      sleep_mcu(sleep_reps);   //go to sleep
      //#############################################3
      //###SLEEPING HERE###  
      //#############################################3
      //prep the board after wakeup
      digitalWrite(comms_pwr_pin, LOW);
      delay(300);
      wdt_enable(9); 
      set_int_8s_no_reset();   
      wdt_timeout_rep_cnt = 0;    //start the watchdog timer
      //###############################################
      */
    }
  
    //######PROCESS AND DUMP###########
    //#############################################3
    //process measurements  
    if (testing) Serial.println("Data Processing");
    pre_dump_processing();
    if (wind_test){
      //just randomly make up some numbers for testing
      w_spd_mean = (double)random(0,30);
      w_dir_mean = (double)random(0,359);
      //temp = (double)random(0,45);
      //hum = (double)random(0,100);
    }
  
    //#############################################3
    //activate the comms and send
    if (testing) Serial.println("Activating Comms");
    digitalWrite(comms_pwr_pin, HIGH);
    delay(2000);  //give it time to connect
    setup_wifi_fast();
    
    //try to connect 3 times, then go into connectivity trouble mode where we wait it out with longer sleep intervals
    connected_flag = false;
    http_ok_flag = false;
    if (!trouble_flag){
      //try to connect to the ap
      /*for (i=0;i<5;i++){
        if (connect_wifi() == 1){
          connected_flag = true;
          break;
        }
      }*/
      connected_flag=true;
      
      if (connected_flag){
        //try to post
        for (i=0;i<5;i++){
          if (http_post() == 1){
            http_ok_flag = true;
            break;
          }
        }
        if (!http_ok_flag){
          //could connect but couldn't send data
          trouble_flag = true;
          if (testing) Serial.println("HTTP fail, Trouble Flag Has Been Activated");
        }
      }
      else{
        //couldn't connect, so give up
        trouble_flag = true;
        if (testing) Serial.println("WIFI Connect Fail, Trouble Flag Has Been Activated");
      }
    }
    else{
      //we are in trouble mode, so try only once and then give up
      if (connect_wifi() == 1){
        if (http_post() == 1){
          trouble_flag = false;
          if (testing) Serial.println("Trouble Flag Has Been Deactivated");
        }
      }
    }
    if (testing) Serial.println("Deactivating Comms");
    digitalWrite(comms_pwr_pin, LOW);
    delay(100);
    //increment the dump rep count to know how many dump reps were done before hte board died and reset itself
    dump_rep_cnt++;
  }   
  /*else{*/
    //for the case of low batt
    //#####SLEEP THE BOARD##################################
    if (testing) Serial.println("Sleeping the board ....");
    wdt_disable();   //disable the wtc function
    delay(300);
    sleep_mcu(sleep_reps*meas_reps);   //go to sleep
    //#############################################3
    //###SLEEPING HERE###  
    //#############################################3
    //prep the board after wakeup
    digitalWrite(status_led_pin, HIGH);
    delay(300);
    digitalWrite(status_led_pin, LOW);
  
    digitalWrite(comms_pwr_pin, LOW);
    delay(500);
    wdt_enable(9); 
    set_int_8s_no_reset();   
    wdt_timeout_rep_cnt = 0;    //start the watchdog timer
    //###############################################
  /*}*/
 }
