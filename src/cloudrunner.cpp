#include "CloudRunner.h"
#include "constants.h"
//#include <ppltasks.h>
//======================BOARD INITIALIZING FUNCTIONS======================
//-------------------------Class constructor-----------------------------
CloudRunner::CloudRunner(){
    return;
}
//-------------------------Initialize Motor output-----------------------------
//-> this function initializes all the motor related pins
void CloudRunner::begin(){
    pinMode(L_SPEED_PIN,OUTPUT);
    pinMode(R_SPEED_PIN,OUTPUT);
    pinMode(L_FORWARD,OUTPUT);
    pinMode(L_BACKWARD,OUTPUT);
    pinMode(R_FORWARD,OUTPUT);
    pinMode(R_BACKWARD,OUTPUT);

    // set the number of sensors operating for PID value
    if(SENSOR_NUM ==5)  pin = fivepin;
    if(SENSOR_NUM ==3)  pin = threepin;
}

//======================SENSOR UTILITY FUNCTIONS======================

// reads all sensors, and stores their raw values to array raw_vals[]
void CloudRunner::read_all_sensors(){
    // Serial.print("R: ");
    for(uint8_t i = 0; i<SENSOR_NUM; i++){
        raw_vals[i] = read_sensor(pin[i]);
        // Serial.print(raw_vals[i]);
        // Serial.print(" ");
    }
    // Serial.println();
}

// normalizes all sensors by subtracting baseline_vals[] from raw_vals[], and stores their value
// in array norm_vals[]
void CloudRunner::norm_sensors(){
    for(uint8_t i = 0; i<SENSOR_NUM; i++){
        norm_vals[i]=raw_vals[i]-baseline_vals[i];
    }
}

// initializes baseline values. the sensors shall be placed on a white surface when this is executed.
void CloudRunner::init_baselines(){
    for(uint8_t i=0; i<SENSOR_NUM; i++){
        baseline_vals[i]=read_sensor(pin[i]);
    }
}

// update baseline with an IIR filter, rejects small and high-frequency variations
// inspiration: see section 5.3.2.3.1 of
// https://www.infineon.com/assets/row/public/documents/30/42/infineon-an85951-psoc-4-psoc-6-capsense-design-guide-applicationnotes-en.pdf
// adjust thresholds in constants.h
void CloudRunner::update_baselines(){
    for(uint8_t i=0; i<SENSOR_NUM; i++){
        uint16_t current_val = raw_vals[i];
        if(current_val > baseline_vals[i] + NOISE_THRESHOLD){
            continue;
        }
        if((current_val < (baseline_vals[i] - NEG_NOISE_THRESHOLD))&&low_baseline_count[i] < LOW_BASELINE_RESET){
            low_baseline_count[i]++;
            continue;
        }
        baseline_vals[i]=(current_val+(SENSOR_BASELINE_FACTOR-1)*baseline_vals[i])/SENSOR_BASELINE_FACTOR;
        low_baseline_count[i] = 0;
    }
}

//set black surface thresholds
void CloudRunner::init_black_thresholds(){
    uint16_t start = millis();
    while (millis() - start < 5000) {
        drive_motor(TURN_RIGHT);
        read_all_sensors();
        //update_baselines();
        norm_sensors();
        
        for(uint8_t i = 0; i < SENSOR_NUM; i++){
            uint16_t val = constrain(norm_vals[i],0, MAX_LIMIT_SENSE);
            if(val > black_thresh_vals[i]+BLACK_THRESHOLD_MARGIN) {
                black_thresh_vals[i] = val-BLACK_THRESHOLD_MARGIN;
            }
        }
    }
    drive_motor(STOP_MOTORS);
}

void CloudRunner::print_black_thresholds(){
    Serial.print("Black Thresholds:");
    for (int i = 0; i < SENSOR_NUM; i++) {
        Serial.print(String(black_thresh_vals[i]) + " ");
    }
}
//-------------------------Sensor Reading code-----------------------------
//This code works for sensor circuit based on 3pi robot by Pololu
//For circuit explanation Refer to : https://www.youtube.com/watch?v=9XjSJV5MPc0&t=543s
int CloudRunner::read_sensor(int p_sensor_pin){
    pinMode(p_sensor_pin,OUTPUT);
    digitalWrite(p_sensor_pin,HIGH);

    unsigned long start = micros();

    delayMicroseconds(10);
    pinMode(p_sensor_pin,INPUT);

    //Wait for the voltage to fall bellow threshold of HIGH, if this takes too long
    //then output maximum value
    while(digitalRead(p_sensor_pin) == HIGH && micros() - start < MAX_LIMIT_SENSE);
    int val = micros() - start;

    return val;
}

void CloudRunner::test_read_sensor(){
    int pin2 = read_sensor(2);
    int pin4 = read_sensor(4);
    int pin5 = read_sensor(5);
    int pin7 = read_sensor(7);
    int pin8 = read_sensor(8);

    //Printing Raw Sensor Values
    Serial.println("Raw values:");
    Serial.print(pin2);
    Serial.print(", ");
    Serial.print(pin4);
    Serial.print(", ");
    Serial.print(pin5);
    Serial.print(", ");
    Serial.print(pin7);
    Serial.print(", ");
    Serial.print(pin8);
    Serial.println();
}

void CloudRunner::print_raw(){
    Serial.print("Raw values: ");
    for(uint8_t i = 0; i<SENSOR_NUM; i++){
    Serial.print(raw_vals[i]);
    Serial.print(", ");
    }
    Serial.println();
}

void CloudRunner::print_baselines(){
    Serial.print("Baselines: ");
    for(uint8_t i = 0; i<SENSOR_NUM; i++){
    Serial.print(baseline_vals[i]);
    Serial.print(" ");
    }
    Serial.println();
}

void CloudRunner::print_norm_vals(){
    Serial.print("Normalized: ");
    for(uint8_t i = 0; i<SENSOR_NUM; i++){
    Serial.print(norm_vals[i]);
    Serial.print(", ");
    }
    Serial.println();
}

void CloudRunner::calc_pos(){
    //Zero out existing Line Position Variables
    mass=0;
    torque=0;
    pos=0;

    //Calculate Position of Line using Centroid method by Kirk Charles
    //For explanation refer to: https://www.youtube.com/watch?v=RFYB0wO9ZSQ&t=1217s
    for(int i=0; i < SENSOR_NUM; i++){
    if(norm_vals[i] < 50 && norm_vals[i] >-50){
        continue;
    }
    mass += norm_vals[i];
    torque += (norm_vals[i] * (i-SENSOR_NUM/2)); //Torque = force * lever arm
    }

    //Note this multiplier may need changing based on the board & surface's behavior
    if(mass == 0){
    mass = 1;
    }
    pos = (torque * torque_multiplier)/ mass;

}

int16_t CloudRunner::get_pos_new(){
    return (int16_t) pos;
}

//------------------Calibration function for PID sensors----------------------
//-->Records lowest and highest values of sensor values during initiation phase (only for PID sensors)
// Note: This sets the lowest and highest values for each sensor, since they are not uniform
void CloudRunner::calibrate_PID_sensors(){
    for (int i = 0; i < SENSOR_NUM; i++) {
    LB_vals[i] = 1000;
    UB_vals[i] = 0;
    }

    uint16_t start = millis();

    while (millis() - start < 5000) {
    drive_motor(TURN_RIGHT);
    for(int i=0, raw_val=0; i < SENSOR_NUM;i++){
        raw_val = read_sensor(pin[i])/32;   //read sensors for raw data

        //Check if raw value is less than Lower bound for this sensor, set new lower bound if yes
        if(LB_vals[i] > raw_val) LB_vals[i] = raw_val;
        //Check if raw value is more than Upper bound for this sensor, set new upper bound if yes
        if(UB_vals[i] < raw_val) UB_vals[i] = raw_val;
    }
    }
    drive_motor(STOP_MOTORS);
    Serial.print("Lower bound: ");
    for (int i = 0; i < SENSOR_NUM; i++) {
        Serial.print(String(LB_vals[i]) + " ");
    }
    Serial.print("Upper bound: ");
    for (int i = 0; i < SENSOR_NUM; i++) {
        Serial.print(String(UB_vals[i]) + " ");
    }
}



//--------------Calibration function for turn sensors-----------------------
//-> separate calibration function for turn sensors
//->this function sets new thresh values as calibration is underway
void CloudRunner::calibrate_turn_sensors(){

    R_onblk_thresh = (LB_vals[0]+ UB_vals[0])/2;
    L_onblk_thresh = (LB_vals[SENSOR_NUM -1]+ UB_vals[SENSOR_NUM - 1])/2;
    Serial.println("Rth: " + String(R_onblk_thresh) + " Lth: " + String(L_onblk_thresh));

}


//----------------Function to Get Line Position----------------------
//-->Determines line position after sensor is read
int CloudRunner::get_pos() {
    int raw[SENSOR_NUM];

    for(int i=0; i < SENSOR_NUM;i++){
    raw[i] = read_sensor(pin[i])/32;   //read sensors for raw data

    }
    //Zero out existing Line Position Variables
    mass=0;
    torque=0;
    pos=0;

    //Calculate Position of Line using Centroid method by Kirk Charles
    //For explanation refer to: https://www.youtube.com/watch?v=RFYB0wO9ZSQ&t=1217s
    for(int i=0; i < SENSOR_NUM; i++){
    mass += raw[i];
    torque += (raw[i] * i); //Torque = force + lever arm
    }

    //Note this multiplier may need changing based on the board & surface's behavior

    pos = (torque * torque_multiplier / mass);


    delay(1);

    return pos;
}


//----------------Function to Get Line Position----------------------
//-->Determines line position after sensor is read using the normalized values
//Note: this assumes you have called the calibrate_sensors() method
int CloudRunner::get_norm_pos() {
    int raw[SENSOR_NUM]= {0};   //contains raw values
    int norm_vals[SENSOR_NUM]={0}; //contains normalized values

    for(int i=0; i < SENSOR_NUM;i++){
    raw[i] = read_sensor(pin[i])/32;   //read sensors for raw data

    }

    //Take raw values from sensors and normalize them given the
    //recorded Lower and Upper bounds for each sensor
    //(from most recent call of calibrate_sensors())
    for(int i=0; i<SENSOR_NUM; i++){
    //Normalize to a 0-100 range of value
    norm_vals[i] = map(raw[i], LB_vals[i], UB_vals[i], 0, 100);
    }

    //Zero out existing Line Position Variables
    mass=0;
    torque=0;
    pos=0;

    //Calculate Position of Line using Centroid method by Kirk Charles
    //For explanation refer to: https://www.youtube.com/watch?v=RFYB0wO9ZSQ&t=1217s
    for(int i=0; i < SENSOR_NUM; i++){
    mass += norm_vals[i];
    torque += (norm_vals[i] * i); //Torque = force + lever arm
    }

    //Note this multiplier may need changing based on the board & surface's behavior
    pos = (torque * torque_multiplier)/ mass;
    delay(1);
    return pos;
}


//-----------------Function to check Left and right turns---------------
//-> Detect turns based on crossing of the threshold set by calibration
void CloudRunner::check_turn(){

    int R_val =0, L_val=0;  //mapped raw values

    R_val = read_sensor(R_TURN_PIN)/32;
  L_val = read_sensor(L_TURN_PIN)/32;

    //turn detection logic
    if(R_val > R_onblk_thresh && L_val < L_onblk_thresh){
        R_turn_detected = true;
    L_turn_detected = false;
    Intersect_detected = false;
        }else if(R_val < R_onblk_thresh && L_val > L_onblk_thresh){
        L_turn_detected = true;
    R_turn_detected = false;
    Intersect_detected = false;
        }else if(R_val> R_onblk_thresh && L_val > L_onblk_thresh){
    Serial.println("turn detected\n");
    Intersect_detected = true;
    R_turn_detected = false;
    L_turn_detected = false;
    
    }
    //Set flags if turn detected

}

//-----------------Function to check Left and right turns---------------
//-> Detect turns based on crossing of the threshold set by calibration
uint8_t CloudRunner::check_turn_new(){
    uint16_t L1_val = constrain(norm_vals[0], 0, MAX_LIMIT_SENSE);
    uint16_t L2_val = constrain(norm_vals[1], 0, MAX_LIMIT_SENSE);
    uint16_t R1_val = constrain(norm_vals[4], 0, MAX_LIMIT_SENSE);
    uint16_t R2_val = constrain(norm_vals[3], 0, MAX_LIMIT_SENSE);

    //turn detection logic
    if(L1_val > black_thresh_vals[0] &&
            L2_val > black_thresh_vals[1] &&
            R1_val > black_thresh_vals[4] &&
            R2_val > black_thresh_vals[3]  ){
        //intersection detected
        
        Serial.println("intersection");
        turn_status = FORWARD_MOTORS;
        return NO_CHANGE;
    }
    else if( L1_val > black_thresh_vals[0] && 
        L2_val < black_thresh_vals[1] &&
        R1_val < black_thresh_vals[4]/2 &&
        R2_val < black_thresh_vals[3]/2){
        //left turn detected
        Serial.println("left turn");
        turn_status = TURN_LEFT;
        return TURN_LEFT;
    } 
    else if(L1_val < black_thresh_vals[0]/2 &&
            L2_val < black_thresh_vals[1]/2 &&
            R1_val > black_thresh_vals[4] &&
            R2_val < black_thresh_vals[3]){
        //right turn detected
        Serial.println("right turn");
        turn_status = TURN_RIGHT;
        return TURN_RIGHT;
    }

    turn_status = NO_CHANGE;
    return NO_CHANGE;
}

bool CloudRunner::has_turn(){
    if(turn_status != 255){
        turn_status = 0;
        digitalWrite(12,HIGH);
        return true;
    }
    return false;
}

//----------------------Function to reset all turn flags------------------
//-> avoid latching by reseting all turn flags
void CloudRunner::reset_turn_detect(){
    R_turn_detected = false;
    L_turn_detected = false;
    Intersect_detected = false;
}




//======================MOTOR UTILITY FUNCTIONS======================


//motor compensation
//-------------Function to steer the motors in a fixed interval-------------
//-> interfaces with motor driver via pwm
void CloudRunner::drive_motor(int p_direction){
    analogWrite(L_SPEED_PIN, 40*L_spd_mult);
    analogWrite(R_SPEED_PIN, 40*R_spd_mult);
    switch(p_direction){
    case STOP_MOTORS:
        digitalWrite(L_FORWARD,LOW);
        digitalWrite(L_BACKWARD,LOW);
        digitalWrite(R_FORWARD,LOW);
        digitalWrite(R_BACKWARD,LOW);
        break;
    case FORWARD_MOTORS:
        digitalWrite(L_FORWARD,HIGH);
        digitalWrite(L_BACKWARD,LOW);
        digitalWrite(R_FORWARD,HIGH);
        digitalWrite(R_BACKWARD,LOW);
        break;
    case REVERSE_MOTORS:
        digitalWrite(L_FORWARD,LOW);
        digitalWrite(L_BACKWARD,HIGH);
        digitalWrite(R_FORWARD,LOW);
        digitalWrite(R_BACKWARD,HIGH);
        break;
    case TURN_RIGHT:
        digitalWrite(L_FORWARD,HIGH);
        digitalWrite(L_BACKWARD,LOW);
        digitalWrite(R_FORWARD,LOW);
        digitalWrite(R_BACKWARD,HIGH);
        break;
    case TURN_LEFT:
        digitalWrite(L_FORWARD,LOW);
        digitalWrite(L_BACKWARD,HIGH);
        digitalWrite(R_FORWARD,HIGH);
        digitalWrite(R_BACKWARD,LOW);
        break;
    default:
        break;
    }
}

//----------------------Function to steer motors using PID------------------
//-> modulates speed based on calculated PID value
void CloudRunner::PID_steer(int p_PID_val) {
    // Calculating the effective motor speed:

    // float Lspeed = (INIT_spd + p_PID_val)*L_spd_mult + L_spd_offset;
    // float Rspeed = (INIT_spd - p_PID_val)*R_spd_mult + R_spd_offset;

    float L_speed = 1.0;
    float R_speed = 1.0;
    if(p_PID_val > 0){
        R_speed = 1.0 - (float) p_PID_val / 100.0;
    }   
    else if(p_PID_val < 0){
        L_speed = 1.0 - (float) p_PID_val / 100.0;
    }
    
    if(L_speed < 0){
        digitalWrite(L_FORWARD, LOW);
        digitalWrite(L_BACKWARD, HIGH);
        digitalWrite(R_FORWARD , HIGH);
        digitalWrite(R_BACKWARD, LOW);
    }
    else{
        digitalWrite(L_FORWARD, HIGH);
        digitalWrite(L_BACKWARD, LOW);
        digitalWrite(R_FORWARD , HIGH);
        digitalWrite(R_BACKWARD, LOW);
    }
    if(R_speed < 0){
        digitalWrite(L_FORWARD, HIGH);
        digitalWrite(L_BACKWARD, LOW);
        digitalWrite(R_FORWARD , LOW);
        digitalWrite(R_BACKWARD, HIGH);
    }
    else{
        digitalWrite(L_FORWARD, HIGH);
        digitalWrite(L_BACKWARD, LOW);
        digitalWrite(R_FORWARD , HIGH);
        digitalWrite(R_BACKWARD, LOW);
    }
    // The motor speed should not exceed the max PWM value
    L_speed = constrain(L_speed*INIT_spd, LB_spd, UB_spd);
    R_speed = constrain(R_speed*INIT_spd, LB_spd, UB_spd);

    Serial.print("L: ");
    Serial.print(L_speed);
    Serial.print("R: ");
    Serial.print(R_speed);
    Serial.println();
    analogWrite(L_SPEED_PIN, L_speed); //Left Motor Speed
    analogWrite(R_SPEED_PIN, R_speed); //Right Motor Speed

    //following lines of code are to make the bot move forward

    // digitalWrite(L_FORWARD, HIGH);
    // digitalWrite(L_BACKWARD, LOW);
    // digitalWrite(R_FORWARD , HIGH);
    // digitalWrite(R_BACKWARD, LOW);

}

//----------------------Function to calculate PID------------------
//-> calculates PID value based on error passed
int CloudRunner::PID_calc(int p_error_val){
    P = p_error_val;
    D = p_error_val - old_error;
    I += P;
    I = constrain(I,-60,60);
    old_error = p_error_val;
    int PID_val = P*Kp + I*Ki + D*Kd;
    return PID_val;
}


//--------------------------Function to use DC motors as mini buzzers-------------------
//-> Use this function to provide feedback and make debugging easier
void CloudRunner::beep_motor(){

    int amplitude = 255;      //PWM value controls amplitude of beep
    int oscillation_delay_us = 500;

    //stop motors
    digitalWrite(L_FORWARD,LOW);
    digitalWrite(L_BACKWARD,LOW);
    digitalWrite(R_FORWARD,LOW);
    digitalWrite(R_BACKWARD,LOW);
    analogWrite(L_SPEED_PIN,0);
    analogWrite(R_SPEED_PIN,0);


    //oscillate motors to achieve beep
    for( int i = 0; i < 20; i++){


    analogWrite(L_SPEED_PIN,amplitude);
    analogWrite(R_SPEED_PIN,amplitude);

         //Run pins motor forward
    digitalWrite(L_FORWARD,HIGH);
    digitalWrite(L_BACKWARD,LOW);
    digitalWrite(R_FORWARD,HIGH);
    digitalWrite(R_BACKWARD,LOW);
    delayMicroseconds(oscillation_delay_us);
        //Run pins motor backwards
    digitalWrite(L_FORWARD,LOW);
    digitalWrite(L_BACKWARD,HIGH);
    digitalWrite(R_FORWARD,LOW);
    digitalWrite(R_BACKWARD,HIGH);
    delayMicroseconds(oscillation_delay_us);
    }

    analogWrite(L_SPEED_PIN,0);
    analogWrite(R_SPEED_PIN,0);
}

//----------------Functions to make quarter second beeps----------
//-> Use motors as buzzers for easier debugging
void CloudRunner::motor_quartersec_beep(int p_num_beep){
    for(int i = 0;i < p_num_beep;i++){
    beep_motor();
    delay(250);
    }
}



//======================PID UTILITY FUNCTIONS======================
//-> setter and getter functions
void CloudRunner::set_Kd(int p_Kd){
    Kd = p_Kd;
}

void CloudRunner::set_Kp(int p_Kp){
    Kp = p_Kp;

}

void CloudRunner::set_Ki(int p_Ki){
    Ki = p_Ki;
}

void CloudRunner::set_count(int val)
{
    count = val;
}

void CloudRunner::set_L_spd_offset(int offset){
    L_spd_offset = offset;
}

void CloudRunner::set_R_spd_offset(int offset){
    R_spd_offset = offset;
}

void CloudRunner::set_L_spd_mult(float mult){
    L_spd_mult = mult;
}

void CloudRunner::set_R_spd_mult(float mult){
    R_spd_mult = mult;
}

void CloudRunner::set_L_onblk_thresh(uint16_t thresh){
    L_onblk_thresh = thresh;
}

void CloudRunner::set_R_onblk_thresh(uint16_t thresh){
    R_onblk_thresh = thresh;
}

void CloudRunner::set_LB_spd(int speed){
    LB_spd = speed;
}

void CloudRunner::set_UB_spd(int speed){
    UB_spd = speed;
}

void CloudRunner::set_INIT_spd(int speed){
    INIT_spd = speed;
}

void CloudRunner::set_with_intersection(boolean statement, int count)
{
    with_intersection = statement;
    intersectionCount = count;
}

int CloudRunner::get_Kp(){
    return Kp;
}

int CloudRunner::get_Kd(){
    return Kd;
}

int CloudRunner::get_Ki(){
    return Ki;
}

int CloudRunner::get_count(){
    return count;
}

int CloudRunner::get_L_spd_offset(){
    return L_spd_offset;
}

int CloudRunner::get_R_spd_offset(){
    return R_spd_offset;
}

int CloudRunner::get_LB_spd(){
    return LB_spd;
}

int CloudRunner::get_UB_spd(){
    return UB_spd;
}

int CloudRunner::get_INIT_spd(){
    return INIT_spd;
}

boolean CloudRunner::get_with_intersection()
{
    return with_intersection;
}

int CloudRunner::get_intersectionCount()
{
    return intersectionCount;
}

boolean CloudRunner::intersection_detected()
{
    return Intersect_detected;
}

void CloudRunner::set_target_pos(int p_pos){
    target_pos = p_pos;
}

int CloudRunner::get_target_pos(){
    return target_pos;
}

void CloudRunner::set_torque_multiplier(int p_multiplier){
    torque_multiplier = p_multiplier;
}

int CloudRunner::get_torque_multiplier(){
    return torque_multiplier;
}