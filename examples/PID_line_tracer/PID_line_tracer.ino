

#include <cloudrunner.h>
CloudRunner board;

#define OUTPUT_UPDATE_RATE 50
uint8_t a = 0;
void setup() {
  Serial.begin(115200);

  //Initialize board and motors
  board.begin();

  //Set the intersection boolean and intersection count
  board.set_with_intersection(true, 3);

  //Set your manual configuration for the motors
  // board.set_L_onblk_thresh(4000);
  // board.set_R_onblk_thresh(4000);
  // board.set_L_spd_offset(9); // Set an offset in the motor's speed 
  //board.set_R_spd_offset(0); // if their actual rpm does not match

  // scalers for motor speed; use if constant offset is not enough
  // board.set_L_spd_mult(1); 
  // board.set_R_spd_mult(1);

  board.set_INIT_spd(30);    // Initial speed
  board.set_LB_spd(0);       // Lower Bound speed
  board.set_UB_spd(255);      // Upper Bound speed

  //Set the PID constants here
  board.set_Kp(1);
  board.set_Kd(4);
  board.set_Ki(0); 
  
  //Set your ideal center position here (for the line)
  board.set_target_pos(30);
  
  //Set the other constants here, this should help you
  // solve any problems with the get pos
  board.set_torque_multiplier(50);
  
  pinMode(12, OUTPUT);
  digitalWrite(12,HIGH);
  delay(3000);
  //sensor calibration if with intersection 
  // if(board.get_with_intersection()){

  //   Serial.println("Start Calibration");
  //   board.calibrate_PID_sensors();
  //   delay(10);
  //   board.calibrate_turn_sensors();
  //   Serial.println("Done Calibration");
  //   delay(5000);

  // }
  digitalWrite(12,LOW);
  Serial.println("Calibration Start:");
  board.init_baselines();
  board.init_black_thresholds();
  board.print_baselines();
  board.print_black_thresholds();
  Serial.println("Calibration End\n");
  
  delay(3000);
}

uint32_t last_update_ms = 0;

void loop() {
  if(millis()-last_update_ms < 1000/OUTPUT_UPDATE_RATE){
    return;
  }
  a--;
  if(a == 0){
    digitalWrite(12, LOW);
  }
  last_update_ms = 0;

  board.read_all_sensors();
  board.update_baselines();
  board.norm_sensors();
  board.calc_pos();
  
  int pos =0 , error =0 , PID_val =0;
  pos = board.get_pos_new();                  //find the current position

  error = pos - board.get_target_pos();   //find the error
  PID_val = board.PID_calc(error);        //find the PID value
  // board.print_norm_vals();
  Serial.print("pos: ");
  Serial.print(pos);
  // Serial.print(", err:");
  // Serial.print(error);
  // Serial.print(", pid:");
  // Serial.print(PID_val);
  Serial.println();

  uint8_t turn = board.check_turn_new();
  if(board.has_turn()){
      board.drive_motor(turn);
      last_update_ms += 30; //delay next output update
      if(turn == FORWARD_MOTORS){
        last_update_ms +=40;
      }
      a = 5;
  }
  else{
    board.PID_steer(PID_val);
  }
  last_update_ms += millis();

  // if(board.get_with_intersection()){

  //   board.check_turn();                   //check if intersection detected

  //   // if(board.intersection_detected()){    // if there's an intersection drive for xxx ms and increment count

  //   //   board.drive_motor(FORWARD_MOTORS);
  //   //   delay(100);
  //   //   board.set_count(board.get_count() + 1);

  //   // }

  //   board.reset_turn_detect();            //reset detection booleans

  //   // if(board.get_count() >= board.get_intersectionCount()){   // check if the robot should stop after x intersection detected

  //   //   board.drive_motor(STOP_MOTORS);
  //   //   delay(10000000);

  //   // }
  // }
  
  
                 //adjust motor speed and runs the robot
  
}
