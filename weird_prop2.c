/*
 * Copyright 2022 Maria
 */ 
    
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h> // not used
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
  
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
 
#define HELIX_VELOCITY 20 // 10


bool checkpoint1; // true or false, true if reached
bool checkpoint2;
bool checkpoint3;

// August 23
int mark_1 = 0;
int mark_2 = 0;
int mark_3 = 0;


// August 20
// some 'static' declaration here:

static WbDeviceTag motor1, motor2, motor3, motor4;

// #define trial_number 3 // to make some test of the goto command  // August 20
 
      
void wb_gps_enable(WbDeviceTag tag, int sampling_period);
WbGpsCoordinateSystem wb_gps_get_coordinate_system(WbDeviceTag tag);

// August 24
void wb_camera_enable(WbDeviceTag tag, int sampling_period); // August 24
int wb_camera_get_sampling_period(WbDeviceTag tag); // August 24
void wb_camera_recognition_enable(WbDeviceTag tag, int time_step); // August 24

double time_All;

float timeS = 0.5; // seconds

int trial_number = 3; // August 20

 
  double getTimeAll() // June 18, 2022, modified on June 29 from int to double
  {
    time_All = (double)wb_robot_get_time(); // changing from int to double, June 29
    return time_All;
  }
 

 // Taking "motor stop" from A. Ailfranta's code

  void motorStop()
{
    wb_motor_set_velocity(motor1, 0);
    wb_motor_set_velocity(motor2, 0);
    wb_motor_set_velocity(motor3, 0);
    wb_motor_set_velocity(motor4, 0);
}



// August 20

void reach_the_target(float gpsSide1x, float gpsSide1z, float gpsSide2x, float gpsSide2z, float gpsSide3x, float gpsSide3z, float gpsSide4x, float gpsSide4z, float coordinate_x, float coordinate_z) {


// August 20
  // If I don't write the device_reference also here, I get an error while executing the code (invalid device tag)
  const WbDeviceTag motor1 = wb_robot_get_device("motor1");
  const WbDeviceTag motor2 = wb_robot_get_device("motor2");
  const WbDeviceTag motor3 = wb_robot_get_device("motor3");
  const WbDeviceTag motor4 = wb_robot_get_device("motor4");

if ((fabs((gpsSide1x + gpsSide3x)/2 - coordinate_x) > 0.1) || (fabs((gpsSide2z + gpsSide4z)/2 - coordinate_z) > 0.1)) { // 0.2, 0.2

if(fabs((gpsSide1x + gpsSide3x)/2 - coordinate_x) > 0.1) // > 0.2 // != 0
  {
    if(fabs(gpsSide1x- coordinate_x) > fabs(gpsSide3x - coordinate_x))
    {
        printf("\nbla\n");
        wb_motor_set_velocity(motor3, 0.5 * HELIX_VELOCITY); // fabs(gpsSide1x - coordinate_x)/1
        //printf("barycenter-x: %f\t target-x: %f\n", (gpsSide1x + gpsSide3x)/2, coordinate_x);
        printf("barycenter-x: %f\n", (gpsSide1x + gpsSide3x)/2);
        if ((gpsSide1x + gpsSide3x)/2 == coordinate_x)
          wb_motor_set_velocity(motor3, 0.0);
      }
      if(fabs(gpsSide3x - coordinate_x) > fabs(gpsSide1x - coordinate_x))
      {
        printf("\nblu\n");
        wb_motor_set_velocity(motor1, 0.5 * HELIX_VELOCITY); // fabs(gpsSide3x - coordinate_x)/1
        //printf("barycenter-x: %f\t target-x: %f\n", (gpsSide1x + gpsSide3x)/2, coordinate_x);
        printf("barycenter-x: %f\n", (gpsSide1x + gpsSide3x)/2);
        if ((gpsSide1x + gpsSide3x)/2 == coordinate_x)
          wb_motor_set_velocity(motor1, 0.0);
      }
      if(fabs(gpsSide3x - coordinate_x) == fabs(gpsSide1x - coordinate_x)) // problem: if it is equidistant!! --> just move forward?
      {
        printf("cip\n");
        //printf("barycenter-x: %f\t target-x: %f\n", (gpsSide1x+ gpsSide3x)/2, coordinate_x);
        printf("barycenter-x: %f\n", (gpsSide1x+ gpsSide3x)/2);
      }
}


if(fabs((gpsSide2z+ gpsSide4z)/2 - coordinate_z) > 0.1) // > 0.2 // != 0
  {
    if(fabs(gpsSide2z - coordinate_z) > fabs(gpsSide4z - coordinate_z))
    {
        printf("\nalpha\n");
        wb_motor_set_velocity(motor4, 0.5 * HELIX_VELOCITY);
        //printf("barycenter-z: %f\t target-z: %f\n", (gpsSide1z + gpsSide3z)/2, coordinate_z);
        printf("barycenter-x: %f\n", (gpsSide1x + gpsSide3x)/2);
        if ((gpsSide2z + gpsSide4z)/2 == coordinate_z)
          wb_motor_set_velocity(motor4, 0.0);
      }
      if(fabs(gpsSide4z - coordinate_z) > fabs(gpsSide2z - coordinate_z))
      {
        printf("\nbeta\n");
        wb_motor_set_velocity(motor2, 0.5 * HELIX_VELOCITY);
        //rintf("barycenter-z: %f\t target-z: %f\n", (gpsSide2z + gpsSide4z)/2, coordinate_z);
        printf("barycenter-z: %f\n", (gpsSide2z + gpsSide4z)/2);
        if ((gpsSide2z+ gpsSide4z)/2 == coordinate_z)
          wb_motor_set_velocity(motor2, 0.0);
      }
      if(fabs(gpsSide4z - coordinate_z) == fabs(gpsSide2z - coordinate_z))  // problem: if it is equidistant!! --> just move forward?
      {
        printf("gamma\n");
        //printf("barycenter-z: %f\t target-z: %f\n", (gpsSide2z + gpsSide4z)/2, coordinate_z);
        printf("barycenter-z: %f\n", (gpsSide2z + gpsSide4z)/2);
      }
}
}
else{
  printf("\n\ntarget reached!\n\n");
  // August 22
  /*

  // what I was thinking is not feasible, because there is no information on the first or second target...
  if ((fabs((gpsSide1x + gpsSide3x)/2 - target_x[0]) <= 0.2) && (fabs((gpsSide2z + gpsSide4z)/2 - coordinate_z) <= 0.2)) {
    int mark_1 = 1;
    printf("Ciao! mark_1 = %d\n", mark_1);
  }
  if ((fabs((gps_side1[0] + gps_side3[0])/2 - target_x[1]) <= 0.2) && (fabs((gps_side2[2] + gps_side4[2])/2 - target_z[1]) <= 0.2)) {
    int mark_2 = 1;
    printf("Ciao! mark_2 = %d\n", mark_2);
  }
  */
  // checkpoint1 = true;
  wb_motor_set_velocity(motor1, 0.0 * HELIX_VELOCITY);
  wb_motor_set_velocity(motor2, 0.0 * HELIX_VELOCITY);
  wb_motor_set_velocity(motor3, 0.0 * HELIX_VELOCITY);
  wb_motor_set_velocity(motor4, 0.0 * HELIX_VELOCITY);
}



}







int main() {   
  wb_robot_init();
  const int time_step = wb_robot_get_basic_time_step();


  FILE *f = fopen("outcome_.txt", "a");
  FILE *f2 = fopen("trash.txt", "a");

  FILE *fx = fopen("outcome_x.csv", "a");
  FILE *fy = fopen("outcome_y.csv", "a");

  FILE *f_x = fopen("fx.txt", "a");
  FILE *f_y = fopen("fy.txt", "a");


  
// helixes motion
  const WbDeviceTag motor1 = wb_robot_get_device("motor1");
  const WbDeviceTag motor2 = wb_robot_get_device("motor2");
  const WbDeviceTag motor3 = wb_robot_get_device("motor3");
  const WbDeviceTag motor4 = wb_robot_get_device("motor4");


  WbDeviceTag gps_1 = wb_robot_get_device("gps_side_1"); // plate_
  wb_gps_enable(gps_1, time_step);
  const double *gps_side1 = wb_gps_get_values(gps_1);


  WbDeviceTag gps_2 = wb_robot_get_device("gps_side_2"); // plate_
  wb_gps_enable(gps_2, time_step);
  const double *gps_side2 = wb_gps_get_values(gps_2);


  WbDeviceTag gps_3 = wb_robot_get_device("gps_side_3"); // plate_
  wb_gps_enable(gps_3, time_step);
  const double *gps_side3 = wb_gps_get_values(gps_3);


  WbDeviceTag gps_4 = wb_robot_get_device("gps_side_4"); // plate_
  wb_gps_enable(gps_4, time_step);
  const double *gps_side4 = wb_gps_get_values(gps_4);
  

  WbDeviceTag ps1 = wb_robot_get_device("ps1_plate");
  WbDeviceTag ps2 = wb_robot_get_device("ps2_plate");
  WbDeviceTag ps3 = wb_robot_get_device("ps3_plate");
  WbDeviceTag ps4 = wb_robot_get_device("ps4_plate");

  wb_distance_sensor_enable(ps1, time_step);
  wb_distance_sensor_enable(ps2, time_step);
  wb_distance_sensor_enable(ps3, time_step);
  wb_distance_sensor_enable(ps4, time_step);

  wb_motor_set_position(motor1, 0.1); // stop
  wb_motor_set_position(motor2, 0.1); // stop
  wb_motor_set_position(motor3, 0.1); // stop
  wb_motor_set_position(motor4, 0.1); // stop
  
  while (wb_robot_step(time_step) != -1) {

  double ps_values1 = wb_distance_sensor_get_value(ps1); // July 26
  double ps_values2 = wb_distance_sensor_get_value(ps2); // July 26
  double ps_values3 = wb_distance_sensor_get_value(ps3); // July 26
  double ps_values4 = wb_distance_sensor_get_value(ps4); // July 26




    
 
  printf("distance-sensor 1 readings: %f\n", ps_values1); // without the "\n" IT DOESN'T WORK 'CAUSE THERE's A BUG!
  printf("distance-sensor 2 readings: %f\n", ps_values2); // without the "\n" IT DOESN'T WORK 'CAUSE THERE's A BUG!
  printf("distance-sensor 3 readings: %f\n", ps_values3); // without the "\n" IT DOESN'T WORK 'CAUSE THERE's A BUG!
  printf("distance-sensor 4 readings: %f\n", ps_values4); // without the "\n" IT DOESN'T WORK 'CAUSE THERE's A BUG!

  printf("\n\n");

  printf("GPS-sensor 1 (turquoise) readings along x: %f\t and z: %f\n", gps_side1[0], gps_side1[2]);
  printf("GPS-sensor 2 (blue) readings along x: %f\t and z: %f\n", gps_side2[0], gps_side2[2]);
  printf("GPS-sensor 3 (violet) readings along x: %f\t and z: %f\n", gps_side3[0], gps_side3[2]);
  printf("GPS-sensor 4 (green) readings along x: %f\t and z: %f\n", gps_side4[0], gps_side4[2]);


  wb_motor_set_velocity(motor1, 0.0 * HELIX_VELOCITY);
  wb_motor_set_velocity(motor2, 0.0 * HELIX_VELOCITY);
  wb_motor_set_velocity(motor3, 0.0 * HELIX_VELOCITY);
  wb_motor_set_velocity(motor4, 0.0 * HELIX_VELOCITY);


  // August 24
  WbDeviceTag camera = wb_robot_get_device("camera"); // plate_
  wb_camera_enable(camera, time_step);
  wb_camera_recognition_enable(camera, time_step);
  // const double *gps_side4 = wb_gps_get_values(gps_4);

  // August 24
const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera); // from https://github.com/Azeem-Salvatore/Wall-following-vehicle/blob/main/Controller%20for%20emitter%20robotic%20vehicle
  int object_num = wb_camera_recognition_get_number_of_objects(camera); // from https://github.com/Azeem-Salvatore/Wall-following-vehicle/blob/main/Controller%20for%20emitter%20robotic%20vehicle
  int message_printed = 0; // same as above



// August 20

  // const double *target_x[2] = {-0.3, 0.15};

  double target_x[2] = {-0.7, -0.7}; // {-0.3, 0.0} 0.3
  double target_y[2] = {0.0, 0.0};
  double target_z[2] = {0.6, 0.6}; // {-0.6, 0.8}; -0.5


  /* Good example: (I'm commenting this on September 27 just to make an example for Apphia)

  double target_x[3] = {-0.7, 0.1, 0.3}; // {-0.3, 0.0} 0.3
  double target_y[3] = {0.0, 0.0, 0.0};
  double target_z[3] = {0.7, 0.7, -0.6}; // {-0.6, 0.8}; -0.5


  */


  printf("target_x[0] is %f\n", target_x[0]);
  printf("target_z[0] is %f\n", target_z[0]);
  printf("target_x[1] is %f\n", target_x[1]);
  printf("target_z[1] is %f\n", target_z[1]);

  printf("%f\t%f\t%f\t%f\t\n\n", target_x[0], target_z[0], target_x[1], target_z[1]);

  // initialization

  checkpoint1 = false;
  checkpoint2 = false;
  checkpoint3 = false;

  // check if the robot is already at the first checkpoint...
  // and we put a "mark" if it has been reached:
  //int mark_1 = 0;
  //int mark_2 = 0;

// camera: August 24
// while doing the tasks (= getting to some checkpoint(s) whose coordinates are known),
// the robot recognized trash according to colors. Recognized objects are surrounded by a red square
// in the robotic camera. How to print out their coordinates?


fprintf(f, "\nthe number of detected objects (trash?) at %f\t%f is %d\n", (gps_side1[0] + gps_side3[0])/2, (gps_side2[2] + gps_side4[2])/2, object_num, "a");
// fprintf(f2, "\nthe number of detected objects (trash?) at %f\t%f is %d\n", (gps_side1[0] + gps_side3[0])/2, (gps_side2[2] + gps_side4[2])/2, object_num, "a");


  fprintf(f, "time time%f\n\n", getTimeAll(), "a");

if (object_num >= 1) {
	fprintf(f2, "\n\nThe robot found some dark-grey trash around coordinates %f\t%f\t no. of items: %d\n", (gps_side1[0] + gps_side3[0])/2, (gps_side2[2] + gps_side4[2])/2, object_num, "a");
	printf("\n\nThe robot found some dark-grey trash around coordinates %f\t%f\t no. of items: %d\n", (gps_side1[0] + gps_side3[0])/2, (gps_side2[2] + gps_side4[2])/2, object_num);
}


// Section added on October 3, 2022



fprintf(f, "the distance-sensory reading no. 1 is: %f\n", ps_values1);
fprintf(f, "the distance-sensory reading no. 2 is: %f\n", ps_values2);
fprintf(f, "the distance-sensory reading no. 3 is: %f\n", ps_values3);
fprintf(f, "the distance-sensory reading no. 4 is: %f\n", ps_values4);

if (ps_values1 < 900)
  {
    fprintf(f, "sensor 1: signaling closeness\n");
}

if (ps_values2 < 900)
  {
    fprintf(f, "sensor 2: signaling closeness!\n");
}

if (ps_values3 < 900)
  {
    fprintf(f, "sensor 3: signaling closeness\n");
}

if (ps_values4 < 900)
  {
    fprintf(f, "sensor 4: signaling closeness\n");
}



// Here, the first task begins

if (mark_1 == 0) {

if ((fabs((gps_side1[0] + gps_side3[0])/2 - target_x[0]) <= 0.1) && (fabs((gps_side2[2] + gps_side4[2])/2 - target_z[0]) <= 0.1)) // 0.2, 0.2
  {
    checkpoint1 = true;
    motorStop(); // August 22
    mark_1 = 1;
    printf("\ncheckpoint 1: true\t mark_1 = %d\n", mark_1);
    fprintf(f, "\nrobot barycenter x: %f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(f, "robot barycenter z: %f\n", (gps_side2[2] + gps_side4[2])/2, "a");
    fprintf(f, "target x: %f\n", target_x[0], "a");
    fprintf(f, "target z: %f\n", target_z[0], "a");
    fprintf(f, "checkpoint 1: true\n", "a");
    fprintf(f, "mark_1 = %d\n", mark_1, "a");
	  fprintf(f, "mark_2 = %d\n", mark_2, "a");
    fprintf(f, "mark_3 = %d\n", mark_3, "a");

    fprintf(fx, "%f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(fy, "%f\n", (gps_side2[2] + gps_side4[2])/2, "a");

    fprintf(f_x, "%f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(f_y, "%f\n", (gps_side2[2] + gps_side4[2])/2, "a");


  }

 //if (mark_1 == 0) {
 	reach_the_target(gps_side1[0], gps_side1[2], gps_side2[0], gps_side2[2], gps_side3[0], gps_side3[2], gps_side4[0], gps_side4[2], target_x[0], target_z[0]);
    printf("\ntarget x: %f\n", target_x[0]);
    printf("\ntarget z: %f\n", target_z[0]);
    fprintf(f, "\nrobot barycenter x: %f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(f, "robot barycenter y: %f\n", (gps_side2[2] + gps_side4[2])/2, "a");
    fprintf(f, "target x: %f\n", target_x[0], "a");
    fprintf(f, "target z: %f\n", target_z[0], "a");
    fprintf(f, "mark_1 = %d\n", mark_1, "a");
	  fprintf(f, "mark_2 = %d\n", mark_2, "a");
    fprintf(f, "mark_3 = %d\n", mark_3, "a");

    fprintf(fx, "%f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(fy, "%f\n", (gps_side2[2] + gps_side4[2])/2, "a");

    fprintf(f_x, "%f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(f_y, "%f\n", (gps_side2[2] + gps_side4[2])/2, "a");
 }  


printf("mark_1 %d\n", mark_1);
//fprintf(f, "mark_1 = %d\n", mark_1, "a");
//fprintf(f, "mark_2 = %d\n", mark_2, "a");


/*
if (mark_1 == 1) {
  printf("\nciao ciao!\n\n");
}
*/


// Here, the second task begins

if (mark_1 == 1 && mark_2 == 0) { // && mark_2 == 0 new

  //usleep(10);

    printf("\n\nlet's keep moving!\n\n");
    reach_the_target(gps_side1[0], gps_side1[2], gps_side2[0], gps_side2[2], gps_side3[0], gps_side3[2], gps_side4[0], gps_side4[2], target_x[1], target_z[1]);
    printf("\ntarget x: %f\n", target_x[1]);
    printf("\ntarget z: %f\n", target_z[1]);
    //fprintf(f, "robot %s\n", wb_robot_get_name(), "a");
	fprintf(f, "\nrobot barycenter x: %f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(f, "robot barycenter y: %f\n", (gps_side2[2] + gps_side4[2])/2, "a");
    fprintf(f, "target x: %f\n", target_x[1], "a");
    fprintf(f, "target z: %f\n", target_z[1], "a");
    printf("mark_1 %d\n", mark_1);
    printf("mark_2 %d\n", mark_2);
    printf("mark_3 %d\n", mark_2);
    fprintf(f, "mark_1 = %d\n", mark_1, "a");
    fprintf(f, "mark_2 = %d\n", mark_2, "a");
    fprintf(f, "mark_3 = %d\n", mark_3, "a");

    fprintf(fx, "%f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(fy, "%f\n", (gps_side2[2] + gps_side4[2])/2, "a");

    fprintf(f_x, "%f\n", (gps_side1[0] + gps_side3[0])/2, "a");
    fprintf(f_y, "%f\n", (gps_side2[2] + gps_side4[2])/2, "a");



    if ((fabs((gps_side1[0] + gps_side3[0])/2 - target_x[1]) <= 0.1) && (fabs((gps_side2[2] + gps_side4[2])/2 - target_z[1]) <= 0.1)) // 0.2, 0.2
  {
    checkpoint2 = true;
    motorStop(); // August 22
    mark_2 = 1;
    printf("\ncheckpoint 2: true\n");
    printf("mark_1 %d\n", mark_1);
    printf("mark_2 %d\n", mark_2);
    fprintf(f, "\ncheckpoint 2: true\n", "a");
    fprintf(f, "mark_1 = %d\n", mark_1, "a");
    fprintf(f, "mark_2 = %d\n", mark_2, "a");
    fprintf(f, "mark_3 = %d\n", mark_3, "a");
  }


  }


	}

 return 0;
}




