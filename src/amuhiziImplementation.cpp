/*******************************************************************************************************************
 *   Assignment 5 - vision-based brick stacking
 *   --------------------------------------------------------------------
 *
 *   Implementation file
 *
 *   Audit Trail
 *   -----------
 *   28 June 2020: re-factored code to separate calculation of the joint angles using the inverse kinematics,  
 *                 from the calculation of servomotor setpoint values.  
 *                 This was done to allow the simulator to be controlled by publishing joint angles on the 
 *                 ROS /lynxmotion_al5d/joints_positions/command topic 
 *
 *   Audit Trail
 *   -----------
 *   Generated a studentid skeleton application by removing the core processing, i.e. brick segmentation, pose estimation in 2D,
 *   and pose estimation in 3D using the inverse perspective transformation 
 *   David Vernon 8 April 2021
 *
 *******************************************************************************************************************/

#ifdef WIN32
#include "amuhizi.h"
#else
#include <assignment5/amuhizi.h>
#endif


/******************************************************************************************************************

   SimulatorCamera class to support image acquisition from the Gazebo Lynxmotion AL5D Simulator 
   --------------------------------------------------------------------------------------------

   Interface to the SimulatorCamera classe required to implement image acquistion
   
   Author: David Vernon
   Date:   04/04/2021

*******************************************************************************************************************/

#ifdef ROS

SimulatorCamera::SimulatorCamera() {

   bool debug = false;
  
   /* note that we can't initialize the subscriber here because the node has to be initializeded first */
   /* and the object is instantiated and the constructor is called  before this happens.               */

   if (debug) printf("SimulatorCamera constuctor\n");

   frameNumber = 0;
   lastFrameAcquired = 0;
};

void SimulatorCamera::initialize() {

   bool debug = false;
  
   /* do initialization in the constructor                            */
   /* declare a node handler and register the image handling callback */
  
   if (debug) printf("SimulatorCamera::initializeCamera\n");
   
   image_transport::ImageTransport it(nh);
   
   sub = it.subscribe("/lynxmotion_al5d/external_vision/image_raw", 1, &SimulatorCamera::imageMessageReceived, this);

   ros::topic::waitForMessage<sensor_msgs::Image>("/lynxmotion_al5d/external_vision/image_raw");

   ros::spinOnce(); 

};

void SimulatorCamera::imageMessageReceived(const sensor_msgs::ImageConstPtr &msg) {
  
   bool debug = false;
  
   cv_bridge::CvImagePtr cv_ptr;
   try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e) {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
   
   frame = cv_ptr->image;

   if (frame.empty()) {
       printf("SimulatorCamera::imageMessageReceived; image is empty\n");
   }

   frameNumber++;

   if (debug) printf("SimulatorCamera::imageMessageReceived: image %ld size is %d x %d\n",frameNumber, frame.rows, frame.cols);

}

void SimulatorCamera::getImage(Mat &image) {

   bool debug = false;

   /* wait until a new frame arrives, i.e. until the imageMessageReceived callback has been called at least once since the last acquisition */
   /* discard the first two frames to make sure we are not reading old images in a buffer; this happens even if the queue length = 1        */
   
   while ((lastFrameAcquired == frameNumber) || (frameNumber < 3)) {
      ros::spinOnce();
   }
   
   image = frame; // copy image from the one stored by the subscriber callback function
   lastFrameAcquired = frameNumber;
	  
   if (debug) printf("SimulatorCamera::getImage: image %ld size is %d x %d\n",lastFrameAcquired, image.rows, image.cols);
};

#endif

/******************************************************************************

 Robot configuration data: global to allow access from implementation functions

*******************************************************************************/
    
struct robotConfigurationDataType robotConfigurationData;

 
/***********************************************************************************************************************

   Frame and vector classes to support task-level robot programming 
   ----------------------------------------------------------------

   Interface to the Frame and Vector classes and auxilliary friend functions required to implement a robot control program
   by specifying the position and orientation of objects and the robot manipulator using homogeneous transformations
   
   Author: David Vernon, Carnegie Mellon University Africa, Rwanda
   Date:   22/02/2017

***********************************************************************************************************************/
float joint_state_[6]; // Array to store the joint states

Vector::Vector(double x, double y, double z, double w) { 
   coefficient[0] = x; 
   coefficient[1] = y;
   coefficient[2] = z;
   coefficient[3] = w;
}

void Vector::setValues(double x, double y, double z, double w) { 
   coefficient[0] = x; 
   coefficient[1] = y;
   coefficient[2] = z;
   coefficient[3] = w;
}

void Vector::getValues(double &x, double &y, double &z, double &w) { 
   x = coefficient[0]; 
   y = coefficient[1];
   z = coefficient[2];
   w = coefficient[3];
}

void Vector::printVector()const {
  int i;
   
   for (i=0; i<4; i++) {
     printf("%4.1f ",coefficient[i]);
   }
   printf("\n\n");
}
 
Vector operator+(Vector &a, Vector &b) { 
   return Vector(a.coefficient[0] / a.coefficient[3] + b.coefficient[0] / b.coefficient[3], 
                 a.coefficient[1] / a.coefficient[3] + b.coefficient[1] / b.coefficient[3], 
                 a.coefficient[2] / a.coefficient[3] + b.coefficient[2] / b.coefficient[3],
                 1); //friend access
}

double dotProduct(Vector &a, Vector &b) {

   double result;

   result = a.coefficient[0] / a.coefficient[3] * b.coefficient[0] / b.coefficient[3] +
            a.coefficient[1] / a.coefficient[3] * b.coefficient[1] / b.coefficient[3] +
            a.coefficient[2] / a.coefficient[3] * b.coefficient[2] / b.coefficient[3]; //friend access;

    return result;
}

Frame::Frame() { 
   int i, j;
   
   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         if (i==j)
            coefficient[i][j] = 1; 
         else
            coefficient[i][j] = 0; 
      }
   }
}

void Frame::printFrame()const {
   int i, j;
   
   printf("\n");
   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         printf("%4.1f ",coefficient[i][j]);
      }
      printf("\n");
   }
   printf("\n");
}
 
Frame &Frame::operator*(Frame const& h) { 

   Frame result;
   double temp;
   int i, j, k;
   bool debug = false;

   if (debug) {
      printf("Operator *\n");
      this->printFrame();
      h.printFrame();
   }

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         temp = 0;
         for (k=0; k<4; k++) { 
            temp = temp + (this->coefficient[i][k]) * (h.coefficient[k][j]);
         } 
         result.coefficient[i][j] = temp; 
      }
   } 

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         this->coefficient[i][j] = result.coefficient[i][j]; 
      }
   } 

   if (debug) this->printFrame();

   return *this;
}

Frame &Frame::operator=(Frame const& h) { 
   int i, j;
   bool debug = false;

   if (debug) {
      printf("Operator =\n");
      h.printFrame();
   }

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         this->coefficient[i][j] = h.coefficient[i][j];
      }
   }

  if (debug) this->printFrame();

   return *this;
}

/* translation by vector (x, y, z) */

Frame trans(float x, float y, float z) {
      
   Frame result;

   int i, j;
   bool debug = false;

   if (debug) {
      printf("trans %f %f %f\n",x, y, z);
   }

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = 1;
   result.coefficient[1][1] = 1;
   result.coefficient[2][2] = 1;
   result.coefficient[3][3] = 1;

   result.coefficient[0][3] = x;
   result.coefficient[1][3] = y;
   result.coefficient[2][3] = z;

  if (debug) result.printFrame();

  return result;
}


/* rotation about x axis by theta degrees */

Frame rotx(float theta) {
         
   Frame result;
   double thetaRadians;
   int i, j;
   bool debug = false;

   if (debug) {
      printf("rotx %f\n",theta);
   }

   /* convert theta to radians */

   thetaRadians = (3.14159 * theta) / 180.0;

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = 1;
   result.coefficient[0][1] = 0;
   result.coefficient[0][2] = 0;

   result.coefficient[1][0] = 0;
   result.coefficient[1][1] = cos(thetaRadians);
   result.coefficient[1][2] = -sin(thetaRadians);;

   result.coefficient[2][0] = 0;
   result.coefficient[2][1] = sin(thetaRadians);
   result.coefficient[2][2] = cos(thetaRadians);

   result.coefficient[3][3] = 1;

  if (debug) result.printFrame();

  return result;
}


/* rotation about y axis by theta degrees */

Frame roty(float theta) {
         
   Frame result;
   double thetaRadians;
   int i, j;
   bool debug = false;

   if (debug) {
      printf("roty %f\n",theta);
   }

   /* convert theta to radians */

   thetaRadians = (3.14159 * theta) / 180.0;

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = cos(thetaRadians);
   result.coefficient[0][1] = 0;
   result.coefficient[0][2] = sin(thetaRadians);

   result.coefficient[1][0] = 0;
   result.coefficient[1][1] = 1;
   result.coefficient[1][2] = 0;

   result.coefficient[2][0] = -sin(thetaRadians);
   result.coefficient[2][1] = 0;
   result.coefficient[2][2] = cos(thetaRadians);

   result.coefficient[3][3] = 1;

  if (debug) result.printFrame();

  return result;
}


Frame inv(Frame h) { 

   Vector u, v;
   Frame result;

   int i, j;
   //double dp;
   bool debug = false;

   if (debug) {
      printf("inv \n");
      h.printFrame();
   }

   for (i=0; i<3; i++) {
      for (j=0; j<3; j++) {
         result.coefficient[j][i] = h.coefficient[i][j]; // transpose the rotation part
      }
   }

   for (j=0; j<4; j++) {
      result.coefficient[3][j] = h.coefficient[3][j];    // copy the scaling part
   }

   
   v.setValues(h.coefficient[0][3], h.coefficient[1][3], h.coefficient[2][3], h.coefficient[3][3]); // p

   u.setValues(h.coefficient[0][0], h.coefficient[1][0], h.coefficient[2][0], 1); // n
   result.coefficient[0][3] = -dotProduct(u,v);
 
   u.setValues(h.coefficient[0][1], h.coefficient[1][1], h.coefficient[2][1], 1); // o 
   result.coefficient[1][3] = -dotProduct(u,v);
 
   u.setValues(h.coefficient[0][2], h.coefficient[1][2], h.coefficient[2][2], 1); // a
   result.coefficient[2][3] = -dotProduct(u,v);
 
   result.coefficient[3][3] = 1;

   if (debug) result.printFrame();

   return result;
}


/* rotation about z axis by theta degrees */

Frame rotz(float theta) {
         
   Frame result;
   double thetaRadians;
   int i, j;
   bool debug = false;

   if (debug) {
      printf("rotz %f\n",theta);
   }

   /* convert theta to radians */

   thetaRadians = (3.14159 * theta) / 180.0;

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = cos(thetaRadians);
   result.coefficient[0][1] = -sin(thetaRadians);
   result.coefficient[0][2] = 0;

   result.coefficient[1][0] = sin(thetaRadians);
   result.coefficient[1][1] = cos(thetaRadians);
   result.coefficient[1][2] = 0;

   result.coefficient[2][0] = 0;
   result.coefficient[2][1] = 0;
   result.coefficient[2][2] = 1;

   result.coefficient[3][3] = 1;

  if (debug) result.printFrame();

  return result;
}


/* move(Frame t5)                                                                                   */
/*                                                                                                  */
/* 1. Extract the pose parameters from the T5 frame and servo the robot to the associated pose      */
/* 2. Call computeJointAngles() to compute the joint angles in radians using the inverse kinematics */
/* 3. Call setJointAngles() to servo to robot arm to the required joint angles                      */
/*                                                                                                  */
/*                                                                                                  */
/* Fixed bug in computation of pitch parameter                                                      */
/* David Vernon 8/6/2018                                                                            */
/*                                                                                                  */
/* Refactored code to use computeJointAngles() and setJointAngles()                                 */
/* David Vernon 28/6//2020                                                                          */


bool move(Frame T5) {

   bool debug = false;

   double ax, ay, az; // components of approach vector
   double ox, oy, oz; // components of orientation vector
   double px, py, pz; // components of position vector

   double tolerance = 0.001;
   double r;
   double pitch;
   double roll;

   double jointAngles[6]; // six angles: five for the pose (joints 1 to 5), all in radians, and one for the gripper in metres 

   /* check to see if the pose is achievable:                                                                */
   /* the approach vector must be aligned with (i.e. in same plane as) the vector from the base to the wrist */
   /* (unless the approach vector is directed vertically up or vertically down                               */

   // T5.printFrame();

   ox = T5.coefficient[0][1];
   oy = T5.coefficient[1][1];
   oz = T5.coefficient[2][1];

   ax = T5.coefficient[0][2];
   ay = T5.coefficient[1][2];
   az = T5.coefficient[2][2];

   px = T5.coefficient[0][3];
   py = T5.coefficient[1][3];
   pz = T5.coefficient[2][3];


   if (debug) {
         T5.printFrame();
         // printf("move(): px,py %4.1f %4.1f  ax,ay %4.1f %4.1f angles  %4.1f %4.1f \n", px, py, ax, ay, 180*atan2(py, px)/3.14159, 180*atan2(ay, ax)/3.14159);
   }

   if (( ax > -tolerance && ax < tolerance && ay > -tolerance && ay < tolerance)  // vertical approach vector
       ||
       (abs(atan2(ay, ax) - atan2(py, px)) < tolerance)) {  

      /* achievable pose                           */
      /* extract the pitch and roll angles from T5 */

      /* pitch */
      r = sqrt(ax*ax + ay*ay);

      if (r < tolerance) {      // vertical orientation vector
         pitch = 0;
         if (az < 0) {          // get direction right
            pitch = -180;
         }
      }
      else {
         pitch = -degrees(atan2(r, az)); // DV change to negative angle since pitch range is -180 to 0     8/6/2018
      }
       
      /* roll */
      roll =   degrees(atan2(ox, oy));

      if (debug) {
         printf("move(): x, y, z, pitch, roll: %4.1f %4.1f %4.1f %4.1f %4.1f \n", px, py, pz, pitch, roll);
      }

      //gotoPose((float) px, (float) py, (float) pz, (float) pitch, (float) roll);

	  computeJointAngles(px, py, pz, pitch, roll, jointAngles);

	  setJointAngles(jointAngles);

      return true;
   }
   else {

      printf("move(): pose not achievable: approach vector and arm are not aligned \n");
      printf("        atan2(py, px) %f; atan2(ay, ax)  %f\n",180*atan2(py, px)/3.14159, 180*atan2(ay, ax)/3.14159);

      return false; // approach vector and arm are not aligned ... pose is not achievable
   }
}


/*********************************************************************/
/*                                                                   */
/* Inverse kinematics for LynxMotion AL5D robot manipulator          */
/*                                                                   */
/*********************************************************************/

double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI; // David Vernon ... cast to float
    return degrees;
}

double radians(double degrees)
{
    double radians = degrees / ((double) 180.0 / (double) M_PI); // David Vernon ... cast to float
    return radians;
}
 

/* computeJointAngles()
   
   Transform from wrist pose (x, y, z, pitch, and roll in degrees) to joint angles using the inverse kinematics

   If resulting arm position is physically unreachable, return false.
   Otherwise, return the corresponding joint angles in radians.

   This code is a modified version of the code written by Oleg Mazurov and Eric Goldsmith (see header)
   The revisions include the removal of the dependency on the end-effector (gripper) distance
   so that it computes the inverse kinematics for T5, the position and orientation of the wrist
   Also, the roll angle was added.  Minor changes have been made to the variable names; more are necessary
   to make it readable and consistent with Denavit-Hartenberg notation 

   David Vernon
   3 March 2017
   
   Audit trail
   -----------

   7 June 2018: modified to use robot configuration data reflecting the calibration of individual robots DV
   8 June 2018: fixed a number of bugs in the inverse kinematic solution
   28 June 2020: refactored code into computeJointAngles() and computeJointPositions(), removing getPose()
*/

bool computeJointAngles(double x, double y, double z, double pitch_angle_d, double roll_angle_d, double joint_angles[]) {
 
    bool debug = false; 
  
    if (debug) printf("computeJointAngles(): x %4.1f, y %4.1f, z %4.1f, pitch %4.1f, roll %4.1f\n", x, y, z, pitch_angle_d, roll_angle_d);

    double hum_sq;
    double uln_sq;
    double wri_roll_angle_d;

    hum_sq = A3 * A3;
    uln_sq = A4 * A4;


    //grip angle in radians for use in calculations
    double pitch_angle_r = radians(pitch_angle_d);  // David Vernon uncommented 
    double roll_angle_r = radians(roll_angle_d);    // David Vernon uncommented


    // Base angle and radial distance from x,y coordinates
	// ---------------------------------------------------
    double bas_angle_r = atan2(x, y);
    double bas_angle_d = degrees(bas_angle_r);

    double rdist = sqrt((x * x) + (y * y));

    // rdist is y coordinate for the arm
    y = (float) rdist; //BAD PRACTICE IN ORIGINAL CODE: OVERWRITING A PARAMETER! Noted by David Vernon

    // Wrist position
    double wrist_z = z - D1;
    double wrist_y = y;

    // Shoulder to wrist distance (AKA sw)
    double s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
    double s_w_sqrt = sqrt(s_w);

    // s_w angle to ground
    double a1 = atan2(wrist_z, wrist_y);  // David Vernon ... alpha in notes

    // s_w angle to A3
    double a2 = (float) acos(((hum_sq - uln_sq) + s_w) / (2 * A3 * s_w_sqrt));  // David Vernon ... cast to float ... this is angle beta in notes

    // Shoulder angle
	//---------------
    double shl_angle_r = a1 + a2; // David Vernon ... theta_2 = alpha + beta in notes
    // If result is NAN or Infinity, the desired arm position is not possible

    if (std::isnan(shl_angle_r) || std::isinf(shl_angle_r))
        return false;             // David Vernon ... not a valid pose 

    double shl_angle_d = degrees(shl_angle_r);

    double a1_d = degrees(a1);    // David Vernon ... alpha in notes
    double a2_d = degrees(a2);    // David Vernon ... beta  in notes


	// Elbow angle
	//------------
	/* Changed original solution to use the negative value of the original angle                                                           */
	/* This is necessary because the shoulder angle is computed using a1 + a2 (theta_2 = alpha + beta in notes)                            */
	/* The original value is correct for the a1 - a2 (theta_2 = alpha - beta in notes)                                                     */
	/* The final servo value was correct because a negative value was used in that calculation                                             */
	/* David Vernon                                                                                                                        */
	/* 28 June 2020                                                                                                                        */           

    //double elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * A3 * A4)); // David Vernon ... cast to float
    double elb_angle_r = acos((s_w - hum_sq - uln_sq) / (2 * A3 * A4)); // Innocent ... calculating the supplementary angle
    
    // If result is NAN or Infinity, the desired arm position is not possible
    if (std::isnan(elb_angle_r) || std::isinf(elb_angle_r)) 
        return false;            // David Vernon ... not a valid pose 

    elb_angle_r = -elb_angle_r;  // David Vernon ... use negative value when shoulder angle = a1 + a2 (theta_2 = alpha + beta in notes)
    //double elb_angle_d = degrees(elb_angle_r);

    //double elb_angle_dn = -((double)180.0 - elb_angle_d);  // Commented in David Vernon's code.

    // double elb_angle_dn = -((double)180.0 + elb_angle_d);    // David Vernon ... adjusted for negative value 

    double elb_angle_dn = degrees(elb_angle_r); // Innocent ... converting the supplementary angle to degrees


    // Wrist angles
    //-------------
    /* Changed original solution by adding 90 degrees to rotation about y axis (pitch) and about z axis (roll) to ensure that              */
    /* the z axis of the wrist (i.e. the approach vector) is aligned with the z axis of the frame in base of the robot, i.e. frame Z       */  
    /* and the y axis of the wrist (i.e. the orientation vector) is aligned with the y axis of frame Z                                     */
    /* Thus, if T5 is pure translation the the wrist is pointing vertically upward and the plane of the gripper is aligned with the y axis */ 
    /* David Vernon                                                                                                                        */
	/* 24 April 2018                                                                                                                       */
                               
	/* Note that this is necessary because the kinematic specification given in M. A. Qassem, I. Abuhadrous, and H. Elaydi,                */
	/* �Modeling and Simulation of 5 DOF educational robot arm�, 2nd International Conference on Advanced Computer Control (ICACC), 2010.  */
	/* has the T5 axes aligned in different directions to the base frame                                                                   */
    /* David Vernon                                                                                                                        */
	/* 25 June 2020                                                                                                                        */         

    //float wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d; // original code
    double wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d + 90; // David Vernon ... added 90

    // if (((int)pitch_angle_d == -90) || ((int)pitch_angle_d == 90)) // original code
    if (((int) pitch_angle_d == 0))  // directed vertically up
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d + bas_angle_d + 90; // gripper orientation aligned with y axis

    }
    else if (((int) pitch_angle_d == -180) || ((int) pitch_angle_d == 180))  // directed vertically down
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d - bas_angle_d + 90; // gripper orientation aligned with y axis
    }
    else
    {
        // should really throw an exception here because this isn't correct
        wri_roll_angle_d = roll_angle_d; // original code
        wri_roll_angle_d = roll_angle_d + 90;
    }

	joint_angles[0] = bas_angle_r;
	joint_angles[1] = shl_angle_r;
	joint_angles[2] = elb_angle_r;
	joint_angles[3] = radians(wri_pitch_angle_d);
	joint_angles[4] = radians(wri_roll_angle_d);

    if (debug) {
      // printf("Joint 1 (degrees): %4.2f \n",degrees(bas_angle_r));
      // printf("Joint 2 (degrees): %4.2f \n",shl_angle_d);
      // printf("Joint 3 (degrees): %4.2f \n",elb_angle_d);
      // printf("Joint 4 (degrees): %4.2f \n",wri_pitch_angle_d);
      // printf("Joint 5 (degrees): %4.2f \n",wri_roll_angle_d);
	   printf("\n");
       printf("Joint 1 (radians): %4.2f \n",joint_angles[0]);
       printf("Joint 2 (radians): %4.2f \n",joint_angles[1]);
       printf("Joint 3 (radians): %4.2f \n",joint_angles[2]);
       printf("Joint 4 (radians): %4.2f \n",joint_angles[3]);
       printf("Joint 5 (radians): %4.2f \n",joint_angles[4]);
       printf("\n");
    }

    return true; // David Vernon ... valid pose
}


/* setJointAngles()
   
   Servo the robot by setting the joint angles.

   If using ROS, this is effected by publishing the joint angles on the /lynxmotion_al5d/joints_positions/command topic 

   If not using ROS but controlling the robot from Windows, this is effected by 
   transforming from joint angles to servo position values and writing the servo position values to the COM port
 
   David Vernon
   28 June 2020

*/

bool setJointAngles(double joint_angles[]) {

	bool debug = false; 
    int positions[6];       
    bool valid_pose;   

    if (debug) printf("setJointAngles(): angles %4.2f %4.2f %4.2f %4.2f %4.2f\n", joint_angles[0], joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4]);

#ifdef ROS

	/* ROS version: PUT ROS PUBLISHER CODE HERE */

	/* copy joint angles to the globally-accessible structure so that they can be used when constucting the topic message in the grasp() function */

	for (int i=0; i<5; i++) {
		robotConfigurationData.current_joint_value[i] = joint_angles[i];
	}

	/* now construct the topic message with all six joint values: five for the joint angles and one for the gripper */
	/* use the values in robotConfigurationData.current_joint_value[]                                               */

		ros::NodeHandle n;                        // Become a node
        ros::Publisher pub_ = n.advertise<std_msgs::Float64MultiArray>("/lynxmotion_al5d/joints_positions/command", 1000);
	ros::Rate rate(2); // Loop at 2Hz until the node is shut down

	/* Create a subscriber object */
	ros :: Subscriber sub = n.subscribe("/lynxmotion_al5d/joint_states", 1000, &jointStates);

	// MultiArray message
	std_msgs::Float64MultiArray msg;

	std::vector<float> joints_values;

	
	/* publish the message on the topic */

	joints_values = {robotConfigurationData.current_joint_value[0], robotConfigurationData.current_joint_value[1], robotConfigurationData.current_joint_value[2], robotConfigurationData.current_joint_value[3], robotConfigurationData.current_joint_value[4], robotConfigurationData.current_joint_value[5]};

	   
	   msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	   msg.layout.dim[0].size = joints_values.size();
	   msg.layout.dim[0].stride = 1;
	   msg.layout.dim[0].label = "joints";

	   msg.data.clear();
	   msg.data.insert(msg.data.end(), joints_values.begin(), joints_values.end());


	   /* Waiting for the publisher to be ready to publish the message */
	   while(pub_.getNumSubscribers()<1)
	     {
	       // waiting for a connection to publisher
	     }


	   
	   pub_.publish(msg); // publishing the message





#else

	/* Windows version */

	valid_pose = computeServoPositions(joint_angles, positions);

    if (valid_pose) {
        
       if (debug) printf("setJointAngles(): servo positions %d %d %d %d %d \n", positions[0], positions[1],  positions[2], positions[3], positions[4]);

       executeCommand(robotConfigurationData.channel, positions, robotConfigurationData.speed, 5);

       return 1;
    }
    else {
       printf("setJointAngles() error: not a valid pose for this robot\n");
       return 0;
    }

#endif 
}

/* computeServoPositions()
   
   Transform from joint angles to servo position values
 
   David Vernon
   28 June 2020

*/

bool computeServoPositions(double joint_angles[], int positions[]) {

    int homeOffset[6];  

    bool debug = false; 
    int i;

    if (debug) 
	   printf("computeServoPositions(): joint angles %4.2f %4.2f %4.2f %4.2f %4.2f\n", joint_angles[0], joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4]);

    double bas_pos;
    double shl_pos;
    double elb_pos;
    double wri_pitch_pos;
    double wri_roll_pos;

    /* Set the servo angles corresponding to joint angles for the home position */

    for (i=0; i<5; i++) {
       homeOffset[i] = (int) ((float)robotConfigurationData.home[i] / robotConfigurationData.degree[i]);
    }


    // Calculate servo positions

    bas_pos       = degrees(joint_angles[0])                  + homeOffset[0];
    shl_pos       = (degrees(joint_angles[1]) - (float) 90.0) + homeOffset[1];  // joint angle after compensating for the -90 degrees for the home configuration
    elb_pos       = (-degrees(joint_angles[2]) - (float) 90.0) + homeOffset[2];  // joint angle after compensating for the -90 degrees for the home configuration
    wri_pitch_pos = degrees(joint_angles[3])                  + homeOffset[3];
    if (robotConfigurationData.lightweightWrist == true) {
       wri_roll_pos  = - degrees(joint_angles[4])             + homeOffset[4];            
    }
    else {
       wri_roll_pos  =   degrees(joint_angles[4])             + homeOffset[4];  // roll servo rotates in reverse with the medium duty wrist attachment
    }

    //if (wri_roll_pos > 90) wri_roll_pos = wri_roll_pos - 180; // NEW CHECK
    //if (wri_roll_pos < 90) wri_roll_pos = wri_roll_pos + 180; // NEW CHECK
	
    positions[0] = (int)(bas_pos       * robotConfigurationData.degree[0]);
    positions[1] = (int)(shl_pos       * robotConfigurationData.degree[1]);
    positions[2] = (int)(elb_pos       * robotConfigurationData.degree[2]);
    positions[3] = (int)(wri_pitch_pos * robotConfigurationData.degree[3]);
    positions[4] = (int)(wri_roll_pos  * robotConfigurationData.degree[4]);

    if (debug) {
	   printf("computeServoPositions(): servo positions ");
	   for (i=0; i<=4; i++) {   
		  printf("%4d ",positions[i]);
	   }
       printf("\n");
    }

    return true;  
}

void grasp(int d) // d is distance between finger tips:  0 <= d <= 30 mm
{

  /* The gripper is controlled by servo 6
   *
   * The calibration data are stored in element 5 of the home[], degree[], and channel[] arrays
   * in the global robotConfigurationData structure
   *
   * The gripper is approximately 30mm apart (i.e. fully open) when at the servo setpoint given by home[5]
   *
   * A (closing) distance of 1 mm is given by the PW defined by degree[5] array
   * and, a gripper opening distance of d mm is given by home[5] + (30-d) * degree[5]
   */

   bool debug = true;

   int pw;

#ifdef ROS

	/* ROS version: PUT ROS PUBLISHER CODE HERE */

	/* copy the gripper distance angles to the globally-accessible structure so that it can be used when constucting the topic message in the setJointAngles() function */

   robotConfigurationData.current_joint_value[5] = ((double) d) / 1000;
   
	

	/* now construct the topic message with all six joint values: five for the joint angles and one for the gripper */
	/* use the values in robotConfigurationData.current_joint_value[]                                               */

		ros::NodeHandle n;                        // Become a node
        ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/lynxmotion_al5d/joints_positions/command", 1000);
	ros::Rate rate(2); // Loop at 2Hz until the node is shut down

	/* Create a subscriber object */
	ros :: Subscriber sub = n.subscribe("/lynxmotion_al5d/joint_states", 1000, jointStates);

	// MultiArray message
	std_msgs::Float64MultiArray msg;

	std::vector<float> joints_values;



	/* publish the message on the topic */

	joints_values = {robotConfigurationData.current_joint_value[0], robotConfigurationData.current_joint_value[1], robotConfigurationData.current_joint_value[2], robotConfigurationData.current_joint_value[3], robotConfigurationData.current_joint_value[4], robotConfigurationData.current_joint_value[5]};
	

	   msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	   msg.layout.dim[0].size = joints_values.size();
	   msg.layout.dim[0].stride = 1;
	   msg.layout.dim[0].label = "joints";

	   msg.data.clear();
	   msg.data.insert(msg.data.end(), joints_values.begin(), joints_values.end());

	   /* Waiting for the publisher to be ready to publish the message */
	   while(pub.getNumSubscribers()<1)
	     {
	       // Waiting for connection to publisher
	     }


	   pub.publish(msg); // publishing the message

	   



#else

   /* Windows version */

   pw = robotConfigurationData.home[5] + (int) (float (30-d) * robotConfigurationData.degree[5]);

   if (debug) {
      printf("grasp: %d\n",d);
     // printf("grasp: d %d  PW %d\n", d, pw);
   }
   executeCommand(robotConfigurationData.channel[5], pw, robotConfigurationData.speed * 2);   

#endif

}


int pose_within_working_env(float x, float y, float z)
{
    if((int)x <= MAX_X && (int) x > MIN_X && (int)y <= MAX_Y && (int) y > MIN_Y && (int)z <= MAX_Z && (int) z > MIN_Z ) return 1;
    else return 0;
}

#ifdef ROS
void jointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
  
    /* The messages are echoed starting from the gripper then the joints */
    joint_state_[5] = msg->position[0];
    joint_state_[0] = msg->position[1];
    joint_state_[1] = msg->position[2];
    joint_state_[2] = msg->position[3];
    joint_state_[3] = msg->position[4];
    joint_state_[4] = msg->position[5];
   
}
#endif




#ifdef COMPILE_LEGACY_VERSION

/* Arm positioning routine using inverse kinematics
 
   Z is height, Y is distance from base center out, X is side to side. Y, Z can only be positive.
   Input dimensions are for the WRIST 
   If resulting arm position is physically unreachable, return false.

   This code is an extended version of the code written by Oleg mazurov and Eric Goldsmith (see header)
   The revisions include the removal of the dependency on the end-effector (gripper) distance
   so that it computes the inverse kinematics for T5, the position and orientation of the wrist
   Also, the roll angle was added.  Minor changes have been made to the variable names; more are necessary
   to make it readable and consistent with Denavit-Hartenberg notation 

   David Vernon
   3 March 2017
   
   Audit trail
   -----------

   7 June 2018: modified to use robot configuration data reflecting the calibration of individual robots DV
   8 June 2018: fixed a number of bugs in the inverse kinematic solution
*/

bool getJointPositions(float x, float y, float z, float pitch_angle_d, float roll_angle_d, int positions[]) {
 
    int homeOffset[6];  

    bool debug = true; 
    int i;

    if (false && debug) printf("getJointPositions(): %4.1f %4.1f %4.1f %4.1f %4.1f\n", x, y, z, pitch_angle_d, roll_angle_d);

    double hum_sq;
    double uln_sq;
    double wri_roll_angle_d;
    double bas_pos;
    double shl_pos;
    double elb_pos;
    double wri_pitch_pos;
    double wri_roll_pos;

    hum_sq = A3 * A3;
    uln_sq = A4 * A4;

    /* Set the servo angles corresponding to joint angles for the home position */

    for (i=0; i<5; i++) {
       homeOffset[i] = (int) ((float)robotConfigurationData.home[i] / robotConfigurationData.degree[i]);
    }

    //grip angle in radians for use in calculations
    double pitch_angle_r = radians(pitch_angle_d);  // David Vernon uncommented 
    double roll_angle_r = radians(roll_angle_d);    // David Vernon uncommented

    // Base angle and radial distance from x,y coordinates
    double bas_angle_r = atan2(x, y);
    double bas_angle_d = degrees(bas_angle_r);

    double rdist = sqrt((x * x) + (y * y));

    // rdist is y coordinate for the arm
    y = (float) rdist; //DV BAD PRACTICE IN ORIGINAL CODE: OVERWRITING A PARAMETER!

    // Wrist position
    double wrist_z = z - D1;
    double wrist_y = y;

    // Shoulder to wrist distance (AKA sw)
    double s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
    double s_w_sqrt = sqrt(s_w);

    // s_w angle to ground
    double a1 = atan2(wrist_z, wrist_y);  // David Vernon ... alpha in notes

    // s_w angle to A3
    double a2 = (float) acos(((hum_sq - uln_sq) + s_w) / (2 * A3 * s_w_sqrt));  // David Vernon ... cast to float ... this is angle beta in notes

    // Shoulder angle
    double shl_angle_r = a1 + a2; // David Vernon ... theta_2 = alpha + beta in notes
    // If result is NAN or Infinity, the desired arm position is not possible

    if (std::isnan(shl_angle_r) || std::isinf(shl_angle_r))
        return false;             // David Vernon ... not a valid pose 

    double shl_angle_d = degrees(shl_angle_r);

    double a1_d = degrees(a1);    // David Vernon ... alpha in notes
    double a2_d = degrees(a2);    // David Vernon ... beta  in notes


	// Elbow angle
	//------------
	/* Changed original solution to use the negative value of the original angle                                                           */
	/* This is necessary because the shoulder angle is computed using a1 + a2 (theta_2 = alpha + beta in notes)                            */
	/* The original value is correct for the a1 - a2 (theta_2 = alpha - beta in notes)                                                     */
	/* The final servo value was correct because a negative value is used in that calculation                                              */
	/* David Vernon                                                                                                                        */
	/* 28 June 2020                                                                                                                        */           

    double elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * A3 * A4)); // David Vernon ... cast to float
    // If result is NAN or Infinity, the desired arm position is not possible
    if (std::isnan(elb_angle_r) || std::isinf(elb_angle_r)) 
        return false;            // David Vernon ... not a valid pose 

	elb_angle_r = -elb_angle_r;  // David Vernon ... use negative value when shoulder angle = a1 + a2 (theta_2 = alpha + beta in notes) 

    double elb_angle_d = degrees(elb_angle_r);
    //double elb_angle_dn = -((double)180.0 - elb_angle_d);  
    double elb_angle_dn = -((double)180.0 + elb_angle_d);    // David Vernon ... adjusted for negative value 


    // Wrist angles
    //-------------
    /* Changed original solution by adding 90 degrees to rotation about y axis (pitch) and about z axis (roll) to ensure that              */
    /* the z axis of the wrist (i.e. the approach vector) is aligned with the z axis of the frame in base of the robot, i.e. frame Z       */  
    /* and the y axis of the wrist (i.e. the orientation vector) is aligned with the y axis of frame Z                                     */
    /* Thus, if T5 is pure translation the the wrist is pointing vertically upward and the plane of the gripper is aligned with the y axis */ 
    /* David Vernon                                                                                                                        */
	/* 24 April 2018                                                                                                                       */
                               
	/* Note that this is necessary because the kinematic specification given in M. A. Qassem, I. Abuhadrous, and H. Elaydi,                */
	/* �Modeling and Simulation of 5 DOF educational robot arm�, 2nd International Conference on Advanced Computer Control (ICACC), 2010.  */
	/* has the T5 axes aligned in different directions to the base frame                                                                   */
    /* David Vernon                                                                                                                        */
	/* 25 June 2020                                                                                                                        */         

    //float wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d; // original code
    double wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d + 90; // David Vernon ... added 90

    // if (((int)pitch_angle_d == -90) || ((int)pitch_angle_d == 90)) // original code
    if (((int) pitch_angle_d == 0))  // directed vertically up
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d + bas_angle_d + 90; // gripper orientation aligned with y axis

    }
    else if (((int) pitch_angle_d == -180) || ((int) pitch_angle_d == 180))  // directed vertically down
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d - bas_angle_d + 90; // gripper orientation aligned with y axis
    }
    else
    {
        // should really throw an exception here because this isn't correct
        wri_roll_angle_d = roll_angle_d; // original code
        wri_roll_angle_d = roll_angle_d + 90;
    }


    // Calculate servo positions

    bas_pos       = bas_angle_d                  + homeOffset[0];
    shl_pos       = (shl_angle_d - (float) 90.0) + homeOffset[1];  // joint angle after compensating for the -90 degrees for the home configuration
    elb_pos       = (elb_angle_d + (float) 90.0) + homeOffset[2];  // joint angle after compensating for the -90 degrees for the home configuration
    wri_pitch_pos = wri_pitch_angle_d            + homeOffset[3];
    if (robotConfigurationData.lightweightWrist == true) {
       wri_roll_pos  = - wri_roll_angle_d        + homeOffset[4];            
    }
    else {
       wri_roll_pos  =   wri_roll_angle_d        + homeOffset[4];  // roll servo rotates in reverse with the medium duty wrist attachment
    }

    //if (wri_roll_pos > 90) wri_roll_pos = wri_roll_pos - 180; // NEW CHECK
    //if (wri_roll_pos < 90) wri_roll_pos = wri_roll_pos + 180; // NEW CHECK
	
    positions[0] = (int)(bas_pos       * robotConfigurationData.degree[0]);
    positions[1] = (int)(shl_pos       * robotConfigurationData.degree[1]);
    positions[2] = (int)(elb_pos       * robotConfigurationData.degree[2]);
    positions[3] = (int)(wri_pitch_pos * robotConfigurationData.degree[3]);
    positions[4] = (int)(wri_roll_pos  * robotConfigurationData.degree[4]);


    if (debug) {
       printf("x:         %5.1f \n",x);
       printf("y:         %5.1f \n",y); // DV THIS HAS BEEN OVERWRITTEN IN ORIGINAL CODE
       printf("z:         %5.1f \n",z);
       printf("pitch:     %5.1f \n",pitch_angle_d);
       printf("roll:      %5.1f \n",roll_angle_d);
       printf("Base Pos:  %5.1f \n",bas_pos);
       printf("Shld Pos:  %5.1f \n",shl_pos);
       printf("Elbw Pos:  %5.1f \n",elb_pos);
       printf("Pitch Pos: %5.1f \n",wri_pitch_pos);
       printf("Roll Pos:  %5.1f \n",wri_roll_pos);
       printf("bas angle: %5.1f \n",degrees(bas_angle_r));
       printf("shl angle: %5.1f \n",shl_angle_d);
       printf("elb angle: %5.1f \n",elb_angle_d);
       printf("Pitch d:   %5.1f \n",wri_pitch_angle_d);
       printf("Roll  d:   %5.1f \n",wri_roll_angle_d);
	   printf("positions: ");
	   for (i=0; i<=4; i++) {   
		  printf("%d ",positions[i]);
	   }
       printf("\n\n");
    }

    return true; // David Vernon ... valid pose
}


int gotoPose(float x, float y, float z, float pitch, float roll)
{
    bool debug = false; 
    int pos[6];       
    bool valid_pose;   

    if (!pose_within_working_env(x, y, z)) {
       printf("gotoPose() error: %4.1f %4.1f %4.1f not in working envelope\n", x, y, z);
       //return 0;
    }

    if (debug) printf("gotoPose(): %4.1f %4.1f %4.1f %4.1f %4.1f\n", x, y, z, pitch, roll);

    valid_pose = getJointPositions(x, y, z, pitch, roll, pos);

    if (valid_pose) {
        
       if (debug) printf("gotoPose(): %d %d %d %d %d \n", pos[0], pos[1],  pos[2], pos[3], pos[4]);

        executeCommand(robotConfigurationData.channel, pos, robotConfigurationData.speed, 5);

        return 1;
    }
    else {
       printf("gotoPose() error: not a valid pose for this robot\n");
       return 0;
    }
}

#endif

/*=======================================================*/
/* Robot configuration function                          */ 
/*=======================================================*/

void readRobotConfigurationData(char filename[]) {
      
   bool debug = false;
   int i; 
   int j;
   int k;

   keyword keylist[NUMBER_OF_KEYS] = {
      "com",
      "baud",
      "speed",
      "channel",
      "home",
      "degree",
      "effector",
      "wrist",
      "current"
   };

   keyword key;                  // the key string when reading parameters
   keyword value;                // the value string, used for the WRIST key

   char input_string[STRING_LENGTH];
   FILE *fp_config;       

   if ((fp_config = fopen(filename,"r")) == 0) {
	   printf("Error can't open robot configuration file %s\n",filename);
      prompt_and_exit(0);
   }

   /*** get the key-value pairs ***/

   for (i=0; i<NUMBER_OF_KEYS; i++) {
		
      fgets(input_string, STRING_LENGTH, fp_config);
      //if (debug)  printf ("Input string: %s",input_string);

      /* extract the key */

      sscanf(input_string, " %s", key);

      for (j=0; j < (int) strlen(key); j++)
         key[j] = tolower(key[j]);
       
      //if (debug)  printf ("key: %s\n",key);

      for (j=0; j < NUMBER_OF_KEYS; j++) {
         if (strcmp(key,keylist[j]) == 0) {
            switch (j) {
            case 0:  sscanf(input_string, " %s %s", key, robotConfigurationData.com);     // com  
                     break;
            case 1:  sscanf(input_string, " %s %d", key, &(robotConfigurationData.baud));  // baud
                     break;
            case 2:  sscanf(input_string, " %s %d", key, &(robotConfigurationData.speed)); // speed
                     break;
            case 3:  sscanf(input_string, " %s %d %d %d %d %d %d", key,                    // channel
                                                                   &(robotConfigurationData.channel[0]), 
                                                                   &(robotConfigurationData.channel[1]), 
                                                                   &(robotConfigurationData.channel[2]), 
                                                                   &(robotConfigurationData.channel[3]), 
                                                                   &(robotConfigurationData.channel[4]), 
                                                                   &(robotConfigurationData.channel[5]));
                     break;
            case 4:  sscanf(input_string, " %s %d %d %d %d %d %d", key,                    // home
                                                                   &(robotConfigurationData.home[0]), 
                                                                   &(robotConfigurationData.home[1]), 
                                                                   &(robotConfigurationData.home[2]), 
                                                                   &(robotConfigurationData.home[3]), 
                                                                   &(robotConfigurationData.home[4]), 
                                                                   &(robotConfigurationData.home[5]));                 
                     break;
            case 5:  sscanf(input_string, " %s %f %f %f %f %f %f", key,                    // degree
                                                                   &(robotConfigurationData.degree[0]), 
                                                                   &(robotConfigurationData.degree[1]), 
                                                                   &(robotConfigurationData.degree[2]), 
                                                                   &(robotConfigurationData.degree[3]), 
                                                                   &(robotConfigurationData.degree[4]), 
                                                                   &(robotConfigurationData.degree[5]));  
                     break;
            case 6:  sscanf(input_string, " %s %d %d %d", key,                             // effector
                                                          &(robotConfigurationData.effector_x),  
                                                          &(robotConfigurationData.effector_y), 
                                                          &(robotConfigurationData.effector_z));                                                    
                     break;
            case 7:  sscanf(input_string, " %s %s ", key, value);                          // wrist
                     for (j=0; j < (int) strlen(value); j++)
                        value[j] = tolower(value[j]);
                     if (strcmp(value,"lightweight")==0) {
                        robotConfigurationData.lightweightWrist = true;
                     }
                     else {
                        robotConfigurationData.lightweightWrist = false;
                     }
                     break;
		     case 8: sscanf(input_string, " %s %f %f %f %f %f %f", key,                    // current_joint_value
                                                                   &(robotConfigurationData.current_joint_value[0]), 
                                                                   &(robotConfigurationData.current_joint_value[1]), 
                                                                   &(robotConfigurationData.current_joint_value[2]), 
                                                                   &(robotConfigurationData.current_joint_value[3]), 
                                                                   &(robotConfigurationData.current_joint_value[4]), 
                                                                   &(robotConfigurationData.current_joint_value[5]));  
                     break;

            }
         }
      }
   }

   if (debug) { 
      printf("COM:      %s\n",robotConfigurationData.com);
      printf("BAUD:     %d\n",robotConfigurationData.baud);
      printf("SPEED:    %d\n",robotConfigurationData.speed);
      printf("CHANNEL:  "); for (k=0; k<6; k++) printf("%d ", robotConfigurationData.channel[k]);   printf("\n");
      printf("HOME:     "); for (k=0; k<6; k++) printf("%d ", robotConfigurationData.home[k]);      printf("\n");
      printf("DEGREE:   "); for (k=0; k<6; k++) printf("%3.1f ", robotConfigurationData.degree[k]); printf("\n");
      printf("EFFECTOR: "); printf("%d %d %d \n", robotConfigurationData.effector_x, robotConfigurationData.effector_y, robotConfigurationData.effector_z);
      printf("WRIST:    "); printf("%s \n", value);
      printf("CURRENT:  "); for (k=0; k<6; k++) printf("%4.3f ", robotConfigurationData.current_joint_value[k]); printf("\n");
   }
}


/******************************************************************************
   
   Serial port interface 

   Based on code written by Victor Akinwande, Carnegie Mellon University Africa
   
   Modified by: David Vernon, Carnegie Mellon University Africa

*******************************************************************************/

/*
 *  intialize the port, baud rate, and speed global variables
 */


void goHome() {

    executeCommand(robotConfigurationData.channel, robotConfigurationData.home, robotConfigurationData.speed, 6);
}


/* execute command for multiple servo motors */

void executeCommand(int *channel, int *pos, int speed, int number_of_servos) {

    char command[COMMAND_SIZE] = {0};

    for(int i =0; i< number_of_servos; i++) {

        char temp[COMMAND_SIZE] = {0};
        sprintf(temp, " #%dP%d", channel[i], pos[i]); // David Vernon ... added space before #; without this port 0 is not affected

        strcat(command, temp);

        sprintf(temp, "S%d", robotConfigurationData.speed); // David Vernon ... append the speed argument to each servo command 
        strcat(command, temp);                              // David Vernon

        strcat(command, " ");
  
    }
     
    sendToSerialPort(command);
}


/* execute command for single servo motor */

void executeCommand(int channel, int pos, int speed) {

    char command[200];

    sprintf(command, " #%dP%dS%d ", channel, pos, speed); // David Vernon ... added space before # ... without this port 0 is not affected
                                                          // also removed the <CR> after the command
    sendToSerialPort(command);
}

/* send the command to the serial port with the echo OS primitive */
/* do this with the system() function                             */

void sendToSerialPort(char *command)
{
    bool debug = false;  
    char execcommand[COMMAND_SIZE];

    if (debug && false) printf("execute(): %s \n", command);

    sprintf(execcommand, "echo \"%s\" > %s", command, robotConfigurationData.com);

    if (debug) printf("%s\n", execcommand);
        
    system(execcommand);

    wait(DEFAULT_SLEEP_TIME); 
}

//helper methods

/**
 * Print failure message and return
 * @param message, output message
 */

void fail(char *message)
{
    printf("%s\n", message);
    printf("Enter any character to finish >>");
    getchar();
    exit(1);
}


/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/


void display_error_and_exit(const char *error_message) {
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(0);
}


void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}


void wait(int ms)
{
#ifdef ROS
    usleep(ms * 1000);
#else
    Sleep(ms);
#endif
}

void print_message_to_file(FILE *fp, char message[]) {
   fprintf(fp,"The message is: %s\n", message);
}

#ifdef ROS
int spawn_brick(std::string name, std::string color, double x, double y, double z, double phi) {

    bool debug = false;

    if (debug) {
       printf("spawn_brick:  %s with color %s at position (%.2f %.2f %.2f %.2f)\n",
	      name.c_str(), color.c_str(), x, y, z, phi);
    }
  
    // The values are expected to be in mm and degrees
    ros::NodeHandle nh;
    ros::service::waitForService("/lynxmotion_al5d/spawn_brick");
    ros::ServiceClient client = nh.serviceClient<lynxmotion_al5d_description::SpawnBrick>("/lynxmotion_al5d/spawn_brick");
    lynxmotion_al5d_description::SpawnBrick srv;

    srv.request.name  = name;
    srv.request.color = color;
    srv.request.pose.position.x = x / 1000.0;
    srv.request.pose.position.y = y / 1000.0;
    srv.request.pose.position.z = z / 1000.0;
    srv.request.pose.orientation.yaw = radians(phi);

    if (client.call(srv))
    {
      if (debug) ROS_INFO("Spawned brick [%s] of color [%s] at position (%.3f %.3f %.3f %.2f %.3f %.3f)", srv.response.name.c_str(), color.c_str(), (x/1000.0), (y/1000.0), (z/1000.0), 0.0, 0.0, radians(phi));
      wait(1000);
    }
    else
    {
        ROS_ERROR("Failed to call the service");
        return 1;
    }
    
    return 0;
}    

int kill_brick(std::string name) {

    bool debug = false;

    if (debug) {
       printf("kill_brick: %s\n",name.c_str());
    }
  
    ros::NodeHandle nh;
    ros::service::waitForService("/lynxmotion_al5d/kill_brick");
    ros::ServiceClient client = nh.serviceClient<lynxmotion_al5d_description::KillBrick>("/lynxmotion_al5d/kill_brick");
    lynxmotion_al5d_description::KillBrick srv;

    srv.request.name  = name.c_str();

    if (client.call(srv))
    {
        if (debug) ROS_INFO("Killed brick [%s]", name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call the service");
        return 1;
    }
    
    return 0;
}    

#endif


void drawCrossHairs(Mat hough,  int x, int y, int size, int red, int green, int blue, int lineWeight) {

   Point pt1, pt2;

   pt1.x = cvRound(x + size/2);
   pt1.y = cvRound(y);
   pt2.x = cvRound(x - size/2);
   pt2.y = cvRound(y);
   line( hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);

   pt1.x = cvRound(x);
   pt1.y = cvRound(y + size/2);
   pt2.x = cvRound(x);
   pt2.y = cvRound(y - size/2);
   line( hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);
}

void drawArrowedLine(Mat hough, int i, int j, float magnitude_value, float phase_value, int red, int green, int blue, int lineWeight) {

   Point pt1, pt2;
   double theta;
   double i_offset;
   double j_offset;
   int i2, j2;
   int i3, j3;
  
   i_offset = magnitude_value * cos(phase_value);
   j_offset = magnitude_value * sin(phase_value);

   i2 = i + (int)(i_offset);
   j2 = j + (int)(j_offset);

   if ((i2 >= 0) && (i2 < hough.cols) && (j2 >= 0) && (j2 < hough.rows)) {

      pt1.x = i;
      pt1.y = j;
      pt2.x = i2;
      pt2.y = j2;
   
      line(hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);

      /* add arrow head */

      theta = phase_value + 3.14159 + ARROW_HEAD_ANGLE;
      i_offset = ARROW_HEAD_SIZE * cos(theta);
      j_offset = ARROW_HEAD_SIZE * sin(theta);

      i3 = i2 + (int)(i_offset);
      j3 = j2 + (int)(j_offset);

      pt1.x = i2;
      pt1.y = j2;
      pt2.x = i3;
      pt2.y = j3;

      line(hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);

      theta = phase_value + 3.14159 - ARROW_HEAD_ANGLE;
      i_offset = ARROW_HEAD_SIZE * cos(theta);
      j_offset = ARROW_HEAD_SIZE * sin(theta);;

      i3 = i2 + (int)(i_offset);
      j3 = j2 + (int)(j_offset);

      pt1.x = i2;
      pt1.y = j2;
      pt2.x = i3;
      pt2.y = j3;

      line(hough, pt1, pt2, Scalar((double)blue,(double)green,(double)red), lineWeight, CV_AA);
   }
}



float  lineLength(Point2f p1, Point2f p2) {

   return(sqrt( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)));
}


/**************************************************************************************************************

Assignment 5 Code

**************************************************************************************************************/


void inversePerspectiveTransformation(Point2f image_sample_point,
                                      float camera_model[][4], 
                                      float z,
                                      Point3f *world_sample_point) {

   bool debug = false;
   int i, j;

   float a1, b1, c1, d1;
   float a2, b2, c2, d2;
   float x, y;


   if (false && debug) {
      printf("Camera model\n");
      for (i=0; i<3; i++) {
         for (j=0; j<4; j++) {
            printf("%f ", camera_model[i][j]);
         }
         printf("\n");
      }
      printf("\n");
      printf("Image point:  %f, %f \n\n", image_sample_point.x, image_sample_point.y);
   }

   a1 = camera_model[0][0] - image_sample_point.x * camera_model[2][0];
   b1 = camera_model[0][1] - image_sample_point.x * camera_model[2][1];
   c1 = camera_model[0][2] - image_sample_point.x * camera_model[2][2];
   d1 = camera_model[0][3] - image_sample_point.x * camera_model[2][3];
  
   a2 = camera_model[1][0] - image_sample_point.y * camera_model[2][0];
   b2 = camera_model[1][1] - image_sample_point.y * camera_model[2][1];
   c2 = camera_model[1][2] - image_sample_point.y * camera_model[2][2];
   d2 = camera_model[1][3] - image_sample_point.y * camera_model[2][3];
  

   /* inverse perspective solution for a given z value  */


   x = (z * (b1*c2 - b2*c1) + (b1*d2 - b2*d1)) / (a1*b2 - a2*b1);
   y = (z * (a2*c1 - a1*c2) + (a2*d1 - a1*d2)) / (a1*b2 - a2*b1); 

   world_sample_point->x = x;
   world_sample_point->y = y;
   world_sample_point->z = z;

   if (debug) {

      printf("(%3d, %3d) -> (%4.1f, %4.1f, %4.1f)\n", (int) image_sample_point.x,  (int) image_sample_point.y,  
                                                              world_sample_point->x, world_sample_point->y, world_sample_point->z);

   }
}

void rgb2hsi(unsigned char red, unsigned char green, unsigned char blue, float *hue, float *saturation, float *intensity){
    double y, h, h_star, c, c1, c2,  s, r, g, b;
    int min = 256;

    //  0 <= hue <= 2 pi
    //  0 <= saturation <= 1

    r = (float) red   / 256;
    g = (float) green / 256;
    b = (float) blue  / 256;

    y  = 0.2125 * r + 0.7154 * g + 0.0721 * b;
    c1 =          r - 0.5    * g - 0.5    * b;
    c2 =            - 0.8660 * g + 0.8660 * b;


    // chroma c: [0,1]

    c = sqrt(c1*c1 + c2*c2);


    // hue h: [0,360]

    if (c == 0) { // h and s are undefined
       *hue        = (float) 0;
       *saturation = (float) 0;
    }
    else {
      if(c2 <= 0) {
         h = acos (c1/c);
      }
      else {
         h = 2*3.14159  - acos (c1/c);
      }

       h = 360 * (h / (2 * 3.14159)); // convert to degrees


      // saturation: [0,1]

      h_star =  (int) h - (int) (60 * (  ((int) h) / 60));  // convert to interval 0,60


       s = (2 * c * sin( 2 * 3.14159 * ((120 - h_star) / 360.0))) / 1.73205;

      //*hue        = (float)  ((h / 360) * 2 * 3.14159); // convert to radians ... for the moment anyway
      *hue        = (float)  h;
      *saturation = (float)  s;
   }

     *intensity  = (float)  (r+g+b)/3;

  // printf("rgb2hsi: (%d, %d, %d) -> (%3.1f, %3.1f, %3.1f)\n", red, green, blue, *hue, *saturation, *intensity);

}

/**
 * 
 * Extracting the countours of the objects in the picture
 * This function retours vector of vectors of points which is the container of all coutours found int the picture.
 * This function is the edited version taken from the notes Introduction Cognitive Robotics Prof. David Vernon.
 * 
*/

void ContourExtraction(cv::Mat src, std::vector<std::vector<cv::Point>> *contours, int thresholdValue)
{
    cv::Mat src_gray;
    cv::Mat src_blur;
    cv::Mat detected_edges;
    
    bool debug = false;
    int ratio = 3;
    int kernel_size = 3;
    int filter_size;
    
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat thresholdedImage;
    filter_size = kernel_size * 4 + 1;
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY) ;
    // multiplier must be even to ensure an odd filter size as required by OpenCV
    // this places an upper limit on gaussian_std dev of 7 to ensure the filter size < 31
    // which is the maximum size for the Laplacian operator
    GaussianBlur(src_gray, src_blur, cv::Size(filter_size,filter_size), kernel_size);
    Canny(src_gray, detected_edges,thresholdValue, thresholdValue*ratio, kernel_size ) ;

    
    cv::Mat canny_edge_image_copy = detected_edges.clone();
    // clone the edge image because findContours overwrites it
    /* see http://docs.opencv.org/2.4/modu1es/imgproc/doc/structura1 analysis and shape descriptors. html#findcontours */
    /* and http://docs.opencv.org/2.4/doc/tutoria1s/imgproc/shapedescriptors/find contours/ find contours. html */
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    findContours (canny_edge_image_copy, *contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat contours_image = cv::Mat::zeros(src.size(), CV_8UC3);
    if(debug)
    {
        imshow("Gray", src_gray);
        imshow("Image brur", src_blur);
        imshow("Image detected_edges", detected_edges);
    }
}

/**
 * Calculating distance between two points using two x,y coordinates.
 * Return the distance between two points
 **/
void distance(int x1, int y1, int x2, int y2, int *distance)
{
   *distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}
/**
 * Getting the center of all object found in the picture
 * Some of the object are ignored since I was tracking brick that are in the range of 3000 area
 *
*/
void getCenter(cv::Mat *src, std::vector<std::vector<cv::Point>> contours,std::vector<cv::Point> *centers, std::vector<cv::Point> *points_on_arcLine){
    int centerX, centerY;
    bool debug = false;
    int distance_1, distance_2;
    std::vector<std::vector<cv::Point>> conPoints(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    cv::Point point_0, point_1, point_2, point_3;
    
    int area;
    float arc;
    
    for (int contour_number=0; (contour_number<(int)contours.size()); contour_number++) {
        cv::Scalar colour( rand()&0xFF, rand()&0xFF, rand()&0xFF ) ;    // use a random colour for each contour
        // calculating the area of the contours
        area = contourArea(contours[contour_number]);
        // calculating the perimeter of the contours function from openCV
        arc = arcLength(contours[contour_number], true);
        // Approximates a polygonal curve(s) with the specified precision.
        approxPolyDP(contours[contour_number], conPoints[contour_number], 0.03 * arc, true);
        boundRect[contour_number] = boundingRect(conPoints[contour_number]);
        if(debug) std::cout << "areas: " << area;
        if(area >= 2000) // Ignore detected contours that may be small which means they are not bricks. since the brick area is around 3000
        {
            // drawing bounding rectangle on each object in the picture.
            // rectangle(*src, boundRect[contour_number].tl(), boundRect[contour_number].br(), colour, 1);
            point_0 = cv::Point(conPoints[contour_number][0].x  , conPoints[contour_number][0].y);
            point_1 = cv::Point(conPoints[contour_number][1].x  , conPoints[contour_number][1].y);
            point_2 = cv::Point(conPoints[contour_number][2].x  , conPoints[contour_number][2].y);
            point_3 = cv::Point(conPoints[contour_number][3].x  , conPoints[contour_number][3].y);
            
            centerX = point_0.x/2 - point_2.x/2 + point_2.x;
            centerY = point_0.y/2 - point_2.y/2 + point_2.y;
            
            if(debug)
            {
                line(*src, cv::Point(centerX - 10.0, centerY), cv::Point(centerX + 10.0, centerY), cv::Scalar(0, 255, 255), 1);
                line(*src, cv::Point(centerX, centerY - 10.0), cv::Point(centerX, centerY + 10.0), cv::Scalar(0, 255, 255), 1);
                line(*src, point_0, point_1, cv::Scalar(0, 0, 0), 5);
                line(*src, point_1, cv::Point(point_1.x + centerX, point_1.y + centerY), cv::Scalar(255, 0, 0), 2);
            }
            // adding a center in vector of points
            centers->push_back(cv::Point(centerX, centerY));
            
            if(debug) std::cout << "Center x_0: " << centerX << " Center y_0: " << centerY<< std::endl;
            distance(point_0.x, point_0.y, point_1.x, point_1.y, &distance_1);
            distance(point_1.x, point_1.y, point_2.x, point_2.y, &distance_2);
            // check the smaller side on the found brick in order to identify the orientation.
            if(distance_1 < distance_2)
            {
                points_on_arcLine->push_back(cv::Point((point_0.x + point_1.x) / 2, (point_0.y + point_1.y)/2));
                if(debug)
                    std::cout<<"Line " << contour_number + 1 << cv::Point(centerX, centerY)<< " ===> " << cv::Point((point_0.x + point_1.x) / 2, (point_0.y + point_1.y)/2) <<std::endl;
            }else{
                points_on_arcLine->push_back(cv::Point((point_0.x + point_3.x) / 2, (point_0.y + point_3.y)/2));
                if(debug)
                    std::cout<<"Line " << contour_number + 1 << cv::Point(centerX, centerY)<< " ===> " << cv::Point((point_0.x + point_3.x) / 2, (point_0.y + point_3.y)/2) <<std::endl;
            }
            if(debug)
            {
                line(*src, point_0, point_1, cv::Scalar(0, 255, 255), 1);
                line(*src, point_1, point_2, cv::Scalar(0, 0, 0), 1);
            }
        }
    }
}
/**
 *  converting from radian to degree
 */
void radToDeg(float rad, int *deg) 
{
    *deg = rad * (180.0/M_PI);
}
/**
 *  converting from degree to radian
 */
float degToRad(int deg)
{
    return deg * (M_PI/180);
}
/**
 * getting the angle between two point and the horizontal.
 */
void getAngle(cv::Point point, cv::Point center, int *thetha)
{
    float deltaY, deltaX, rad;
    deltaY = point.y - center.y;
    deltaX = point.x - center.x;
    rad = abs(atan2(deltaY, deltaX));
    radToDeg(rad, thetha);
}
void hueMagnitude(float *hue)
{
    if(*hue > 180) *hue = 360 - *hue;
}
void getRGB(cv::Mat img, int x, int y, unsigned char *red, unsigned char *green, unsigned char *blue) {
    bool debug = true;
    cv::Vec3b color = img.at<cv::Vec3b>(cv::Point(x,y));
    *red   = color[2];
    *green = color[1];
    *blue  = color[0];
    if(debug)
    {
        printf("RGB( %d, %d, %d ) \n", (int) color[2], (int) color[1], (int) color[0]);
    }
}
bool smallHue(PIC_VALUES pv1, PIC_VALUES pv2)
{
    return (pv1.hue < pv2.hue);
}

void pick_and_place(float object_x, float object_y, float object_z, float object_phi, 
                    float destination_x, float destination_y, float destination_z, float destination_phi,
                    float grasp_x, float grasp_y, float grasp_z, float grasp_theta) {
    /* now start the pick and place task */
    /* --------------------------------- */

    float effector_length = (float) robotConfigurationData.effector_z; // initialized from robot configuration data
    float initial_approach_distance = 20;
    float final_depart_distance     = 20;
    float delta = 2;

    float approach_distance;
    float depart_distance;
  
    Frame E               = trans(0.0, 0.0, effector_length);                                           // end-effector (gripper) frame 
    Frame Z               = trans(0.0 ,0.0, 0.0);                                                       // robot base frame
    Frame object          = trans(object_x,      object_y,      object_z)      * rotz(object_phi);      // object pose
    Frame destination     = trans(destination_x, destination_y, destination_z) * rotz(destination_phi); // destination pose
    Frame object_grasp    = trans(grasp_x,       grasp_y,       grasp_z)       * roty(grasp_theta);     // object grasp frame w.r.t. both object and destination frames
    Frame object_approach = trans(0,0,-initial_approach_distance);                                      // frame defined w.r.t. grasp frame
    Frame object_depart   = trans(0,0,-final_depart_distance);                                          // frame defined w.r.t. grasp frame
 
    bool debug = true;
    bool continuous_path = true;

    /* open the gripper */
    /* ----------------- */

    if (debug) printf("Opening gripper\n");
   
    grasp(GRIPPER_OPEN);

#ifdef ROS
    wait(5000); // wait to allow the simulator to go to the home pose before beginning
                // we need to do this because the simulator does not initialize in the home pose
#endif

   
    /* move to the pick approach pose */
    /* ------------------------------ */
   
    if (debug) printf("Moving to object approach pose\n");

    Frame T6 = inv(Z) * object * object_grasp * object_approach * inv(E);

    if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;

    wait(2000); 


    if (continuous_path) {
        
        /* incrementally decrease the approach distance */
     
        approach_distance = initial_approach_distance - delta;
   
        while (approach_distance >= 0) {
            
            object_approach = trans(0,0,-approach_distance);
	 
            T6 = inv(Z) * object * object_grasp * object_approach * inv(E);
	 
            if (move(T6) == false) display_error_and_exit("move error ... quitting\n");  

            approach_distance = approach_distance - delta;                              
        }
    }

   
    /* move to the pick pose */
    /* --------------------- */
      
    if (debug) printf("Moving to object pose\n");

    T6 = inv(Z) * object * object_grasp * inv(E);

    if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

    wait(1000); 

   
    /* close the gripper */
    /* ----------------- */

    if (debug) printf("Closing gripper\n");
    
    grasp(GRIPPER_CLOSED);

    wait(1000);

           
    /* move to pick depart pose */
    /* ------------------------ */

    if (debug) printf("Moving to object depart pose\n");

    if (continuous_path) {
        
        /* incrementally increase depart distance */

        depart_distance = delta;
      
        while (depart_distance <= final_depart_distance) {
            
            object_depart   = trans(0,0,-depart_distance);

            T6 = inv(Z) * object * object_grasp * object_depart * inv(E);          
                                                                              
            if (move(T6) == false) display_error_and_exit("move error ... quitting\n"); 
            
            depart_distance = depart_distance + delta;
        }
    }

   
    T6 = inv(Z) * object * object_grasp * object_depart * inv(E);

    if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;

    wait(2000);

   
    /* move to destination approach pose */
    /* --------------------------------- */

    if (debug) printf("Moving to destination approach pose\n");

    object_approach = trans(0,0,-initial_approach_distance);
 
    T6 = inv(Z) * destination * object_grasp * object_approach * inv(E);

    if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

    wait(2000);

      
    /* move to the destination pose */
    /* ---------------------------- */

    if (debug) printf("Moving to destination pose\n");
      
    if (continuous_path) {
        
        /* incrementally decrease approach distance */

        approach_distance = initial_approach_distance - delta;
   
        while (approach_distance >= 0) {
            
            object_approach   = trans(0,0,-approach_distance);
	
            T6 = inv(Z) * destination * object_grasp * object_approach * inv(E);   
                                                                                  
            if (move(T6) == false) display_error_and_exit("move error ... quitting\n");  

            approach_distance = approach_distance - delta;
        }
    }


    T6 = inv(Z) * destination * object_grasp * inv(E);

    if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;
 
    wait(1000);

   
    /* open the gripper */
    /* ---------------- */

    if (debug) printf("Opening gripper\n");

    grasp(GRIPPER_OPEN);     
    wait(1000); 

   
    /* move to depart pose */
    /* ------------------- */

    if (debug) printf("Moving to destination depart pose\n");

    if (continuous_path) {
        
        depart_distance = delta;
        
        while (depart_distance <= final_depart_distance) {
            
            object_depart   = trans(0,0,-depart_distance);

            T6 = inv(Z) * destination * object_grasp * object_depart * inv(E);          
                                                                              
            if (move(T6) == false) display_error_and_exit("move error ... quitting\n"); 
            
            depart_distance = depart_distance + delta;
        }
    }
   
    object_depart   = trans(0,0,-final_depart_distance);
      
    T6 = inv(Z) * destination * object_grasp * object_depart * inv(E);

    if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

    wait(1000);

    
    goHome(); // this returns the robot to the home position so that when it's switched off 
              // it's in a pose that is close to the one that the servo-controller uses as its initial state
              // could also do this with a move() as show above
}