/*******************************************************************************************************************
*   Assignment 5: vision-based pick-and-place program for a LynxMotion AL5D robot arm to stack three bricks
*   ----------------------------------------------------------------------------------------------------------------
*
*   This application implements a robot control program for the LynxMotion AL5D robot arm to pick up three coloured bricks,
*   one red, one green, one blue, and stack them on top of each other, with the red brick on top and the blue brick at the base.
*
*   The filename of the robot configuration file is specificed on the first line of the input file.
*   The filename of the camera model file is specified on the second line of the input file.
*   The (x, y, z) position and orientation phi of the first brick in the stack is specified on the third line of the input file.
*
*   It is assumed that the x axis of the brick is aligned with its major axis. 
*   It is also assumed that the dimensions of the brick in the x, y, and z directions are 31.8 mm,  15.8 mm, 11.4 mm respectively. 
* 
*   All poses – T5, gripper, and bricks – are specified in a Cartesian frame of reference using frames (i.e. homogenous transformations).
*
*   David Vernon
*   5 April 2021
*
*   Audit Trail
*   -----------
*   Generated a amuhizi skeleton application by removing the core processing, i.e. brick segmentation, pose estimation in 2D,
*   and pose estimation in 3D using the inverse perspective transformation
*   David Vernon 8 April 2021
*
*******************************************************************************************************************/

#include <stdlib.h>
#include <time.h>
#ifdef WIN32
    #include "amuhizi.h"
#else
    #include <assignment5/amuhizi.h>
#endif


int main(int argc, char ** argv) {

   /* initialize the ROS node  */
   /* ------------------------ */

   /* we do this first in case one of the classes creates a node handle when the object is instantiated */

   #ifdef ROS
       ros::init(argc, argv, "amuhizi"); // Initialize the ROS system
   #endif

   /* now come the declarations */

   extern robotConfigurationDataType robotConfigurationData;
   
   bool debug = true;
   
   FILE *fp_in;                    // assignment 5 input file
   FILE *fp_camera_model;          // camera model file
   int  end_of_file; 
   char robot_configuration_filename[MAX_FILENAME_LENGTH];
   char camera_model_filename[MAX_FILENAME_LENGTH] = {};
   char filename[MAX_FILENAME_LENGTH] = {};
   char directory[MAX_FILENAME_LENGTH] = {};

   const char* scene_window_name  = "Simulator Image";

   /* Frame objects */
   
   Frame brick1;
   Frame brick2;
   Frame brick3;
   Frame destination;
   Frame E;
   Frame T5;
   Frame Z;
   Frame home;
   Frame out_of_view;

   /* camera object for simulator */

   SimulatorCamera camera;

   /* image */

   Mat frame;
   cv::Mat src;
   cv::Mat src_bgr;
   cv::Mat srcGray;

   std::vector<std::vector<cv::Point>> contours;
   int theta = 0;
   std::vector<cv::Point> centers;
   std::vector<cv::Point> arcLine_points;
   unsigned char red, green, blue;
   float hue, saturation, intensity;

   /* camera model */

   float camera_model[3][4];

   /* data variables */
      
   float destination_x;                    
   float destination_y;
   float destination_z;
   float destination_phi;

   float brick_height = 11;
   
   bool continuous_path = false;    // if true, implement approximation of continuous path control
                                    // when approaching and departing the grasp pose
                                    // otherwise just move directly from the initial approach pose to the grasp pose
                                    // and directly from the grasp pose to the final depart pose
   
   int number_of_bricks = 0;        // number of brick detected in the scene

   int i, j;


   
   /* open the input file */
   /* ------------------- */

   /* Set the filename; different directories for ROS and Windows versions */
   
#ifdef ROS
   strcat(directory, (ros::package::getPath(ROS_PACKAGE_NAME) + "/data/").c_str());
#else
   strcat(directory, "../data/"); // On Windows the exec is in bin, so we go in the parent directory first
#endif

   strcpy(filename, directory);
   strcat(filename, "assignment5Input.txt"); // Input filename matches the application name

   if ((fp_in = fopen(filename, "r")) == 0) {
      printf("Error can't open input %s\n",filename);
      prompt_and_exit(0);
   }

   
   /* get the robot configuration data */
   /* -------------------------------- */

   end_of_file = fscanf(fp_in, "%s", filename); // read the configuration filename   

   if (end_of_file == EOF) {   
     printf("Fatal error: unable to read the robot configuration filename %s\n",filename);
      prompt_and_exit(1);
   }

   if (debug) printf("Robot configuration filename %s\n", filename);

   strcpy(robot_configuration_filename, directory);
   strcat(robot_configuration_filename, filename);

   readRobotConfigurationData(robot_configuration_filename); // there is a dedicated function to read the configuration data
                                                             // and store it in a global structure
                                                            
   
   /* get the camera model data  */
   /* -------------------------- */

   end_of_file = fscanf(fp_in, "%s", filename); // read the configuration filename   

   if (end_of_file == EOF) {   
      printf("Fatal error: unable to read the camera model filename, %s\n", filename);
      prompt_and_exit(1);
   }

   if (debug) printf("Camera model filename %s\n", filename);

   strcpy(camera_model_filename, directory);
   strcat(camera_model_filename, filename);

   if ((fp_camera_model = fopen(camera_model_filename, "r")) == 0) {
      printf("Error can't open input %s\n",camera_model_filename);
      prompt_and_exit(0);
   }

   for (i=0; i<3; i++) {
      for (j=0; j<4; j++) {
         fscanf(fp_camera_model, "%f ", &(camera_model[i][j]));
      }
   }

   if (debug) {
      printf("Camera model\n");
      for (i=0; i<3; i++) {
         for (j=0; j<4; j++) {
            printf("%6.3f ", camera_model[i][j]);
         }
	 printf("\n");
      }
      printf("\n");
   }

   fclose(fp_camera_model);  

   
   /* get the destination pose data */
   /* ----------------------------- */

   end_of_file = fscanf(fp_in, "%f %f %f %f", &destination_x, &destination_y, &destination_z, &destination_phi);

   if (end_of_file == EOF) {   
	  printf("Fatal error: unable to read the destination position and orientation\n");
      prompt_and_exit(1);
   }
   
   if (debug) printf("Destination pose %6.3f %6.3f %6.3f %6.3f\n", destination_x, destination_y, destination_z, destination_phi);

   /* done reading the input file so close it */
   
   fclose(fp_in);


      
   /* now start the pick and place task */
   /* --------------------------------- */


   E    = trans((float) robotConfigurationData.effector_x,       // end-effector (gripper) frame
		(float) robotConfigurationData.effector_y,       // is initialized from data
		(float) robotConfigurationData.effector_z);      // in the robot configuration file
   
   Z    = trans(0.0, 0.0, 0.0);                                  // robot base frame

   /* move to the home pose                                                   */
   /*                                                                         */
   /* Define the home pose with the wrist T6 positioned over theworls  Y axis */
   /* so that link a3 is vertical and link a4 is  horizontal                  */
   /* with the approach vector directed along the world  Y axis               */
   /* and the orientation vector directed aloing the the X axis               */
   /* thus, the normal vector is directed in the - Z direction                */
   
   home = trans(0, 187, 216) *  roty(90.0) * rotx(-90);
   
   T5   = inv(Z) * home; // no need to include inv(E) here since we are specifying the wrist pose directly

   if (move(T5) == false) display_error_and_exit("move error ... quitting\n");

   wait(3000);

   /* move out of the field of view of the camera                                    */
   /*                                                                                */
   /* Define the out_of_view pose with the wrist T6 positioned over the world X axis */
   /* so that link a3 is vertical and link a4 is  horizontal                         */
   /* with the approach vector directed along the world  -Z axis                     */
   /* and the orientation vector directed in the  with the -Y axis                   */
   /* thus, the normal vector is directed in the - X direction                       */
   
   out_of_view = trans(187, 0, 216) * roty(180.0);

   T5   = inv(Z) * out_of_view; // no need to include inv(E) here since we are specifying the wrist pose directly

   if (move(T5) == false) display_error_and_exit("move error ... quitting\n");

   wait(3000);


   /* create a window to see the acquired image */
   
   namedWindow(scene_window_name, CV_WINDOW_AUTOSIZE);
   waitKey(30);
   moveWindow(scene_window_name, 0, 0);
   waitKey(30);

   /* initialize the camera */

   camera.initialize();
   
   /* acquire an image */

   camera.getImage(frame);
   imshow (scene_window_name, frame);
   waitKey(3000);  // three second delay so that we can see the image before the window is destroyed.
          
   destroyWindow(scene_window_name);

   
   /* analyze the image to determine the pose of the three bricks in the world or robot frame of reference */
   /* sort the poses by increasing value of  hue                                                           */
   /* ---------------------------------------------------------------------------------------------------- */

   
   /* insert your code here */

   std::vector<PIC_VALUES> pic_vals;
   src = frame; // cv::imread(path + endString.at(0));
   src_bgr = frame; // cv::imread(path + endString.at(0));
   ContourExtraction(src, &contours, 150);
   getCenter(&src, contours, &centers, &arcLine_points);
   
   if (debug) printf("Number of contours %lu: \n", contours.size());
   for(int index = 0; index < centers.size(); index++)
   {
      getAngle(arcLine_points.at(index), centers.at(index), &theta);
      drawCrossHairs(src, centers.at(index).x, centers.at(index).y, 10, 255, 255, 0, 1);
      drawArrowedLine(src, centers.at(index).x, centers.at(index).y, 90, - degToRad(theta), 255, 255, 0, 1);
      getRGB(src_bgr, centers.at(index).x, centers.at(index).y, &red, &green, &blue);
      rgb2hsi(red, green, blue, &hue, &saturation, &intensity);
      hueMagnitude(&hue);
      pic_vals.push_back({centers.at(index).x, centers.at(index).y, theta, hue});
   }

    /* get the object pose data */
    /* ------------------------ */
    const int NUM_BRICKS = 3;           // we have 3 bricks. this parameter should perhaps be read from the input file?
                                        // name and colors arrays should be of size NUM_BRICKS

   float bricks_pose[NUM_BRICKS][4];   // read and store the bricks poses in an array



   sort(pic_vals.begin(), pic_vals.end(), smallHue);
   for (auto pic : pic_vals){
      if (debug) printf("( %3d, %3d, %3d, %3f)", pic.x, pic.y, pic.theta, pic.hue);
      bricks_pose[i][0] = pic.x;
      bricks_pose[i][1] = pic.y;
      bricks_pose[i][2] = 0;
      bricks_pose[i][3] = pic.theta;
      printf("location %d = %f %f %f %f\n", i+1, bricks_pose[i][0], bricks_pose[i][1], bricks_pose[i][2], bricks_pose[i][3]);
   }
   printf("\n");
   pic_vals.clear();
   imshow( "Src", src);
   if(centers.size() != 0) centers.clear();
   if(arcLine_points.size() != 0) arcLine_points.clear();
   cv::waitKey(0);
   
   /* stack the bricks: red on top, green in the middle, and blue at the bottom  */
   /* -------------------------------------------------------------------------  */


   /* insert your code here */

 
    /* get the destination pose data */
    /* ----------------------------- */

    
    if (debug) printf("Destination pose %f %f %f %f\n", destination_x, destination_y, destination_z, destination_phi);
   
    float bricks_dest_z[NUM_BRICKS]; // each bricks destination z coordinate will depend on its position in the stack given
    for (int i = 0; i < NUM_BRICKS; i++){
        bricks_dest_z[i] = destination_z + i*11.4; // all bricks have a height of 11.4 mm
    }
   /* Call the utility function to pick and place the spawned bricks */
    for (int i = 0; i < NUM_BRICKS; i++) {
        pick_and_place(bricks_pose[i][0], bricks_pose[i][1], bricks_pose[i][2], bricks_pose[i][3],
                       destination_x, destination_y, bricks_dest_z[i], destination_phi,
                       0, 0, 5, 180);
    }

   return 0;
}
