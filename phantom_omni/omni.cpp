#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include <bullet/LinearMath/btMatrix3x3.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include "phantom_omni/PhantomButtonEvent.h"
#include "phantom_omni/LockFunction.h"

#include <pthread.h>


struct OmniState
{
    hduVector3Dd position;  //3x1 vector of position
    hduVector3Dd velocity;  //3x1 vector of velocity
    hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
    hduVector3Dd inp_vel2;  
    hduVector3Dd inp_vel3;  
    hduVector3Dd out_vel1;  
    hduVector3Dd out_vel2;  
    hduVector3Dd out_vel3;
    hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity 
    hduVector3Dd pos_hist2; 
    hduVector3Dd rot;
    hduVector3Dd joints;
    hduVector3Dd joint_torques;    
    hduVector3Dd force;     //3 element double vector force[0], force[1], force[2]
    int buttons[2];
    int buttons_prev[2];
    bool lock;
    bool lock_init;
    bool lock_function;
    hduVector3Dd lock_pos;
    double pose_transform_matrix[16];
    bool workspace_data_set;
    double max_workspace[6];
    double usable_workspace[6];    
};


class PhantomROS {

    public:

    ros::NodeHandle n;

    ros::Publisher pose_publisher, button_publisher, omni_angles, tip_pose_publisher, lock_publisher, max_workspace_publisher, usable_workspace_publisher;    
    ros::Subscriber joint_torque_sub, force_sub, lockpose_sub;
    ros::ServiceServer lock_function_service;

    OmniState *state;


    void init(OmniState *s) 
    {
        //Publishers
        pose_publisher = n.advertise<geometry_msgs::PoseStamped>("pose", 100);
        button_publisher = n.advertise<phantom_omni::PhantomButtonEvent>("button", 100);
        omni_angles = n.advertise<sensor_msgs::JointState>("angles", 100);
        tip_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("tip_pose", 100);
        lock_publisher = n.advertise<std_msgs::Bool>("lock", 100);
        max_workspace_publisher = n.advertise<std_msgs::Float64MultiArray>("max_workspace", 100);
        usable_workspace_publisher = n.advertise<std_msgs::Float64MultiArray>("usable_workspace", 100); 

        //Subscribers
        joint_torque_sub = n.subscribe("set_joint_torques", 100, &PhantomROS::joint_torque_callback, this);             
        force_sub = n.subscribe("set_forces", 100, &PhantomROS::force_callback, this);          
        lockpose_sub = n.subscribe("set_lock_pose", 100, &PhantomROS::lockpose_callback, this); 

        // Services
        lock_function_service = n.advertiseService("lock_function", &PhantomROS::lock_function_callback, this);


        state = s;
        state->buttons[0] = 0;
        state->buttons[1] = 0;
        state->buttons_prev[0] = 0;
        state->buttons_prev[1] = 0;
        hduVector3Dd zeros(0, 0, 0);
        state->velocity = zeros;
        state->inp_vel1 = zeros;  //3x1 history of velocity
        state->inp_vel2 = zeros;  //3x1 history of velocity
        state->inp_vel3 = zeros;  //3x1 history of velocity
        state->out_vel1 = zeros;  //3x1 history of velocity
        state->out_vel2 = zeros;  //3x1 history of velocity
        state->out_vel3 = zeros;  //3x1 history of velocity
        state->pos_hist1 = zeros; //3x1 history of position 
        state->pos_hist2 = zeros; //3x1 history of position
        state->lock = true;
        state->lock_init = true;
        state->lock_function = true;
        state->lock_pos = zeros;
        for (unsigned int i=0; i<16; i++)       state->pose_transform_matrix[i] = 0;
        state->workspace_data_set = false;
    }


    /*******************************************************************************
     ROS node callbacks  
    *******************************************************************************/
    void joint_torque_callback(const sensor_msgs::JointState& omnifeed_joint)
    {
        for (unsigned int i=0; i<3; i++)        state->joint_torques[i] = omnifeed_joint.effort[i];
    }


    void force_callback(const geometry_msgs::Vector3& omnifeed_forces)
    {
        // extra damping
        state->force[0] = omnifeed_forces.x - 0.001*state->velocity[0];
        state->force[1] = omnifeed_forces.y - 0.001*state->velocity[1];
        state->force[2] = omnifeed_forces.z - 0.001*state->velocity[2];
    }


    void lockpose_callback(const geometry_msgs::Vector3& omnifeed_lockpose)
    {
        state->lock_pos[0] = omnifeed_lockpose.x;
        state->lock_pos[1] = omnifeed_lockpose.y;
        state->lock_pos[2] = omnifeed_lockpose.z;
    }    


    bool lock_function_callback(phantom_omni::LockFunction::Request  &req,
                                phantom_omni::LockFunction::Response &res)
    {
        if (req.lock)   state->lock_function = true;
        else            state->lock_function = false;

        res.lock_state = state->lock_function;

        return true;
    }


    void publish_omni_state()
    {
        // Angles
        sensor_msgs::JointState angles;
        angles.position.resize(6);
        for (unsigned int i=0; i<3; i++)
        {
          angles.position[i] = state->joints[i];
          angles.position[i+3] = state->rot[i];
        }
//      for (unsigned int i=0; i<6; i++)        angles.position[i] *= (180/M_PI);       // For visualization
        omni_angles.publish(angles);


        // Workspace data: [Xmin, Ymin, Zmin, Xmax, Ymax, Zmax]
        if (state->workspace_data_set)
        {
          std_msgs::Float64MultiArray workspace_data;

          for (unsigned int i=0; i<6; i++)      workspace_data.data.push_back(state->max_workspace[i]);
          max_workspace_publisher.publish(workspace_data);

          for (unsigned int i=0; i<6; i++)      workspace_data.data.push_back(state->usable_workspace[i]);        
          usable_workspace_publisher.publish(workspace_data);     
        }


        // Pose
        tf::Transform l0_pose(tf::Matrix3x3(state->pose_transform_matrix[0],state->pose_transform_matrix[1],state->pose_transform_matrix[2],
                                            state->pose_transform_matrix[4],state->pose_transform_matrix[5],state->pose_transform_matrix[6],
                                            state->pose_transform_matrix[8],state->pose_transform_matrix[9],state->pose_transform_matrix[10]),
                              tf::Vector3(state->pose_transform_matrix[12], state->pose_transform_matrix[13], state->pose_transform_matrix[14]));

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "pose";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = l0_pose.getOrigin()[0];
        pose_stamped.pose.position.y = l0_pose.getOrigin()[1];
        pose_stamped.pose.position.z = l0_pose.getOrigin()[2];
        pose_stamped.pose.orientation.x = l0_pose.getRotation()[0];
        pose_stamped.pose.orientation.y = l0_pose.getRotation()[1];
        pose_stamped.pose.orientation.z = l0_pose.getRotation()[2];     
        pose_stamped.pose.orientation.w = l0_pose.getRotation()[3];
        pose_publisher.publish(pose_stamped);


        // Tip pose = pose - pen_length * Z_axis
        double pen_length = 0.0375;
        l0_pose.setOrigin(tf::Vector3(state->pose_transform_matrix[12] - pen_length * state->pose_transform_matrix[8],
                                      state->pose_transform_matrix[13] - pen_length * state->pose_transform_matrix[9],
                                      state->pose_transform_matrix[14] - pen_length * state->pose_transform_matrix[10]));

        pose_stamped.header.frame_id = "tip_pose";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = l0_pose.getOrigin()[0];
        pose_stamped.pose.position.y = l0_pose.getOrigin()[1];
        pose_stamped.pose.position.z = l0_pose.getOrigin()[2];
        pose_stamped.pose.orientation.x = l0_pose.getRotation()[0];
        pose_stamped.pose.orientation.y = l0_pose.getRotation()[1];
        pose_stamped.pose.orientation.z = l0_pose.getRotation()[2];     
        pose_stamped.pose.orientation.w = l0_pose.getRotation()[3];
        tip_pose_publisher.publish(pose_stamped);


        // Buttons
        if (state->lock_function)
        {
          if (state->lock_init){
            std_msgs::Bool is_lock;
            is_lock.data = true;
            lock_publisher.publish(is_lock);
          }
          {
            // If any button is pressed
            if ((state->buttons[0] != state->buttons_prev[0]) or (state->buttons[1] != state->buttons_prev[1]))
            {

              // If both buttons are pressed
              if ((state->buttons[0] == state->buttons[1]) and (state->buttons[0] == 1))
              {
                state->lock_init = false;

                // lock/unlock
                state->lock = !(state->lock);

                // Publish it
                std_msgs::Bool is_lock;
                is_lock.data = false;
                if (state->lock)        is_lock.data = true;
                lock_publisher.publish(is_lock);

                // If omni just unlocked then set forces and joint torques to zero
                if (!state->lock)
                {
                  for (unsigned int i=0; i<3; i++)
                  {
                    state->joint_torques[i] = 0.0;
                    state->force[i] = 0.0;
                  }
                }
              }

              phantom_omni::PhantomButtonEvent button_event;
              button_event.grey_button = state->buttons[0];
              button_event.white_button = state->buttons[1];
              state->buttons_prev[0] = state->buttons[0];
              state->buttons_prev[1] = state->buttons[1];
              button_publisher.publish(button_event);
            }
          }

        }
        else
        {
          state->lock = false;
        }
    }
};


HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
    OmniState *omni_state = static_cast<OmniState *>(pUserData);

    hdBeginFrame(hdGetCurrentDevice());

    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  omni_state->joints);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);    
    hdGetDoublev(HD_CURRENT_POSITION,      omni_state->position);    
    hdGetDoublev(HD_CURRENT_TRANSFORM,     omni_state->pose_transform_matrix);


    // Position to metres
    for (unsigned int i=0; i<3; i++)
    {
      omni_state->position[i] /= 1000;
      omni_state->pose_transform_matrix[i+12] /= 1000;
    }

    // Decouple joint 3
    omni_state->joints[2] = omni_state->joints[2] - omni_state->joints[1];


    // Workspace dimensions
    if (!omni_state->workspace_data_set)
    {
      HDdouble hddata[6];
      hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, hddata);
      for (unsigned int i=0; i<6; i++)          omni_state->max_workspace[i] = hddata[i]/1000;          // Set data in metres

      hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, hddata);
      for (unsigned int i=0; i<6; i++)          omni_state->usable_workspace[i] = hddata[i]/1000;       // Set data in metres

      omni_state->workspace_data_set = true;
    }


    // Historial & velocity
    hduVector3Dd vel_buff(0, 0, 0);
    vel_buff = (omni_state->position*3 - 4*omni_state->pos_hist1 + omni_state->pos_hist2)/0.002;  //mm/s, 2nd order backward dif
    omni_state->velocity = (.2196*(vel_buff+omni_state->inp_vel3)+.6588*(omni_state->inp_vel1+omni_state->inp_vel2))/1000.0-(-2.7488*omni_state->out_vel1+2.5282*omni_state->out_vel2 - 0.7776*omni_state->out_vel3);  //cutoff freq of 20 Hz
    omni_state->pos_hist2 = omni_state->pos_hist1;
    omni_state->pos_hist1 = omni_state->position;
    omni_state->inp_vel3 = omni_state->inp_vel2;
    omni_state->inp_vel2 = omni_state->inp_vel1;
    omni_state->inp_vel1 = vel_buff;
    omni_state->out_vel3 = omni_state->out_vel2;
    omni_state->out_vel2 = omni_state->out_vel1;
    omni_state->out_vel1 = omni_state->velocity;


    //Buttons
    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;    


    // Lock
    if (omni_state->lock == true)
    {
        omni_state->force = 1000*(0.08*(omni_state->lock_pos-omni_state->position) - 0.001*omni_state->velocity);       // Magnitudes in mm and mm/s

        hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

//      printf("joint forces: %3.2f %3.2f %3.2f\n", omni_state->force[0], omni_state->force[1], omni_state->force[2]);  
    }
    // Set force/torque
    else
    {

      double f_norm = fabs(omni_state->force[0]) + fabs(omni_state->force[1]) + fabs(omni_state->force[2]);
      double t_norm = fabs(omni_state->joint_torques[0]) + fabs(omni_state->joint_torques[1]) + fabs(omni_state->joint_torques[2]);       

/*      printf("joint torques / forces: %3.2f %3.2f %3.2f   -   %3.2f %3.2f %3.2f   -   %f %f\n", omni_state->joint_torques[0], omni_state->joint_torques[1], omni_state->joint_torques[2],
                                                                                                      omni_state->force[0], omni_state->force[1], omni_state->force[2],
                                                                                                      t_norm, f_norm); */  

      double epsilon = 0.0001;

      if ( (f_norm > epsilon) && (t_norm < epsilon) )
      {
        hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);
      }
      else if ( (t_norm > epsilon) && (f_norm < epsilon) )
      {
        hdSetDoublev(HD_CURRENT_JOINT_TORQUE, omni_state->joint_torques);
      }      
      else
      {
        hduVector3Dd joint_torques_zero, force_zero;  
        for (unsigned int i=0; i<3; i++)
        {
          joint_torques_zero[i] = 0.0;
          force_zero[i] = 0.0;    
        }

        hdSetDoublev(HD_CURRENT_FORCE, force_zero);
        hdSetDoublev(HD_CURRENT_JOINT_TORQUE, joint_torques_zero);
      }
    }


    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during main scheduler callback\n");
        if (hduIsSchedulerError(&error))        return HD_CALLBACK_DONE;
    }


    // Plot values
//     printf("angles:  %3.1f %3.1f %3.1f %3.1f %3.1f %3.1f\n", omni_state->joints[0]*(180/M_PI), omni_state->joints[1]*(180/M_PI), (omni_state->joints[2])*(180/M_PI), 
//                                                            omni_state->rot[0]*(180/M_PI), omni_state->rot[1]*(180/M_PI), omni_state->rot[2]*(180/M_PI));
//     printf("position x, y, z: %f %f %f \n", omni_state->position[0], omni_state->position[1], omni_state->position[2]);
//     printf("velocity x, y, z, time: %f %f %f \n", omni_state->velocity[0], omni_state->velocity[1],omni_state->velocity[2]);
//     std::cout << " --------------------------------- " << std::setw(4) << std::endl;
//     for (unsigned int i=0; i<4; i++){
//       for (unsigned int j=0; j<4; j++)
//       std::cout << std::setw(5) << std::fixed << std::setprecision(2) << omni_state->pose_transform_matrix[i+4*j] << "  ";
//       std::cout << std::endl;
//     }
//     std::cout << std::endl;    
//     printf("lock position x, y, z: %f %f %f \n", omni_state->lock_pos[0], omni_state->lock_pos[1], omni_state->lock_pos[2]);


    return HD_CALLBACK_CONTINUE;
}


/*******************************************************************************
Automatic Calibration of Phantom Device - No character inputs
*******************************************************************************/
void HHD_Auto_Calibration()
{
   int calibrationStyle;
   int supportedCalibrationStyles;
   HDErrorInfo error;

   hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
   if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
   {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESE..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
   {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL..\n\n");
   }
   if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
   {
        calibrationStyle = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO..\n\n");
   }

   do 
   {
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus in well)\n");
      if (HD_DEVICE_ERROR(error = hdGetError()))
      {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;           
      }
   }  while (hdCheckCalibration() != HD_CALIBRATION_OK);

   ROS_INFO("\n\nCalibration complete.\n");
}


void *ros_publish(void *ptr)
{
   PhantomROS *omni_ros = (PhantomROS *) ptr;
   int publish_rate;
   omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
   ros::Rate loop_rate(publish_rate);
   ros::AsyncSpinner spinner(2);
   spinner.start();

   while(ros::ok())
   {
       omni_ros->publish_omni_state();
       ros::spinOnce();
       loop_rate.sleep();
   }

   return NULL;
}


int main(int argc, char** argv)
{
   ////////////////////////////////////////////////////////////////
   // Init Phantom
   ////////////////////////////////////////////////////////////////
   HDErrorInfo error;
   HHD hHD;
   hHD = hdInitDevice(HD_DEFAULT_DEVICE);
   if (HD_DEVICE_ERROR(error = hdGetError())) 
   {
       //hduPrintError(stderr, &error, "Failed to initialize haptic device");
       ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
       return -1;
   }

   ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
   hdEnable(HD_FORCE_OUTPUT);
   hdStartScheduler(); 
   if (HD_DEVICE_ERROR(error = hdGetError()))
   {
       ROS_ERROR("Failed to start the scheduler");//, &error);
       return -1;           
   }
   HHD_Auto_Calibration();

   ////////////////////////////////////////////////////////////////
   // Init ROS 
   ////////////////////////////////////////////////////////////////
   ros::init(argc, argv, "omni_haptic_node");
   OmniState state;
   PhantomROS omni_ros;

   omni_ros.init(&state);
   hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

   ////////////////////////////////////////////////////////////////
   // Loop and publish 
   ////////////////////////////////////////////////////////////////
   pthread_t publish_thread;
   pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
   pthread_join(publish_thread, NULL);

   ROS_INFO("Ending Session....\n");
   hdStopScheduler();
   hdDisableDevice(hHD);

   return 0;
}
