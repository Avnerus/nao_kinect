/*
* Paul Bartholomew Feb 2014
* Read Joint Angles from OpenNi skeletal
XBOX Kinect data
translate the information to joint angles to determine the human pose
tracking software received from
*

https://smartech.gatech.edu/bitstream/handle/1853/51872/BARTHOLOMEW-THESIS-2014.pdf

*/

#include "ros/ros.h"
#include "tf/tfMessage.h"
#include "naoqi_bridge_msgs/JointAnglesWithSpeed.h"
#include <string>
#include <vector>
#include <math.h>
#include <ros/console.h>

// //LIST OF VARIABLES

// extraction of kinect joint name
std::string tfJointName;
std::string tfFrameName;

// constant "/camera" we don’t want to include these elements in the kinect skeletal array
const std::string dontIncludeCameraTFs ("/camera_frame") ;
const std::string includeDepthFrame ("/tracker_depth_frame") ;

// constant "/head" which is the first element published by the kinect
const std::string head ("/tracker/user_1/head") ;

// extractions for kinect translational vector data
float Depth_X;
float Right_Y;
float Height_Z;

// arrays which keep all joint names and angles to be sent to NAO robot
std::vector<std::string> naoJointNames(0) ;
std::vector<float> naoJointAngles(0) ;

// array to keep list of kinect joint names
std::vector<std::string> tfNames(0) ;

// array to keep list of kinect joint angles
std::vector<float> tfDepth(0) ;
std::vector<float> tfRight (0) ;
std::vector<float> tfHeight (0) ;

//variables for Left Shoulder the xyz vectors

// Left Shoulder
float LS_Depth, LS_Right , LS_Height;
// Left Elbow
float LE_Depth, LE_Right , LE_Height;
// Left Hand
float LH_Depth, LH_Right, LH_Height;

// Right Shoulder
float RS_Depth, RS_Right, RS_Height;
// Right Elbow
float RE_Depth, RE_Right, RE_Height;
// Right Hand
float RH_Depth, RH_Right, RH_Height;

// variables for squaring up model
float Zeroed_Depth, Zeroed_Right , SquareRoll;


// Arm Segments
float LBicep_Right_unsq , LBicep_Depth_unsq, LForearm_Right_unsq ,LForearm_Depth_unsq;

float RBicep_Right_unsq , RBicep_Depth_unsq, RForearm_Right_unsq, RForearm_Depth_unsq;

float LBicep_Height , LBicep_Right , LBicep_Depth , LForearm_Height , LForearm_Right , LForearm_Depth;

float RBicep_Height , RBicep_Right , RBicep_Depth, RForearm_Height, RForearm_Right, RForearm_Depth;

// un-Pitched Arm segments
float LBicep_Depth_unpitch , RBicep_Depth_unpitch , LForearm_Depth_unpitch , RForearm_Depth_unpitch , LForearm_Height_unpitch , RForearm_Height_unpitch;

float LForearm_Depth_unroll , RForearm_Depth_unroll , LForearm_Right_unroll , RForearm_Right_unroll , LForearm_Right_unyaw, RForearm_Right_unyaw;

// variables for NAO angles
float LShoulderRoll , LShoulderPitch , RShoulderRoll , RShoulderPitch;
float LElbowRoll , LElbowYaw, RElbowRoll , RElbowYaw;



// variables for NAO msg parameters
// fraction of maximum joint velocity [0:1]
float speed;

//absolute angle (0 is default) or relative change
uint8_t rel ;

// Function to convert Kinect Vectors into Pitch and Roll angles for NAO joints
void convertXYZvectorsToAngles() {

    // get a shoulder , elbow, and hand vectors from Kinect
    // Note: Data is Mirrored switch Right and Left to unmirror

// Using this Tracker: https://github.com/Avnerus/skeleton_tracker
    //ROS_INFO("GO!!");

    // left shoulder is element 6
    RS_Depth = tfDepth.at(6);
    RS_Right = tfRight.at(6);
    RS_Height = tfHeight.at(6);



    // left elbow is element 1
    RE_Depth = tfDepth.at(1);
    RE_Right = tfRight.at(1);
    RE_Height = tfHeight.at(1);

    // left hand is element 3
    RH_Depth = tfDepth.at(3);
    RH_Right = tfRight.at(3);
    RH_Height = tfHeight.at(3);

    // right shoulder is element 13
    LS_Depth = tfDepth.at(13);
    LS_Right = tfRight.at(13);
    LS_Height = tfHeight.at(13);

    // right elbow is element 8
    LE_Depth = tfDepth.at(8);
    LE_Right = tfRight.at(8);
    LE_Height = tfHeight.at(8);

    // right hand is element 10
    LH_Depth = tfDepth.at(10);
    LH_Right = tfRight.at(10);
    LH_Height = tfHeight.at(10);

    // //BUILD ARMS FROM SEGMENTS
    // Arm is generated from one bone extending from Shoulder to Elbow and another from the Elbow to the Hand

    // Left Arm
    LBicep_Height = LE_Height-LS_Height;
    LBicep_Right_unsq = LE_Right-LS_Right;
    LBicep_Depth_unsq = LE_Depth-LS_Depth;
    LForearm_Height = LH_Height-LE_Height;
    LForearm_Right_unsq = LH_Right-LE_Right;
    LForearm_Depth_unsq = LH_Depth-LE_Depth;

    // Right Arm
    RBicep_Height = RE_Height-RS_Height;
    RBicep_Right_unsq = RE_Right-RS_Right;
    RBicep_Depth_unsq = RE_Depth-RS_Depth;
    RForearm_Height = RH_Height-RE_Height;
    RForearm_Right_unsq = RH_Right-RE_Right;
    RForearm_Depth_unsq = RH_Depth-RE_Depth;

    // First Find the vector between the shoulders and use it to square up the model with the camera perpendicular

    //Assume the shoulders are at the same height , find the rotation perpendicular to the camera to square up the model
    Zeroed_Depth = RS_Depth - LS_Depth;
    Zeroed_Right = RS_Right - LS_Right;
    SquareRoll = atan2(-Zeroed_Depth,Zeroed_Right) ;

    /*ROS_INFO_STREAM("Right Shoulder XYZ = (" << RS_Right << "," << RS_Height << "," << RS_Depth << ")");
    ROS_INFO_STREAM("Left Shoulder XYZ = (" << LS_Right << "," << LS_Height << "," << LS_Depth << ")");


    ROS_INFO_STREAM("SquareRoll Angel  = atan2(" << -Zeroed_Depth << "," << Zeroed_Right << ") = " << SquareRoll << "(" << SquareRoll * (180.0 / M_PI) << ")" );*/

    //Use a rotation about
    // Use the standard rotation matrix since both are rotation about the Height Axis to square up the model
    // Here we use a transpose matrix about the vertical axis
    //
    //NOTE: Height Remains the same

    //Square up Left Side

    LBicep_Depth = cos(SquareRoll)*LBicep_Depth_unsq+sin(SquareRoll) * LBicep_Right_unsq;
    LBicep_Right = -sin(SquareRoll)*LBicep_Depth_unsq+cos(SquareRoll) * LBicep_Right_unsq;
    LForearm_Depth = cos(SquareRoll)*LForearm_Depth_unsq+sin(SquareRoll) * LForearm_Right_unsq;
    LForearm_Right = -sin(SquareRoll)*LForearm_Depth_unsq+cos(SquareRoll) * LForearm_Right_unsq;

    //Square up Right Side
    RBicep_Depth = cos(SquareRoll)*RBicep_Depth_unsq+sin(SquareRoll) * RBicep_Right_unsq;
    RBicep_Right = -sin(SquareRoll)*RBicep_Depth_unsq+cos(SquareRoll) * RBicep_Right_unsq;
    RForearm_Depth = cos(SquareRoll)*RForearm_Depth_unsq+sin(SquareRoll) * RForearm_Right_unsq;
    RForearm_Right = -sin(SquareRoll)*RForearm_Depth_unsq+cos(SquareRoll) * RForearm_Right_unsq;

    // //OBTAIN PITCH AND ROLL ANGLES OF THE SHOULDER JOINTS
    // Pitch angle is obtained by the arc tangent of the Depth vector and the Height vector
    LShoulderPitch = atan2(-LBicep_Height , -LBicep_Depth) ;
    RShoulderPitch = atan2(-RBicep_Height , -RBicep_Depth) ;

    //Next un-pitch the arms about the " right " direction axis
    //NOTE: Right remains the same, and Height should now be practically zero
    LBicep_Depth_unpitch = cos(LShoulderPitch)*LBicep_Depth+sin(LShoulderPitch)*LBicep_Height;
    RBicep_Depth_unpitch = cos(RShoulderPitch)*RBicep_Depth+sin(RShoulderPitch)*RBicep_Height;
    // Also un-pitch forearm segments
    LForearm_Depth_unpitch = cos(LShoulderPitch)*LForearm_Depth+sin(LShoulderPitch)*LForearm_Height;
    LForearm_Height_unpitch = -sin(LShoulderPitch)*LForearm_Depth+cos(LShoulderPitch)*LForearm_Height;
    RForearm_Depth_unpitch = cos(RShoulderPitch)*RForearm_Depth+sin(RShoulderPitch)*RForearm_Height;
    RForearm_Height_unpitch = -sin(RShoulderPitch)*RForearm_Depth+cos(RShoulderPitch)*RForearm_Height;


    // Roll angle can be calculated from the arc tangent between what in the Right direction VS Depth direction
    LShoulderRoll = atan2(-LBicep_Right,-LBicep_Depth_unpitch);
    RShoulderRoll = atan2(-RBicep_Right,-RBicep_Depth_unpitch);

    // //OBTAIN YAW AND ROLL OF THE ELBOW JOINTS
    LForearm_Depth_unroll = cos(LShoulderRoll)*LForearm_Depth_unpitch+sin(LShoulderRoll)*LForearm_Right;
    LForearm_Right_unroll = -sin(LShoulderRoll)*LForearm_Depth_unpitch+cos(LShoulderRoll)*LForearm_Right;
    RForearm_Depth_unroll = cos(RShoulderRoll)*RForearm_Depth_unpitch+sin(RShoulderRoll)*RForearm_Right;
    RForearm_Right_unroll = -sin(RShoulderRoll)*RForearm_Depth_unpitch+cos(RShoulderRoll)*RForearm_Right;

    // Find the Elbow Yaw angles
    LElbowYaw = atan2(-LForearm_Height_unpitch ,LForearm_Right_unroll) ;
    RElbowYaw = atan2( RForearm_Height_unpitch , -RForearm_Right_unroll) ;
    //NOTE: This should make the forearm height data practically zero
    LForearm_Right_unyaw = cos(LElbowYaw)*LForearm_Right_unroll-sin(LElbowYaw)*LForearm_Height_unpitch;
    RForearm_Right_unyaw = cos(RElbowYaw)*RForearm_Right_unroll-sin(RElbowYaw)*RForearm_Height_unpitch;

    // Lastly , compute the Elbow Roll angles
    LElbowRoll = atan2(-LForearm_Right_unyaw,-LForearm_Depth_unroll) ;
    RElbowRoll = atan2(-RForearm_Right_unyaw,-RForearm_Depth_unroll) ;

    //Emptying the old values from the name and angle arrays

    while (!naoJointNames.empty() ) {
        naoJointNames.pop_back() ;
        naoJointAngles.pop_back() ;
    }

    // Put the new name and angle values into their corresponding structures
    naoJointNames.push_back("LShoulderPitch") ;
    naoJointAngles.push_back(LShoulderPitch) ; 
    naoJointNames.push_back("LShoulderRoll") ;
    naoJointAngles.push_back(LShoulderRoll) ;
    naoJointNames.push_back("LElbowYaw") ;
    naoJointAngles.push_back(LElbowYaw) ;
    naoJointNames.push_back("LElbowRoll") ;
    naoJointAngles.push_back(LElbowRoll) ;
    naoJointNames.push_back("RShoulderPitch") ;
    naoJointAngles.push_back(RShoulderPitch) ;
    naoJointNames.push_back("RShoulderRoll") ;
    naoJointAngles.push_back(RShoulderRoll) ;
    naoJointNames.push_back("RElbowYaw") ;
    naoJointAngles.push_back(RElbowYaw) ;
    naoJointNames.push_back("RElbowRoll") ;
    naoJointAngles.push_back(RElbowRoll) ;

}

// Subscriber: function called each time the Kinect publishes new skeletal data
void getTFvectors(const tf::tfMessage::ConstPtr& msg) {
    // get the joint name
    tfJointName = msg->transforms[0].child_frame_id;
    tfFrameName = msg->transforms[0].header.frame_id;

    //check to see if is a "/camera..." element
    if (tfFrameName.compare(includeDepthFrame) == 0 && tfJointName.compare(dontIncludeCameraTFs)!=0) {
        // get the X, Y, and Z coordinates for the kinect joint

	// Had to swtich Z and Y. Y is height and Z is right
        Depth_X = msg->transforms[0].transform.translation.x;
        Right_Y = msg->transforms[0].transform.translation.z;
        Height_Z = msg->transforms[0].transform.translation.y;


        //The head element is first it will allow us to begin our parsing
        //
        if (tfJointName.compare(head)==0)
        {
            //so call a function to empty the vector and transform the vectors
            // into angles theta and phi
            // wait to be length of 14 before making function call
            if (tfNames.size ()==15)
            {
                //New head element means we should publish the old kinect
                // skeleton and clear out the array before starting again
                convertXYZvectorsToAngles();
            }

            //empty the X-Y-Z coordinate arrays
            while (!tfNames.empty()) {
                tfNames.pop_back() ;
                tfDepth.pop_back() ;
                tfRight.pop_back() ;
                tfHeight.pop_back() ;
            }

            // store "/head" element and xyz vector
            tfNames.push_back(tfJointName);
            tfDepth.push_back(Depth_X);
            tfRight.push_back(Right_Y);
            tfHeight.push_back(Height_Z);

        }
        else
        {
            //isn ’t "/head" or "/camera.." store it in order it arrives 
            tfNames.push_back(tfJointName) ;
            tfDepth.push_back(Depth_X) ;
            tfRight.push_back(Right_Y) ;
            tfHeight.push_back(Height_Z) ;
        }  
    }
}

void initializeArms() {
    // initialize all the joints to be controlled to zero
    naoJointNames.push_back("LShoulderRoll") ;
    naoJointAngles.push_back(0.0) ;
    naoJointNames.push_back("LShoulderPitch") ;
    naoJointAngles.push_back(0.0) ;
    naoJointNames.push_back("LElbowYaw") ;
    naoJointAngles.push_back(0.0) ;
    naoJointNames.push_back("LElbowRoll") ;
    naoJointAngles.push_back(0.0) ;
    naoJointNames.push_back("RShoulderRoll") ;
    naoJointAngles.push_back(0.0) ;
    naoJointNames.push_back("RShoulderPitch") ;
    naoJointAngles.push_back(0.0) ;
    naoJointNames.push_back("RElbowYaw") ;
    naoJointAngles.push_back(0.0) ;
    naoJointNames.push_back("RElbowRoll") ;
    naoJointAngles.push_back(0.0) ;
}
int main( int argc , char **argv) {

    // initialize a node with name
    ros::init (argc, argv, "KinectRawJointAngles") ;

    ROS_INFO("Nao Kinect node Active");

    // create node handle
    ros::NodeHandle n;

    // create a function to subscribe to a topic
    ros::Subscriber sub = n.subscribe("tf" , 1000, getTFvectors);

    // create a function to advertise on a given topic
    //ros::Publisher joint_angles_pub = n.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/nao_robot/pose/joint_angles" ,1000);
    ros::Publisher joint_angles_pub = n.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles" ,1000);

    //choose the looping rate
    ros::Rate loop_rate(30.0);

    // create message element to be filled with appropriate data to be published
    naoqi_bridge_msgs::JointAnglesWithSpeed msg;

    // initialize arms to zero;
    initializeArms();

    // loop
    while(ros::ok()) {
        // Put elements into message for publishing topic
        msg.joint_names = naoJointNames;
        msg.joint_angles = naoJointAngles;  // float [] -In Radians (must be array)
        speed = 0.5;
        rel = 0;

        msg.speed = speed; // float 
        msg.relative = rel; // unit8

        // publish
        joint_angles_pub.publish(msg);

        // spin once
        ros::spinOnce() ;

        // sleep
        loop_rate.sleep() ;
    }
    return 0;
}
