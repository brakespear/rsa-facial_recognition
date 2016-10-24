/*
 * face_finder.cpp
 * Client to use the Fserver provided in the facial recognition package to find 
 * (turn the robot to be facing) a given person from a group of people standing in a circle around the robot
 * Written by: Elliott Smith
 * For COMP3431 Assignment 2
 * Date: 26/09/2016
*/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FRClientGoal.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <signal.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>


face_recognition::FaceRecognitionGoal goal; //Goal message
actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> * ac; //action lib client
char *findName;
bool faceFound = false;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const face_recognition::FaceRecognitionResultConstPtr& result)
{
	ROS_INFO("Continous face recognition goal has finished?!");
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Begin looking for faces");
}

/*
 * Called every time feedback is received for the goal
 * Compares the name of any person detected with the name of that being searched for
 * And sets a flag if they match (that will stop the rotation of the robot)
*/
void feedbackCb(const face_recognition::FaceRecognitionFeedbackConstPtr& feedback)
{
	ROS_INFO("Received feedback from Goal [%d] ", feedback->order_id);
	if(feedback->order_id==1 ) {
		ROS_INFO("%s was recognized with confidence %f", feedback->names[0].c_str(),feedback->confidence[0]);     
		ROS_INFO("Looking for %s found %s",findName,feedback->names[0].c_str());
		if (strcmp(feedback->names[0].c_str(),findName)==0) {
			ROS_INFO("Recognised person being searched for %s",feedback->names[0].c_str());
			faceFound = true;
		}
	}
	
}


//shut down
void exit_handler(int s)
{
	delete(ac);
	ros::shutdown();
}


/* 
 * main function expects name of person to find as a command line argument
 * Name must match that used when training the persons face
 * Send the continuous facial recognition goal to the server
 * And spins the robot anticlockwise
 * Until the feedback from the server indicates that the person being searched for has been found
*/
int main (int argc, char **argv)
{
	ros::init(argc, argv, "face_finder");
	if (argc != 2) {
		ROS_INFO("Usage: face_finder name.");
		return 1;
	} else {
		findName = argv[1];
	}
	ros::NodeHandle n;
	ac = new actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction>("face_recognition", true);
	//for proper shutdown exit_handler is used
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
	//wait for the server 
	ac->waitForServer();
	//send goal to continuously recognises faces to the server
	goal.order_id = 1; //recognise continuous
	goal.order_argument = "none"; //no additional information required
	ROS_INFO("Sending continous face recognition goal");
	ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);  
	ROS_INFO("Goal sent");
	//
	ros::Publisher twistPub = n.advertise< geometry_msgs::Twist >("cmd_vel_mux/input/navi", 1, false);
	while (ros::ok() && !faceFound) {
		geometry_msgs::Twist t;
		t.linear.x = t.linear.y = t.linear.z = 0;
		t.angular.x = t.angular.y = 0;
    	t.angular.z = 0.1; //adjust to change rate of turning
		twistPub.publish(t);
		ros::spinOnce();
	}
		
	return 0;
}

