/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/

#include <stdr_robot/stdr_robot.h>
#include <nodelet/NodeletUnload.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(stdr_robot::Robot, nodelet::Nodelet) 

namespace stdr_robot {
	
void Robot::onInit() {
	ros::NodeHandle n = getMTNodeHandle();
	
	_registerClientPtr.reset( new RegisterRobotClient(n, "stdr_server/register_robot", true) );
	_registerClientPtr->waitForServer();
	
	stdr_msgs::RegisterRobotGoal goal;
	goal.name = getName();
	_registerClientPtr->sendGoal(goal, boost::bind(&Robot::initializeRobot, this, _1, _2));	

	_currentPosePtr.reset( new geometry_msgs::Pose2D ); 

	_mapSubscriber = n.subscribe("map", 1, &Robot::mapCallback, this);
	_moveRobotService = n.advertiseService(getName() + "/replace", &Robot::moveRobotCallback, this);
	_collisionTimer = n.createTimer(ros::Duration(0.1), &Robot::checkCollision, this, false, false);
	_startTimer = true;
	_tfTimer = n.createTimer(ros::Duration(0.1), &Robot::publishRobotTf, this);
}

void Robot::initializeRobot(const actionlib::SimpleClientGoalState& state, const stdr_msgs::RegisterRobotResultConstPtr result) {
	
	if (state == state.ABORTED) {
		NODELET_ERROR("Something really bad happened...");
		return;
	}
	
	NODELET_INFO("Loaded new robot, %s", getName().c_str());
	
	// use result to initialize sensors
	
	_currentPosePtr->x = result->description.initialPose.x;
	_currentPosePtr->y = result->description.initialPose.y;
	_currentPosePtr->theta = result->description.initialPose.theta;
	
	_robotDescription = result->description;
	
	ros::NodeHandle n = getMTNodeHandle();
	_motionControllerPtr.reset( new IdealMotionController(_currentPosePtr, _tfBroadcaster, n, getName()) );

}

void Robot::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
	ros::NodeHandle n = getMTNodeHandle();
	if ( _startTimer ){
		_startTimer = false;
		_collisionTimer.start();
	}
	_map = *msg;
}

bool Robot::moveRobotCallback(stdr_msgs::MoveRobot::Request& req,
							stdr_msgs::MoveRobot::Response& res)
{
	_currentPosePtr->x = req.newPose.x;
	_currentPosePtr->y = req.newPose.y;
	_currentPosePtr->theta = req.newPose.theta;
	return true;
}

void Robot::checkCollision(const ros::TimerEvent&) {
	int xMapPixel;
	int yMapPixel;
	
	if ( _robotDescription.footprint.points.size() == 0 ){
		float angleStep = 1.0;
		float radStep = angleStep * STDR_PI / 180.0;
		
		for (float angleIter = 0.0; angleIter < STDR_PI * 2; angleIter += radStep){
			
			xMapPixel = ( _currentPosePtr->x + _robotDescription.footprint.radius * cos( angleIter ) ) / _map.info.resolution;
			yMapPixel = ( _currentPosePtr->y + _robotDescription.footprint.radius * sin( angleIter ) ) / _map.info.resolution;
			
			if ( xMapPixel >= _map.info.width || yMapPixel >=  _map.info.height || xMapPixel < 0 || yMapPixel < 0){
				ROS_DEBUG_NAMED("stdr_robot","[stdr_robot %s %d]: Footprint values outside map limits ", __FUNCTION__, __LINE__ );
				_motionControllerPtr->stop();
				break;
			}
			
			if ( _map.data[ yMapPixel * _map.info.width + xMapPixel ] > 70 ){
				
				ROS_DEBUG_NAMED("stdr_robot","[stdr_robot %s  %d]: Collision Detected!!!", __FUNCTION__, __LINE__ );
				_motionControllerPtr->stop();
				break;
			}
		}
	}
}

void Robot::publishRobotTf(const ros::TimerEvent&) {
		
	tf::Vector3 translation(_currentPosePtr->x, _currentPosePtr->y, 0);
	tf::Quaternion rotation;
	rotation.setRPY(0, 0, _currentPosePtr->theta);
	
	tf::Transform transform(rotation, translation);
	
	_tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", getName()));	
}

Robot::~Robot() {
	// cleanup
}
	
}
