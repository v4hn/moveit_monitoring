/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Universitaet Hamburg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner */

#include <moveit/move_group/move_group_capability.h>
#include <pluginlib/class_list_macros.hpp>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <thread>

class Monitoring : public move_group::MoveGroupCapability {
public:
	Monitoring() :
	   move_group::MoveGroupCapability{ "Monitoring" }
	{}

	~Monitoring() override
	{
		t.join();
	}

	void initialize() override {
		t = std::thread([this]{ this->monitoringThread(); });
		diagnostics = root_node_handle_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1, true);
		log_msgs = node_handle_.param("monitoring/log_msgs", false);
		rate = node_handle_.param("monitoring/rate", 0.5);
	};

	void monitoringThread();

private:
	void fillInSceneStatus(const planning_scene::PlanningScene& scene, diagnostic_msgs::DiagnosticStatus& status);
	void fillInLimitsStatus(const planning_scene::PlanningScene& scene, diagnostic_msgs::DiagnosticStatus& status);

	std::thread t;

	ros::Publisher diagnostics;
	bool log_msgs;
	double rate;
};
PLUGINLIB_EXPORT_CLASS(Monitoring, move_group::MoveGroupCapability);

void Monitoring::monitoringThread()
{
	diagnostic_msgs::DiagnosticArray da;
	da.header.stamp = ros::Time::now();
	da.status.resize(2);
	auto& scene_status{ da.status[0] };
	scene_status.name = "MoveGroupMonitoring.PlanningSceneCheck";
	auto& limits_status{ da.status[1] };
	limits_status.name = "MoveGroupMonitoring.LimitsCheck";
	auto sendDiagnostics = [&] { diagnostics.publish(da); };

	ros::Rate r(rate);
	while(ros::ok()){
		r.sleep();
		planning_scene_monitor::LockedPlanningSceneRO locked_scene{ context_->planning_scene_monitor_ };

		const auto& scene { *locked_scene->shared_from_this() };
		fillInSceneStatus(scene, scene_status);
		fillInLimitsStatus(scene, limits_status);
		sendDiagnostics();
	}
}

void Monitoring::fillInSceneStatus(const planning_scene::PlanningScene& scene, diagnostic_msgs::DiagnosticStatus& status)
{
	collision_detection::CollisionRequest req;
	// we need to request contacts to get link pairs to report
	req.contacts = true;
	req.max_contacts = 5;
	req.max_contacts_per_pair = 1;

	collision_detection::CollisionResult res;
	scene.checkCollision(req, res);

	if(res.collision){
		status.level = status.ERROR;
		status.message = "in collision (" + std::to_string( res.contact_count ) + " contacts found)";
		status.values.clear();
		status.values.reserve(res.contacts.size());
		size_t id = 0;
		for(const auto& pair : res.contacts){
			status.values.emplace_back();
			status.values.back().key = "contact " + std::to_string(id++);
			status.values.back().value = pair.first.first + " - " + pair.first.second;
		}
		if(log_msgs)
			ROS_ERROR_STREAM_NAMED(getName(), status.message);
	}
	else {
		status.level = status.OK;
		status.message = "no collision";
		status.values.clear();
	}
}

void Monitoring::fillInLimitsStatus(const planning_scene::PlanningScene&, diagnostic_msgs::DiagnosticStatus& status)
{
	status.level = status.OK;
	status.message = "not implemented";
}
