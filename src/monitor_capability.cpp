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
		joint_limit_soft_margin = node_handle_.param("monitoring/joint_limit_soft_margin", 0.05);
		joint_close_to_limit_fraction = node_handle_.param("monitoring/joint_close_to_limit_fraction", 0.05);
	};

	void monitoringThread();

private:
	void fillInSceneStatus(const planning_scene::PlanningScene& scene, diagnostic_msgs::DiagnosticStatus& status);
	void fillInLimitsStatus(const planning_scene::PlanningScene& scene, diagnostic_msgs::DiagnosticStatus& status);

	std::thread t;

	ros::Publisher diagnostics;
	bool log_msgs;
	double rate;
	double joint_limit_soft_margin;
	double joint_close_to_limit_fraction;
};
PLUGINLIB_EXPORT_CLASS(Monitoring, move_group::MoveGroupCapability);

void Monitoring::monitoringThread()
{
	diagnostic_msgs::DiagnosticArray da;
	da.status.resize(2);
	auto& scene_status{ da.status[0] };
	scene_status.name = "MoveGroupMonitoring.PlanningSceneCheck";
	auto& limits_status{ da.status[1] };
	limits_status.name = "MoveGroupMonitoring.LimitsCheck";
	auto sendDiagnostics = [&](const ros::Time& time) {
		da.header.stamp = time;
		diagnostics.publish(da);
	};

	ros::Time now;

	ros::Rate r(rate);
	while(ros::ok()){
		r.sleep();
		{
			planning_scene_monitor::LockedPlanningSceneRO locked_scene{ context_->planning_scene_monitor_ };
			now = ros::Time::now();
			const auto& scene { *locked_scene->shared_from_this() };
			fillInSceneStatus(scene, scene_status);
			fillInLimitsStatus(scene, limits_status);
		}
		sendDiagnostics(now);
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
			status.values.back().value = pair.first.first + " <> " + pair.first.second;
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

template <typename T, typename Q>
inline T max(T x, Q y) {
	return std::max(static_cast<int>(x), static_cast<int>(y));
}

void Monitoring::fillInLimitsStatus(const planning_scene::PlanningScene& scene, diagnostic_msgs::DiagnosticStatus& status)
{
	status.level = status.OK;

	const auto& state{ scene.getCurrentState() };

	status.values.clear();
	status.values.reserve(state.getRobotModel()->getJointModels().size());

	auto makeKV = [](std::string key, std::string value){
		diagnostic_msgs::KeyValue kv;
		kv.key = key;
		kv.value = value;
		return kv;
	};

	std::vector<diagnostic_msgs::KeyValue> out_of_bounds;
	std::vector<diagnostic_msgs::KeyValue> close_to_limits;

	for(const auto& joint : state.getRobotModel()->getJointModels()) {
		const double* positions{ state.getJointPositions(joint) };
		const std::vector<moveit::core::VariableBounds>& bounds{ joint->getVariableBounds() };

		for(size_t i = 0; i < joint->getVariableCount(); ++i){
			if(!bounds[i].position_bounded_)
				continue;
			if(positions[i] < bounds[i].min_position_ - joint_limit_soft_margin){
				status.level = status.ERROR;
				std::stringstream report;
				report << positions[i] << " below lower limit " << bounds[i].min_position_;
				out_of_bounds.push_back(makeKV(joint->getVariableNames()[i], report.str()));
			}
			else if(positions[i] > bounds[i].max_position_ + joint_limit_soft_margin){
				status.level = status.ERROR;
				std::stringstream report;
				report << positions[i] << " above upper limit " << bounds[i].max_position_;
				out_of_bounds.push_back(makeKV(joint->getVariableNames()[i], report.str()));
			}
			else if(positions[i] < bounds[i].min_position_ + joint_close_to_limit_fraction*joint->getMaximumExtent()){
				status.level = max(status.level, status.WARN);
				std::stringstream report;
				report << positions[i] << " close to lower limit " << bounds[i].min_position_;
				close_to_limits.push_back(makeKV(joint->getVariableNames()[i], report.str()));
			}
			else if(positions[i] > bounds[i].max_position_ - joint_close_to_limit_fraction*joint->getMaximumExtent()){
				status.level = max(status.level, status.WARN);
				std::stringstream report;
				report << positions[i] << " close to upper limit " << bounds[i].max_position_;
				close_to_limits.push_back(makeKV(joint->getVariableNames()[i], report.str()));
			}
		}
	}

	// always list out of bounds errors first
	status.values.insert(status.values.end(), out_of_bounds.begin(), out_of_bounds.end());
	status.values.insert(status.values.end(), close_to_limits.begin(), close_to_limits.end());

	switch(status.level){
	case diagnostic_msgs::DiagnosticStatus::OK:
		status.message = "joints within limits";
		break;
	case diagnostic_msgs::DiagnosticStatus::WARN:
		status.message = "joints close to limit";
		break;
	case diagnostic_msgs::DiagnosticStatus::ERROR:
		status.message = "joints outside configured limits";
		break;
	default:
		break;
	}

	if(status.level != status.OK && log_msgs){
		std::stringstream ss;
		ss << status.message << "\n";
		for(const auto& entry : status.values)
			ss << "- " << entry.key << " " << entry.value << "\n";
		if(status.level == status.WARN)
			ROS_WARN_STREAM_NAMED(status.name, ss.str());
		else
			ROS_ERROR_STREAM_NAMED(status.name, ss.str());
	}
}
