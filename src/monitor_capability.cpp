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
		log_msgs = node_handle_.param("monitoring/log_msgs", false);
		rate = node_handle_.param("monitoring/rate", 0.5);
		joint_limit_soft_margin = node_handle_.param("monitoring/joint_limit_soft_margin", 0.05);
		joint_close_to_limit_fraction = node_handle_.param("monitoring/joint_close_to_limit_fraction", 0.05);

		diagnostics = root_node_handle_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1, true);
		t = std::thread([this]{ this->monitoringThread(); });
	};

	void monitoringThread();

private:
	template <typename Inserter>
	void fillInSceneStatus(const planning_scene::PlanningScene& scene, Inserter msgs);
	template <typename Inserter>
	void fillInLimitsStatus(const planning_scene::PlanningScene& scene, Inserter msgs);

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
	diagnostic_msgs::DiagnosticArray msgs;

	ros::Time now;

	ros::Rate r(rate);
	while(ros::ok()){
		r.sleep();
		{
			planning_scene_monitor::LockedPlanningSceneRO locked_scene{ context_->planning_scene_monitor_ };
			now = ros::Time::now();
			const auto& scene { *locked_scene->shared_from_this() };
			msgs.status.clear();
			fillInSceneStatus(scene, std::back_inserter(msgs.status));
			fillInLimitsStatus(scene, std::back_inserter(msgs.status));
		}
		msgs.header.stamp = now;
		diagnostics.publish(msgs);
	}
}

template <typename Inserter>
void Monitoring::fillInSceneStatus(const planning_scene::PlanningScene& scene, Inserter msgs)
{
	const size_t number_of_reported_collisions{ 3 };
	const std::string prefix{ "move_group.SceneCollision" };

	collision_detection::CollisionRequest req;
	// we need to request contacts to get link pairs to report
	req.contacts = true;
	req.max_contacts = number_of_reported_collisions;
	req.max_contacts_per_pair = 1;

	collision_detection::CollisionResult res;
	scene.checkCollision(req, res);

	size_t entry_id = 0;

	if(res.collision){
		for(const auto& pair : res.contacts){
			diagnostic_msgs::DiagnosticStatus entry;
			entry.level = entry.ERROR;
			entry.name.append(prefix).append(std::to_string(entry_id++));
			entry.message.append(pair.first.first).append(" <> ").append(pair.first.second);
			ROS_ERROR_STREAM_COND_NAMED(log_msgs, "monitoring.SceneCollision", "unwanted collision " << entry.message);
			msgs = std::move(entry);
		}
	}

	while(entry_id < number_of_reported_collisions){
		diagnostic_msgs::DiagnosticStatus entry;
		entry.level = entry.OK;
		entry.name.append(prefix).append(std::to_string(entry_id++));
		entry.message = "no collision";
		msgs = std::move(entry);
	}
}

template <typename Inserter>
void Monitoring::fillInLimitsStatus(const planning_scene::PlanningScene& scene, Inserter msgs)
{
	const std::string prefix{ "move_group.JointLimits." };
  const std::string log_name{ "monitoring.JointLimits" };

	auto makeKV = [](std::string key, const auto& value){
		diagnostic_msgs::KeyValue kv;
		kv.key = key;
		kv.value = std::to_string(value);
		return kv;
	};

	const auto& state{ scene.getCurrentState() };

	for(const auto& joint : state.getRobotModel()->getJointModels()) {
		const double* positions{ state.getJointPositions(joint) };
		const std::vector<moveit::core::VariableBounds>& bounds{ joint->getVariableBounds() };

		for(size_t i = 0; i < joint->getVariableCount(); ++i){
			if(!bounds[i].position_bounded_)
				continue;

			diagnostic_msgs::DiagnosticStatus entry;
			entry.name = prefix + joint->getVariableNames()[i];

			if(positions[i] < bounds[i].min_position_ - joint_limit_soft_margin){
				entry.level = entry.ERROR;
				std::stringstream report;
				report << positions[i] << " outside lower limit " << bounds[i].min_position_;
				entry.message = report.str();
				entry.values.push_back(makeKV("lower_limit", bounds[i].min_position_));
				entry.values.push_back(makeKV("position", positions[i]));
				ROS_ERROR_STREAM_COND_NAMED(log_msgs, log_name, entry.name + ": " + entry.message);
			}
			else if(positions[i] > bounds[i].max_position_ + joint_limit_soft_margin){
				entry.level = entry.ERROR;
				std::stringstream report;
				report << positions[i] << " outside upper limit " << bounds[i].min_position_;
				entry.message = report.str();
				entry.values.push_back(makeKV("upper_limit", bounds[i].max_position_));
				entry.values.push_back(makeKV("position", positions[i]));
				ROS_ERROR_STREAM_COND_NAMED(log_msgs, log_name, entry.name + ": " + entry.message);
			}
			else if(positions[i] < bounds[i].min_position_ + joint_close_to_limit_fraction*joint->getMaximumExtent()){
				entry.level = entry.WARN;
				std::stringstream report;
				report << positions[i] << " close to lower limit " << bounds[i].min_position_;
				entry.message = report.str();
				entry.values.push_back(makeKV("lower_limit", bounds[i].max_position_));
				entry.values.push_back(makeKV("position", positions[i]));
				ROS_WARN_STREAM_COND_NAMED(log_msgs, log_name, entry.name + ": " + entry.message);
			}
			else if(positions[i] > bounds[i].max_position_ - joint_close_to_limit_fraction*joint->getMaximumExtent()){
				entry.level = entry.WARN;
				std::stringstream report;
				report << positions[i] << " close to upper limit " << bounds[i].min_position_;
				entry.message = report.str();
				entry.values.push_back(makeKV("upper_limit", bounds[i].max_position_));
				entry.values.push_back(makeKV("position", positions[i]));
				ROS_WARN_STREAM_COND_NAMED(log_msgs, log_name, entry.name + ": " + entry.message);
			}
			else {
				entry.level = entry.OK;
			}
			msgs = entry;
		}
	}
}
