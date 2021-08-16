/*
Created on Mon Dec  2

@author: mahmoud
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>

#include<pedsim_msgs/TrackedPersons.h>
#include<pedsim_msgs/AgentStates.h>
#include<pedsim_msgs/AgentState.h>


namespace gazebo
{
    class ActorPosesPlugin : public WorldPlugin{
        public:
            ActorPosesPlugin() : WorldPlugin(){}

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
            this->world_ = _world;
            if (!ros::isInitialized()){
                ROS_ERROR("ROS not initialized");
                return;
            }
            rosNode.reset(new ros::NodeHandle("gazebo_client"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents", 1,boost::bind(&ActorPosesPlugin::OnRosMsg, this, _1), ros::VoidPtr(),&rosQueue);
            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&ActorPosesPlugin::QueueThread, this));
            // in case you need to change/modify model on update
            // this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ActorPosesPlugin::OnUpdate, this));
        }


        public:
            // call back function when receive rosmsg
            void OnRosMsg( const pedsim_msgs::AgentStatesConstPtr msg) {
//              ROS_INFO ("OnRosMsg ... ");
                std::string model_name;
#if GAZEBO_MAJOR_VERSION < 9
                for(unsigned int mdl = 0; mdl < world_->GetModelCount(); mdl++) {
#else
                for(unsigned int mdl = 0; mdl < world_->ModelCount(); mdl++) {
#endif
                    physics::ModelPtr  tmp_model;
#if GAZEBO_MAJOR_VERSION < 9
                    tmp_model = world_->GetModel(mdl);
#else
                    tmp_model = world_->ModelByIndex(mdl);
#endif
                    std::string frame_id;
                    frame_id = tmp_model->GetName();

                    // Samliu 20210617
                    int agent_idx;
                    if(sscanf(frame_id.c_str(), "actor_%d%*s", &agent_idx)){
                        for(int i = 0; i < msg->agent_states.size(); i++){
                            if(agent_idx == msg->agent_states[i].id){
                                ignition::math::Pose3d gzb_pose;
                                gzb_pose.Pos().Set( msg->agent_states[i].pose.position.x,
                                                    msg->agent_states[i].pose.position.y,
                                                    msg->agent_states[i].pose.position.z + kModelOffsetZ);
                                gzb_pose.Rot().Set(msg->agent_states[i].pose.orientation.w,
                                                   msg->agent_states[i].pose.orientation.x,
                                                   msg->agent_states[i].pose.orientation.y,
                                                   msg->agent_states[i].pose.orientation.z);

                                try {
                                    tmp_model->SetWorldPose(gzb_pose);
                                } catch(gazebo::common::Exception gz_ex) {
                                    ROS_ERROR("Error setting pose %s - %s", frame_id.c_str(), gz_ex.GetErrorStr().c_str());
                                }
                            }
                        }
                    }   // endif sscanf success
               } // end if callback func

          }


        // ROS helper function that processes messages
        private: void QueueThread() {
            static const double timeout = 0.2;
            while (rosNode->ok()) {
                rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    private:
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        physics::WorldPtr world_;
        event::ConnectionPtr updateConnection_;
        const float kModelOffsetZ = 0; // 0.75;

    };
    GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
}


