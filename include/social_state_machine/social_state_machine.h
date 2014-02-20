#ifndef SSB_SOCIAL_STATE_MACHINE_
#define SSB_SOCIAL_STATE_MACHINE_

#include<move_base/state_machine.h>
#include<social_state_machine/SocialStateMachineConfig.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/JointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajectoryClient;

enum SocialState { S_PLANNING, S_LOOKUP, S_LOOK_AROUND, S_SPINE_UP, S_LOOK_AT_PATH, 
                   S_NORMAL, S_NEAR_END, S_SPINE_DOWN, S_END_LOOK_AROUND, S_END_LOOK_DOWN, 
                   S_AT_GOAL};
int NUM_STATES = 11;

namespace social_state_machine {

  class SocialStateMachine : public move_base::StateMachine {
    public:
      SocialStateMachine();
      ~SocialStateMachine();
      
      virtual void initialize(tf::TransformListener* tf, move_base::GlobalNavigator* planner, move_base::LocalNavigator* controller);

      virtual void reset();
      void advance();
      virtual void executeCycle(int* status, std::string* message);
      
      
    private:
    void lookdown(){}
    void lookup(){}
    void lookaround(){}
    void spineup(){}
    void lookatpath(){}
    void spinedown(){}
    
    double end_distance_;


    int state_;
    bool* enabled_;
    
    TrajectoryClient head_, spine_;
    control_msgs::JointTrajectoryGoal head_goal_, spine_goal_;

  dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig> *dsrv_;
  void reconfigureCB(social_state_machine::SocialStateMachineConfig &config, uint32_t level);
  
  };
};

#endif
