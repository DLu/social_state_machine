#ifndef SSB_SOCIAL_STATE_MACHINE_
#define SSB_SOCIAL_STATE_MACHINE_

#include<move_base/state_machine.h>
#include<social_state_machine/SocialStateMachineConfig.h>

namespace social_state_machine {

  class SocialStateMachine : public move_base::StateMachine {
    public:
      SocialStateMachine();
      ~SocialStateMachine();
      
      virtual void initialize(tf::TransformListener* tf, move_base::GlobalNavigator* planner, move_base::LocalNavigator* controller);

      virtual void reset();
      virtual std::string executeCycle();

    private:


  dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig> *dsrv_;
  void reconfigureCB(social_state_machine::SocialStateMachineConfig &config, uint32_t level);
  
  };
};

#endif
