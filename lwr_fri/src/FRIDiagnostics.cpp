
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <diagnostic_msgs/typekit/Types.h>
#include <kuka_lwr_fri/friComm.h>
#include <boost/lexical_cast.hpp>
#include <string>
#include <bitset>
// End of user code

class FRIDiagnostics : public RTT::TaskContext {
public:
  FRIDiagnostics(const std::string & name) : TaskContext(name) {

    prop_prefix = "lwr";
	
    this->addProperty("prefix", prop_prefix);

    this->ports()->addPort("FRIState", port_FRIState).doc("");
    this->ports()->addPort("RobotState", port_RobotState).doc("");

    this->ports()->addPort("Diagnostics", port_Diagnostics).doc("");
  }

  ~FRIDiagnostics(){
  }

  bool configureHook() {
    // Start of user code configureHook
    diagnostic_.status.resize(2);
    diagnostic_.status[0].values.resize(11);
    diagnostic_.status[1].values.resize(4);

    diagnostic_.status[0].name = prop_prefix + " FRI state";
    diagnostic_.status[0].values[0].key = "Kuka System Time";
    diagnostic_.status[0].values[1].key = "State";
    diagnostic_.status[0].values[2].key = "Quality";
    diagnostic_.status[0].values[3].key = "Desired Send Sample Time";
    diagnostic_.status[0].values[4].key = "Desired Command Sample Time";
    diagnostic_.status[0].values[5].key = "Safety Limits";
    diagnostic_.status[0].values[6].key = "Answer Rate";
    diagnostic_.status[0].values[7].key = "Latency";
    diagnostic_.status[0].values[8].key = "Jitter";
    diagnostic_.status[0].values[9].key = "Average Missed Answer Packages";
    diagnostic_.status[0].values[10].key = "Total Missed Packages";

    diagnostic_.status[1].name = prop_prefix + " robot state";
    diagnostic_.status[1].values[0].key = "Power";
    diagnostic_.status[1].values[1].key = "Control Strategy";
    diagnostic_.status[1].values[2].key = "Error";
    diagnostic_.status[1].values[3].key = "Warning";
    // End of user code
    return true;
  }

  bool startHook() {
    // Start of user code startHook
    // TODO Put implementation of startHook here !!!
    // End of user code
    return true;
  }

  void stopHook() {
    // Start of user code stopHook
	  // TODO Put implementation of stopHook here !!!
	  // End of user code
  }

  void updateHook() {
	if( true) {
      doDiagnostics();
    }
  }

private:

  void doDiagnostics() {
    // Start of user code Diagnostics
    tFriIntfState fristate;
    tFriRobotState robotstate;

    port_FRIState.read(fristate);
    port_RobotState.read(robotstate);

    fri_comm_diagnostics(fristate);
    fri_robot_diagnostics(robotstate);

    port_Diagnostics.write(diagnostic_);
	  // End of user code
  }


  RTT::InputPort<tFriIntfState > port_FRIState;
  RTT::InputPort<tFriRobotState > port_RobotState;

  RTT::OutputPort<diagnostic_msgs::DiagnosticArray > port_Diagnostics;

  std::string prop_prefix;


  // Start of user code userData
  diagnostic_msgs::DiagnosticArray diagnostic_;

  void fri_comm_diagnostics(const tFriIntfState &fristate){

    if(fristate.quality==FRI_QUALITY_PERFECT){
      diagnostic_.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
      diagnostic_.status[0].message = "Communication quality PERFECT";
      diagnostic_.status[0].values[2].value = "PERFECT";
    }else if(fristate.quality==FRI_QUALITY_OK){
      diagnostic_.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
      diagnostic_.status[0].message = "Communication quality OK";
      diagnostic_.status[0].values[2].value = "OK";
    }else if(fristate.quality==FRI_QUALITY_BAD){
      diagnostic_.status[0].level = diagnostic_msgs::DiagnosticStatus::WARN;
      diagnostic_.status[0].message = "Communication quality BAD";
      diagnostic_.status[0].values[2].value = "BAD";
    }else{
      diagnostic_.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diagnostic_.status[0].message = "Communication quality UNACCEPTABLE";
      diagnostic_.status[0].values[2].value = "UNACCEPTABLE";
    }

    diagnostic_.status[0].values[0].value = boost::lexical_cast<std::string>(fristate.timestamp);

    if(fristate.state == FRI_STATE_MON){
      diagnostic_.status[0].values[1].value = "monitor";
    }else if(fristate.state == FRI_STATE_CMD){
      diagnostic_.status[0].values[1].value = "command";
    }else{
      diagnostic_.status[0].values[1].value = "invalid";
    }

    diagnostic_.status[0].values[3].value = boost::lexical_cast<std::string>(fristate.desiredMsrSampleTime);
    diagnostic_.status[0].values[4].value = boost::lexical_cast<std::string>(fristate.desiredCmdSampleTime);
    diagnostic_.status[0].values[5].value = boost::lexical_cast<std::string>(fristate.safetyLimits);
    diagnostic_.status[0].values[6].value = boost::lexical_cast<std::string>(fristate.stat.answerRate);
    diagnostic_.status[0].values[7].value = boost::lexical_cast<std::string>(fristate.stat.latency);
    diagnostic_.status[0].values[8].value = boost::lexical_cast<std::string>(fristate.stat.jitter);
    diagnostic_.status[0].values[9].value = boost::lexical_cast<std::string>(fristate.stat.missRate);
    diagnostic_.status[0].values[10].value = boost::lexical_cast<std::string>(fristate.stat.missCounter);

//    stat.add("Kuka System Time",fristate.timestamp);
//    stat.add("State",fristate.state);
//    stat.add("Quality",fristate.quality);
//    stat.add("Desired Send Sample Time",fristate.desiredMsrSampleTime);
//    stat.add("Desired Command Sample Time",fristate.desiredCmdSampleTime);
//    stat.add("Safety Limits",fristate.safetyLimits);
//    stat.add("Answer Rate",fristate.stat.answerRate);
//    stat.add("Latency",fristate.stat.latency);
//    stat.add("Jitter",fristate.stat.jitter);
//    stat.add("Average Missed Answer Packages",fristate.stat.missRate);
//    stat.add("Total Missed Packages",fristate.stat.missCounter);
  }

  void fri_robot_diagnostics(const tFriRobotState &robotstate){

    std::bitset<7> power(robotstate.power);
    std::bitset<7> error(robotstate.error);
    std::bitset<7> warning(robotstate.warning);

    std::bitset<7> clear;
    if(clear==error){
      if(clear==warning){
        diagnostic_.status[1].level = diagnostic_msgs::DiagnosticStatus::OK;
        diagnostic_.status[1].message = "All drives OK";
      }else{
        diagnostic_.status[1].level = diagnostic_msgs::DiagnosticStatus::WARN;
        diagnostic_.status[1].message = "Drives warning";
      }
    }else{
      diagnostic_.status[1].level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diagnostic_.status[1].message = "Drives Error";
    }

    diagnostic_.status[1].values[0].value = power.to_string();

    if(robotstate.control == FRI_CTRL_POSITION){
      diagnostic_.status[1].values[1].value = "Position";
    } else if(robotstate.control == FRI_CTRL_CART_IMP){
      diagnostic_.status[1].values[1].value = "Cartesian impedance";
    } else if(robotstate.control == FRI_CTRL_JNT_IMP){
      diagnostic_.status[1].values[1].value = "Joint impedance";
    } else {
      diagnostic_.status[1].values[1].value = "Invalid";
    }

    diagnostic_.status[1].values[2].value = error.to_string();
    diagnostic_.status[1].values[3].value = error.to_string();

//    stat.add("Power",power.to_string());
//    stat.add("Control Strategy",robotstate.control);
//    stat.add("Error",error.to_string());
//    stat.add("Warning",warning.to_string());
//    stat.add("Temperature Joint 1",robotstate.temperature[0]);
//    stat.add("Temperature Joint 2",robotstate.temperature[1]);
//    stat.add("Temperature Joint 3",robotstate.temperature[2]);
//    stat.add("Temperature Joint 4",robotstate.temperature[3]);
//    stat.add("Temperature Joint 5",robotstate.temperature[4]);
//    stat.add("Temperature Joint 6",robotstate.temperature[5]);
//    stat.add("Temperature Joint 7",robotstate.temperature[6]);

  }

  // End of user code

};

ORO_CREATE_COMPONENT(FRIDiagnostics)

