
#include <boost/lexical_cast.hpp>
#include <string>
#include <bitset>

#include <rtt/Component.hpp>
#include "FRIDiagnostics.hpp"

ORO_CREATE_COMPONENT(LWR::FRIDiagnostics);

namespace LWR{
  
  FRIDiagnostics::FRIDiagnostics(const std::string& name):
    TaskContext(name, PreOperational),
    RobotStatePort("RobotState"),
    FriStatePort("FriState")
  {
    this->addPort("RobotState", RobotStatePort);
	this->addPort("FriState", FriStatePort);
	this->addPort("diagnostic", port_diagnostic);

	this->addProperty("diagnostic_prefix", prop_diagnostic_prefix);
  }

  FRIDiagnostics::~FRIDiagnostics(){
  }

	bool FRIDiagnostics::configureHook(){

		diagnostic.status.resize(2);
		diagnostic.status[0].values.resize(11);
		diagnostic.status[1].values.resize(4);

		diagnostic.status[0].name = prop_diagnostic_prefix + " FRI state";
		diagnostic.status[0].values[0].key = "Kuka System Time";
		diagnostic.status[0].values[1].key = "State";
		diagnostic.status[0].values[2].key = "Quality";
		diagnostic.status[0].values[3].key = "Desired Send Sample Time";
		diagnostic.status[0].values[4].key = "Desired Command Sample Time";
		diagnostic.status[0].values[5].key = "Safety Limits";
		diagnostic.status[0].values[6].key = "Answer Rate";
		diagnostic.status[0].values[7].key = "Latency";
		diagnostic.status[0].values[8].key = "Jitter";
		diagnostic.status[0].values[9].key = "Average Missed Answer Packages";
		diagnostic.status[0].values[10].key = "Total Missed Packages";

		diagnostic.status[1].name = prop_diagnostic_prefix + " robot state";
		diagnostic.status[1].values[0].key = "Power";
		diagnostic.status[1].values[1].key = "Control Strategy";
		diagnostic.status[1].values[2].key = "Error";
		diagnostic.status[1].values[3].key = "Warning";
		return true;
	}

  void FRIDiagnostics::updateHook(){
    RobotStatePort.read(robotstate);
    FriStatePort.read(fristate);
		fri_comm_diagnostics();
		fri_robot_diagnostics();

		port_diagnostic.write(diagnostic);
	}

  void FRIDiagnostics::fri_comm_diagnostics(){
    
    if(fristate.quality==FRI_QUALITY_PERFECT){
			diagnostic.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
			diagnostic.status[0].message = "Communication quality PERFECT";
			diagnostic.status[0].values[2].value = "PERFECT";
    }else if(fristate.quality==FRI_QUALITY_OK){
			diagnostic.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
			diagnostic.status[0].message = "Communication quality OK";
			diagnostic.status[0].values[2].value = "OK";
    }else if(fristate.quality==FRI_QUALITY_BAD){
			diagnostic.status[0].level = diagnostic_msgs::DiagnosticStatus::WARN;
			diagnostic.status[0].message = "Communication quality BAD";
			diagnostic.status[0].values[2].value = "BAD";
    }else{
			diagnostic.status[0].level = diagnostic_msgs::DiagnosticStatus::ERROR;
			diagnostic.status[0].message = "Communication quality UNACCEPTABLE";
			diagnostic.status[0].values[2].value = "UNACCEPTABLE";
    }

		diagnostic.status[0].values[0].value = boost::lexical_cast<std::string>(fristate.timestamp);

		if(fristate.state == FRI_STATE_MON){
			diagnostic.status[0].values[1].value = "monitor";
		}else if(fristate.state == FRI_STATE_CMD){
			diagnostic.status[0].values[1].value = "command";
		}else{
			diagnostic.status[0].values[1].value = "invalid";
		}

		diagnostic.status[0].values[3].value = boost::lexical_cast<std::string>(fristate.desiredMsrSampleTime);
		diagnostic.status[0].values[4].value = boost::lexical_cast<std::string>(fristate.desiredCmdSampleTime);
		diagnostic.status[0].values[5].value = boost::lexical_cast<std::string>(fristate.safetyLimits);
		diagnostic.status[0].values[6].value = boost::lexical_cast<std::string>(fristate.stat.answerRate);
		diagnostic.status[0].values[7].value = boost::lexical_cast<std::string>(fristate.stat.latency);
		diagnostic.status[0].values[8].value = boost::lexical_cast<std::string>(fristate.stat.jitter);
		diagnostic.status[0].values[9].value = boost::lexical_cast<std::string>(fristate.stat.missRate);
		diagnostic.status[0].values[10].value = boost::lexical_cast<std::string>(fristate.stat.missCounter);

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

  void FRIDiagnostics::fri_robot_diagnostics(){
    
    std::bitset<7> power(robotstate.power);
    std::bitset<7> error(robotstate.error);
    std::bitset<7> warning(robotstate.warning);
    
    std::bitset<7> clear;
    if(clear==error){
      if(clear==warning){
				diagnostic.status[1].level = diagnostic_msgs::DiagnosticStatus::OK;
				diagnostic.status[1].message = "All drives OK";
			}else{
				diagnostic.status[1].level = diagnostic_msgs::DiagnosticStatus::WARN;
				diagnostic.status[1].message = "Drives warning";
			}
		}else{
			diagnostic.status[1].level = diagnostic_msgs::DiagnosticStatus::ERROR;
			diagnostic.status[1].message = "Drives Error";
    }
    
		diagnostic.status[1].values[0].value = power.to_string();
		
		if(robotstate.control == FRI_CTRL_POSITION){
			diagnostic.status[1].values[1].value = "Position";
		} else if(robotstate.control == FRI_CTRL_CART_IMP){
			diagnostic.status[1].values[1].value = "Cartesian impedance";
		} else if(robotstate.control == FRI_CTRL_JNT_IMP){
			diagnostic.status[1].values[1].value = "Joint impedance";
		} else {
			diagnostic.status[1].values[1].value = "Invalid";
		}

		diagnostic.status[1].values[2].value = error.to_string();
		diagnostic.status[1].values[3].value = error.to_string();

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
}


