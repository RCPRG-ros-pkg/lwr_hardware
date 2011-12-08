// Copyright  (C)  2010  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>, 

// Author: Ruben Smits
// Maintainer: Ruben Smits

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <string>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <diagnostic_msgs/typekit/Types.h>
#include <kuka_lwr_fri/friComm.h>

namespace LWR{

using namespace RTT;

  class FRIDiagnostics:public RTT::TaskContext{
  public:
    FRIDiagnostics(const std::string& name="FRIDiagnostics");
    virtual ~FRIDiagnostics();

    virtual bool configureHook();
    virtual bool startHook(){return true;};
    virtual void updateHook();
    virtual void stopHook(){};
    virtual void cleanupHook(){};

  private:

    std::string prop_diagnostic_prefix;

    InputPort<tFriRobotState> RobotStatePort;
    InputPort<tFriIntfState> FriStatePort;

    OutputPort<diagnostic_msgs::DiagnosticArray> port_diagnostic;

    tFriIntfState fristate;
    tFriRobotState robotstate;

    diagnostic_msgs::DiagnosticArray diagnostic;

    void fri_robot_diagnostics();
    void fri_comm_diagnostics();
  };
}
    
