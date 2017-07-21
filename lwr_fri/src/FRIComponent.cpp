#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include <lwr_msgs/FriRobotState.h>
#include <lwr_msgs/FriIntfState.h>

#include <tf_conversions/tf_kdl.h>

#include <Eigen/Dense>

#include <vector>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>

#ifndef HAVE_RTNET

#define rt_dev_socket socket
#define rt_dev_setsockopt setsockopt
#define rt_dev_bind bind
#define rt_dev_recvfrom recvfrom
#define rt_dev_sendto sendto
#define rt_dev_close close

#else
#include <rtdm/rtdm.h>
#endif

typedef Eigen::Matrix<double, 7, 7> Matrix77d;
// End of user code

class FRIComponent : public RTT::TaskContext {
public:
  FRIComponent(const std::string & name) :
    TaskContext(name, PreOperational),
    port_CartesianImpedanceCommand("CartesianImpedanceCommand_INPORT"),
    port_CartesianWrenchCommand("CartesianWrenchCommand_INPORT"),
    port_CartesianPositionCommand("CartesianPositionCommand_INPORT"),
    port_JointImpedanceCommand("JointImpedanceCommand_INPORT"),
    port_JointPositionCommand("JointPositionCommand_INPORT"),
    port_JointTorqueCommand("JointTorqueCommand_INPORT"),
    port_KRL_CMD("KRL_CMD_INPORT"),
    port_CartesianWrench("CartesianWrench_OUTPORT", true),
    port_RobotState("RobotState_OUTPORT", true),
    port_FRIState("FRIState_OUTPORT", true),
    port_JointVelocity("JointVelocity_OUTPORT", true),
    port_CartesianVelocity("CartesianVelocity_OUTPORT", true),
    port_CartesianPosition("CartesianPosition_OUTPORT", true),
    port_MassMatrix("MassMatrix_OUTPORT", true),
    port_Jacobian("Jacobian_OUTPORT", true),
    port_JointTorque("JointTorque_OUTPORT", true),
    port_GravityTorque("GravityTorque_OUTPORT", true),
    port_JointPosition("JointPosition_OUTPORT", true) {

    prop_fri_port = -1;
	
    this->addProperty("fri_port", prop_fri_port);
    this->addProperty("joint_offset", prop_joint_offset);

    this->ports()->addPort(port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort(port_CartesianWrenchCommand).doc("");
    this->ports()->addPort(port_CartesianPositionCommand).doc("");
    this->ports()->addPort(port_JointImpedanceCommand).doc("");
    this->ports()->addPort(port_JointPositionCommand).doc("");
    this->ports()->addPort(port_JointTorqueCommand).doc("");
    this->ports()->addPort(port_KRL_CMD).doc("");

    this->ports()->addPort(port_CartesianWrench).doc("");
    this->ports()->addPort(port_RobotState).doc("");
    this->ports()->addPort(port_FRIState).doc("");
    this->ports()->addPort(port_JointVelocity).doc("");
    this->ports()->addPort(port_CartesianVelocity).doc("");
    this->ports()->addPort(port_CartesianPosition).doc("");
    this->ports()->addPort(port_MassMatrix).doc("");
    this->ports()->addPort(port_Jacobian).doc("");
    this->ports()->addPort(port_JointTorque).doc("");
    this->ports()->addPort(port_GravityTorque);
    this->ports()->addPort(port_JointPosition).doc("");
  }

  ~FRIComponent(){
  }

  bool configureHook() {
    RTT::Logger::In in(getName() + "::configureHook");

    // Start of user code configureHook
    jac_.resize(LBR_MNJ);

    port_Jacobian.setDataSample(jac_);

    if (prop_fri_port <= 0) {
        RTT::log(RTT::Error) << "property 'fri_port' is not set" << RTT::endlog();
        return false;
    }

    if (prop_joint_offset.size() != 7) {
        RTT::log(RTT::Error) << "property 'joint_offset' is not set" << RTT::endlog();
        return false;
    }

    if (fri_create_socket() != 0) {
        RTT::log(RTT::Error) << "could not create socket" << RTT::endlog();
        return false;
    }
    // End of user code
    return true;
  }

  bool startHook() {
    // Start of user code startHook
    // End of user code
    return true;
  }

  void stopHook() {
    // Start of user code stopHook
    // End of user code
  }

  void updateHook() {
    if( true) {
      doComm();
    }
  }

private:

  void doComm() {
    // Start of user code Comm

    geometry_msgs::Pose cart_pos, cart_pos_cmd;
    geometry_msgs::Wrench cart_wrench, cart_wrench_cmd;
    geometry_msgs::Twist cart_twist;
    Matrix77d mass;
    lwr_fri::FriJointImpedance jnt_imp_cmd;
    lwr_fri::CartesianImpedance cart_imp_cmd;

//    std::cout << "reading fri..." << std::endl;
    //Read:
    if (fri_recv() == 0) {
//      std::cout << "reading fri ok" << std::endl;

      KDL::Frame baseFrame(
          KDL::Rotation::RPY(m_msr_data.krl.realData[3] * M_PI / 180.0,
              m_msr_data.krl.realData[4] * M_PI / 180.0,
              m_msr_data.krl.realData[5] * M_PI / 180.0),
          KDL::Vector(m_msr_data.krl.realData[0] / 1000.0,
              m_msr_data.krl.realData[1] / 1000.0,
              m_msr_data.krl.realData[2] / 1000.0));

      // Fill in fri_joint_state and joint_state
      for (unsigned int i = 0; i < LBR_MNJ; i++) {
        grav_trq_[i] = m_msr_data.data.gravity[i];
        jnt_trq_[i] = m_msr_data.data.estExtJntTrq[i];
        jnt_pos_[i] = m_msr_data.data.msrJntPos[i] + prop_joint_offset[i];
        jnt_vel_[i] = (jnt_pos_[i] - jnt_pos_old_[i]) / m_msr_data.intf.desiredMsrSampleTime;
        jnt_pos_old_[i] = jnt_pos_[i];
      }

      geometry_msgs::Quaternion quat;
      KDL::Frame cartPos;
      cartPos.M = KDL::Rotation(m_msr_data.data.msrCartPos[0],
          m_msr_data.data.msrCartPos[1], m_msr_data.data.msrCartPos[2],
          m_msr_data.data.msrCartPos[4], m_msr_data.data.msrCartPos[5],
          m_msr_data.data.msrCartPos[6], m_msr_data.data.msrCartPos[8],
          m_msr_data.data.msrCartPos[9], m_msr_data.data.msrCartPos[10]);
      cartPos.p.x(m_msr_data.data.msrCartPos[3]);
      cartPos.p.y(m_msr_data.data.msrCartPos[7]);
      cartPos.p.z(m_msr_data.data.msrCartPos[11]);
      cartPos = baseFrame * cartPos;
      tf::PoseKDLToMsg(cartPos, cart_pos);

      KDL::Twist v = KDL::diff(T_old, cartPos, m_msr_data.intf.desiredMsrSampleTime);
      v = cartPos.M.Inverse() * v;
      T_old = cartPos;
      tf::TwistKDLToMsg(v, cart_twist);

      cart_wrench.force.x = m_msr_data.data.estExtTcpFT[0];
      cart_wrench.force.y = m_msr_data.data.estExtTcpFT[1];
      cart_wrench.force.z = m_msr_data.data.estExtTcpFT[2];
      cart_wrench.torque.x = m_msr_data.data.estExtTcpFT[5];
      cart_wrench.torque.y = m_msr_data.data.estExtTcpFT[4];
      cart_wrench.torque.z = m_msr_data.data.estExtTcpFT[3];

      for (int i = 0; i < FRI_CART_VEC; i++)
        for (int j = 0; j < LBR_MNJ; j++)
          jac_(i, j) = m_msr_data.data.jacobian[i * LBR_MNJ + j];
      //Kuka uses Tx, Ty, Tz, Rz, Ry, Rx convention, so we need to swap Rz and Rx
      jac_.data.row(3).swap(jac_.data.row(5));

      for (unsigned int i = 0; i < LBR_MNJ; i++) {
        for (unsigned int j = 0; j < LBR_MNJ; j++) {
          mass(i, j) = m_msr_data.data.massMatrix[LBR_MNJ * i + j];
        }
      }

      //Fill in datagram to send:
      m_cmd_data.head.datagramId = FRI_DATAGRAM_ID_CMD;
      m_cmd_data.head.packetSize = sizeof(tFriCmdData);
      m_cmd_data.head.sendSeqCount = ++counter;
      m_cmd_data.head.reflSeqCount = m_msr_data.head.sendSeqCount;

      //Process KRL CMD

      if (!(m_msr_data.krl.boolData & (1 << 0))) {
//        std::cout << "can send KRL command" << std::endl;
        std_msgs::Int32 x;
        if (port_KRL_CMD.read(x) == RTT::NewData) {
          std::cout << "read KRL command" << std::endl;
          m_cmd_data.krl.intData[0] = x.data;
          m_cmd_data.krl.boolData |= (1 << 0);
        }
      } else {
        m_cmd_data.krl.boolData &= ~(1 << 0);
      }

      if (!isPowerOn()) {
        // necessary to write cmd if not powered on. See kuka FRI user manual p6 and friremote.cpp:
        for (int i = 0; i < LBR_MNJ; i++) {
          m_cmd_data.cmd.jntPos[i] = m_msr_data.data.cmdJntPos[i]
              + m_msr_data.data.cmdJntPosFriOffset[i];
        }
      }
      if (m_msr_data.intf.state == FRI_STATE_MON || !isPowerOn()) {
        // joint position control capable modes:
        if (m_msr_data.robot.control == FRI_CTRL_POSITION
            || m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTPOS;
          for (unsigned int i = 0; i < LBR_MNJ; i++) {
            // see note above with !isPowerOn()
            // the user manual speaks of 'mimic msr.data.msrCmdJntPos' which is ambiguous.
            // on the other hand, the friremote.cpp will send this whenever (!isPowerOn() || state != FRI_STATE_CMD)
            // so we mimic the kuka reference code here...
            m_cmd_data.cmd.jntPos[i] = m_msr_data.data.cmdJntPos[i]
                + m_msr_data.data.cmdJntPosFriOffset[i];
          }
        }
        // Additional flags are set in joint impedance mode:
        if (m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          m_cmd_data.cmd.cmdFlags |= FRI_CMD_JNTTRQ;
          m_cmd_data.cmd.cmdFlags |= FRI_CMD_JNTSTIFF | FRI_CMD_JNTDAMP;
          for (unsigned int i = 0; i < LBR_MNJ; i++) {
            m_cmd_data.cmd.addJntTrq[i] = 0.0;
            m_cmd_data.cmd.jntStiffness[i] = 0;
            m_cmd_data.cmd.jntDamping[i] = 0.7;
          }
        }
        if (m_msr_data.robot.control == FRI_CTRL_CART_IMP) {
          m_cmd_data.cmd.cmdFlags = FRI_CMD_CARTPOS | FRI_CMD_TCPFT;
          m_cmd_data.cmd.cmdFlags |= FRI_CMD_CARTSTIFF | FRI_CMD_CARTDAMP;
          for (unsigned int i = 0; i < FRI_CART_FRM_DIM; i++)
            m_cmd_data.cmd.cartPos[i] = m_msr_data.data.msrCartPos[i];
          for (unsigned int i = 0; i < FRI_CART_VEC; i++)
            m_cmd_data.cmd.addTcpFT[i] = 0.0;
          for (unsigned int i = 0; i < FRI_CART_VEC / 2; i++) {
            //Linear part;
            m_cmd_data.cmd.cartStiffness[i] = 0;
            m_cmd_data.cmd.cartDamping[i] = 0.7;
            //rotational part;
            m_cmd_data.cmd.cartStiffness[i + FRI_CART_VEC / 2] = 0;
            m_cmd_data.cmd.cartDamping[i + FRI_CART_VEC / 2] = 0.7;
          }
        }
      }
      //Only send if state is in FRI_STATE_CMD and drives are powerd
      if ((m_msr_data.intf.state == FRI_STATE_CMD) && isPowerOn()) {
        //Valid ports in joint position and joint impedance mode
        if (m_msr_data.robot.control == FRI_CTRL_POSITION
            || m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          //Read desired positions
          if (port_JointPositionCommand.read(jnt_pos_cmd_) == RTT::NewData) {
            if (jnt_pos_cmd_.size() == LBR_MNJ) {
              for (unsigned int i = 0; i < LBR_MNJ; i++)
                m_cmd_data.cmd.jntPos[i] = jnt_pos_cmd_[i];
            } else
              RTT::log(RTT::Warning) << "Size of " << port_JointPositionCommand.getName()
                  << " not equal to " << LBR_MNJ << RTT::endlog();
          }
        }
        //Valid ports only in joint impedance mode
        if (m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          //Read desired additional joint torques
          if (port_JointTorqueCommand.read(jnt_trq_cmd_)
              == RTT::NewData) {
            //Check size
            if (jnt_trq_cmd_.size() == LBR_MNJ) {
              for (unsigned int i = 0; i < LBR_MNJ; i++)
                m_cmd_data.cmd.addJntTrq[i] = jnt_trq_cmd_[i];
            } else
              RTT::log(RTT::Warning) << "Size of " << port_JointTorqueCommand.getName()
                  << " not equal to " << LBR_MNJ << RTT::endlog();

          }
          else {
              RTT::log(RTT::Error) << "could not read joint torque command" << RTT::endlog();
            return;
          }

          //Read desired joint impedance
          if (port_JointImpedanceCommand.read(jnt_imp_cmd) == RTT::NewData) {
            for (unsigned int i = 0; i < LBR_MNJ; i++) {
              m_cmd_data.cmd.jntStiffness[i] =
                  jnt_imp_cmd.stiffness[i];
              m_cmd_data.cmd.jntDamping[i] = jnt_imp_cmd.damping[i];
            }
          }
        } else if (m_msr_data.robot.control == FRI_CTRL_CART_IMP) {
          if (port_CartesianPositionCommand.read(cart_pos_cmd) == RTT::NewData) {
            KDL::Rotation rot = KDL::Rotation::Quaternion(
                cart_pos_cmd.orientation.x, cart_pos_cmd.orientation.y,
                cart_pos_cmd.orientation.z, cart_pos_cmd.orientation.w);
            m_cmd_data.cmd.cartPos[0] = rot.data[0];
            m_cmd_data.cmd.cartPos[1] = rot.data[1];
            m_cmd_data.cmd.cartPos[2] = rot.data[2];
            m_cmd_data.cmd.cartPos[4] = rot.data[3];
            m_cmd_data.cmd.cartPos[5] = rot.data[4];
            m_cmd_data.cmd.cartPos[6] = rot.data[5];
            m_cmd_data.cmd.cartPos[8] = rot.data[6];
            m_cmd_data.cmd.cartPos[9] = rot.data[7];
            m_cmd_data.cmd.cartPos[10] = rot.data[8];

            m_cmd_data.cmd.cartPos[3] = cart_pos_cmd.position.x;
            m_cmd_data.cmd.cartPos[7] = cart_pos_cmd.position.y;
            m_cmd_data.cmd.cartPos[11] = cart_pos_cmd.position.z;
          }
          else {
            return;
          }

          if (port_CartesianWrenchCommand.read(cart_wrench_cmd) == RTT::NewData) {
            m_cmd_data.cmd.addTcpFT[0] = cart_wrench_cmd.force.x;
            m_cmd_data.cmd.addTcpFT[1] = cart_wrench_cmd.force.y;
            m_cmd_data.cmd.addTcpFT[2] = cart_wrench_cmd.force.z;
            m_cmd_data.cmd.addTcpFT[3] = cart_wrench_cmd.torque.z;
            m_cmd_data.cmd.addTcpFT[4] = cart_wrench_cmd.torque.y;
            m_cmd_data.cmd.addTcpFT[5] = cart_wrench_cmd.torque.x;
          }

          if (port_CartesianImpedanceCommand.read(cart_imp_cmd) == RTT::NewData) {
            m_cmd_data.cmd.cartStiffness[0] = cart_imp_cmd.stiffness.linear.x;
            m_cmd_data.cmd.cartStiffness[1] = cart_imp_cmd.stiffness.linear.y;
            m_cmd_data.cmd.cartStiffness[2] = cart_imp_cmd.stiffness.linear.z;
            m_cmd_data.cmd.cartStiffness[5] = cart_imp_cmd.stiffness.angular.x;
            m_cmd_data.cmd.cartStiffness[4] = cart_imp_cmd.stiffness.angular.y;
            m_cmd_data.cmd.cartStiffness[3] = cart_imp_cmd.stiffness.angular.z;
            m_cmd_data.cmd.cartDamping[0] = cart_imp_cmd.damping.linear.x;
            m_cmd_data.cmd.cartDamping[1] = cart_imp_cmd.damping.linear.y;
            m_cmd_data.cmd.cartDamping[2] = cart_imp_cmd.damping.linear.z;
            m_cmd_data.cmd.cartDamping[5] = cart_imp_cmd.damping.angular.x;
            m_cmd_data.cmd.cartDamping[4] = cart_imp_cmd.damping.angular.y;
            m_cmd_data.cmd.cartDamping[3] = cart_imp_cmd.damping.angular.z;
          }
        } else if (m_msr_data.robot.control == FRI_CTRL_OTHER) {
          this->error();
        }
      }						//End command mode

      //Put robot and fri state on the ports(no parsing)
      lwr_msgs::FriRobotState rs;
      rs.power = m_msr_data.robot.power;
      rs.control = m_msr_data.robot.control;
      rs.error = m_msr_data.robot.error;
      rs.warning = m_msr_data.robot.warning;
      for (int i = 0; i < 7; ++i) {
          rs.temperature[i] = m_msr_data.robot.temperature[i];
      }
      port_RobotState.write(rs);

      lwr_msgs::FriIntfState fs;
      fs.timestamp = m_msr_data.intf.timestamp;
      fs.state = m_msr_data.intf.state;
      fs.quality = m_msr_data.intf.quality;
      fs.desiredMsrSampleTime = m_msr_data.intf.desiredMsrSampleTime;
      fs.desiredCmdSampleTime = m_msr_data.intf.desiredCmdSampleTime;
      fs.safetyLimits = m_msr_data.intf.safetyLimits;
      fs.stat.answerRate = m_msr_data.intf.stat.answerRate;
      fs.stat.latency = m_msr_data.intf.stat.latency;
      fs.stat.jitter = m_msr_data.intf.stat.jitter;
      fs.stat.missRate = m_msr_data.intf.stat.missRate;
      fs.stat.missCounter = m_msr_data.intf.stat.missCounter;
      port_FRIState.write(fs);

      port_JointPosition.write(jnt_pos_);
      port_JointVelocity.write(jnt_vel_);
      port_JointTorque.write(jnt_trq_);
      port_GravityTorque.write(grav_trq_);

      port_CartesianPosition.write(cart_pos);
      port_CartesianVelocity.write(cart_twist);
      port_CartesianWrench.write(cart_wrench);

      port_Jacobian.write(jac_);
      port_MassMatrix.write(mass);

      fri_send();
    }
    else {
//      std::cout << "reading fri failed" << std::endl;
    }

    // End of user code
  }

  typedef Eigen::Matrix<double, LBR_MNJ, 1>  Joints;

  RTT::InputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
  RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
  RTT::InputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
  RTT::InputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
  RTT::InputPort<Joints > port_JointPositionCommand;
  RTT::InputPort<Joints > port_JointTorqueCommand;
  RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;

  RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench;
  RTT::OutputPort<lwr_msgs::FriRobotState > port_RobotState;
  RTT::OutputPort<lwr_msgs::FriIntfState > port_FRIState;
  RTT::OutputPort<Joints > port_JointVelocity;
  RTT::OutputPort<geometry_msgs::Twist > port_CartesianVelocity;
  RTT::OutputPort<geometry_msgs::Pose > port_CartesianPosition;
  RTT::OutputPort<Matrix77d > port_MassMatrix;
  RTT::OutputPort<KDL::Jacobian > port_Jacobian;
  RTT::OutputPort<Joints > port_JointTorque;
  RTT::OutputPort<Joints > port_GravityTorque;
  RTT::OutputPort<Joints > port_JointPosition;

  int prop_fri_port;
  std::vector<double> prop_joint_offset;

  // Start of user code userData
  Joints jnt_pos_;
  Joints jnt_pos_old_;
  Joints jnt_trq_;
  Joints grav_trq_;
  Joints jnt_vel_;

  Joints jnt_pos_cmd_;
  Joints jnt_trq_cmd_;

  KDL::Jacobian jac_;

  KDL::Frame T_old;

  int m_socket, m_remote_port, m_control_mode;
  std::string joint_names_prefix;
  uint16_t counter, fri_state_last;
  struct sockaddr_in m_remote_addr;
  socklen_t m_sock_addr_len;

  tFriMsrData m_msr_data;
  tFriCmdData m_cmd_data;

  int fri_recv() {
    int n = rt_dev_recvfrom(m_socket, (void*) &m_msr_data, sizeof(m_msr_data),
        0, (sockaddr*) &m_remote_addr, &m_sock_addr_len);
    if (sizeof(tFriMsrData) != n) {
      RTT::log(RTT::Error) << "bad packet length: " << n << ", expected: "
          << sizeof(tFriMsrData) << "errno: " << errno << RTT::endlog();
      return -1;
    }
    return 0;
  }

  int fri_send() {
    if (0
        > rt_dev_sendto(m_socket, (void*) &m_cmd_data, sizeof(m_cmd_data), 0,
            (sockaddr*) &m_remote_addr, m_sock_addr_len)) {
      RTT::log(RTT::Error) << "Sending datagram failed."
          << ntohs(m_remote_addr.sin_port) << RTT::endlog();
      return -1;
    }
    return 0;
  }

  bool isPowerOn() { return m_msr_data.robot.power!=0; }

  int fri_create_socket() {

    if (m_socket != 0) {
      rt_dev_close(m_socket);
    }
    m_socket = rt_dev_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    rt_dev_setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 5000;

    if (setsockopt (m_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) {
        RTT::log(RTT::Error) << "setsockopt failed" << RTT::endlog();
        return -2;
    }

    struct sockaddr_in local_addr;
    bzero((char *) &local_addr, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(prop_fri_port);

    if (rt_dev_bind(m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
      RTT::log(RTT::Error) << "Binding of port failed with errno " << errno << RTT::endlog();
      return -1;
    }

    return 0;
  }
  // End of user code

};

ORO_CREATE_COMPONENT(FRIComponent)

