#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include "ros/ros.h"
#include <protobuf_comm/peer.h>

#include <rockin_msgs/AttentionMessage.pb.h>
#include <rockin_msgs/BeaconSignal.pb.h>
#include <rockin_msgs/BenchmarkState.pb.h>
#include <rockin_msgs/BenchmarkFeedback.pb.h>
#include <rockin_msgs/Camera.pb.h>
#include <rockin_msgs/ConveyorBelt.pb.h>
#include <rockin_msgs/CompressedImage.pb.h>
#include <rockin_msgs/DrillingMachine.pb.h>
#include <rockin_msgs/ForceFittingMachine.pb.h>
#include <rockin_msgs/Image.pb.h>
#include <rockin_msgs/Inventory.pb.h>
#include <rockin_msgs/LoggingStatus.pb.h>
#include <rockin_msgs/Order.pb.h>
#include <rockin_msgs/Pose3D.pb.h>
#include <rockin_msgs/Position3D.pb.h>
#include <rockin_msgs/Quaternion.pb.h>
#include <rockin_msgs/RobotInfo.pb.h>
#include <rockin_msgs/RobotStatusReport.pb.h>
#include <rockin_msgs/Time.pb.h>
#include <rockin_msgs/VersionInfo.pb.h>


//publisher
#include <cfh_node_example/AttentionMessage.h>

//subscribe
#include <cfh_node_example/CameraControl.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <unistd.h>

#include <sstream>

using namespace protobuf_comm;
using namespace rockin_msgs;

std::string name, team_name;
unsigned long seq_ = 0;
ProtobufBroadcastPeer *peer_public = NULL, *peer_team = NULL;

//Ros Publisher
ros::Publisher AttentionMessage_pub;

//Ros Subscriber
ros::Subscriber CameraConrol_sub;
/*
 * Handler for send errors.
 *
 * This handler is called for send errors.
 * Print the message that should be send as Ros Warning
 *
 */
void handle_send_error(std::string msg){
  ROS_WARN("Send error: %s\n", msg.c_str());
}

/*
 * Handler for receive errors.
 *
 * This handler is called for receive errors.
 * Print the message that should be received as a Ros Warning
 *
 */
void handle_recv_error(boost::asio::ip::udp::endpoint &endpoint,
                        std::string msg){
  ROS_WARN("Recv error: %s\n", msg.c_str());

}

/*
 * Handler for receive messages .
 *
 * This handler is called for receiving messages.
 * Main function in this node.
 *
 */
void handle_message(boost::asio::ip::udp::endpoint &sender,
                    uint16_t component_id, uint16_t msg_type,
                    std::shared_ptr<google::protobuf::Message> msg){

  std::shared_ptr<AttentionMessage> am;                         /* shared_ptr on the message type */
  if((am = std::dynamic_pointer_cast<AttentionMessage>(msg))){  /* test it is the right message type */
    cfh_node_example::AttentionMessage attention_msg;        /* ros message handling */
    attention_msg.message      = am->message();
    attention_msg.time_to_show = am->time_to_show();
    attention_msg.team         = am->team();
    AttentionMessage_pub.publish(attention_msg);
  }
}

/*
 * Example for a send command
 *
 * This is the subscriber to control the QA camera.
 * Use subscriber or Action server/client to send commands to the refbox
 *
 */
void CameraControl(cfh_node_example::CameraControl msg){
  std::shared_ptr<CameraCommand> cam_control(new CameraCommand); /*create a new message */
  peer_team->send(cam_control);                                  /*send the Message over team peer */
}

/*
 * Beacon Signal
 *
 * This function sends the beacon signal
 * The function is called from the main loop
 */
void send_beacon(){
  timespec start;                            /*generate the timestamp over boost or chrono (C++11) */
  clock_gettime(CLOCK_REALTIME, &start);
  int32_t sec = start.tv_sec;
  int32_t nsec = start.tv_nsec;

  std::shared_ptr<BeaconSignal> signal(new BeaconSignal());  /*create the message */

  Time *time = signal->mutable_time();  /*seperate the time segment of the message */
  time->set_sec(sec);                   /*write the timestamp into the message*/
  time->set_nsec(nsec);

  signal->set_peer_name(name);           /*write the name und co into the message*/
  signal->set_team_name(team_name);
  signal->set_seq(++seq_);               /*increase the sequence number*/
  peer_team->send(signal);               /*send over team peer*/
}
/*
 * Main function
 *
 * Use to create and configure the peers
 *
 */
int main(int argc, char **argv){

  ros::init(argc, argv, "cfh_node");
  ROS_INFO("CFH is running!");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1); // one Hz
  int count = 0;

  /*
   * Confgiuration Parameters
   * - can use the ros parameter
   *   Server to handle this
   */
  std::string hostname = "127.0.0.1";
  int team_port = 4450, public_port = 4444;

  name      = "spqr";
  team_name = "SPQR";

  ROS_INFO("Hostname: %s", hostname.c_str());
  ROS_INFO("Team Port: %i", team_port);
  ROS_INFO("Public Port: %i", public_port);
  ROS_INFO("Name: %s", name.c_str());
  ROS_INFO("Team Name: %s", team_name.c_str());
  /*
   *  peer creation
   *  Section of peer creation
   *  creation the public and team peer
   *
   */
  int public_send_port = 4445, public_recv_port = 4444;
  peer_public = new ProtobufBroadcastPeer(hostname, public_send_port, public_recv_port);       /*create public peer*/

  MessageRegister &message_register = peer_public->message_register();  /*create internal message handler*/
  message_register.add_message_type<AttentionMessage>();                /*added messagetype to the handler*/
  message_register.add_message_type<BeaconSignal>();
  message_register.add_message_type<BenchmarkState>();
  message_register.add_message_type<CompressedImage>();
  message_register.add_message_type<ConveyorBeltStatus>();
  message_register.add_message_type<DrillingMachineStatus>();
  message_register.add_message_type<ForceFittingMachineStatus>();
  message_register.add_message_type<Inventory>();
  message_register.add_message_type<OrderInfo>();
  message_register.add_message_type<RobotInfo>();
  message_register.add_message_type<VersionInfo>();

  int team_send_port = 4451, team_recv_port = 4450;
  peer_team = new ProtobufBroadcastPeer(hostname, team_send_port, team_recv_port, &message_register); /*create team peer and linked to internal message handler*/

  peer_public->signal_received().connect(handle_message);              /*bind the peers to the callback funktions*/
  peer_public->signal_send_error().connect(handle_send_error);
  peer_public->signal_recv_error().connect(handle_recv_error);

  peer_team->signal_received().connect(handle_message);
  peer_team->signal_send_error().connect(handle_send_error);
  peer_team->signal_recv_error().connect(handle_recv_error);

  /*
   * Ros Part
   * creation the Publisher und Subscriber
   * for Ros
   */
  //Publisher
  AttentionMessage_pub = nh.advertise<cfh_node_example::AttentionMessage> ("attention_message", 10);

  //Subscriber
  CameraConrol_sub = nh.subscribe<cfh_node_example::CameraControl>("camera_control", 1000, CameraControl);

  /*
   *  Main loop
   *
   */
  while (ros::ok()) {
    send_beacon();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  delete peer_public;
  delete peer_team;

  // Delete all global objects allocated by libprotobuf
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
