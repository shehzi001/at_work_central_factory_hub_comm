#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG
#include <ros/ros.h>
#include <protobuf_comm/peer.h>

#include <rockin_msgs/AttentionMessage.pb.h>
#include <rockin_msgs/BeaconSignal.pb.h>
#include <rockin_msgs/BenchmarkState.pb.h>
#include <rockin_msgs/BenchmarkFeedback.pb.h>
#include <rockin_msgs/Camera.pb.h>
#include <rockin_msgs/ConveyorBelt.pb.h>
#include <rockin_msgs/CompressedImage.pb.h>
#include <rockin_msgs/DrillingMachine.pb.h>
#include <rockin_msgs/Image.pb.h>
#include <rockin_msgs/Inventory.pb.h>
#include <rockin_msgs/Order.pb.h>
#include <rockin_msgs/Pose3D.pb.h>
#include <rockin_msgs/Position3D.pb.h>
#include <rockin_msgs/Quaternion.pb.h>
#include <rockin_msgs/RobotInfo.pb.h>
#include <rockin_msgs/Time.pb.h>
#include <rockin_msgs/VersionInfo.pb.h>


//publisher
#include <cfh_robot/AttentionMessage.h>

//subscribe
#include <cfh_robot/CameraControl.h>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <unistd.h>

#include <sstream>

using namespace protobuf_comm;
using namespace rockin_msgs;

class CFHRobot
{
    public:
        /**
         * Ctor.
         */
        CFHRobot(const ros::NodeHandle &nh);

        /**
         * Dtor.
         */
        ~CFHRobot();

        void initialize();

        void createCFHRobot();

        /**
         * Beacon Signal
         *
         * This function sends the beacon signal
         * The function is called from the main loop
         */
        void sendBeacon();

    private:
        /**
         * Copy Ctor.
         */
        CFHRobot(const CFHRobot &other);

        /**
         * Assignment operator
         */
        CFHRobot &operator=(const CFHRobot &other);

        /**
         * Handler for send errors.
         *
         * This handler is called for send errors.
         * Print the message that should be send as Ros Warning
         *
         */
        void handleSendError(std::string msg);

        /**
         * Handler for receive errors.
         *
         * This handler is called for receive errors.
         * Print the message that should be received as a Ros Warning
         *
         */
        void handleReceiveError(boost::asio::ip::udp::endpoint &endpoint,
                                std::string msg);

        /**
         * Handler for receive messages .
         *
         * This handler is called for receiving messages.
         * Main function in this node.
         *
         */
        void handleMessage(boost::asio::ip::udp::endpoint &sender,
                            uint16_t component_id, uint16_t msg_type,
                            std::shared_ptr<google::protobuf::Message> msg);
        /**
         * Example for a send command
         *
         * This is the subscriber to control the QA camera.
         * Use subscriber or Action server/client to send commands to the refbox
         */
         
        void cameraControlCB(cfh_robot::CameraControl msg);

    private:
        /**
         * ROS node handle.
         */
        ros::NodeHandle nh_;

        /**
         * Stores sequence of the bacon messages.
         */
        unsigned long seq_;

        /**
         * The interface to the Protobuf Broadcast peer to communicate with Refbox.
         */
        std::shared_ptr<ProtobufBroadcastPeer> peer_public_;

        /**
         * The interface to the Protobuf Broadcast peer to communicate with Team.
         */
        std::shared_ptr<ProtobufBroadcastPeer> peer_team_;

        /**
         * Stores robot name.
         */
        std::string robot_name_;

        /**
         * Stores team name.
         */
        std::string team_name_;

        /**
         * Publisher to publish attenetion messages.
         */
        ros::Publisher attention_message_pub;

        /**
         * Subscriber to control camera over network through ref box.
         */
        ros::Subscriber camera_conrol_sub;

        /**
         * Parameter to check if refbox is running on local or another machine.
         */
        bool remote_refbox_;

        /**
         * Parameter to store host name(IP address).
         */
        std::string host_name_;

        /**
         * Stores port number on which refbox is receiving or sending messages
         * when the refbox is running on another machine. 
         */
        int public_port_;

        /**
         * Stores port number on which robot/team is receiving or sending messages. 
         * when the refbox is running on another machine.
         */
        int team_port_;

        /**
         * Stores port number on which refbox is sending messages. 
         * when the refbox is running on local machine.
         */
        int public_send_port_;

        /**
         * Stores port number on which refbox is receiving messages. 
         * when the refbox is running on local machine.
         */
        int public_recv_port_;
 
        /**
         * Stores port number on which the team is sending messages. 
         * when the refbox is running on local machine.
         */
        int team_send_port_;
 
        /**
         * Stores port number on which team is receiving messages. 
         * when the refbox is running on local machine.
         */
        int team_recv_port_;
};