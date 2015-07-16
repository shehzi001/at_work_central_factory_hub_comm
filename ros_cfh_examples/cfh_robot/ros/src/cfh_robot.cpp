#include <cfh_robot/cfh_robot.h>


CFHRobot::CFHRobot(const ros::NodeHandle &nh):
    nh_(nh), seq_(0), 
    peer_public_(NULL),
    peer_team_(NULL)
{
    initialize();

    //Publisher
    attention_message_pub = nh_.advertise<cfh_robot::AttentionMessage> ("attention_message", 10);

    //Subscriber
    camera_conrol_sub = nh_.subscribe<cfh_robot::CameraControl>("camera_control", 1000, 
                                                                        &CFHRobot::cameraControlCB, this);

    createCFHRobot();
}

CFHRobot::~CFHRobot()
{
    // Delete all global objects allocated by libprotobuf
    google::protobuf::ShutdownProtobufLibrary();
}

void CFHRobot::initialize()
{
    ros::param::param<bool>("~remote_refbox", remote_refbox_, false);
    ros::param::param<std::string>("~host_name", host_name_, "localhost");

    //Paramters to use when ref box is running on remote machine.
    ros::param::param<int>("~public_port", public_port_, 4444);
    ros::param::param<int>("~team_port", team_port_, 4450);

    //Paramters to use when ref box is running on same machine as client.
    ros::param::param<int>("~refbox_send_port", public_recv_port_ , 4444);
    ros::param::param<int>("~refbox_recv_port", public_send_port_, 4445);
    ros::param::param<int>("~team_send_port", team_send_port_, 4451);
    ros::param::param<int>("~team_recv_port", team_recv_port_, 4450);

    ros::param::param<std::string>("~robot_name", robot_name_, "spqr");
    ros::param::param<std::string>("~team_name", team_name_, "SPQR");

    ROS_INFO("Hostname: %s", host_name_.c_str());
    ROS_INFO("Team Port: %i", team_port_);
    ROS_INFO("Public Port: %i", public_port_);
    ROS_INFO("Name: %s", robot_name_.c_str());
    ROS_INFO("Team Name: %s", team_name_.c_str());
}

void CFHRobot::createCFHRobot()
{
    //create public peer
    if (remote_refbox_) {
        //ref box is running on remote machine.
        peer_public_.reset(new ProtobufBroadcastPeer(host_name_, 
                                                    public_port_));
    } else {
        //ref box is running on same machine as client.
        peer_public_.reset(new ProtobufBroadcastPeer(host_name_, 
                                                    public_send_port_, 
                                                    public_recv_port_));    
    }
    

    //create internal message handler
    MessageRegister &message_register = peer_public_->message_register();
    //added messagetype to the handler
    message_register.add_message_type<AttentionMessage>();
    message_register.add_message_type<BeaconSignal>();
    message_register.add_message_type<BenchmarkState>();
    message_register.add_message_type<CompressedImage>();
    message_register.add_message_type<Inventory>();
    message_register.add_message_type<OrderInfo>();
    message_register.add_message_type<RobotInfo>();
    message_register.add_message_type<VersionInfo>();

    //create team peer and linked to internal message handler
    if (remote_refbox_) {
        //ref box is running on remote machine.
        peer_team_.reset(new ProtobufBroadcastPeer(host_name_,
                                                    team_port_, 
                                                    &message_register));
    } else {
        //ref box is running on same machine as client.
        peer_team_.reset(new ProtobufBroadcastPeer(host_name_,
                                                    team_send_port_, 
                                                    team_recv_port_, 
                                                    &message_register));
    }

    //bind the peers to the callback funktions
    peer_public_->signal_received().connect(boost::bind(&CFHRobot::handleMessage, this, _1, _2, _3, _4));
    peer_public_->signal_send_error().connect(boost::bind( &CFHRobot::handleSendError, this, _1));
    peer_public_->signal_recv_error().connect(boost::bind(&CFHRobot::handleReceiveError, this, _1, _2));

    peer_team_->signal_received().connect(boost::bind(&CFHRobot::handleMessage, this, _1, _2, _3, _4));
    peer_team_->signal_send_error().connect(boost::bind( &CFHRobot::handleSendError, this, _1));
    peer_team_->signal_recv_error().connect(boost::bind(&CFHRobot::handleReceiveError, this, _1, _2));
}

void CFHRobot::sendBeacon()
{
    //generate the timestamp
    timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    int32_t sec = start.tv_sec;
    int32_t nsec = start.tv_nsec;

    //create the message
    std::shared_ptr<BeaconSignal> signal(new BeaconSignal());

    //seperate the time segment of the message
    Time *time = signal->mutable_time(); 
    //write the timestamp into the message
    time->set_sec(sec);
    time->set_nsec(nsec);

    //write the name und co into the message
    signal->set_peer_name(robot_name_);
    signal->set_team_name(team_name_);
    //increase the sequence number
    signal->set_seq(++seq_);
    //send over team peer       
    peer_team_->send(signal);
}

void CFHRobot::handleSendError(std::string msg)
{
    ROS_WARN("Send error: %s\n", msg.c_str());
}

void CFHRobot::handleReceiveError(boost::asio::ip::udp::endpoint &endpoint,
                        std::string msg)
{
    ROS_WARN("Recv error: %s\n", msg.c_str());
}

void CFHRobot::handleMessage(boost::asio::ip::udp::endpoint &sender,
                    uint16_t component_id, uint16_t msg_type,
                    std::shared_ptr<google::protobuf::Message> msg)
{
    std::shared_ptr<AttentionMessage> am;
    // test it is the right message type              
    if ((am = std::dynamic_pointer_cast<AttentionMessage>(msg))) {

        cfh_robot::AttentionMessage attention_msg;
        attention_msg.message      = am->message();
        attention_msg.time_to_show = am->time_to_show();
        attention_msg.team         = am->team();
        attention_message_pub.publish(attention_msg);
    }
}

 
void CFHRobot::cameraControlCB(cfh_robot::CameraControl msg)
{
    //create a new message
    std::shared_ptr<CameraCommand> cam_control(new CameraCommand);
    //send the Message over team peer
    peer_team_->send(cam_control);
}