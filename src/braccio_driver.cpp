#include <braccio/braccio_driver.hpp>
#include <iterator>
#include <vector>

#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
//https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

Braccio::Braccio(): Node("Braccio")
{
    // Set srv_server
    srv_server_ = this->create_service<braccio::srv::Status>("Braccio/status", std::bind(&Braccio::process_service_request, this, _1,_2,_3));

    // Set action_server
    this->action_server_ = rclcpp_action::create_server<BraccioAction>(
      this,
      "braccio_action",
      std::bind(&Braccio::handle_goal, this, _1, _2),
      std::bind(&Braccio::handle_cancel, this, _1),
      std::bind(&Braccio::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "[Braccio] srv_server and action_server ready and operative.");

    //Arduino SerialPort
    serialPortFilename = "/dev/ttyACM0";
    bool sp = config_serial(serialPortFilename);
    if (!sp)
	{
		printf("[Braccio] ERROR opening serial port: %s \n",serialPortFilename.c_str());
		return;
	}else{
        RCLCPP_INFO(this->get_logger(), "[Braccio] Arduino SerialPort %s Open\n", serialPortFilename.c_str());
    }
}


bool Braccio::config_serial(std::string portName)
{
    // Open Serial Port
    serial_port = open(portName.c_str(), O_RDWR);

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return false;
    }

    tty.c_cflag &= ~PARENB;     // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;     // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;      // Clear all bits that set the data size
    tty.c_cflag |= CS8;         // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;    // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;       // Disable echo
    tty.c_lflag &= ~ECHOE;      // Disable erasure
    tty.c_lflag &= ~ECHONL;     // Disable new-line echo
    tty.c_lflag &= ~ISIG;       // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return false;
    }

    // flush before start
    sleep(1); //required to make flush work, for some reason
    tcflush(serial_port,TCIOFLUSH);
    //manual flush
    int num_bytes = 1;
    while (num_bytes>0){
        char read_buf[256];
        memset(&read_buf, '\0', sizeof(read_buf));
        num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
    }

    // Serial Port config Ok
    return true;
}


Braccio::~Braccio()
{
    // nodo cerrandose.
    printf("\nNode Braccio leaving gently \n");
    // close serial port with Arduino
    //fclose(serPortFile);
    close(serial_port);
}


void Braccio::process_service_request(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<braccio::srv::Status::Request> request,
        std::shared_ptr<braccio::srv::Status::Response> response)
{
    // Service has been called!
    RCLCPP_INFO(this->get_logger(), "[Braccio] Service has been called!");

    // Clean buffers:
    tcflush(serial_port,TCIOFLUSH);

    // write serialPort
    char writeBuffer[80];
    sprintf(writeBuffer,"STATUS");
    write(serial_port, writeBuffer, sizeof(writeBuffer));
    RCLCPP_INFO(this->get_logger(), "[Braccio] Sending request through SerialPort");
    sleep(3);   //Give at least 3 seconds

    // read Response "T1 T2 T3 T4 T5 T6"
    char read_buf[256];
    memset(&read_buf, '\0', sizeof(read_buf));
    bool read_done = false;
    int num_bytes;
    RCLCPP_INFO(this->get_logger(), "[Braccio] Reading response through SerialPort");
    while (!read_done){
        num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
        if (num_bytes > 0)
            read_done = true;
        else if (num_bytes < 0)
            break;
        else
            sleep(0.1);
    }

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        printf("[Braccio] Error reading from serial port: %s", strerror(errno));
    }else{
        RCLCPP_INFO(this->get_logger(), "[Braccio] Status response is %d bytes with content: %s", num_bytes,read_buf);
        char *strings[10];
        uint index = 0;
        const char *delimiter =" ";
        char* ptr = strtok(read_buf, delimiter);
        current_joints.clear();
        while (ptr != NULL)
        {
            strings[index] = ptr;
            current_joints.push_back(atoi(strings[index]));
            index++;
            ptr = strtok(NULL, delimiter);
        }

        if (current_joints.size() == 6){
            // end-efector position (MCD)
            current_position = braccio_mcd(current_joints);

            // Fill response
            response->current_position = current_position;
            response->current_joints = current_joints;
        }else{
            printf("[Braccio] Error reading Status. Not correct format: %s", read_buf);
        }
    }
}


rclcpp_action::GoalResponse Braccio::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const BraccioAction::Goal> goal)
{
    // Action has been called
    RCLCPP_INFO(this->get_logger(), "[Braccio] Action goal received");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse Braccio::handle_cancel(
        const std::shared_ptr<GoalHandleBraccio> goal_handle)
{
    // Request to cancell current action
    RCLCPP_INFO(this->get_logger(), "[Braccio] Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}


void Braccio::handle_accepted(const std::shared_ptr<GoalHandleBraccio> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Braccio::execute, this, _1), goal_handle}.detach();
}


void Braccio::execute(const std::shared_ptr<GoalHandleBraccio> goal_handle)
{
    // This will run on a new thread!
    RCLCPP_INFO(this->get_logger(), "[Braccio] Executing goal");

    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<BraccioAction::Feedback>();
    auto result = std::make_shared<BraccioAction::Result>();

    // Clean buffers:
    tcflush(serial_port,TCIOFLUSH);

    //1. check if joints are set. If so, ignore pose.
    if (goal->goal_joints.size() == 6)
    {
        // Setting joint angles
        //std::cout << "[Braccio] Goal is joint set = { ";
        //for (float n : goal->goal_joints)
        //    std::cout << n << ", ";
        //std::cout << "}; \n";

        char writeBuffer[80];
        sprintf(writeBuffer,"JOINTS %.2f %.2f %.2f %.2f %.2f %.2f", goal->goal_joints[0], goal->goal_joints[1], goal->goal_joints[2], goal->goal_joints[3], goal->goal_joints[4], goal->goal_joints[5]);
        //puts(writeBuffer);

        // write serialPort
        write(serial_port, writeBuffer, sizeof(writeBuffer));

    }else{
        // Calculate joints for position_goal (MCI)
        RCLCPP_INFO(this->get_logger(), "[Braccio] Goal is Position (%.2f, %.2f, %.2f)", goal->goal_position.x, goal->goal_position.y, goal->goal_position.z);
        //auto &angles = braccio_mci(goal->goal_position);
        // Setting joint angles
    }


    // Wait Arduino Confirmation (movement completed)
    RCLCPP_INFO(this->get_logger(), "[Braccio] Action CMD sent. Waiting Braccio confirmation");
    sleep(3);

    // read Response
    char read_buf[256];
    memset(&read_buf, '\0', sizeof(read_buf));
    bool read_done = false;
    int num_bytes;
    while (!read_done){
        num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
        if (num_bytes > 0)
            read_done = true;
        else if (num_bytes < 0)
            break;
        else
            sleep(1);
    }

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        printf("[Braccio] Error reading from serial port: %s \n", strerror(errno));
        result->goal_achieved = false;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Failed");
    }else{
        RCLCPP_INFO(this->get_logger(), "[Braccio] Arduino confirmed that movement is completed: %s", read_buf);
        result->goal_achieved = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

geometry_msgs::msg::Point Braccio::braccio_mcd(std::vector<float> joints)
{
    // Kinematic Direct Model of Braccio Arm

    // Arm Link lengths (m)
    float L1=0.076;
    float L2=0.124;
    float L3=0.124;
    float L4=0.060;
    float L5=0.130;

    const double pi = std::acos(-1);

    // joints (rad)
    float theta1 = joints[0]*pi/180;
    float theta2 = joints[1]*pi/180;
    float theta3 = joints[2]*pi/180;
    float theta4 = joints[3]*pi/180;
    float theta5 = joints[4]*pi/180;
    float theta6 = joints[5]*pi/180;

    // just X Y Z components! No orientation needed here!
    float x = L2*std::cos(theta1)*std::cos(theta2) - (std::cos(theta1 + theta2 + theta3 + theta4)/2 + std::cos(theta2 - theta1 + theta3 + theta4)/2)*(L4 + L5) + L3*std::cos(theta1)*std::cos(theta2)*std::sin(theta3) + L3*std::cos(theta1)*std::cos(theta3)*std::sin(theta2);

    float y = (std::sin(theta2 - theta1 + theta3 + theta4)/2 - std::sin(theta1 + theta2 + theta3 + theta4)/2)*(L4 + L5) + L2*std::cos(theta2)*std::sin(theta1) + L3*std::cos(theta2)*std::sin(theta1)*std::sin(theta3) + L3*std::cos(theta3)*std::sin(theta1)*std::sin(theta2);

    float z = L3*std::cos(theta2 + theta3) - L1 - L2*std::sin(theta2) + L4*std::sin(theta2 + theta3 + theta4) + L5*std::sin(theta2 + theta3 + theta4);

    // Set result
    geometry_msgs::msg::Point target;
    target.x = x;
    target.y = y;
    target.z = z;
    return target;
}

std::vector<float> Braccio::braccio_mci(geometry_msgs::msg::Point p)
{
}

int main (int argc, char* argv[]){
    //inicializamos ROS2
    rclcpp::init(argc,argv);
    auto p = std::make_shared<Braccio>();
    rclcpp::spin(p);

    //Al terminar cerramos todo
    rclcpp::shutdown();
    return 0;
}

