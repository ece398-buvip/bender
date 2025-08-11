#include <memory.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

static double clampDouble(double val, double min, double max)
{
    if (val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    return val;
}

class Drivetrain
{
private:
    bool setupSerialPort();
    bool writeToPort(const std::string &val);

    // TODO: Allow for this to be configured
    std::string _filePath = "/dev/ttyS0";
    int _fd = -1;
    bool _serialUp = false;
public:
    Drivetrain();
    bool WriteDutyToPort();
    bool SerialLinkOk();
    bool SendHeartbeat();

    int32_t m_leftDuty;
    int32_t m_rightDuty;
};

Drivetrain::Drivetrain() : m_leftDuty{0}, m_rightDuty{0}
{
    // Open serial port file
    setupSerialPort();
}

bool Drivetrain::SerialLinkOk()
{
    return _serialUp;
}

bool
Drivetrain::setupSerialPort()
{
   int file, rc;
   struct termios options;

   file = open(_filePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
   if (file < 0)
   {
      return false;
   }
   _fd = file;

   tcgetattr(file, &options);

   // Set baud rate to 115200
   cfsetispeed(&options, B115200);
   cfsetospeed(&options, B115200);

   // Input settings
   options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IXON | IXOFF | IXANY);
   options.c_oflag &= ~OPOST;
   options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
   options.c_cflag &= ~(CSIZE | PARENB);
   options.c_cflag |= (CS8 | CLOCAL | CREAD);
   // Disable RTS/CTS
   options.c_cflag &= ~CRTSCTS;

   options.c_cc[VMIN] = 0;
   options.c_cc[VTIME] = 0;

   rc = tcsetattr(file, TCSANOW, &options);

   if (rc == 0)
   {
      // Flush the current buffer of data
      // See: https://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
      sleep(2);
      tcflush(_fd, TCIOFLUSH);
      _serialUp = true;
      return true;
   }
   else
   {
      return false;
   }
}

bool
Drivetrain::writeToPort(const std::string &val)
{
   if (!_serialUp)
   {
      if (!setupSerialPort())
      {
         return false;
      }
   }

   if (val.size() <= 0)
   {
      return false;
   }

   size_t txNum = write(_fd, val.c_str(), val.size());

   return txNum == val.size();
}


bool Drivetrain::WriteDutyToPort() {
    char buf[64] = {0};
    // Write duty to serial port file
    if (!_serialUp)
    {
        return false;
    }

    // snprintf(buf, sizeof(buf), "%03d,%03d\n", m_leftDuty, m_rightDuty);
    snprintf(buf, sizeof(buf), "%03d,%03d\n", m_leftDuty, m_rightDuty);

    writeToPort(buf);

    return true;
}

bool Drivetrain::SendHeartbeat()
{
    if (!_serialUp)
    {
        return false;
    }

    writeToPort("H\n");

    return true;
}

class DrivetrainSub : public rclcpp::Node{
public:
    DrivetrainSub();
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist;
    Drivetrain _driveTrain;
    rclcpp::TimerBase::SharedPtr _heartbeatTimer;
    void heartbeat_callback();
};


DrivetrainSub::DrivetrainSub() : Node("drivetrain_sub"), _driveTrain{}
{
    // Check if drivetrain constructed OK
    if (!_driveTrain.SerialLinkOk())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize drivetrain serial link");
    }

    // Create the callback that happens whenever a message is received on
    //   left/right publisher interfaces
    auto twist_callback = [this](geometry_msgs::msg::Twist::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "RECEIVED A TWIST MESSAGE: [linear.x: %0.2f, angular.z: %0.2f]", msg->linear.x, msg->angular.z);

        double vX = clampDouble(msg->linear.x, -1.0, 1.0);
        double wZ = clampDouble(msg->angular.z, -1.0, 1.0);

        // Calculate left and right duty cycles from the twist velocities.
        double left = clampDouble(vX - wZ, -1.0, 1.0);
        double right = clampDouble(vX + wZ, -1.0, 1.0);

        _driveTrain.m_leftDuty = left * 55;
        _driveTrain.m_rightDuty = right * 55;

        if (!_driveTrain.WriteDutyToPort())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write duty cycle to port!");
        }

        RCLCPP_INFO(this->get_logger(), "Left Duty: %d, Right Duty: %d", _driveTrain.m_leftDuty, _driveTrain.m_rightDuty);
    };

    _twist = this->create_subscription<geometry_msgs::msg::Twist>("bender/twist", 1, twist_callback);

    // Create a timer that runs every 500ms
    _heartbeatTimer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&DrivetrainSub::heartbeat_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Starting Drivetrain Listener");
}

void DrivetrainSub::heartbeat_callback()
{
    if (_driveTrain.SendHeartbeat())
    {
        RCLCPP_INFO(this->get_logger(), "Sent Heartbeat");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send heartbeat");
    }
}

// Main program for package

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivetrainSub>());
    rclcpp::shutdown();
}
