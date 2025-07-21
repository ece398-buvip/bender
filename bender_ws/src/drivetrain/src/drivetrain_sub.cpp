#include <memory.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

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
    bool WriteDutyToPort(); bool SerialLinkOk();

    uint32_t m_leftDuty;
    uint32_t m_rightDuty;
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

   // Set baud rate to 115200
   cfsetispeed(&options, B115200);
   cfsetospeed(&options, B115200);

   // Input settings
   options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IXON | IXOFF | IXANY);
   options.c_oflag &= ~OPOST;
   options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
   options.c_cflag &= ~(CSIZE | PARENB);
   options.c_cflag |= (CS8 | CLOCAL | CREAD);

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

    snprintf(buf, sizeof(buf), "%d,%d\n", m_leftDuty, m_rightDuty);

    writeToPort(buf);

    return true;
}

class DrivetrainSub : public rclcpp::Node{
public:
    DrivetrainSub();
private:
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr _leftDuty;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr _rightDuty;
    Drivetrain _driveTrain;
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
    auto left_callback = [this](std_msgs::msg::UInt32::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "RECEIVED A LEFT MESSAGE: [%d]", msg->data);

        if (msg->data <= 255)
        {
            _driveTrain.m_leftDuty = msg->data;
            if (!_driveTrain.WriteDutyToPort())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write duty cycle to port!");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid duty cycle");
        }
    };

    auto right_callback = [this](std_msgs::msg::UInt32::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "RECEIVED A RIGHT MESSAGE: [%d]", msg->data);

        if (msg->data <= 255)
        {
            _driveTrain.m_rightDuty = msg->data;
            if (!_driveTrain.WriteDutyToPort())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write duty cycle to port!");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid duty cycle");
        }

    };

    _leftDuty = this->create_subscription<std_msgs::msg::UInt32>("bender/left_duty", 1, left_callback);
    _rightDuty = this->create_subscription<std_msgs::msg::UInt32>("bender/right_duty", 1, right_callback);

    RCLCPP_INFO(this->get_logger(), "Starting Drivetrain Listener");
}

// Main program for package

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrivetrainSub>());
    rclcpp::shutdown();
}
