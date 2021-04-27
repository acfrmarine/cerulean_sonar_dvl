#include <string_view>
#include <string>

#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <cerulean_sonar_dvl_msgs/DVLExtendedData.h>

namespace
{
std::pair<std::string_view, int> getSentenceAndChecksum(const std::string_view& sentence_full)
{
  size_t checksum_delimiter_pos = sentence_full.rfind('*');

  if (checksum_delimiter_pos == std::string_view::npos)
  {
    return { {}, -1 };
  }

  // Strip first character and checksum
  std::string_view sentence = sentence_full.substr(1u, checksum_delimiter_pos - 1u);

  const std::string_view checksum_string = sentence_full.substr(checksum_delimiter_pos + 1u);
  int checksum = static_cast<int>(std::stol(std::string(checksum_string), nullptr, 16));

  return { sentence, checksum };
}

int calcChecksum(const std::string_view& sentence)
{
  int checksum = 0;

  for (const char c : sentence)
  {
    checksum ^= c;
  }

  return checksum;
}

}  // namespace

namespace cerulean_sonar_dvl_driver
{
class DvlNmeaSentenceParser
{
public:
  DvlNmeaSentenceParser(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  diagnostic_msgs::DiagnosticStatus imu_status_msg_{};
  cerulean_sonar_dvl_msgs::DVLExtendedData dvl_extended_data_msg_{};

  ros::Time last_imu_status_publish_time_{};

  ros::Publisher imu_status_pub_{};
  ros::Publisher dvl_extended_data_pub_{};

  ros::Subscriber nmea_sentence_sub_{};

  void handleImuCalibrationField(const std::string_view& field);
  void handleDvlExtendedDataMessage(const std::string_view& message);
  void nmeaSentenceCallback(const nmea_msgs::SentenceConstPtr& msg);
};

DvlNmeaSentenceParser::DvlNmeaSentenceParser(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  imu_status_msg_.level = diagnostic_msgs::DiagnosticStatus::STALE;
  imu_status_msg_.name = "DVL";

  imu_status_msg_.values.resize(4u);
  imu_status_msg_.values[0].key = "system_calibration_level";
  imu_status_msg_.values[0].value = "unknown";
  imu_status_msg_.values[1].key = "gyroscope_calibration_level";
  imu_status_msg_.values[1].value = "unknown";
  imu_status_msg_.values[2].key = "accelerometer_calibration_level";
  imu_status_msg_.values[2].value = "unknown";
  imu_status_msg_.values[3].key = "magnetometer_calibration_level";
  imu_status_msg_.values[3].value = "unknown";

  imu_status_pub_ = nh.advertise<diagnostic_msgs::DiagnosticStatus>("imu_status", 10);
  dvl_extended_data_pub_ = nh.advertise<cerulean_sonar_dvl_msgs::DVLExtendedData>("dvl_extended_data", 10);

  nmea_sentence_sub_ = nh.subscribe("nmea_sentence", 10, &DvlNmeaSentenceParser::nmeaSentenceCallback, this);
}

void DvlNmeaSentenceParser::handleImuCalibrationField(const std::string_view& field)
{
  if (field.length() != 4)
  {
    ROS_WARN("Incorrect number of character in IMU field");
    imu_status_msg_.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }
  else
  {
    imu_status_msg_.level = diagnostic_msgs::DiagnosticStatus::OK;

    for (size_t idx = 0u; idx < 4u; ++idx)
    {
      int value = static_cast<int>(field.at(idx) - 48u);

      switch (value)
      {
        case 0:
          imu_status_msg_.values[idx].value = "uncalibrated";
          imu_status_msg_.level = diagnostic_msgs::DiagnosticStatus::WARN;
          break;
        case 1:
          imu_status_msg_.values[idx].value = "poorly calibrated";
          imu_status_msg_.level = diagnostic_msgs::DiagnosticStatus::WARN;
          break;
        case 2:
          imu_status_msg_.values[idx].value = "well calibrated";
          break;
        case 3:
          imu_status_msg_.values[idx].value = "fully calibrated";
          break;
        default:
          imu_status_msg_.values[idx].value = "unknown";
      }
    }
  }

  // Publish IMU status if it hasn't been published for a second
  if (ros::Time::now() - last_imu_status_publish_time_ > ros::Duration{ 1.0 })
  {
    imu_status_pub_.publish(imu_status_msg_);
    last_imu_status_publish_time_ = ros::Time::now();
  }
}

void DvlNmeaSentenceParser::handleDvlExtendedDataMessage(const std::string_view& message)
{
  size_t prev_comma_pos = 0u;

  for (size_t f = 0u; f < 34u; ++f)
  {
    size_t next_comma_pos = message.find(',', prev_comma_pos + 1u);

    if (next_comma_pos == std::string_view::npos)
    {
      ROS_WARN("Incorrect number of fields in DVL extended data message sentence");
      return;
    }

    if (next_comma_pos != prev_comma_pos + 1u)
    {
      const std::string_view field = message.substr(prev_comma_pos + 1u, next_comma_pos - prev_comma_pos - 1);

      switch (f)
      {
        case 0u:  // DVL Lock
          dvl_extended_data_msg_.lock = field.at(0) == 'T';
          break;
        case 2u:  // IMU Calibration
          handleImuCalibrationField(field);
          break;
        case 3u:  // Roll
          dvl_extended_data_msg_.roll = std::stof(std::string(field));
          break;
        case 4u:  // Pitch
          dvl_extended_data_msg_.pitch = std::stof(std::string(field));
          break;
        case 5u:  // Heading
          dvl_extended_data_msg_.yaw = std::stof(std::string(field));
          break;
        case 6u:  // Data skips
          dvl_extended_data_msg_.data_skips = std::stoi(std::string(field));
          break;
        case 8u:  // Altitude
          dvl_extended_data_msg_.altitude = std::stof(std::string(field));
          break;
        case 9u:  // Velocity north (y)
          dvl_extended_data_msg_.velocity_y = std::stof(std::string(field));
          break;
        case 10u:  // Velocity east (x)
          dvl_extended_data_msg_.velocity_x = std::stof(std::string(field));
          break;
        case 18u:  // Channel A gain (port)
          dvl_extended_data_msg_.gain_a = static_cast<int8_t>(std::stoi(std::string(field)));
          break;
        case 19u:  // Channel B gain (stern)
          dvl_extended_data_msg_.gain_b = static_cast<int8_t>(std::stoi(std::string(field)));
          break;
        case 20u:  // Channel C gain (starboard)
          dvl_extended_data_msg_.gain_c = static_cast<int8_t>(std::stoi(std::string(field)));
          break;
        case 21u:  // Channel D gain (bow)
          dvl_extended_data_msg_.gain_d = static_cast<int8_t>(std::stoi(std::string(field)));
          break;
        case 22u:  // Channel A lock
          dvl_extended_data_msg_.lock_a = field.at(0) == 'T';
          break;
        case 23u:  // Channel B lock
          dvl_extended_data_msg_.lock_b = field.at(0) == 'T';
          break;
        case 24u:  // Channel C lock
          dvl_extended_data_msg_.lock_c = field.at(0) == 'T';
          break;
        case 25u:  // Channel D lock
          dvl_extended_data_msg_.lock_d = field.at(0) == 'T';
          break;
        case 26u:  // Channel A velocity
          dvl_extended_data_msg_.velocity_a = std::stof(std::string(field));
          break;
        case 27u:  // Channel B velocity
          dvl_extended_data_msg_.velocity_b = std::stof(std::string(field));
          break;
        case 28u:  // Channel C velocity
          dvl_extended_data_msg_.velocity_c = std::stof(std::string(field));
          break;
        case 29u:  // Channel D velocity
          dvl_extended_data_msg_.velocity_d = std::stof(std::string(field));
          break;
        case 30u:  // Channel A range
          dvl_extended_data_msg_.range_a = std::stof(std::string(field));
          break;
        case 31u:  // Channel B range
          dvl_extended_data_msg_.range_b = std::stof(std::string(field));
          break;
        case 32u:  // Channel C range
          dvl_extended_data_msg_.range_c = std::stof(std::string(field));
          break;
        case 33u:  // Channel D range
          dvl_extended_data_msg_.range_d = std::stof(std::string(field));
          break;
        default:
          // Ignore other fields
          break;
      }
    }

    prev_comma_pos = next_comma_pos;
  }

  // Publish message
  dvl_extended_data_pub_.publish(dvl_extended_data_msg_);
}

void DvlNmeaSentenceParser::nmeaSentenceCallback(const nmea_msgs::SentenceConstPtr& msg)
{
  // Ignore messages that don't start with '$'
  if (msg->sentence[0] != '$')
  {
    return;
  }

  std::string_view sentence_full{ msg->sentence };

  auto [sentence, checksum] = getSentenceAndChecksum(sentence_full);

  if (checksum < 0 || checksum != calcChecksum(sentence))
  {
    return;
  }

  const std::string_view message_type = sentence.substr(0, 5);
  const std::string_view message = sentence.substr(5);

  if (message_type.compare("DVEXT") == 0)
  {
    dvl_extended_data_msg_.header.stamp = msg->header.stamp;
    dvl_extended_data_msg_.header.seq = msg->header.seq;
    dvl_extended_data_msg_.header.frame_id = msg->header.frame_id;

    handleDvlExtendedDataMessage(message);
    return;
  }
}

}  // namespace cerulean_sonar_dvl_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dvl_nmea_sentence_parser");

  ros::NodeHandle nh{};
  ros::NodeHandle private_nh{ "~" };

  cerulean_sonar_dvl_driver::DvlNmeaSentenceParser receiver_nmea_sentence_parser{ nh, private_nh };

  ros::spin();

  return 0;
}
