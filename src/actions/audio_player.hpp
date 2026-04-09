#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <qi/session.hpp>
#include <naoqi_bridge_msgs/action/play_audio.hpp>
#include <naoqi_bridge_msgs/srv/load_audio_file.hpp>
#include <naoqi_bridge_msgs/srv/unload_audio_file.hpp>
#include <naoqi_bridge_msgs/srv/set_audio_volume.hpp>
#include <naoqi_bridge_msgs/srv/stop_audio.hpp>
#include <naoqi_bridge_msgs/srv/send_audio_buffer.hpp>
#include <naoqi_bridge_msgs/srv/pause_audio.hpp>
#include <naoqi_bridge_msgs/srv/resume_audio.hpp>

namespace naoqi
{
namespace action
{

struct AudioPlayerServers
{
  rclcpp_action::Server<naoqi_bridge_msgs::action::PlayAudio>::SharedPtr  play_audio;
  rclcpp::Service<naoqi_bridge_msgs::srv::LoadAudioFile>::SharedPtr       load_file;
  rclcpp::Service<naoqi_bridge_msgs::srv::UnloadAudioFile>::SharedPtr     unload_file;
  rclcpp::Service<naoqi_bridge_msgs::srv::SetAudioVolume>::SharedPtr      set_volume;
  rclcpp::Service<naoqi_bridge_msgs::srv::StopAudio>::SharedPtr           stop_audio;
  rclcpp::Service<naoqi_bridge_msgs::srv::SendAudioBuffer>::SharedPtr     send_buffer;
  rclcpp::Service<naoqi_bridge_msgs::srv::PauseAudio>::SharedPtr          pause_audio;
  rclcpp::Service<naoqi_bridge_msgs::srv::ResumeAudio>::SharedPtr         resume_audio;
};

AudioPlayerServers createAudioPlayerServers(rclcpp::Node* node, qi::SessionPtr session);

} // namespace action
} // namespace naoqi
