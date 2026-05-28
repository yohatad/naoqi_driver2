#include "audio_player.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>
#include <random>
#include <sstream>

#include <qi/buffer.hpp>
#include <qi/future.hpp>

#include <naoqi_bridge_msgs/srv/pause_audio.hpp>
#include <naoqi_bridge_msgs/srv/resume_audio.hpp>

using PlayAudio       = naoqi_bridge_msgs::action::PlayAudio;
using PlayGoalHandle  = rclcpp_action::ServerGoalHandle<PlayAudio>;
using LoadAudioFile   = naoqi_bridge_msgs::srv::LoadAudioFile;
using UnloadAudioFile = naoqi_bridge_msgs::srv::UnloadAudioFile;
using SetAudioVolume  = naoqi_bridge_msgs::srv::SetAudioVolume;
using StopAudio       = naoqi_bridge_msgs::srv::StopAudio;
using SendAudioBuffer = naoqi_bridge_msgs::srv::SendAudioBuffer;
using PauseAudio      = naoqi_bridge_msgs::srv::PauseAudio;
using ResumeAudio     = naoqi_bridge_msgs::srv::ResumeAudio;

// Default timeout for playback (60 seconds)
static constexpr float DEFAULT_PLAYBACK_TIMEOUT = 60.0f;

namespace naoqi
{
namespace action
{
namespace
{

struct AudioPlayerState
{
  AudioPlayerState(rclcpp::Node* node, qi::SessionPtr session)
  : node(node), session(std::move(session)), logger(node->get_logger())
  {
    robot_ip   = node->get_parameter("nao_ip").as_string();
    robot_user = node->get_parameter("user").as_string();
  }

  rclcpp::Node*  node;
  qi::SessionPtr session;
  rclcpp::Logger logger;

  std::string robot_ip;
  std::string robot_user;

  std::mutex                           mutex;
  std::shared_ptr<PlayGoalHandle>      current_goal;
  std::atomic<bool>                    cancel_requested{false};
  std::atomic<bool>                    pause_requested{false};
  std::atomic<bool>                    is_paused{false};
  qi::AnyObject                        audio_player;
  qi::AnyObject                        audio_device;

  // Tracks remote temp files that need deletion: file_id → remote_path
  std::map<int32_t, std::string>       remote_temp_files;

};

// ── helpers ───────────────────────────────────────────────────────────────────

// Thread-safe getter for ALAudioPlayer service
qi::AnyObject getAudioPlayer(std::shared_ptr<AudioPlayerState> state)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->audio_player) {
    state->audio_player = state->session->service("ALAudioPlayer").value();
  }
  return state->audio_player;
}

// Thread-safe getter for ALAudioDevice service
qi::AnyObject getAudioDevice(std::shared_ptr<AudioPlayerState> state)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->audio_device) {
    state->audio_device = state->session->service("ALAudioDevice").value();
  }
  return state->audio_device;
}

// Generate unique temp file path
std::string generateUniqueTempPath()
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<> dis(0, 999999);
  
  std::ostringstream oss;
  oss << "/tmp/ros2_audio_" << dis(gen) << "_" << dis(gen) << ".wav";
  return oss.str();
}

// Clamp volume to [0..1] as expected by ALAudioPlayer.setVolume / setMasterVolume
float clampVolume(float v)
{
  return std::clamp(v, 0.0f, 1.0f);
}

// ── action: handle_goal ───────────────────────────────────────────────────────

rclcpp_action::GoalResponse handle_goal(
  std::shared_ptr<AudioPlayerState> state,
  const rclcpp_action::GoalUUID& uuid,
  const std::shared_ptr<const PlayAudio::Goal> goal)
{
  const std::string goal_id = rclcpp_action::to_string(uuid);
  RCLCPP_INFO(state->logger, "Received play_audio goal %s (file_id=%d loop=%s)",
              goal_id.c_str(), goal->file_id, goal->loop ? "true" : "false");

  if (goal->file_id < 0) {
    RCLCPP_WARN(state->logger, "Rejecting %s: invalid file_id %d", goal_id.c_str(), goal->file_id);
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::lock_guard<std::mutex> lock(state->mutex);
  if (state->current_goal) {
    RCLCPP_INFO(state->logger, "Rejecting %s: already playing for %s",
                goal_id.c_str(),
                rclcpp_action::to_string(state->current_goal->get_goal_id()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ── action: handle_cancel ─────────────────────────────────────────────────────

rclcpp_action::CancelResponse handle_cancel(
  std::shared_ptr<AudioPlayerState> state,
  const std::shared_ptr<PlayGoalHandle> goal_handle)
{
  RCLCPP_INFO(state->logger, "Cancel requested for play_audio %s",
              rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());

  state->cancel_requested.store(true);

  // Stop playback immediately so the execute thread unblocks
  try {
    getAudioPlayer(state).call<void>("stopAll");
  } catch (const std::exception& e) {
    RCLCPP_WARN(state->logger, "stopAll during cancel failed: %s", e.what());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

// ── action: handle_accepted / execute ────────────────────────────────────────

void handle_accepted(
  std::shared_ptr<AudioPlayerState> state,
  const std::shared_ptr<PlayGoalHandle> goal_handle)
{
  std::thread([state, goal_handle]()
  {
    const auto goal     = goal_handle->get_goal();
    auto       feedback = std::make_shared<PlayAudio::Feedback>();
    auto       result   = std::make_shared<PlayAudio::Result>();

    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->current_goal = goal_handle;
      state->cancel_requested.store(false);
      state->pause_requested.store(false);
      state->is_paused.store(false);
    }

    const auto start = std::chrono::steady_clock::now();
    auto elapsed_secs = [&]() -> float {
      return std::chrono::duration<float>(
        std::chrono::steady_clock::now() - start).count();
    };

    try {
      auto player = getAudioPlayer(state);

      // Apply volume and pan before starting
      player.call<void>("setVolume",   goal->file_id, clampVolume(goal->volume));
      player.call<void>("setPanorama", goal->pan);

      // Start playback — async so we can poll for cancel without blocking
      qi::Future<void> play_future;
      if (goal->loop) {
        play_future = player.async<void>("playInLoop", goal->file_id);
      } else {
        play_future = player.async<void>("play", goal->file_id);
      }

      RCLCPP_INFO(state->logger, "Playback started: file_id=%d", goal->file_id);

      // Poll at ~10 Hz: check cancel, wait 100 ms for future to finish
      // Also check for timeout to prevent indefinite blocking
      while (rclcpp::ok()) {
        // Check timeout
        if (elapsed_secs() > DEFAULT_PLAYBACK_TIMEOUT) {
          player.call<void>("stopAll");
          result->success     = false;
          result->status      = "timeout: playback exceeded maximum duration";
          result->played_secs = elapsed_secs();

          {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->current_goal.reset();
            state->cancel_requested.store(false);
          }

          goal_handle->abort(result);
          RCLCPP_WARN(state->logger, "Playback timed out after %.1fs", result->played_secs);
          return;
        }

        if (state->cancel_requested.load() || goal_handle->is_canceling()) {
          // stopAll was already called in handle_cancel; wait for future to settle
          play_future.wait(500 /*ms*/);

          result->success     = true;
          result->status      = "cancelled";
          result->played_secs = elapsed_secs();

          {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->current_goal.reset();
            state->cancel_requested.store(false);
          }

          goal_handle->canceled(result);
          RCLCPP_INFO(state->logger, "Playback cancelled at %.1fs", result->played_secs);
          return;
        }

        const auto fstate = play_future.wait(100 /*ms*/);

        if (fstate == qi::FutureState_FinishedWithValue) {
          break;
        }
        if (fstate == qi::FutureState_FinishedWithError) {
          throw std::runtime_error(play_future.error());
        }

        // Publish elapsed time as feedback
        feedback->position_secs = elapsed_secs();
        goal_handle->publish_feedback(feedback);
      }

      result->success     = true;
      result->status      = "completed";
      result->played_secs = elapsed_secs();

      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->current_goal.reset();
        state->cancel_requested.store(false);
      }

      goal_handle->succeed(result);
      RCLCPP_INFO(state->logger, "Playback completed: file_id=%d (%.1fs)",
                  goal->file_id, result->played_secs);

    } catch (const std::exception& e) {
      result->success = false;
      result->status  = std::string("failed: ") + e.what();

      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->current_goal.reset();
        state->cancel_requested.store(false);
      }

      goal_handle->abort(result);
      RCLCPP_ERROR(state->logger, "Playback failed: %s", e.what());
    }
  }).detach();
}

// ── service handlers ──────────────────────────────────────────────────────────

void handleLoadFile(
  std::shared_ptr<AudioPlayerState> state,
  std::shared_ptr<LoadAudioFile::Request>  req,
  std::shared_ptr<LoadAudioFile::Response> res)
{
  // Determine the path to pass to ALAudioPlayer.loadFile().
  // If audio_data bytes were provided, write them to a temp file on the robot
  // via ALFileManager so the caller never needs SCP/SSH access.
  std::string load_path = req->remote_path;

  try {
    if (!req->audio_data.empty()) {
      if (state->robot_ip.empty()) {
        throw std::runtime_error(
          "nao_ip parameter is not set — cannot SCP audio to robot");
      }

      // Write bytes to a local temp file
      const std::string local_path = generateUniqueTempPath();
      {
        std::ofstream out(local_path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(req->audio_data.data()),
                  req->audio_data.size());
        if (!out) {
          throw std::runtime_error("Failed to write local temp file: " + local_path);
        }
      }

      // Derive the remote path (same filename, /tmp on robot)
      const std::string filename    = local_path.substr(local_path.rfind('/') + 1);
      const std::string remote_path = "/tmp/" + filename;
      const std::string scp_cmd =
        "scp " + local_path + " " +
        state->robot_user + "@" + state->robot_ip + ":" + remote_path;

      RCLCPP_INFO(state->logger, "SCPing %zu bytes → %s@%s:%s",
                  req->audio_data.size(),
                  state->robot_user.c_str(), state->robot_ip.c_str(),
                  remote_path.c_str());

      int ret = std::system(scp_cmd.c_str());
      std::remove(local_path.c_str());  // always clean up local file

      if (ret != 0) {
        throw std::runtime_error(
          "SCP failed (exit=" + std::to_string(ret) + "). "
          "Ensure passwordless SSH is set up: ssh-copy-id " +
          state->robot_user + "@" + state->robot_ip);
      }

      load_path = remote_path;
      RCLCPP_INFO(state->logger, "Uploaded to robot:%s", remote_path.c_str());
    }

    const int32_t id = getAudioPlayer(state).call<int>("loadFile", load_path);
    res->success = true;
    res->file_id = id;
    RCLCPP_INFO(state->logger, "Loaded '%s' → file_id=%d", load_path.c_str(), id);

    // Track remote temp files so they can be deleted on unload
    if (!req->audio_data.empty()) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->remote_temp_files[id] = load_path;
    }
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(state->logger, "loadFile failed: %s", e.what());
  }
}

void handleUnloadFile(
  std::shared_ptr<AudioPlayerState> state,
  std::shared_ptr<UnloadAudioFile::Request>  req,
  std::shared_ptr<UnloadAudioFile::Response> res)
{
  try {
    getAudioPlayer(state).call<void>("unloadFile", req->file_id);
    res->success = true;
    RCLCPP_INFO(state->logger, "Unloaded file_id=%d", req->file_id);
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(state->logger, "unloadFile failed: %s", e.what());
  }

  // Delete the remote temp file on the robot (if it was uploaded via SCP)
  std::string remote_path;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    auto it = state->remote_temp_files.find(req->file_id);
    if (it != state->remote_temp_files.end()) {
      remote_path = it->second;
      state->remote_temp_files.erase(it);
    }
  }

  if (!remote_path.empty()) {
    const std::string rm_cmd =
      "ssh " + state->robot_user + "@" + state->robot_ip +
      " rm -f " + remote_path;
    if (std::system(rm_cmd.c_str()) == 0) {
      RCLCPP_INFO(state->logger, "Deleted robot:%s", remote_path.c_str());
    } else {
      RCLCPP_WARN(state->logger, "Failed to delete robot:%s", remote_path.c_str());
    }
  }
}

void handleSetVolume(
  std::shared_ptr<AudioPlayerState> state,
  std::shared_ptr<SetAudioVolume::Request>  req,
  std::shared_ptr<SetAudioVolume::Response> res)
{
  try {
    const float vol = clampVolume(req->volume);
    if (req->file_id < 0) {
      getAudioPlayer(state).call<void>("setMasterVolume", vol);
      RCLCPP_INFO(state->logger, "Master volume → %.2f", vol);
    } else {
      getAudioPlayer(state).call<void>("setVolume", req->file_id, vol);
      RCLCPP_INFO(state->logger, "Volume file_id=%d → %.2f", req->file_id, vol);
    }
    res->success = true;
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(state->logger, "setVolume failed: %s", e.what());
  }
}

void handleStopAudio(
  std::shared_ptr<AudioPlayerState> state,
  std::shared_ptr<StopAudio::Request>,
  std::shared_ptr<StopAudio::Response> res)
{
  try {
    getAudioPlayer(state).call<void>("stopAll");
    res->success = true;
    RCLCPP_INFO(state->logger, "stopAll called");
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(state->logger, "stopAll failed: %s", e.what());
  }
}

// ── new service handlers ─────────────────────────────────────────────────────

void handlePauseAudio(
  std::shared_ptr<AudioPlayerState> state,
  std::shared_ptr<PauseAudio::Request>,
  std::shared_ptr<PauseAudio::Response> res)
{
  try {
    // ALAudioPlayer has no native pause — stop playback and record paused state.
    getAudioPlayer(state).call<void>("stopAll");
    state->pause_requested.store(true);
    state->is_paused.store(true);
    res->success = true;
    RCLCPP_INFO(state->logger, "Audio paused");
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(state->logger, "pause failed: %s", e.what());
  }
}

void handleResumeAudio(
  std::shared_ptr<AudioPlayerState> state,
  std::shared_ptr<ResumeAudio::Request>,
  std::shared_ptr<ResumeAudio::Response> res)
{
  try {
    // Clear paused state; caller must re-send a play_audio goal to restart.
    state->is_paused.store(false);
    state->pause_requested.store(false);
    res->success = true;
    RCLCPP_INFO(state->logger, "Audio resume requested");
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(state->logger, "resume failed: %s", e.what());
  }
}

void handleSendBuffer(
  std::shared_ptr<AudioPlayerState> state,
  std::shared_ptr<SendAudioBuffer::Request>  req,
  std::shared_ptr<SendAudioBuffer::Response> res)
{
  try {
    auto audio_device = getAudioDevice(state);

    // Audio data is 16-bit stereo interleaved: frames = bytes / (2ch × 2B)
    const int nb_of_frames = static_cast<int>(req->audio_data.size() / 4);
    if (nb_of_frames == 0 || nb_of_frames > 16384) {
      throw std::runtime_error(
        "Invalid buffer size: got " + std::to_string(nb_of_frames) +
        " frames, must be in (0, 16384]");
    }

    // ALAudioDevice.sendRemoteBufferToOutput expects the buffer as an ALValue
    // converted to binary (raw bytes). Wrap the int16 PCM data in a qi::Buffer.
    qi::Buffer buf;
    void* raw = buf.reserve(req->audio_data.size());
    std::memcpy(raw, req->audio_data.data(), req->audio_data.size());

    bool success = audio_device.call<bool>(
      "sendRemoteBufferToOutput", nb_of_frames, buf);

    res->success = success;
    if (success) {
      RCLCPP_DEBUG(state->logger, "Sent %d frames to audio output", nb_of_frames);
    } else {
      RCLCPP_WARN(state->logger, "sendRemoteBufferToOutput returned false");
    }
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(state->logger, "sendBuffer failed: %s", e.what());
  }
}

}  // namespace

// ── public factory ────────────────────────────────────────────────────────────

AudioPlayerServers createAudioPlayerServers(rclcpp::Node* node, qi::SessionPtr session)
{
  namespace ph = std::placeholders;
  auto state = std::make_shared<AudioPlayerState>(node, std::move(session));

  AudioPlayerServers servers;

  servers.play_audio = rclcpp_action::create_server<PlayAudio>(
    node, "/naoqi_driver/play_audio",
    std::bind(handle_goal,     state, ph::_1, ph::_2),
    std::bind(handle_cancel,   state, ph::_1),
    std::bind(handle_accepted, state, ph::_1)
  );

  servers.load_file = node->create_service<LoadAudioFile>(
    "/naoqi_driver/load_audio_file",
    std::bind(handleLoadFile, state, ph::_1, ph::_2));

  servers.unload_file = node->create_service<UnloadAudioFile>(
    "/naoqi_driver/unload_audio_file",
    std::bind(handleUnloadFile, state, ph::_1, ph::_2));

  servers.set_volume = node->create_service<SetAudioVolume>(
    "/naoqi_driver/set_audio_volume",
    std::bind(handleSetVolume, state, ph::_1, ph::_2));

  servers.stop_audio = node->create_service<StopAudio>(
    "/naoqi_driver/stop_audio",
    std::bind(handleStopAudio, state, ph::_1, ph::_2));

  servers.send_buffer = node->create_service<SendAudioBuffer>(
    "/naoqi_driver/send_audio_buffer",
    std::bind(handleSendBuffer, state, ph::_1, ph::_2));

  servers.pause_audio = node->create_service<PauseAudio>(
    "/naoqi_driver/pause_audio",
    std::bind(handlePauseAudio, state, ph::_1, ph::_2));

  servers.resume_audio = node->create_service<ResumeAudio>(
    "/naoqi_driver/resume_audio",
    std::bind(handleResumeAudio, state, ph::_1, ph::_2));

  return servers;
}

}  // namespace action
}  // namespace naoqi
