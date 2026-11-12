#include "hik_camera/hik_camera_node.hpp"

#include <MvCameraControl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace hik_camera
{
namespace
{

std::string errorCode(const int code)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << static_cast<unsigned int>(code);
  return stream.str();
}

template<typename T, std::size_t N>
std::string fixedString(const T (& value)[N])
{
  const auto * text = reinterpret_cast<const char *>(value);
  return std::string(text, strnlen(text, N));
}

std::string serialNumber(const MV_CC_DEVICE_INFO & info)
{
  if (info.nTLayerType == MV_USB_DEVICE) {
    return fixedString(info.SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  if (info.nTLayerType == MV_GIGE_DEVICE) {
    return fixedString(info.SpecialInfo.stGigEInfo.chSerialNumber);
  }
  return {};
}

std::string modelName(const MV_CC_DEVICE_INFO & info)
{
  if (info.nTLayerType == MV_USB_DEVICE) {
    return fixedString(info.SpecialInfo.stUsb3VInfo.chModelName);
  }
  if (info.nTLayerType == MV_GIGE_DEVICE) {
    return fixedString(info.SpecialInfo.stGigEInfo.chModelName);
  }
  return "unknown";
}

}  // namespace

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("hik_camera", options)
{
  declareParameters();
  initializeCameraInfo();

  const int initialize_result = MV_CC_Initialize();
  if (initialize_result != MV_OK) {
    throw std::runtime_error("MV_CC_Initialize failed: " + errorCode(initialize_result));
  }
  sdk_initialized_ = true;

  try {
    openCamera();
    configureCamera();

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);

    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&HikCameraNode::onParameters, this, std::placeholders::_1));
    startCapture();
  } catch (...) {
    stopAndCloseCamera();
    MV_CC_Finalize();
    sdk_initialized_ = false;
    throw;
  }
}

HikCameraNode::~HikCameraNode()
{
  running_.store(false);
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_handle_ != nullptr && grabbing_) {
      MV_CC_StopGrabbing(camera_handle_);
      grabbing_ = false;
    }
  }
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  stopAndCloseCamera();
  if (sdk_initialized_) {
    MV_CC_Finalize();
  }
}

void HikCameraNode::declareParameters()
{
  declare_parameter<std::string>("serial_number", "");
  declare_parameter<int>("device_index", 0);
  declare_parameter<std::string>("frame_id", "camera_optical_frame");
  declare_parameter<std::string>("image_topic", "/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/camera_info");
  declare_parameter<std::string>("pixel_format", "BayerRG8");
  declare_parameter<int>("width", 0);
  declare_parameter<int>("height", 0);
  declare_parameter<int>("offset_x", 0);
  declare_parameter<int>("offset_y", 0);
  declare_parameter<int>("buffer_count", 2);
  declare_parameter<int>("bayer_quality", 0);
  declare_parameter<int>("grab_timeout_ms", 100);

  exposure_time_ = declare_parameter<double>("exposure_time", 2000.0);
  gain_ = declare_parameter<double>("gain", 8.0);
  frame_rate_ = declare_parameter<double>("frame_rate", 0.0);

  declare_parameter<std::string>("camera_info.distortion_model", "plumb_bob");
  declare_parameter<std::vector<double>>(
    "camera_info.d", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>(
    "camera_info.k", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>(
    "camera_info.r", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>(
    "camera_info.p",
    std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0});

  frame_id_ = get_parameter("frame_id").as_string();
  image_topic_ = get_parameter("image_topic").as_string();
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  grab_timeout_ms_ = std::max(1, static_cast<int>(get_parameter("grab_timeout_ms").as_int()));
}

void HikCameraNode::initializeCameraInfo()
{
  camera_info_template_.header.frame_id = frame_id_;
  camera_info_template_.distortion_model =
    get_parameter("camera_info.distortion_model").as_string();
  camera_info_template_.d = get_parameter("camera_info.d").as_double_array();

  const auto k = get_parameter("camera_info.k").as_double_array();
  const auto r = get_parameter("camera_info.r").as_double_array();
  const auto p = get_parameter("camera_info.p").as_double_array();
  if (k.size() != 9 || r.size() != 9 || p.size() != 12) {
    throw std::runtime_error("camera_info.k/r/p must contain 9/9/12 numbers");
  }
  std::copy(k.begin(), k.end(), camera_info_template_.k.begin());
  std::copy(r.begin(), r.end(), camera_info_template_.r.begin());
  std::copy(p.begin(), p.end(), camera_info_template_.p.begin());

  if (camera_info_template_.k[0] <= 0.0 || camera_info_template_.k[4] <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Camera calibration is unset. Fill camera_info.k/d in config/camera.yaml before PnP use.");
  }
}

void HikCameraNode::openCamera()
{
  MV_CC_DEVICE_INFO_LIST devices{};
  const int enum_result = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
  if (enum_result != MV_OK) {
    throw std::runtime_error("MV_CC_EnumDevices failed: " + errorCode(enum_result));
  }
  if (devices.nDeviceNum == 0) {
    throw std::runtime_error("No HIKROBOT GigE/USB camera was found");
  }

  const std::string requested_serial = get_parameter("serial_number").as_string();
  const int requested_index = static_cast<int>(get_parameter("device_index").as_int());
  int selected_index = -1;

  for (unsigned int index = 0; index < devices.nDeviceNum; ++index) {
    if (devices.pDeviceInfo[index] == nullptr) {
      continue;
    }
    const auto & info = *devices.pDeviceInfo[index];
    const auto serial = serialNumber(info);
    RCLCPP_INFO(
      get_logger(), "Camera [%u]: model=%s serial=%s transport=%s", index,
      modelName(info).c_str(), serial.c_str(),
      info.nTLayerType == MV_USB_DEVICE ? "USB3" : "GigE");
    if ((!requested_serial.empty() && requested_serial == serial) ||
      (requested_serial.empty() && requested_index == static_cast<int>(index)))
    {
      selected_index = static_cast<int>(index);
    }
  }

  if (selected_index < 0) {
    throw std::runtime_error(
            requested_serial.empty() ? "device_index is outside the enumerated camera list" :
            "No camera matches serial_number=" + requested_serial);
  }

  auto * selected_info = devices.pDeviceInfo[selected_index];
  int result = MV_CC_CreateHandle(&camera_handle_, selected_info);
  if (result != MV_OK) {
    throw std::runtime_error("MV_CC_CreateHandle failed: " + errorCode(result));
  }
  result = MV_CC_OpenDevice(camera_handle_);
  if (result != MV_OK) {
    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;
    throw std::runtime_error("MV_CC_OpenDevice failed: " + errorCode(result));
  }

  if (selected_info->nTLayerType == MV_GIGE_DEVICE) {
    const int packet_size = MV_CC_GetOptimalPacketSize(camera_handle_);
    if (packet_size > 0) {
      const int packet_result = MV_CC_SetIntValueEx(camera_handle_, "GevSCPSPacketSize", packet_size);
      if (packet_result != MV_OK) {
        RCLCPP_WARN(
          get_logger(), "Failed to set optimal GigE packet size: %s",
          errorCode(packet_result).c_str());
      }
    }
  }

  RCLCPP_INFO(
    get_logger(), "Opened camera model=%s serial=%s", modelName(*selected_info).c_str(),
    serialNumber(*selected_info).c_str());
}

void HikCameraNode::configureCamera()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  auto require = [this](const int result, const char * operation) {
      if (result != MV_OK) {
        throw std::runtime_error(std::string(operation) + " failed: " + errorCode(result));
      }
    };

  require(MV_CC_SetEnumValueByString(camera_handle_, "AcquisitionMode", "Continuous"),
    "Set AcquisitionMode=Continuous");
  require(MV_CC_SetEnumValueByString(camera_handle_, "TriggerMode", "Off"),
    "Set TriggerMode=Off");

  const int width = static_cast<int>(get_parameter("width").as_int());
  const int height = static_cast<int>(get_parameter("height").as_int());
  const int offset_x = static_cast<int>(get_parameter("offset_x").as_int());
  const int offset_y = static_cast<int>(get_parameter("offset_y").as_int());
  if (width > 0 || height > 0) {
    MV_CC_SetIntValueEx(camera_handle_, "OffsetX", 0);
    MV_CC_SetIntValueEx(camera_handle_, "OffsetY", 0);
  }
  if (width > 0) {
    require(MV_CC_SetIntValueEx(camera_handle_, "Width", width), "Set Width");
  }
  if (height > 0) {
    require(MV_CC_SetIntValueEx(camera_handle_, "Height", height), "Set Height");
  }
  if (offset_x > 0) {
    require(MV_CC_SetIntValueEx(camera_handle_, "OffsetX", offset_x), "Set OffsetX");
  }
  if (offset_y > 0) {
    require(MV_CC_SetIntValueEx(camera_handle_, "OffsetY", offset_y), "Set OffsetY");
  }

  const auto pixel_format = get_parameter("pixel_format").as_string();
  if (!pixel_format.empty()) {
    const int pixel_result =
      MV_CC_SetEnumValueByString(camera_handle_, "PixelFormat", pixel_format.c_str());
    if (pixel_result != MV_OK) {
      RCLCPP_WARN(
        get_logger(), "PixelFormat=%s is unavailable (%s); keeping the camera's current format",
        pixel_format.c_str(), errorCode(pixel_result).c_str());
    }
  }

  const auto buffer_count = static_cast<unsigned int>(
    std::max<int64_t>(1, get_parameter("buffer_count").as_int()));
  const int buffer_result = MV_CC_SetImageNodeNum(camera_handle_, buffer_count);
  if (buffer_result != MV_OK) {
    RCLCPP_WARN(get_logger(), "Failed to set buffer_count: %s", errorCode(buffer_result).c_str());
  }

  const auto bayer_quality = static_cast<unsigned int>(
    std::clamp<int64_t>(get_parameter("bayer_quality").as_int(), 0, 3));
  const int bayer_result = MV_CC_SetBayerCvtQuality(camera_handle_, bayer_quality);
  if (bayer_result != MV_OK) {
    RCLCPP_WARN(get_logger(), "Failed to set Bayer conversion quality: %s", errorCode(bayer_result).c_str());
  }

  require(MV_CC_SetEnumValueByString(camera_handle_, "ExposureAuto", "Off"),
    "Set ExposureAuto=Off");
  require(MV_CC_SetEnumValueByString(camera_handle_, "GainAuto", "Off"),
    "Set GainAuto=Off");

  std::string reason;
  if (!setFloatFeatureLocked("ExposureTime", exposure_time_, reason)) {
    throw std::runtime_error(reason);
  }
  if (!setFloatFeatureLocked("Gain", gain_, reason)) {
    throw std::runtime_error(reason);
  }
  if (!setFrameRateLocked(frame_rate_, reason)) {
    throw std::runtime_error(reason);
  }

  RCLCPP_INFO(
    get_logger(), "Camera configured: exposure=%.1f us gain=%.2f dB frame_rate=%s",
    exposure_time_, gain_, frame_rate_ <= 0.0 ? "maximum" : std::to_string(frame_rate_).c_str());
}

void HikCameraNode::startCapture()
{
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    const int result = MV_CC_StartGrabbing(camera_handle_);
    if (result != MV_OK) {
      throw std::runtime_error("MV_CC_StartGrabbing failed: " + errorCode(result));
    }
    grabbing_ = true;
  }
  running_.store(true);
  capture_thread_ = std::thread(&HikCameraNode::captureLoop, this);
}

void HikCameraNode::stopAndCloseCamera()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (camera_handle_ == nullptr) {
    return;
  }
  if (grabbing_) {
    MV_CC_StopGrabbing(camera_handle_);
    grabbing_ = false;
  }
  MV_CC_CloseDevice(camera_handle_);
  MV_CC_DestroyHandle(camera_handle_);
  camera_handle_ = nullptr;
}

void HikCameraNode::captureLoop()
{
  using SteadyClock = std::chrono::steady_clock;
  auto report_start = SteadyClock::now();
  std::uint64_t published_frames = 0;
  std::uint64_t dropped_frames = 0;
  std::uint32_t previous_frame_number = 0;
  unsigned int consecutive_failures = 0;

  while (rclcpp::ok() && running_.load()) {
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    MV_FRAME_OUT frame{};
    int get_result = MV_E_CALLORDER;
    int convert_result = MV_OK;
    rclcpp::Time capture_stamp(0, 0, get_clock()->get_clock_type());

    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (!running_.load() || camera_handle_ == nullptr || !grabbing_) {
        break;
      }
      get_result = MV_CC_GetImageBuffer(camera_handle_, &frame, grab_timeout_ms_);
      if (get_result == MV_OK) {
        capture_stamp = now();
        const auto width = static_cast<std::uint32_t>(frame.stFrameInfo.nWidth);
        const auto height = static_cast<std::uint32_t>(frame.stFrameInfo.nHeight);
        const std::size_t rgb_size = static_cast<std::size_t>(width) * height * 3U;
        if (rgb_size > std::numeric_limits<unsigned int>::max()) {
          convert_result = MV_E_PARAMETER;
        } else {
          image->data.resize(rgb_size);
          if (frame.stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed &&
            frame.stFrameInfo.nFrameLen >= rgb_size)
          {
            std::memcpy(image->data.data(), frame.pBufAddr, rgb_size);
          } else {
            MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
            conversion.nWidth = width;
            conversion.nHeight = height;
            conversion.enSrcPixelType = frame.stFrameInfo.enPixelType;
            conversion.pSrcData = frame.pBufAddr;
            conversion.nSrcDataLen = frame.stFrameInfo.nFrameLen;
            conversion.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            conversion.pDstBuffer = image->data.data();
            conversion.nDstBufferSize = static_cast<unsigned int>(rgb_size);
            convert_result = MV_CC_ConvertPixelTypeEx(camera_handle_, &conversion);
            if (convert_result == MV_OK && conversion.nDstLen != rgb_size) {
              image->data.resize(conversion.nDstLen);
            }
          }
        }

        if (previous_frame_number != 0 && frame.stFrameInfo.nFrameNum > previous_frame_number + 1) {
          dropped_frames += frame.stFrameInfo.nFrameNum - previous_frame_number - 1;
        }
        previous_frame_number = frame.stFrameInfo.nFrameNum;
        MV_CC_FreeImageBuffer(camera_handle_, &frame);
      }
    }

    if (get_result != MV_OK) {
      ++consecutive_failures;
      if (consecutive_failures % 20 == 0) {
        RCLCPP_WARN(
          get_logger(), "Image acquisition failed %u consecutive times (last=%s)",
          consecutive_failures, errorCode(get_result).c_str());
      }
      continue;
    }
    consecutive_failures = 0;
    if (convert_result != MV_OK) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "RGB conversion failed: %s",
        errorCode(convert_result).c_str());
      continue;
    }

    const auto width = static_cast<std::uint32_t>(frame.stFrameInfo.nWidth);
    const auto height = static_cast<std::uint32_t>(frame.stFrameInfo.nHeight);
    image->header.stamp = capture_stamp;
    image->header.frame_id = frame_id_;
    image->height = height;
    image->width = width;
    image->encoding = "rgb8";
    image->is_bigendian = false;
    image->step = width * 3U;

    auto camera_info = camera_info_template_;
    camera_info.header = image->header;
    camera_info.width = width;
    camera_info.height = height;
    camera_info_pub_->publish(camera_info);
    image_pub_->publish(std::move(image));
    ++published_frames;

    const auto current_time = SteadyClock::now();
    const double elapsed = std::chrono::duration<double>(current_time - report_start).count();
    if (elapsed >= 2.0) {
      RCLCPP_INFO(
        get_logger(), "Publishing %.1f FPS (SDK frame gaps: %lu)",
        static_cast<double>(published_frames) / elapsed,
        static_cast<unsigned long>(dropped_frames));
      report_start = current_time;
      published_frames = 0;
      dropped_frames = 0;
    }
  }
}

bool HikCameraNode::setFloatFeatureLocked(
  const char * name, const double value, std::string & reason)
{
  MVCC_FLOATVALUE range{};
  const int get_result = MV_CC_GetFloatValue(camera_handle_, name, &range);
  if (get_result != MV_OK) {
    reason = std::string("Read ") + name + " range failed: " + errorCode(get_result);
    return false;
  }
  if (value < range.fMin || value > range.fMax) {
    std::ostringstream stream;
    stream << name << '=' << value << " is outside [" << range.fMin << ", " << range.fMax << ']';
    reason = stream.str();
    return false;
  }
  const int set_result = MV_CC_SetFloatValue(camera_handle_, name, static_cast<float>(value));
  if (set_result != MV_OK) {
    reason = std::string("Set ") + name + " failed: " + errorCode(set_result);
    return false;
  }
  return true;
}

bool HikCameraNode::setFrameRateLocked(const double value, std::string & reason)
{
  if (value <= 0.0) {
    const int result = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
    if (result != MV_OK) {
      reason = "Disable AcquisitionFrameRateEnable failed: " + errorCode(result);
      return false;
    }
    return true;
  }

  const int enable_result =
    MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
  if (enable_result != MV_OK) {
    reason = "Enable AcquisitionFrameRateEnable failed: " + errorCode(enable_result);
    return false;
  }
  return setFloatFeatureLocked("AcquisitionFrameRate", value, reason);
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(camera_mutex_);
  for (const auto & parameter : parameters) {
    std::string reason;
    if (parameter.get_name() == "exposure_time") {
      const double value = parameter.as_double();
      if (!setFloatFeatureLocked("ExposureTime", value, reason)) {
        result.successful = false;
        result.reason = reason;
        return result;
      }
      exposure_time_ = value;
    } else if (parameter.get_name() == "gain") {
      const double value = parameter.as_double();
      if (!setFloatFeatureLocked("Gain", value, reason)) {
        result.successful = false;
        result.reason = reason;
        return result;
      }
      gain_ = value;
    } else if (parameter.get_name() == "frame_rate") {
      const double value = parameter.as_double();
      if (!setFrameRateLocked(value, reason)) {
        result.successful = false;
        result.reason = reason;
        return result;
      }
      frame_rate_ = value;
    }
  }
  return result;
}

}  // namespace hik_camera

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
