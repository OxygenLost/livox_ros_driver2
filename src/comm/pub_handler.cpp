//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "pub_handler.h"

#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>

namespace livox_ros {

namespace {

constexpr uint64_t kDriverDiagSummaryPeriodNs = 1000000000ULL;
constexpr uint64_t kDriverTimestampGapWarnNs = 100000000ULL;
constexpr size_t kDriverRawQueueWarnDepth = 100;

struct PacketStreamDiag {
  bool initialized = false;
  uint8_t time_type = kTimestampTypeNoSync;
  uint64_t last_timestamp = 0;
  uint64_t packet_count = 0;
  uint64_t rollback_count = 0;
  uint64_t gap_count = 0;
  uint64_t delta_min = std::numeric_limits<uint64_t>::max();
  uint64_t delta_max = 0;
};

struct DriverPacketDiag {
  PacketStreamDiag lidar;
  PacketStreamDiag imu;
  uint64_t raw_processed = 0;
  size_t raw_queue_depth = 0;
  size_t raw_queue_max = 0;
  bool time_type_mismatch = false;
  bool mismatch_initialized = false;
  uint64_t last_raw_queue_warn_steady_ns = 0;
  uint64_t last_summary_steady_ns = 0;
};

std::mutex g_driver_diag_mutex;
std::map<uint32_t, DriverPacketDiag> g_driver_diag;

uint64_t SteadyNowNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint64_t SystemNowNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

const char* TimestampTypeName(uint8_t time_type) {
  switch (time_type) {
    case kTimestampTypeNoSync:
      return "no_sync_host_clock";
    case kTimestampTypeGptpOrPtp:
      return "ptp";
    case kTimestampTypeGps:
      return "gps";
    default:
      return "unknown";
  }
}

void MaybePrintDriverDiagLocked(uint32_t handle, DriverPacketDiag& diag, uint64_t now_ns) {
  if (diag.last_summary_steady_ns != 0 &&
      now_ns - diag.last_summary_steady_ns < kDriverDiagSummaryPeriodNs) {
    return;
  }

  const auto normalize_min = [](const PacketStreamDiag& stream) {
    return stream.delta_min == std::numeric_limits<uint64_t>::max() ? 0 : stream.delta_min;
  };

  std::cout << "[LIVOX_DRIVER_DIAG] ip=" << IpNumToString(handle)
            << " lidar_packets=" << diag.lidar.packet_count
            << " imu_packets=" << diag.imu.packet_count
            << " lidar_time_type=" << TimestampTypeName(diag.lidar.time_type)
            << " imu_time_type=" << TimestampTypeName(diag.imu.time_type)
            << " lidar_ts=" << diag.lidar.last_timestamp
            << " imu_ts=" << diag.imu.last_timestamp
            << " lidar_dt_min_ns=" << normalize_min(diag.lidar)
            << " lidar_dt_max_ns=" << diag.lidar.delta_max
            << " imu_dt_min_ns=" << normalize_min(diag.imu)
            << " imu_dt_max_ns=" << diag.imu.delta_max
            << " lidar_rollbacks=" << diag.lidar.rollback_count
            << " imu_rollbacks=" << diag.imu.rollback_count
            << " lidar_gaps=" << diag.lidar.gap_count
            << " imu_gaps=" << diag.imu.gap_count
            << " time_type_mismatch=" << (diag.time_type_mismatch ? 1 : 0)
            << " raw_queue=" << diag.raw_queue_depth
            << " raw_queue_max=" << diag.raw_queue_max
            << " raw_processed=" << diag.raw_processed
            << std::endl;

  diag.lidar.delta_min = std::numeric_limits<uint64_t>::max();
  diag.lidar.delta_max = 0;
  diag.imu.delta_min = std::numeric_limits<uint64_t>::max();
  diag.imu.delta_max = 0;
  diag.raw_queue_max = diag.raw_queue_depth;
  diag.last_summary_steady_ns = now_ns;
}

void TrackIncomingPacket(uint32_t handle, bool is_imu, uint8_t time_type,
                         uint64_t timestamp, uint32_t dot_num,
                         uint32_t packet_time_interval, uint64_t point_interval) {
  const uint64_t now_ns = SteadyNowNs();
  const uint64_t host_wall_ns = SystemNowNs();
  std::lock_guard<std::mutex> lock(g_driver_diag_mutex);
  DriverPacketDiag& diag = g_driver_diag[handle];
  PacketStreamDiag& stream = is_imu ? diag.imu : diag.lidar;
  const char* stream_name = is_imu ? "imu" : "lidar";

  if (!stream.initialized) {
    stream.initialized = true;
    stream.time_type = time_type;
  } else if (stream.time_type != time_type) {
    std::cerr << "[LIVOX_CLOCK_SOURCE] ip=" << IpNumToString(handle)
              << " stream=" << stream_name
              << " old=" << TimestampTypeName(stream.time_type)
              << " new=" << TimestampTypeName(time_type)
              << " sdk_timestamp_ns=" << timestamp
              << " host_wall_ns=" << host_wall_ns
              << std::endl;
    stream.time_type = time_type;
  }

  if (stream.last_timestamp != 0) {
    if (timestamp <= stream.last_timestamp) {
      ++stream.rollback_count;
      const uint64_t backward_ns = stream.last_timestamp - timestamp;
      std::cerr << "[LIVOX_TIME_EVENT] reason=timestamp_rollback"
                << " ip=" << IpNumToString(handle)
                << " stream=" << stream_name
                << " time_type=" << TimestampTypeName(time_type)
                << " previous_ns=" << stream.last_timestamp
                << " current_ns=" << timestamp
                << " backward_ns=" << backward_ns
                << " host_wall_ns=" << host_wall_ns
                << " dot_num=" << dot_num
                << " packet_interval=" << packet_time_interval
                << " point_interval_ns=" << point_interval
                << std::endl;
    } else {
      const uint64_t delta = timestamp - stream.last_timestamp;
      stream.delta_min = std::min(stream.delta_min, delta);
      stream.delta_max = std::max(stream.delta_max, delta);
      if (delta > kDriverTimestampGapWarnNs) {
        ++stream.gap_count;
        std::cerr << "[LIVOX_TIME_EVENT] reason=timestamp_gap"
                  << " ip=" << IpNumToString(handle)
                  << " stream=" << stream_name
                  << " time_type=" << TimestampTypeName(time_type)
                  << " previous_ns=" << stream.last_timestamp
                  << " current_ns=" << timestamp
                  << " delta_ns=" << delta
                  << " host_wall_ns=" << host_wall_ns
                  << " dot_num=" << dot_num
                  << " packet_interval=" << packet_time_interval
                  << " point_interval_ns=" << point_interval
                  << std::endl;
      }
    }
  }

  stream.last_timestamp = timestamp;
  ++stream.packet_count;
  if (diag.lidar.initialized && diag.imu.initialized) {
    const bool mismatch = diag.lidar.time_type != diag.imu.time_type;
    if (!diag.mismatch_initialized || mismatch != diag.time_type_mismatch) {
      std::cerr << "[LIVOX_CLOCK_SOURCE] reason=stream_time_type_mismatch"
                << " ip=" << IpNumToString(handle)
                << " lidar=" << TimestampTypeName(diag.lidar.time_type)
                << " imu=" << TimestampTypeName(diag.imu.time_type)
                << " mismatch=" << (mismatch ? 1 : 0)
                << std::endl;
      diag.mismatch_initialized = true;
      diag.time_type_mismatch = mismatch;
    }
  }
  MaybePrintDriverDiagLocked(handle, diag, now_ns);
}

void TrackRawQueue(uint32_t handle, size_t queue_depth) {
  const uint64_t now_ns = SteadyNowNs();
  std::lock_guard<std::mutex> lock(g_driver_diag_mutex);
  DriverPacketDiag& diag = g_driver_diag[handle];
  diag.raw_queue_depth = queue_depth;
  diag.raw_queue_max = std::max(diag.raw_queue_max, queue_depth);
  if (queue_depth >= kDriverRawQueueWarnDepth &&
      (diag.last_raw_queue_warn_steady_ns == 0 ||
       now_ns - diag.last_raw_queue_warn_steady_ns >= kDriverDiagSummaryPeriodNs)) {
    std::cerr << "[LIVOX_QUEUE_EVENT] type=raw_packet_backlog"
              << " ip=" << IpNumToString(handle)
              << " depth=" << queue_depth
              << std::endl;
    diag.last_raw_queue_warn_steady_ns = now_ns;
  }
  MaybePrintDriverDiagLocked(handle, diag, now_ns);
}

void TrackRawProcessed(uint32_t handle, size_t queue_depth) {
  const uint64_t now_ns = SteadyNowNs();
  std::lock_guard<std::mutex> lock(g_driver_diag_mutex);
  DriverPacketDiag& diag = g_driver_diag[handle];
  ++diag.raw_processed;
  diag.raw_queue_depth = queue_depth;
  MaybePrintDriverDiagLocked(handle, diag, now_ns);
}

}  // namespace

std::atomic<bool> PubHandler::is_timestamp_sync_;

PubHandler &pub_handler() {
  static PubHandler handler;
  return handler;
}

static inline void CalcRotationMatrixDegRPY(const ExtParameter& param, double R[3][3]) {
  const double roll = static_cast<double>(param.roll) * PI / 180.0;
  const double pitch = static_cast<double>(param.pitch) * PI / 180.0;
  const double yaw = static_cast<double>(param.yaw) * PI / 180.0;

  const double cos_roll = cos(roll);
  const double cos_pitch = cos(pitch);
  const double cos_yaw = cos(yaw);
  const double sin_roll = sin(roll);
  const double sin_pitch = sin(pitch);
  const double sin_yaw = sin(yaw);

  // Same convention as LidarPubHandler::SetLidarsExtParam()
  R[0][0] = cos_pitch * cos_yaw;
  R[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  R[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  R[1][0] = cos_pitch * sin_yaw;
  R[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  R[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  R[2][0] = -sin_pitch;
  R[2][1] = sin_roll * cos_pitch;
  R[2][2] = cos_roll * cos_pitch;
}

static inline void RotateVectorInPlace(const double R[3][3], float& x, float& y, float& z) {
  const double ox = static_cast<double>(x);
  const double oy = static_cast<double>(y);
  const double oz = static_cast<double>(z);
  x = static_cast<float>(R[0][0] * ox + R[0][1] * oy + R[0][2] * oz);
  y = static_cast<float>(R[1][0] * ox + R[1][1] * oy + R[1][2] * oz);
  z = static_cast<float>(R[2][0] * ox + R[2][1] * oy + R[2][2] * oz);
}

void PubHandler::Init() {
}

void PubHandler::Uninit() {
  if (lidar_listen_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_);
    lidar_listen_id_ = 0;
  }

  RequestExit();

  if (point_process_thread_ &&
    point_process_thread_->joinable()) {
    point_process_thread_->join();
    point_process_thread_ = nullptr;
  } else {
    /* */
  }
}

void PubHandler::RequestExit() {
  is_quit_.store(true);
}

void PubHandler::SetPointCloudConfig(const double publish_freq) {
  publish_interval_ = (kNsPerSecond / (publish_freq * 10)) * 10;
  publish_interval_tolerance_ = publish_interval_ - kNsTolerantFrameTimeDeviation;
  publish_interval_ms_ = publish_interval_ / kRatioOfMsToNs;
  if (!point_process_thread_) {
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this);
  }
  return;
}

void PubHandler::SetImuDataCallback(ImuDataCallback cb, void* client_data) {
  imu_client_data_ = client_data;
  imu_callback_ = cb;
}

void PubHandler::AddLidarsExtParam(LidarExtParameter& lidar_param) {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  uint32_t id = 0;
  GetLidarId(lidar_param.lidar_type, lidar_param.handle, id);
  lidar_extrinsics_[id] = lidar_param;
}

void PubHandler::ClearAllLidarsExtrinsicParams() {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  lidar_extrinsics_.clear();
}

void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void* client_data) {
  pub_client_data_ = client_data;
  points_callback_ = cb;
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this);
}

void PubHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                                LivoxLidarEthernetPacket *data, void *client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self || !data) {
    if (!data) {
      std::cerr << "[LIVOX_DRIVER_EVENT] reason=null_packet ip=" << IpNumToString(handle)
                << std::endl;
    }
    return;
  }

  if (data->time_type != kTimestampTypeNoSync) {
    is_timestamp_sync_.store(true);
  } else {
    is_timestamp_sync_.store(false);
  }

  if (data->data_type == kLivoxLidarImuData) {
    if (self->imu_callback_) {
      RawImuPoint* imu = (RawImuPoint*) data->data;
      ImuData imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType);
      imu_data.handle = handle;
      imu_data.time_stamp = GetEthPacketTimestamp(data->time_type,
                                                  data->timestamp, sizeof(data->timestamp));
      TrackIncomingPacket(handle, true, data->time_type, imu_data.time_stamp,
                          data->dot_num, data->time_interval, 0);
      imu_data.gyro_x = imu->gyro_x;
      imu_data.gyro_y = imu->gyro_y;
      imu_data.gyro_z = imu->gyro_z;
      imu_data.acc_x = imu->acc_x;
      imu_data.acc_y = imu->acc_y;
      imu_data.acc_z = imu->acc_z;

      // Apply the same extrinsic rotation used for point cloud compensation,
      // so the IMU data is corrected at the source level (bag/storage layer).
      {
        std::unique_lock<std::mutex> lock(self->packet_mutex_);
        auto it = self->lidar_extrinsics_.find(handle);
        if (it != self->lidar_extrinsics_.end()) {
          double R[3][3];
          CalcRotationMatrixDegRPY(it->second.param, R);
          RotateVectorInPlace(R, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
          RotateVectorInPlace(R, imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
        }
      }

      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }
  RawPacket packet = {};
  packet.handle = handle;
  packet.lidar_type = LidarProtoType::kLivoxLidarType;
  packet.extrinsic_enable = false; 
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) {
    packet.line_num = kLineNumberHAP;
  } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360 ||
             dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360s ||
             dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360l) {
    packet.line_num = kLineNumberMid360;
  } else {
    packet.line_num = kLineNumberDefault;
  }
  packet.data_type = data->data_type;
  packet.point_num = data->dot_num;
  if (data->dot_num == 0) {
    const uint64_t timestamp = GetEthPacketTimestamp(data->time_type,
                                                     data->timestamp, sizeof(data->timestamp));
    TrackIncomingPacket(handle, false, data->time_type, timestamp,
                        data->dot_num, data->time_interval, 0);
    std::cerr << "[LIVOX_DRIVER_EVENT] reason=zero_dot_num"
              << " ip=" << IpNumToString(handle)
              << " data_type=" << static_cast<int>(data->data_type)
              << " time_type=" << TimestampTypeName(data->time_type)
              << " timestamp_ns=" << timestamp
              << std::endl;
    return;
  }
  packet.point_interval = data->time_interval * 100 / data->dot_num;  //ns
  packet.time_stamp = GetEthPacketTimestamp(data->time_type,
                                            data->timestamp, sizeof(data->timestamp));
  TrackIncomingPacket(handle, false, data->time_type, packet.time_stamp,
                      data->dot_num, data->time_interval, packet.point_interval);
  uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  size_t raw_queue_depth = 0;
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_packet_queue_.push_back(packet);
    raw_queue_depth = self->raw_packet_queue_.size();
  }
  TrackRawQueue(handle, raw_queue_depth);
  self->packet_condition_.notify_one();

  return;
}

void PubHandler::PublishPointCloud() {
  //publish point
  if (points_callback_) {
    points_callback_(&frame_, pub_client_data_);
  }
  return;
}

void PubHandler::CheckTimer(uint32_t id) {

  if (PubHandler::is_timestamp_sync_.load()) { // Enable time synchronization
    auto& process_handler = lidar_process_handlers_[id];
    uint64_t recent_time_ms = process_handler->GetRecentTimeStamp() / kRatioOfMsToNs;
    if ((recent_time_ms % publish_interval_ms_ != 0) || recent_time_ms == 0) {
      return;
    }

    uint64_t diff = process_handler->GetRecentTimeStamp() - process_handler->GetLidarBaseTime();
    if (diff < publish_interval_tolerance_) {
      return;
    }

    frame_.base_time[frame_.lidar_num] = process_handler->GetLidarBaseTime();
    points_[id].clear();
    process_handler->GetLidarPointClouds(points_[id]);
    if (points_[id].empty()) {
      return;
    }
    PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
    lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
    lidar_point.handle = id;
    lidar_point.points_num = points_[id].size();
    lidar_point.points = points_[id].data();
    frame_.lidar_num++;
    
    if (frame_.lidar_num != 0) {
      PublishPointCloud();
      frame_.lidar_num = 0;
    }
  } else { // Disable time synchronization
    auto now_time = std::chrono::high_resolution_clock::now();
    //First Set
    static bool first = true;
    if (first) {
      last_pub_time_ = now_time;
      first = false;
      return;
    }
    if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) {
      return;
    }
    last_pub_time_ += std::chrono::nanoseconds(publish_interval_);
    for (auto &process_handler : lidar_process_handlers_) {
      frame_.base_time[frame_.lidar_num] = process_handler.second->GetLidarBaseTime();
      uint32_t handle = process_handler.first;
      points_[handle].clear();
      process_handler.second->GetLidarPointClouds(points_[handle]);
      if (points_[handle].empty()) {
        continue;
      }
      PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
      lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
      lidar_point.handle = handle;
      lidar_point.points_num = points_[handle].size();
      lidar_point.points = points_[handle].data();
      frame_.lidar_num++;
    }
    PublishPointCloud();
    frame_.lidar_num = 0;
  }
  return;
}

void PubHandler::RawDataProcess() {
  RawPacket raw_data;
  while (!is_quit_.load()) {
    size_t queue_depth_after_pop = 0;
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      if (raw_packet_queue_.empty()) {
        packet_condition_.wait_for(lock, std::chrono::milliseconds(500));
        if (raw_packet_queue_.empty()) {
          continue;
        }
      }
      raw_data = raw_packet_queue_.front();
      raw_packet_queue_.pop_front();
      queue_depth_after_pop = raw_packet_queue_.size();
    }
    TrackRawProcessed(raw_data.handle, queue_depth_after_pop);
    uint32_t id = 0;
    GetLidarId(raw_data.lidar_type, raw_data.handle, id);
    if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) {
      lidar_process_handlers_[id].reset(new LidarPubHandler());
    }
    auto &process_handler = lidar_process_handlers_[id];
    if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) {
        lidar_process_handlers_[id]->SetLidarsExtParam(lidar_extrinsics_[id]);
    }
    process_handler->PointCloudProcess(raw_data);
    CheckTimer(id);
  }
}

bool PubHandler::GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id) {
  if (lidar_type == kLivoxLidarType) {
    id = handle;
    return true;
  }
  return false;
}

uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);

  if (timestamp_type == kTimestampTypeGptpOrPtp ||
      timestamp_type == kTimestampTypeGps) {
    return time.stamp;
  }

  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

/*******************************/
/*  LidarPubHandler Definitions*/
LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {}

uint64_t LidarPubHandler::GetLidarBaseTime() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.at(0).offset_time;
}

void LidarPubHandler::GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds) {
  std::lock_guard<std::mutex> lock(mutex_);
  points_clouds.swap(points_clouds_);
}

uint64_t LidarPubHandler::GetRecentTimeStamp() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.back().offset_time;
}

uint32_t LidarPubHandler::GetLidarPointCloudsSize() {
  std::lock_guard<std::mutex> lock(mutex_);
  return points_clouds_.size();
}

//convert to standard format and extrinsic compensate
void LidarPubHandler::PointCloudProcess(RawPacket & pkt) {
  if (pkt.lidar_type == LidarProtoType::kLivoxLidarType) {
    LivoxLidarPointCloudProcess(pkt);
  } else {
    static bool flag = false;
    if (!flag) {
      std::cout << "error, unsupported protocol type: " << static_cast<int>(pkt.lidar_type) << std::endl;
      flag = true;      
    }
  }
}

void LidarPubHandler::LivoxLidarPointCloudProcess(RawPacket & pkt) {
  switch (pkt.data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type)
                << " !!" << std::endl;
      break;
  }
}

void LidarPubHandler::SetLidarsExtParam(LidarExtParameter lidar_param) {
  if (is_set_extrinsic_params_) {
    return;
  }
  extrinsic_.trans[0] = lidar_param.param.x;
  extrinsic_.trans[1] = lidar_param.param.y;
  extrinsic_.trans[2] = lidar_param.param.z;

  double cos_roll = cos(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double cos_pitch = cos(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double cos_yaw = cos(static_cast<double>(lidar_param.param.yaw * PI / 180.0));
  double sin_roll = sin(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double sin_pitch = sin(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double sin_yaw = sin(static_cast<double>(lidar_param.param.yaw * PI / 180.0));

  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  is_set_extrinsic_params_ = true;
}

void LidarPubHandler::ProcessCartesianHighPoint(RawPacket & pkt) {
  LivoxLidarCartesianHighRawPoint* raw = (LivoxLidarCartesianHighRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessCartesianLowPoint(RawPacket & pkt) {
  LivoxLidarCartesianLowRawPoint* raw = (LivoxLidarCartesianLowRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessSphericalPoint(RawPacket& pkt) {
  LivoxLidarSpherPoint* raw = (LivoxLidarSpherPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

} // namespace livox_ros
