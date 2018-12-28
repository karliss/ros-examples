/****************************************************************************
 *   Copyright (c) 2016 Ramakrishna Kintada. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "SnapdragonVislamManager.hpp"
#include "SnapdragonDebugPrint.h"
#include "drivers/drv_hrt.h"

#include <px4_tasks.h>
#include <px4_includes.h>
#include <px4_posix.h>
#include <uORB/topics/vehicle_odometry.h>

Snapdragon::VislamManager::VislamManager() {
  cam_man_ptr_ = nullptr;
  vislam_ptr_ = nullptr;
  initialized_ = false;
  image_buffer_size_bytes_ = 0;
  image_buffer_ = nullptr;
}

Snapdragon::VislamManager::~VislamManager() {
  CleanUp();
}

void Snapdragon::VislamManager::ImuCallback(
    const sensor_combined_s& msg)
{
  // Convert from ENU (mavros) to NED (vislam expectation) frame
  int64_t current_timestamp_ns = msg.timestamp * 1000;

  static int64_t last_timestamp = 0;
  float delta = 0.f;

  // Sanity check on IMU timestamp
  if (last_timestamp != 0)
  {
    if (current_timestamp_ns < last_timestamp)
    {
      PX4_WARN("Bad IMU timestamp order, dropping data [ns]\t%lld %lld", last_timestamp, current_timestamp_ns);
      return;
    }

    delta = (current_timestamp_ns - last_timestamp) * 1e-6;
    const float imu_sample_dt_reasonable_threshold_ms = 2.5;
    if (delta > imu_sample_dt_reasonable_threshold_ms)
    {
      if (cam_params_.verbose)
      {
        WARN_PRINT("IMU sample dt > %f ms -- %f ms",
                   (double)imu_sample_dt_reasonable_threshold_ms, (double)delta);
      }
    }
  }

  // Feed IMU message to VISLAM
  std::lock_guard<std::mutex> lock(sync_mutex_);
  sensor_queue.push(msg);
}

void Snapdragon::VislamManager::ImuLoop() {
  int all_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));

  const int message_count = 1;
  px4_pollfd_struct_t fds[message_count];

  // Setup of loop
  fds[0].fd = all_sensors_sub;
  fds[0].events = POLLIN;
  while (!stop_imu) {
    int poll_ret = px4_poll(fds, message_count, 500);
    if (poll_ret == 0) {
      PX4_WARN("Vislam not getting any data");
    } else if (poll_ret < 0) {
      PX4_ERR("px4_poll failed");
      break;
    } else {
      if (fds[0].revents & POLLIN) {
        sensor_combined_s sensor_data;
        orb_copy(ORB_ID(sensor_combined), all_sensors_sub, &sensor_data);
        ImuCallback(sensor_data);
      }
    }
  }

  orb_unsubscribe(all_sensors_sub);

}

int32_t Snapdragon::VislamManager::CleanUp() {
  //stop the camera.
  if( cam_man_ptr_ != nullptr ) {
    WARN_PRINT( "Stopping Camera...." );
    cam_man_ptr_->Terminate();
    WARN_PRINT( "Deleting Camera Pointer" );
    delete cam_man_ptr_;
    cam_man_ptr_ = nullptr;
  }

  //stop the vislam engine.
  if( vislam_ptr_ != nullptr ) {
    mvVISLAM_Deinitialize( vislam_ptr_ );
    vislam_ptr_ = nullptr;
  }

  if( image_buffer_ != nullptr ) {
    delete[] image_buffer_;
    image_buffer_ = nullptr;
    image_buffer_size_bytes_ = 0;
  }
  return 0;
}

int32_t Snapdragon::VislamManager::Initialize
(
  const Snapdragon::CameraParameters& cam_params,
  const Snapdragon::VislamManager::InitParams& vislam_params
) {
  cam_params_ = cam_params;
  vislam_params_ = vislam_params;
  int32_t rc = 0;
  if( rc == 0 ) { //initialize the camera module.
    cam_man_ptr_ = new Snapdragon::CameraManager( &cam_params_ ) ;
    if( cam_man_ptr_ != nullptr ) {
      rc = cam_man_ptr_->Initialize();
    }
    else {
      rc = -1;
    }
  }

  //now intialize the VISLAM module.
  if( rc == 0 ) {
    vislam_ptr_ = mvVISLAM_Initialize
    (
      &(cam_params_.mv_camera_config), 0,
      vislam_params_.tbc, vislam_params_.ombc, vislam_params_.delta,
      vislam_params_.std0Tbc, vislam_params_.std0Ombc, vislam_params_.std0Delta,
      vislam_params_.accelMeasRange, vislam_params_.gyroMeasRange,
      vislam_params_.stdAccelMeasNoise, vislam_params_.stdGyroMeasNoise,
      vislam_params_.stdCamNoise, vislam_params_.minStdPixelNoise, vislam_params_.failHighPixelNoiseScaleFactor,
      vislam_params_.logDepthBootstrap, vislam_params_.useLogCameraHeight, vislam_params_.logCameraHeightBootstrap,
      vislam_params_.noInitWhenMoving,
      vislam_params_.limitedIMUbWtrigger,
      vislam_params_.staticMaskFileName,
      vislam_params_.gpsImuTimeAlignment,
      vislam_params_.tba,
      vislam_params_.mapping
    );
    if( vislam_ptr_ == nullptr ) {
      rc = -1;
    }
  }

  if( rc != 0 ) {
    ERROR_PRINT( "Error initializing the Vislam Manager." );
    CleanUp();
  }
  else {
    initialized_ = true;
  }
  return 0;
}

int32_t Snapdragon::VislamManager::Start() {
  int32_t rc = 0;
  if( initialized_ ) {
    //start the camera
    rc |= cam_man_ptr_->Start();

    //wait till we get the first frame.
    while( cam_man_ptr_->GetLatestFrameId()  < 10 ) {
      std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
    }
    // allocate the image buffer here.
    image_buffer_size_bytes_ = cam_man_ptr_->GetImageSize();
    INFO_PRINT( "image Size: %d frameId: %lld", cam_man_ptr_->GetImageSize(), cam_man_ptr_->GetLatestFrameId() );
    image_buffer_ = new uint8_t[ image_buffer_size_bytes_ ];

    // Setup publishers/subscribers
    // Vislam image processing takes longer than time between IMU samples so
    // put IMU gathering in seperate thread to avoid droping samples.
    imu_read_thread = std::thread(&VislamManager::ImuLoop, this);
  }
  else {
    ERROR_PRINT( "Calling Start without calling intialize" );
    rc = -1;
  }
  return rc;
}

int32_t Snapdragon::VislamManager::Stop() {
  CleanUp();

  // Unsubscribe from IMU topic
  stop_imu = true;
  if (imu_read_thread.joinable()) {
    imu_read_thread.join();
  }

  return 0;
}

int32_t Snapdragon::VislamManager::GetPointCloud( mvVISLAMMapPoint* points, uint32_t max_points ) {
  std::lock_guard<std::mutex> lock( sync_mutex_ );
  return mvVISLAM_GetPointCloud( vislam_ptr_, points, max_points );
}

int32_t Snapdragon::VislamManager::HasUpdatedPointCloud() {
  std::lock_guard<std::mutex> lock( sync_mutex_ );
  return mvVISLAM_HasUpdatedPointCloud(vislam_ptr_);
}

int32_t Snapdragon::VislamManager::GetPose( mvVISLAMPose& pose, int64_t& pose_frame_id, uint64_t& timestamp_ns ) {
  int32_t rc = 0;
  if( !initialized_ ) {
    WARN_PRINT( "VislamManager not initialize" );
    return -1;
  }

  // first set the Image from the camera.
  // Next add it to the mvVISLAM
  // Then call the API to get the Pose.
  int64_t frame_id;
  uint32_t used = 0;
  uint64_t frame_ts_ns;
  static int64_t prev_frame_id = 0;
  rc = cam_man_ptr_->GetNextImageData( &frame_id, &frame_ts_ns, image_buffer_, image_buffer_size_bytes_ , &used );

  if( rc != 0 ) {
    WARN_PRINT( "Error Getting the image from camera" );
  }
  else {
    if( prev_frame_id != 0 && (prev_frame_id + 1 != frame_id ) ) {
      WARN_PRINT( "Warning: Missed/Dropped Camera Frames.  recvd(%lld) expected(%lld)", frame_id, (prev_frame_id+1) );
    }

    // adjust the frame-timestamp for VISLAM at it needs the time at the center of the exposure and not the sof.
    // Correction from exposure time
    float correction = 1e3f * (cam_man_ptr_->GetExposureTimeUs()/2.f);

    uint64_t modified_timestamp = frame_ts_ns - static_cast<uint64_t>(correction);// + clock_offset_ns;

    {
      {
        std::lock_guard<std::mutex> lock( sync_mutex_ );
        while (!sensor_queue.empty() && 1000*sensor_queue.front().timestamp < modified_timestamp) {
          auto& msg = sensor_queue.front();
          mvVISLAM_AddGyro(vislam_ptr_, msg.timestamp*1000 + msg.accelerometer_timestamp_relative * 1000LL, msg.gyro_rad[0], msg.gyro_rad[1], msg.gyro_rad[2]);
          mvVISLAM_AddAccel(vislam_ptr_, msg.timestamp*1000, msg.accelerometer_m_s2[0], msg.accelerometer_m_s2[1], msg.accelerometer_m_s2[2]);
          sensor_queue.pop();
        }
      }
      mvVISLAM_AddImage(vislam_ptr_, modified_timestamp, image_buffer_ );
      pose = mvVISLAM_GetPose(vislam_ptr_);
      pose_frame_id = frame_id;
      timestamp_ns = static_cast<uint64_t>(modified_timestamp);
    }
  }
  return rc;
}
