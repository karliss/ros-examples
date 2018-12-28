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
#include <mvVISLAM.h>
#include "SnapdragonVislamManager.hpp"
#include <unistd.h>
#include <px4_tasks.h>
#include <px4_includes.h>
#include <px4_posix.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/snap_vislam_status.h>
#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <cmath>

#define MIN_NUM_FEATURES 20

class Vislam
{
public:
  Vislam();
  ~Vislam();
  int start();
  void status();
private:
  int    _main_task = -1;      /**< handle for task */
  std::atomic<bool> thread_stop_;
  int previous_mv_tracking_state_ = MV_TRACKING_STATE_FAILED;
  orb_advert_t pose_pub = nullptr;
  orb_advert_t status_pub = nullptr;

  void    task_main();
  static int  task_main_trampoline(int argc, char *argv[]);

  int32_t PublishVislamData( mvVISLAMPose& vislamPose, int64_t vislamFrameId,
                            uint64_t timestamp_ns, int32_t num_tracked_points );
  int32_t PublishVislamStatus( mvVISLAMPose& vislamPose, uint64_t timestamp_ns,
  int32_t statusCode, int32_t num_tracked_points);
};


Vislam::Vislam()
  : thread_stop_(false)
{
}

Vislam::~Vislam()
{
  if (_main_task != -1) {
    /* task wakes up every 100ms or so at the longest */
    thread_stop_ = true;

    /* wait for a second for the task to quit at our request */
    unsigned i = 0;

    do {
      /* wait 20ms */
      usleep(20000);

      /* if we have given up, kill it */
      if (++i > 500) {
        PX4_WARN("Snapdragon vislam didn't exit in time, killing it");
        px4_task_delete(_main_task);
        break;
      }
    } while (_main_task != -1);
  }
}

int
Vislam::start()
{
  ASSERT(_main_task == -1);

  /* start the task */
  _main_task = px4_task_spawn_cmd("snapdragon_mv",
          SCHED_DEFAULT,
          SCHED_PRIORITY_DEFAULT + 15,
          8*0x100000,
          (px4_main_t)&Vislam::task_main_trampoline,
          nullptr);

  if (_main_task < 0) {
    warn("task start failed");
    return -errno;
  }

  return OK;
}

void
Vislam::status()
{
  const char* state = "unknown";
  switch (previous_mv_tracking_state_) {
    case MV_TRACKING_STATE_FAILED:
      state = "failed";
      break;

    case MV_TRACKING_STATE_INITIALIZING:
      state = "initializing";
      break;

    case MV_TRACKING_STATE_HIGH_QUALITY:
      state = "high quality";
      break;

    case MV_TRACKING_STATE_LOW_QUALITY:
      state = "low quality";
      break;
  }
  PX4_INFO("Vislam state state: %s(%d)", state, previous_mv_tracking_state_);
}


void Vislam::task_main()
{
  mvCameraConfiguration config;
  // Set up camera configuraiton (snapdragon down facing camera)
  memset(&config, 0, sizeof(config));
  config.pixelWidth = 640;
  config.pixelHeight = 480;
  config.memoryStride = 640;

  //config.principalPoint[0] = 320;
  //config.principalPoint[1] = 240;
  config.principalPoint[0] = 339.312073;
  config.principalPoint[1] = 258.424683;

  //config.focalLength[0] = 275;
  //config.focalLength[1] = 275;
  config.focalLength[0] = 281.514832;
  config.focalLength[1] = 281.514832;

  config.distortion[0] = 0.007518;
  config.distortion[1] = 0.018835;
  config.distortion[2] = -0.015182;
  config.distortion[3] = 0.003097;
  config.distortion[4] = 0;
  config.distortion[5] = 0;
  config.distortion[6] = 0;
  config.distortion[7] = 0;
  config.distortionModel = 10;

  Snapdragon::VislamManager::InitParams vislamParams;

  // Transformation between camera and IMU frame (forward-right-down).
  // axis angle format
  vislamParams.tbc[0] = -0.01213f;
  vislamParams.tbc[1] = 0.01982f;
  vislamParams.tbc[2] = 0.00717f;
  vislamParams.ombc[0] = 0.2417f;
  vislamParams.ombc[1] = 0.27189f;
  vislamParams.ombc[2] = 1.59864f;

  vislamParams.delta = -0.0087f;

  vislamParams.std0Tbc[0] = 0.005;
  vislamParams.std0Tbc[1] = 0.005;
  vislamParams.std0Tbc[2] = 0.005;

  vislamParams.std0Ombc[0] = 0.04;
  vislamParams.std0Ombc[1] = 0.04;
  vislamParams.std0Ombc[2] = 0.04;

  vislamParams.std0Delta = 0.001;
  vislamParams.accelMeasRange = 156;
  vislamParams.gyroMeasRange = 34;

  vislamParams.stdAccelMeasNoise = 0.316227766016838; // sqrt(1e-1);
  vislamParams.stdGyroMeasNoise = 1e-2; // sqrt(1e-4);

  vislamParams.stdCamNoise = 100;
  vislamParams.minStdPixelNoise = 0.5;
  vislamParams.failHighPixelNoiseScaleFactor = 1.6651f;

  vislamParams.logDepthBootstrap = -3.912;// 2cm;
  vislamParams.useLogCameraHeight = false;
  vislamParams.logCameraHeightBootstrap = -3.22;
  vislamParams.noInitWhenMoving = true;
  vislamParams.limitedIMUbWtrigger = 35.0;

  vislamParams.staticMaskFileName = "";
  vislamParams.gpsImuTimeAlignment = 0.0;
  vislamParams.tba[0] = 0.0;
  vislamParams.tba[1] = 0.0;
  vislamParams.tba[2] = 0.0;
  vislamParams.mapping = true;

  Snapdragon::CameraParameters param;
  param.enable_cpa = true;
  param.camera_config.fps = 30;
  param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
  param.mv_camera_config = config;

  //set the cpa configuration.
  mvCPA_Configuration cpaConfig;
  cpaConfig.cpaType = MVCPA_MODE_HISTOGRAM;//MVCPA_MODE_COST;
  cpaConfig.legacyCost.startExposure = param.camera_config.exposure;
  cpaConfig.legacyCost.startGain = param.camera_config.gain;
  cpaConfig.legacyCost.filterSize = 1;
  cpaConfig.legacyCost.exposureCost = 1.0f;
  cpaConfig.legacyCost.gainCost = 0.3333f;

  param.mv_cpa_config = cpaConfig;
  Snapdragon::VislamManager vislam_man;
  if( vislam_man.Initialize( param, vislamParams ) != 0  ) {
    PX4_ERR( "Snapdragon::RosNodeVislam::VislamThreadMain: Error initializing the VISLAM Manager " );
    return;
  }

  // start the VISLAM processing.
  if( vislam_man.Start() != 0 ) {
    PX4_ERR( "Snapdragon::RosNodeVislam::VislamThreadMain: Error Starting the VISLAM manager" );
    return;
  }

  mvVISLAMPose vislamPose;
  int64_t vislamFrameId;
  uint64_t timestamp_ns;
  int32_t vislam_ret;

  while( !thread_stop_ ) {
    vislam_ret = vislam_man.GetPose( vislamPose, vislamFrameId, timestamp_ns );

    // get the total number of tracked points
    int32_t num_tracked_points = vislam_man.HasUpdatedPointCloud();

    PublishVislamStatus(vislamPose, timestamp_ns, vislam_ret, num_tracked_points);

    if( vislam_ret == 0 ) {
      //check if the pose quality is good.  If not do not publish the data.
      if( vislamPose.poseQuality != MV_TRACKING_STATE_FAILED  &&
          vislamPose.poseQuality != MV_TRACKING_STATE_INITIALIZING ) {
          // Publish Pose Data
          PublishVislamData( vislamPose, vislamFrameId, timestamp_ns, num_tracked_points );
      }

      // Log changes in tracking state
      if (previous_mv_tracking_state_ != vislamPose.poseQuality)
      {
        switch (vislamPose.poseQuality)
        {
          case MV_TRACKING_STATE_FAILED:
            PX4_INFO("VISLAM TRACKING FAILED");
            break;

          case MV_TRACKING_STATE_INITIALIZING:
            PX4_INFO("VISLAM INITIALIZING");
            break;

          case MV_TRACKING_STATE_HIGH_QUALITY:
            PX4_INFO("VISLAM TRACKING HIGH QUALITY");
            break;

          case MV_TRACKING_STATE_LOW_QUALITY:
            PX4_INFO("VISLAM TRACKING LOW QUALITY");
            break;
        }
      }
      previous_mv_tracking_state_ = vislamPose.poseQuality;
    }
    else {
      PX4_WARN( "Snapdragon::RosNodeVislam::VislamThreadMain: Warning Getting Pose Information" );
    }
  }

  // the thread is shutting down. Stop the vislam Manager.
  vislam_man.Stop();
  if (pose_pub) {
    orb_unadvertise(pose_pub);
  }
  if (status_pub) {
    orb_unadvertise(status_pub);
  }

  PX4_INFO( "Snapdragon::RosNodeVislam::VislamThreadMain: Exising VISLAM Thread" );
  _main_task = -1;
  return;
}

int32_t Vislam::PublishVislamData( mvVISLAMPose& vislamPose, int64_t vislamFrameId,
                                  uint64_t timestamp_ns, int32_t num_tracked_points )
{
  vehicle_odometry_s result_pose = {};
  result_pose.timestamp = timestamp_ns / 1000;

  matrix::Dcmf orientationMatrix;
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
      orientationMatrix(i,j) = vislamPose.bodyPose.matrix[i][j];
    }
  }
  matrix::Quaternionf orientation(orientationMatrix);
  result_pose.x = vislamPose.bodyPose.matrix[0][3];
  result_pose.y = vislamPose.bodyPose.matrix[1][3];
  result_pose.z = vislamPose.bodyPose.matrix[2][3];

  orientation.copyTo(result_pose.q);

  // Rbc estimation
  //matrix::AxisAnglef rbc = matrix::Dcmf(vislamPose.Rbc);
  //PX4_INFO("tr: %+0.5f %+0.5f %+0.5f | %+0.5f %+0.5f %+0.5f | %+0.5f", (double)vislamPose.tbc[0], (double)vislamPose.tbc[1], (double)vislamPose.tbc[2],
  //(double)rbc(0), (double)rbc(1), (double)rbc(2), (double)vislamPose.timeAlignment
  //);

  result_pose.vx = vislamPose.velocity[0];
  result_pose.vy = vislamPose.velocity[1];
  result_pose.vz = vislamPose.velocity[2];
  result_pose.rollspeed = vislamPose.angularVelocity[0];
  result_pose.pitchspeed = vislamPose.angularVelocity[1];
  result_pose.yawspeed = vislamPose.angularVelocity[2];

  // convert pose covariance from 6x6 matrix to upper triangle
  int covariance_index = 0;
  for( int16_t i = 0; i < 6; i++ ) {
    for( int16_t j = i; j < 6; j++ ) {
      float value = 10001.0f;
      if (num_tracked_points > MIN_NUM_FEATURES)
        value = vislamPose.errCovPose[i][j];
      result_pose.pose_covariance[covariance_index++] = value;
    }
  }

  // Same as pose covariance but only position speed <-> position speed part
  // is available.
  for( int16_t i = 0; i < 6; i++ ) {
    for( int16_t j = i; j < 6; j++ ) {
      float value = NAN;
      if (i < 3 && j < 3)
        value = vislamPose.errCovVelocity[i][j];
      result_pose.pose_covariance[covariance_index++] = value;
    }
  }

  if (pose_pub) {
    orb_publish(ORB_ID(vehicle_visual_odometry), pose_pub, &result_pose);
  } else {
    pose_pub = orb_advertise(ORB_ID(vehicle_visual_odometry), &result_pose);
  }

 return 0;
}


int32_t Vislam::PublishVislamStatus( mvVISLAMPose& vislamPose, uint64_t timestamp_ns,
  int32_t statusCode,
  int32_t num_tracked_points )
{
  snap_vislam_status_s msg = {};
  msg.timestamp = timestamp_ns / 1000;

  msg.status = statusCode;
  msg.quality = (uint32_t)vislamPose.poseQuality;
  msg.error_code = vislamPose.errorCode;
  msg.tracked_points = num_tracked_points;

  msg.time_alignment = vislamPose.timeAlignment;
  memcpy(msg.gravity, vislamPose.gravity, sizeof(msg.gravity));
  memcpy(msg.w_bias, vislamPose.wBias, sizeof(msg.w_bias));
  memcpy(msg.a_bias, vislamPose.aBias, sizeof(msg.a_bias));

  memcpy(msg.rbg, vislamPose.Rbg, sizeof(msg.rbg));
  memcpy(msg.tbc, vislamPose.tbc, sizeof(msg.tbc));

  memcpy(msg.rbc, vislamPose.Rbc, sizeof(msg.rbc));
  matrix::AxisAnglef rbcAA = matrix::Dcmf(vislamPose.Rbc);
  msg.rbc_aa[0] = rbcAA(0);
  msg.rbc_aa[1] = rbcAA(1);
  msg.rbc_aa[2] = rbcAA(2);

  if (status_pub) {
    orb_publish(ORB_ID(snap_vislam_status), status_pub, &msg);
  } else {
    status_pub = orb_advertise(ORB_ID(snap_vislam_status), &msg);
  }

  return 0;
}

namespace snapdragon_mv {
  Vislam *g_vislam;
}

int
Vislam::task_main_trampoline(int argc, char *argv[])
{
  snapdragon_mv::g_vislam->task_main();
  return 0;
}


static int usage()
{
  PX4_ERR("usage: snapdragon_mv {start|stop|status}");
  return 1;
}

extern "C" __EXPORT int snapdragon_mv_main(int argc, char *argv[]);

int snapdragon_mv_main(int argc, char *argv[])
{
  if (argc < 2) {
    return usage();
  }

  if (!strcmp(argv[1], "start")) {

    if (snapdragon_mv::g_vislam != nullptr) {
      PX4_ERR("already running");
      return 1;
    }

    snapdragon_mv::g_vislam = new Vislam;

    if (snapdragon_mv::g_vislam == nullptr) {
      PX4_ERR("alloc failed");
      return 1;
    }

    if (OK != snapdragon_mv::g_vislam->start()) {
      delete snapdragon_mv::g_vislam;
      snapdragon_mv::g_vislam = nullptr;
      PX4_ERR("start failed %d", errno);
      return 1;
    }

    return 0;
  }

  if (snapdragon_mv::g_vislam == nullptr) {
    PX4_ERR("not running");
    return 1;
  }

  if (!strcmp(argv[1], "stop")) {
    delete snapdragon_mv::g_vislam;
    PX4_INFO("stop done cli");
    snapdragon_mv::g_vislam = nullptr;
  } else if (!strcmp(argv[1], "status")) {
    snapdragon_mv::g_vislam->status();
  }else {
    usage();
  }

  return 0;
}
