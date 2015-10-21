/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef VITAL_VIDEO_METADATA_H
#define VITAL_VIDEO_METADATA_H

namespace kwiver {
namespace vital {

// ----------------------------------------------------------------
/**
 * @brief
 *
 */
class video_metadata
{
public:
  video_metadata();
  ~video_metadata();

  uint64_t timeUTC() const { return ( this->m_timeUTC ); }
  video_netadata & timeUTC(uint64_t const& v) { this->m_timeUTC = v; return ( *this ); }

  uint64_t get_time_UTC() const { return (m_timeUTC); }
  video_metadata & set_time_UTC(uint64_t const& v) { m_timeUTC = v; return (*this); }

  // Add more methods

private:
  // Time from the metadata
  uint64_t m_timeUTC;

  // platform position (in degrees)
  geo_coord::geo_lat_lon m_platformLatLon;
  double m_platformAltitude; // meters ASL/AGL??

  // platform orientation (RPY) (in degrees)
  // Using a right handed coordinate system. Platform axis is along the X axis.
  // The Z axis is down, making the Y axis point to the right.
  double m_platformRoll;      // About the X axis. Positive roll is to the right
  double m_platformPitch;     // About the Y axis. Positive pitch is up.
  double m_platformYaw;       // About the Z axis. Positive yas is to the Right.

  // sensor orientation WRT the platform (in degrees)
  double m_sensorRoll;        // Relative to the platform
  double m_sensorPitch;       // Relative to the platform - Elevation
  double m_sensorYaw;         // Relative to the platform - Azimuth

  // Corner points of the associated image.
  // Corners start in upper-left and go clockwise.
  geo_coord::geo_lat_lon m_corner_ul;
  geo_coord::geo_lat_lon m_corner_ur;
  geo_coord::geo_lat_lon m_corner_lr;
  geo_coord::geo_lat_lon m_corner_ll;

  geo_coord::geo_lat_lon m_frameCenter;

  // Field of view and slant angle parameters for the camera.
  double m_slantRange;
  double m_sensorHorizFOV;
  double m_sensorVertFOV;

  // External device recommended ground sample distances (GSDs).
  double m_horizontal_gsd;
  double m_vertical_gsd;

  // String indicating the camera mode (typically the zoom level).
  std::string m_camera_mode;

  bool m_is_valid;

  // Platform velocity
  /// TBD

}; // end class video_metadata

} } // end namespace

#endif /* VITAL_VIDEO_METADATA_H */
