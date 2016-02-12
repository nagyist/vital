/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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

/**
 * \file
 * \brief This file contains the implementation for vital video metadata.
 */

#include "video_metadata.h"
#include "convert_metadata.h"

#include <vital/klv/klv_0104.h>
#include <vital/klv/klv_data.h>

#include <vital/logger/logger.h>
#include <vital/exceptions/klv.h>

namespace kwiver {
namespace vital {

// ------------------------------------------------------------------
void convert_0104_metadata( klv_uds_vector_t const& uds, video_metadata& metadata )
{
  static kwiver::vital::logger_handle_t logger( kwiver::vital::get_logger( "vital.convert_metadata" ) );

  //
  // Data items that are used to collect multi-value metadataa items
  // such as lat-lon points and image corner points.
  //
  geo_lat_lon sensor_location;
  geo_lat_lon frame_center;
  geo_lat_lon corner_pt1; // really offsets
  geo_lat_lon corner_pt2;
  geo_lat_lon corner_pt3;
  geo_lat_lon corner_pt4;

  for ( auto itr = uds.begin(); itr != uds.end(); ++itr )
  {
    klv_0104::tag tag;
    kwiver::vital::any data;

    try
    {
      tag = klv_0104::instance()->get_tag( itr->first );
      if ( tag == klv_0104::UNKNOWN )
      {
        LOG_WARN( logger, "Unknown key: " << itr->first << "Length: " << itr->second.size() << "bytes" );
        continue;
      }

      data = klv_0104::instance()->get_value( tag, &itr->second[0], itr->second.size() );
    }
    catch ( kwiver::vital::klv_exception const& e )
    {
      LOG_WARN( logger, "Exception caught parsing 0104 klv: " << e.what() );
      continue;
    }

    //
    // Data items that are used to collect multi-value metadataa items
    // such as lat-lon points and image corner points.
    //

    switch (tag)
    {
// Refine simple case to a define
#define CASE(N)                                                 \
case klv_0104::N:                                               \
  metadata.add( NEW_METADATA_ITEM( VITAL_META_ ## N, data ) );  \
  break

#define CASE2(KN,MN)                                                    \
      case klv_0104::KN:                                                \
    metadata.add( NEW_METADATA_ITEM( VITAL_META_ ## MN, data ) );       \
    break

      CASE( UNIX_TIMESTAMP );
      CASE( MISSION_ID );
      CASE( PLATFORM_TAIL_NUMBER );
      CASE( PLATFORM_HEADING_ANGLE );
      CASE( PLATFORM_PITCH_ANGLE );
      CASE( PLATFORM_ROLL_ANGLE );
      CASE( PLATFORM_TRUE_AIRSPEED );
      CASE( PLATFORM_INDICATED_AIRSPEED );
      CASE( PLATFORM_DESIGNATION );
      CASE( IMAGE_SOURCE_SENSOR );
      CASE( IMAGE_COORDINATE_SYSTEM );
      CASE( SENSOR_ALTITUDE );
      CASE( SENSOR_HORIZONTAL_FOV );
      CASE( SENSOR_VERTICAL_FOV );
      CASE( SENSOR_ROLL_ANGLE );
      CASE2( SENSOR_RELATIVE_ROLL_ANGLE, SENSOR_REL_ROLL_ANGLE );
      CASE( SLANT_RANGE );
      CASE( TARGET_WIDTH );
      CASE( WIND_DIRECTION );
      CASE( WIND_SPEED );
      CASE( PLATFORM_CALL_SIGN );
      CASE2( FOV_NAME, SENSOR_FOV_NAME );
      // CASE2( CLASSIFICATION, SECURITY_LOCAL_MD_SET );

      CASE( SENSOR_TYPE );


#undef CASE
#undef CASE2

    case klv_0104::SENSOR_LATITUDE:
      sensor_location.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::SENSOR_LONGITUDE:
      sensor_location.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::FRAME_CENTER_LATITUDE:
      frame_center.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::FRAME_CENTER_LONGITUDE:
      frame_center.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::UPPER_LEFT_CORNER_LAT:
      corner_pt1.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::UPPER_LEFT_CORNER_LON:
      corner_pt1.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::UPPER_RIGHT_CORNER_LAT:
      corner_pt2.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::UPPER_RIGHT_CORNER_LON:
      corner_pt2.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::LOWER_RIGHT_CORNER_LAT:
      corner_pt3.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::LOWER_RIGHT_CORNER_LON:
      corner_pt3.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::LOWER_LEFT_CORNER_LAT:
      corner_pt4.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case klv_0104::LOWER_LEFT_CORNER_LON:
      corner_pt4.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    default:
      LOG_WARN( logger, "Unknown key: " << itr->first << "Length: " << itr->second.size() << "bytes" );
      break;
    } // end switch

  } // end for

    //
  // Process composite metadata
  //
  if ( ! sensor_location.is_empty() )
  {
    if ( ! sensor_location.is_valid() )
    {
      LOG_WARN( logger, "Sensor location lat/lon is not valid coordinate: " << sensor_location );
    }
    else
    {
      metadata.add( NEW_METADATA_ITEM( VITAL_META_SENSOR_LOCATION, sensor_location ) );
    }
  }

  if ( ! frame_center.is_empty() )
  {
    if ( ! frame_center.is_valid() )
    {
      LOG_WARN( logger, "Frame Center lat/lon is not valid coordinate: " << frame_center );
    }
    else
    {
      metadata.add( NEW_METADATA_ITEM( VITAL_META_FRAME_CENTER, frame_center ) );
    }
  }

  //
  // If none of the points are set, then that is o.k.
  //
  if ( ! corner_pt1.is_empty()
       && ! corner_pt2.is_empty()
       && ! corner_pt3.is_empty()
       && ! corner_pt4.is_empty() )
  {
    // If any one of the points are invalid, then decode which one
    if ( ! corner_pt1.is_valid()
         || ! corner_pt2.is_valid()
         || ! corner_pt3.is_valid()
         || ! corner_pt4.is_valid() )
    {
      // Decode which one(s) are not valie
      if ( ! corner_pt1.is_valid() )
      {
        LOG_WARN( logger, "Corner point 1 lat/lon is not valid coordinate: " << corner_pt1 );
      }

      if ( ! corner_pt2.is_valid() )
      {
        LOG_WARN( logger, "Corner point 2 lat/lon is not valid coordinate: " << corner_pt1 );
      }

      if ( ! corner_pt3.is_valid() )
      {
        LOG_WARN( logger, "Corner point 3 lat/lon is not valid coordinate: " << corner_pt1 );
      }

      if ( ! corner_pt4.is_valid() )
      {
        LOG_WARN( logger, "Corner point 4 lat/lon is not valid coordinate: " << corner_pt1 );
      }
    }
    else
    {
      // If all points are set and valid, then build corner point structure
      video_metadata::geo_corner_points corners;
      corners.p1 = corner_pt1;
      corners.p2 = corner_pt2;
      corners.p3 = corner_pt3;
      corners.p4 = corner_pt4;

      metadata.add( NEW_METADATA_ITEM( VITAL_META_CORNER_POINTS, corners ) );
    }
  }
}

} } // end namespace
