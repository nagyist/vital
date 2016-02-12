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

#include <vital/klv/klv_0601.h>
#include <vital/klv/klv_data.h>

#include <vital/logger/logger.h>

namespace kwiver {
namespace vital {

// ------------------------------------------------------------------
void convert_0601_metadata( klv_lds_vector_t const& lds, video_metadata& metadata )
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
  geo_lat_lon target_location;

  for ( auto itr = lds.begin(); itr != lds.end(); ++itr )
  {
    if ( ( itr->first <= KLV_0601_UNKNOWN ) || ( itr->first >= KLV_0601_ENUM_END ) )
    {
      LOG_WARN( logger, "KLV 0601 key: " << int(itr->first) << " is not supported" );
      continue;
    }

    // Convert a single tag
    const klv_0601_tag tag( klv_0601_get_tag( itr->first ) ); // get tag code from key

    // Extract relevant data from associated data bytes.
    kwiver::vital::any data = klv_0601_value( tag,
                                              &itr->second[0], itr->second.size() );

    switch (tag)
    {
// Refine simple case to a define
#define CASE(N)                                                         \
  case KLV_0601_ ## N:                                                  \
    metadata.add( NEW_METADATA_ITEM( VITAL_META_ ## N, data ) );        \
    break

#define CASE2(KN,MN)                                                    \
  case KLV_0601_ ## KN:                                                 \
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
      CASE2( SENSOR_TRUE_ALTITUDE, SENSOR_ALTITUDE );
      CASE( SENSOR_HORIZONTAL_FOV );
      CASE( SENSOR_VERTICAL_FOV );
      CASE( SENSOR_REL_AZ_ANGLE );
      CASE( SENSOR_REL_EL_ANGLE );
      CASE( SENSOR_REL_ROLL_ANGLE );
      CASE( SLANT_RANGE );
      CASE( TARGET_WIDTH );
      CASE( FRAME_CENTER_ELEV );
      CASE( ICING_DETECTED);
      CASE( WIND_DIRECTION );
      CASE( WIND_SPEED );
      CASE( STATIC_PRESSURE );
      CASE( DENSITY_ALTITUDE );
      CASE( OUTSIDE_AIR_TEMPERATURE );
      CASE( TARGET_LOCATION_ELEV );
      CASE( TARGET_TRK_GATE_WIDTH );
      CASE( TARGET_TRK_GATE_HEIGHT );
//      CASE( SECURITY_LOCAL_MD_SET );
      CASE( TARGET_ERROR_EST_CE90 );
      CASE( TARGET_ERROR_EST_LE90 );
      CASE( DIFFERENTIAL_PRESSURE );
      CASE( PLATFORM_ANG_OF_ATTACK );
      CASE( PLATFORM_VERTICAL_SPEED );
      CASE( PLATFORM_SIDESLIP_ANGLE );
      CASE( AIRFIELD_BAROMET_PRESS );
      CASE( AIRFIELD_ELEVATION );
      CASE( RELATIVE_HUMIDITY );
      CASE( PLATFORM_GROUND_SPEED );
      CASE( GROUND_RANGE );
      CASE( PLATFORM_FUEL_REMAINING );
      CASE( PLATFORM_CALL_SIGN );
      CASE( WEAPON_LOAD );
      CASE( WEAPON_FIRED );
      CASE( LASER_PRF_CODE );
      CASE( SENSOR_FOV_NAME );
      CASE( PLATFORM_MAGNET_HEADING );
      CASE( UAS_LDS_VERSION_NUMBER );

#undef CASE
#undef CASE2

    case KLV_0601_SENSOR_LATITUDE:
      sensor_location.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_SENSOR_LONGITUDE:
      sensor_location.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_FRAME_CENTER_LAT:
      frame_center.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_FRAME_CENTER_LONG:
      frame_center.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LAT_PT_1:
      corner_pt1.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LONG_PT_1:
      corner_pt1.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LAT_PT_2:
      corner_pt2.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LONG_PT_2:
      corner_pt2.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LAT_PT_3:
      corner_pt3.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LONG_PT_3:
      corner_pt3.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LAT_PT_4:
      corner_pt4.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_OFFSET_CORNER_LONG_PT_4:
      corner_pt4.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_TARGET_LOCATION_LAT:
      target_location.set_latitude( kwiver::vital::any_cast< double >(data) );
      break;

    case KLV_0601_TARGET_LOCATION_LONG:
      target_location.set_longitude( kwiver::vital::any_cast< double >(data) );
      break;

    default:
      LOG_WARN( logger, "KLV 0601 key: " << int(itr->first) << " is not supported" );
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

  if ( ! target_location.is_empty() )
  {
    if ( ! target_location.is_valid() )
    {
      LOG_WARN( logger, "Target location lat/lon is not valid coordinate: " << target_location );
    }
    else
    {
      metadata.add( NEW_METADATA_ITEM( VITAL_META_TARGET_LOCATION, target_location ) );
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
      // Decode which one(s) are not valid
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
      if ( ! frame_center.is_valid() )
      {
        LOG_WARN( logger, "Frame center not valid. Can not adjust frame corner offsets." );
      }
      else
      {
        // If all points are set and valid, then build corner point structure
        video_metadata::geo_corner_points corners;

        corners.p1.set_latitude( corner_pt1.latitude() + frame_center.latitude() );
        corners.p1.set_longitude( corner_pt1.longitude() + frame_center.longitude() );

        corners.p2.set_latitude( corner_pt2.latitude() + frame_center.latitude() );
        corners.p2.set_longitude( corner_pt2.longitude() + frame_center.longitude() );

        corners.p3.set_latitude( corner_pt3.latitude() + frame_center.latitude() );
        corners.p3.set_longitude( corner_pt3.longitude() + frame_center.longitude() );

        corners.p4.set_latitude( corner_pt4.latitude() + frame_center.latitude() );
        corners.p4.set_longitude( corner_pt4.longitude() + frame_center.longitude() );

        metadata.add( NEW_METADATA_ITEM( VITAL_META_CORNER_POINTS, corners ) );
      }
    }
  }
}

} } // end namespace
