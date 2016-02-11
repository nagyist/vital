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

#include <vital/klv/klv_0601.h>
#include <vital/klv/klv_0104.h>
#include <vital/klv/klv_data.h>

#include <vital/exceptions/klv.h>
#include <vital/logger/logger.h>


namespace kwiver {
namespace vital {

// ------------------------------------------------------------------
// vital meta traits
//
// Need to describe rationale for metadata items
//
template <vital_metadata_tag tag> struct vital_meta_trait;

#define DEFINE_VITAL_META_TRAIT(TAG, NAME, T)                           \
  template <>                                                           \
  struct vital_meta_trait<VITAL_META_ ## TAG>                           \
  {                                                                     \
    static inline std::string name() { return NAME; }                   \
    typedef T type;                                                     \
    static inline vital_metadata_tag tag() { return VITAL_META_ ## TAG; } \
  }

// Define all VITAL metadata tags
//                        tag                          string name                        type
//                        ---                          -----------                        ----
DEFINE_VITAL_META_TRAIT( UNKNOWN,                     "Unknown/ Undefined entry",        void);
DEFINE_VITAL_META_TRAIT( UNIX_TIMESTAMP,              "Unix Time Stamp",                 uint64_t);
DEFINE_VITAL_META_TRAIT( MISSION_ID,                  "Mission ID",                      std::string);
DEFINE_VITAL_META_TRAIT( PLATFORM_TAIL_NUMBER,        "Platform Tail Number",            std::string);
DEFINE_VITAL_META_TRAIT( PLATFORM_HEADING_ANGLE,      "Platform Heading Angle",          double);
DEFINE_VITAL_META_TRAIT( PLATFORM_PITCH_ANGLE,        "Platform Pitch Angle",            double);
DEFINE_VITAL_META_TRAIT( PLATFORM_ROLL_ANGLE,         "Platform Roll Angle",             double);
DEFINE_VITAL_META_TRAIT( PLATFORM_TRUE_AIRSPEED,      "Platform True Airspeed",          double);
DEFINE_VITAL_META_TRAIT( PLATFORM_INDICATED_AIRSPEED, "Platform Indicated Airspeed",     double);
DEFINE_VITAL_META_TRAIT( PLATFORM_DESIGNATION,        "Platform Designation",            std::string);
DEFINE_VITAL_META_TRAIT( IMAGE_SOURCE_SENSOR,         "Image Source Sensor",             std::string);
DEFINE_VITAL_META_TRAIT( IMAGE_COORDINATE_SYSTEM,     "Image Coordinate System",         std::string);
DEFINE_VITAL_META_TRAIT( SENSOR_LOCATION,             "Sensor Location Lat/Lon",         geo_lat_lon);
DEFINE_VITAL_META_TRAIT( SENSOR_ALTITUDE,             "Sensor Altitude",                 double);
DEFINE_VITAL_META_TRAIT( SENSOR_HORIZONTAL_FOV,       "Sensor Horizontal Field of View", double);
DEFINE_VITAL_META_TRAIT( SENSOR_VERTICAL_FOV,         "Sensor Vertical Field of View",   double);
DEFINE_VITAL_META_TRAIT( SENSOR_REL_AZ_ANGLE,         "Sensor Relative Azimuth Angle",   double);
DEFINE_VITAL_META_TRAIT( SENSOR_REL_EL_ANGLE,         "Sensor Relative Elevation Angle", double);
DEFINE_VITAL_META_TRAIT( SENSOR_REL_ROLL_ANGLE,       "Sensor Relative Roll Angle",      double);
DEFINE_VITAL_META_TRAIT( SENSOR_ROLL_ANGLE,           "Sensor Roll Angle",               double);
DEFINE_VITAL_META_TRAIT( SENSOR_TYPE,                 "Sensor Type",                     std::string);
DEFINE_VITAL_META_TRAIT( SLANT_RANGE,                 "Slant Range",                     double);
DEFINE_VITAL_META_TRAIT( TARGET_WIDTH,                "Target Width",                    double);
DEFINE_VITAL_META_TRAIT( FRAME_CENTER,                "Frame Center Lat/Lon",            geo_lat_lon);
DEFINE_VITAL_META_TRAIT( FRAME_CENTER_ELEV,           "Frame Center Elevation",          double);
DEFINE_VITAL_META_TRAIT( CORNER_POINTS,               "Corner points in lat/lon",        video_metadata::geo_corner_points);
DEFINE_VITAL_META_TRAIT( ICING_DETECTED,              "Icing Detected",                  bool);
DEFINE_VITAL_META_TRAIT( WIND_DIRECTION,              "Wind Direction",                  double);
DEFINE_VITAL_META_TRAIT( WIND_SPEED,                  "Wind Speed",                      double);
DEFINE_VITAL_META_TRAIT( STATIC_PRESSURE,             "Static Pressure",                 double);
DEFINE_VITAL_META_TRAIT( DENSITY_ALTITUDE,            "Density Altitude",                double);
DEFINE_VITAL_META_TRAIT( OUTSIDE_AIR_TEMPERATURE,     "Outside Air Temperature",         double);
DEFINE_VITAL_META_TRAIT( TARGET_LOCATION,             "Target Location Lat/Lon",         geo_lat_lon);
DEFINE_VITAL_META_TRAIT( TARGET_LOCATION_ELEV,        "Target Location Elevation",       double);
DEFINE_VITAL_META_TRAIT( TARGET_TRK_GATE_WIDTH,       "Target Track Gate Width",         double);
DEFINE_VITAL_META_TRAIT( TARGET_TRK_GATE_HEIGHT,      "Target Track Gate Height",        double);
DEFINE_VITAL_META_TRAIT( TARGET_ERROR_EST_CE90,       "Target Error Estimate - CE90",    double);
DEFINE_VITAL_META_TRAIT( TARGET_ERROR_EST_LE90,       "Target Error Estimate - LE90",    double);
DEFINE_VITAL_META_TRAIT( DIFFERENTIAL_PRESSURE,       "Differential Pressure",           double);
DEFINE_VITAL_META_TRAIT( PLATFORM_ANG_OF_ATTACK,      "Platform Angle of Attack",        double);
DEFINE_VITAL_META_TRAIT( PLATFORM_VERTICAL_SPEED,     "Platform Vertical Speed",         double);
DEFINE_VITAL_META_TRAIT( PLATFORM_SIDESLIP_ANGLE,     "Platform Sideslip Angle",         double);
DEFINE_VITAL_META_TRAIT( AIRFIELD_BAROMET_PRESS,      "Airfield Barometric Pressure",    double);
DEFINE_VITAL_META_TRAIT( AIRFIELD_ELEVATION,          "Airfield Elevation",              double);
DEFINE_VITAL_META_TRAIT( RELATIVE_HUMIDITY,           "Relative Humidity",               double);
DEFINE_VITAL_META_TRAIT( PLATFORM_GROUND_SPEED,       "Platform Ground Speed",           double);
DEFINE_VITAL_META_TRAIT( GROUND_RANGE,                "Ground Range",                    double);
DEFINE_VITAL_META_TRAIT( PLATFORM_FUEL_REMAINING,     "Platform Fuel Remaining",         double);
DEFINE_VITAL_META_TRAIT( PLATFORM_CALL_SIGN,          "Platform Call Sign",              std::string);
DEFINE_VITAL_META_TRAIT( WEAPON_LOAD,                 "Weapon Load",                     bool);
DEFINE_VITAL_META_TRAIT( WEAPON_FIRED,                "Weapon Fired",                    bool);
DEFINE_VITAL_META_TRAIT( LASER_PRF_CODE,              "Laser PRF Code",                  uint32_t);
DEFINE_VITAL_META_TRAIT( SENSOR_FOV_NAME,             "Sensor Field of View Name",       uint32_t);
DEFINE_VITAL_META_TRAIT( PLATFORM_MAGNET_HEADING,     "Platform Magnetic Heading",       double);
DEFINE_VITAL_META_TRAIT( UAS_LDS_VERSION_NUMBER,      "UAS LDS Version Number",          uint8_t);
DEFINE_VITAL_META_TRAIT( ANGLE_TO_NORTH,              "Angle to North",                  double);


#undef DEFINE_VITAL_META_TRAIT

// usage for creating metadata items
#define NEW_METADATA_ITEM( TAG, DATA )                    \
  new typed_metadata< TAG, vital_meta_trait<TAG>::type >  \
  ( vital_meta_trait<TAG>::name(), DATA )

// ==================================================================
video_metadata
::video_metadata()
{ }


video_metadata
::~video_metadata()
{

}

void
video_metadata
::add( metadata_item* item )
{
  this->m_metadata_map[item->tag()] = std::unique_ptr< metadata_item >(item);
}


bool
video_metadata
::has( vital_metadata_tag tag )
{
  return m_metadata_map.find( tag ) != m_metadata_map.end();
}


metadata_item const&
video_metadata
::find( vital_metadata_tag tag )
{
  static typed_metadata< VITAL_META_UNKNOWN, void > unknown_item(
    vital_meta_trait<VITAL_META_UNKNOWN>::name(), kwiver::vital::any(0) );

  const_iterator_t it = m_metadata_map.find( tag );
  if ( it == m_metadata_map.end() )
  {
    return unknown_item;
  }

  return *(it->second);
}


video_metadata::const_iterator_t
video_metadata
::begin() const
{
  return m_metadata_map.begin();
}


video_metadata::const_iterator_t
video_metadata
::end() const
{
  return m_metadata_map.end();
}



// ==================================================================
void convert_metadata( klv_data const& klv, video_metadata& metadata )
{
  static kwiver::vital::logger_handle_t logger( kwiver::vital::get_logger( "vital.convert_metadata" ) );

  klv_uds_key uds_key( klv ); // create key from raw data

  if ( is_klv_0601_key( uds_key ) )
  {
    if ( ! klv_0601_checksum( klv ) )
    {
      throw klv_exception( "checksum error on 0601 packet");
    }

    klv_lds_vector_t lds = parse_klv_lds( klv );
    convert_0601_metadata( lds, metadata );
  }
  else if ( klv_0104::is_key( uds_key ) )
  {
    klv_uds_vector_t uds = parse_klv_uds( klv );
    convert_0104_metadata( uds,  metadata );
  }
  else
  {
    LOG_WARN( logger, "Unsupported UDS Key: "
              << uds_key << " data size is "
              << klv.value_size() );
  }

}


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
