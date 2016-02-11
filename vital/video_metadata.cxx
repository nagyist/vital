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
//                        tag                      string name                        type
//                        ---                      -----------                        ----
DEFINE_VITAL_META_TRAIT( UNIX_TIMESTAMP,          "Unix Time Stamp",                 uint64_t);
DEFINE_VITAL_META_TRAIT( MISSION_ID,              "Mission ID",                      std::string);
DEFINE_VITAL_META_TRAIT( PLATFORM_TAIL_NUMBER,    "Platform Tail Number",            std::string);
DEFINE_VITAL_META_TRAIT( PLATFORM_HEADING_ANGLE,  "Platform Heading Angle",          double);
DEFINE_VITAL_META_TRAIT( PLATFORM_PITCH_ANGLE,    "Platform Pitch Angle",            double);
DEFINE_VITAL_META_TRAIT( PLATFORM_ROLL_ANGLE,     "Platform Roll Angle",             double);
DEFINE_VITAL_META_TRAIT( PLATFORM_TRUE_AIRSPEED,  "Platform True Airspeed",          double);
DEFINE_VITAL_META_TRAIT( PLATFORM_IND_AIRSPEED,   "Platform Indicated Airspeed",     double);
DEFINE_VITAL_META_TRAIT( PLATFORM_DESIGNATION,    "Platform Designation",            std::string);
DEFINE_VITAL_META_TRAIT( IMAGE_SOURCE_SENSOR,     "Image Source Sensor",             std::string);
DEFINE_VITAL_META_TRAIT( IMAGE_COORDINATE_SYSTEM, "Image Coordinate System",         std::string);
DEFINE_VITAL_META_TRAIT( SENSOR_LOCATION,         "Sensor Location Lat/Lon",         kwiver::vital::geo_point);
DEFINE_VITAL_META_TRAIT( SENSOR_TRUE_ALTITUDE,    "Sensor True Altitude",            double);
DEFINE_VITAL_META_TRAIT( SENSOR_HORIZONTAL_FOV,   "Sensor Horizontal Field of View", double);
DEFINE_VITAL_META_TRAIT( SENSOR_VERTICAL_FOV,     "Sensor Vertical Field of View",   double);
DEFINE_VITAL_META_TRAIT( SENSOR_REL_AZ_ANGLE,     "Sensor Relative Azimuth Angle",   double);
DEFINE_VITAL_META_TRAIT( SENSOR_REL_EL_ANGLE,     "Sensor Relative Elevation Angle", double);
DEFINE_VITAL_META_TRAIT( SENSOR_REL_ROLL_ANGLE,   "Sensor Relative Roll Angle",      double);
DEFINE_VITAL_META_TRAIT( SLANT_RANGE,             "Slant Range",                     double);
DEFINE_VITAL_META_TRAIT( TARGET_WIDTH,            "Target Width",                    double);
DEFINE_VITAL_META_TRAIT( FRAME_CENTER,            "Frame Center Lat/Lon",            kwiver::vital::geo_point);
DEFINE_VITAL_META_TRAIT( FRAME_CENTER_ELEV,       "Frame Center Elevation",          double);
DEFINE_VITAL_META_TRAIT( CORNER_POINTS,           "Corner points in lat/lon",        kwiver::vital::geo_box);
DEFINE_VITAL_META_TRAIT( ICING_DETECTED,          "Icing Detected",                  bool);
DEFINE_VITAL_META_TRAIT( WIND_DIRECTION,          "Wind Direction",                  double);
DEFINE_VITAL_META_TRAIT( WIND_SPEED,              "Wind Speed",                      double);
DEFINE_VITAL_META_TRAIT( STATIC_PRESSURE,         "Static Pressure",                 double);
DEFINE_VITAL_META_TRAIT( DENSITY_ALTITUDE,        "Density Altitude",                double);
DEFINE_VITAL_META_TRAIT( OUTSIDE_AIR_TEMPERATURE, "Outside Air Temperature",         double);
DEFINE_VITAL_META_TRAIT( TARGET_LOCATION,         "Target Location Lat/Lon",         kwiver::vital::geo_point);
DEFINE_VITAL_META_TRAIT( TARGET_LOCATION_ELEV,    "Target Location Elevation",       double);
DEFINE_VITAL_META_TRAIT( TARGET_TRK_GATE_WIDTH,   "Target Track Gate Width",         double);
DEFINE_VITAL_META_TRAIT( TARGET_TRK_GATE_HEIGHT,  "Target Track Gate Height",        double);
DEFINE_VITAL_META_TRAIT( TARGET_ERROR_EST_CE90,   "Target Error Estimate - CE90",    double);
DEFINE_VITAL_META_TRAIT( TARGET_ERROR_EST_LE90,   "Target Error Estimate - LE90",    double);
DEFINE_VITAL_META_TRAIT( DIFFERENTIAL_PRESSURE,   "Differential Pressure",           double);
DEFINE_VITAL_META_TRAIT( PLATFORM_ANG_OF_ATTACK,  "Platform Angle of Attack",        double);
DEFINE_VITAL_META_TRAIT( PLATFORM_VERTICAL_SPEED, "Platform Vertical Speed",         double);
DEFINE_VITAL_META_TRAIT( PLATFORM_SIDESLIP_ANGLE, "Platform Sideslip Angle",         double);
DEFINE_VITAL_META_TRAIT( AIRFIELD_BAROMET_PRESS,  "Airfield Barometric Pressure",    double);
DEFINE_VITAL_META_TRAIT( AIRFIELD_ELEVATION,      "Airfield Elevation",              double);
DEFINE_VITAL_META_TRAIT( RELATIVE_HUMIDITY,       "Relative Humidity",               double);
DEFINE_VITAL_META_TRAIT( PLATFORM_GROUND_SPEED,   "Platform Ground Speed",           double);
DEFINE_VITAL_META_TRAIT( GROUND_RANGE,            "Ground Range",                    double);
DEFINE_VITAL_META_TRAIT( PLATFORM_FUEL_REMAINING, "Platform Fuel Remaining",         double);
DEFINE_VITAL_META_TRAIT( PLATFORM_CALL_SIGN,      "Platform Call Sign",              std::string);
DEFINE_VITAL_META_TRAIT( WEAPON_LOAD,             "Weapon Load",                     bool);
DEFINE_VITAL_META_TRAIT( WEAPON_FIRED,            "Weapon Fired",                    bool);
DEFINE_VITAL_META_TRAIT( LASER_PRF_CODE,          "Laser PRF Code",                  uint32_t);
DEFINE_VITAL_META_TRAIT( SENSOR_FOV_NAME,         "Sensor Field of View Name",       uint32_t);
DEFINE_VITAL_META_TRAIT( PLATFORM_MAGNET_HEADING, "Platform Magnetic Heading",       double);
DEFINE_VITAL_META_TRAIT( UAS_LDS_VERSION_NUMBER,  "UAS LDS Version Number",          uint8_t);

#undef DEFINE_VITAL_META_TRAIT

// usage for creating metadata items
#define METADATA_ITEM( TAG, DATA )                    \
  typed_metadata< TAG, vital_meta_trait<TAG>::type >  \
  ( vital_meta_trait<TAG>::name(), DATA )


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

    //
    // Data items that are used to collect multi-value metadataa items
    // such as lat-lon points and image corner points.
    //


    switch (tag)
    {
    case KLV_0601_SLANT_RANGE:
      metadata.add( METADATA_ITEM( VITAL_META_SLANT_RANGE, data ) );
      break;


    default:
      LOG_WARN( logger, "KLV 0601 key: " << int(itr->first) << " is not supported" );
      break;
    } // end switch


  } // end for

}


// ------------------------------------------------------------------
void convert_0104_metadata( klv_uds_vector_t const& uds, video_metadata& metadata )
{
  static kwiver::vital::logger_handle_t logger( kwiver::vital::get_logger( "vital.convert_metadata" ) );

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
    case klv_0104::SLANT_RANGE:
      metadata.add( METADATA_ITEM( VITAL_META_SLANT_RANGE, data ) );
      break;

    default:
      LOG_WARN( logger, "Unknown key: " << itr->first << "Length: " << itr->second.size() << "bytes" );
      break;
    } // end switch

  } // end for
}


#if 0
// -- using the metadata map --

auto ix = metdata_map.find( VITAL_META_SLANT_RANGE );
if (ix != metadata_map.end() )
{
  double range = ix->second.as_double();
}

#endif

} } // end namespace
