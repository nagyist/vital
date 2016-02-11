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
 * \brief This file contains the interface for vital video metadata.
 */

#ifndef KWIVER_VITAL_VIDEO_METADATA_H
#define KWIVER_VITAL_VIDEO_METADATA_H

#include <vital/any.h>
#include <vital/klv/klv_parse.h>

#include <map>
#include <string>
#include <typeinfo>

namespace kwiver {
namespace vital {

  enum vital_metadata_tag {
    VITAL_META_UNKNOWN,
    VITAL_META_UNIX_TIMESTAMP,
    VITAL_META_MISSION_ID,
    VITAL_META_PLATFORM_TAIL_NUMBER,
    VITAL_META_PLATFORM_HEADING_ANGLE,
    VITAL_META_PLATFORM_PITCH_ANGLE,
    VITAL_META_PLATFORM_ROLL_ANGLE,
    VITAL_META_PLATFORM_TRUE_AIRSPEED,
    VITAL_META_PLATFORM_IND_AIRSPEED,
    VITAL_META_PLATFORM_DESIGNATION,
    VITAL_META_IMAGE_SOURCE_SENSOR,
    VITAL_META_IMAGE_COORDINATE_SYSTEM,
    VITAL_META_SENSOR_LOCATION,
    VITAL_META_SENSOR_TRUE_ALTITUDE,
    VITAL_META_SENSOR_HORIZONTAL_FOV,
    VITAL_META_SENSOR_VERTICAL_FOV,
    VITAL_META_SENSOR_REL_AZ_ANGLE,
    VITAL_META_SENSOR_REL_EL_ANGLE,
    VITAL_META_SENSOR_REL_ROLL_ANGLE,
    VITAL_META_SLANT_RANGE,
    VITAL_META_TARGET_WIDTH,
    VITAL_META_FRAME_CENTER,
    VITAL_META_FRAME_CENTER_ELEV,
    VITAL_META_CORNER_POINTS,
    VITAL_META_ICING_DETECTED,
    VITAL_META_WIND_DIRECTION,
    VITAL_META_WIND_SPEED,
    VITAL_META_STATIC_PRESSURE,
    VITAL_META_DENSITY_ALTITUDE,
    VITAL_META_OUTSIDE_AIR_TEMPERATURE,
    VITAL_META_TARGET_LOCATION,
    VITAL_META_TARGET_LOCATION_ELEV,
    VITAL_META_TARGET_TRK_GATE_WIDTH,
    VITAL_META_TARGET_TRK_GATE_HEIGHT,
    VITAL_META_TARGET_ERROR_EST_CE90,
    VITAL_META_TARGET_ERROR_EST_LE90,
    VITAL_META_GENERIC_FLAG_DATA_01,
    VITAL_META_SECURITY_LOCAL_MD_SET,
    VITAL_META_DIFFERENTIAL_PRESSURE,
    VITAL_META_PLATFORM_ANG_OF_ATTACK,
    VITAL_META_PLATFORM_VERTICAL_SPEED,
    VITAL_META_PLATFORM_SIDESLIP_ANGLE,
    VITAL_META_AIRFIELD_BAROMET_PRESS,
    VITAL_META_AIRFIELD_ELEVATION,
    VITAL_META_RELATIVE_HUMIDITY,
    VITAL_META_PLATFORM_GROUND_SPEED,
    VITAL_META_GROUND_RANGE,
    VITAL_META_PLATFORM_FUEL_REMAINING,
    VITAL_META_PLATFORM_CALL_SIGN,
    VITAL_META_WEAPON_LOAD,
    VITAL_META_WEAPON_FIRED,
    VITAL_META_LASER_PRF_CODE,
    VITAL_META_SENSOR_FOV_NAME,
    VITAL_META_PLATFORM_MAGNET_HEADING,
    VITAL_META_UAS_LDS_VERSION_NUMBER,

    // NOTE  Add the rest of the fields here
    VITAL_META_ENUM_END };


// -----------------------------------------------------------------
/// Abstract base class for video metadata items
/**
 * This class is the abstract base class for a single metadata
 * item. This mainly provides the interface for the type specific
 * derived classes.
 */
class metadata_item
{
public:
  virtual ~metadata_item() { }

  /**
   * @brief Get name of metadata item.
   *
   *
   * @return Descriptive name of this metadata entry.
   */
  std::string const& name() const { return this->m_name; }

  /**
   * @brief Get vital metadata tag,
   *
   * This method returns the vital metadata tag enum value.
   *
   * @return
   */
  virtual vital_metadata_tag tag() const = 0;

  /**
   * @brief Get metadata data type.
   *
   *
   * @return
   */
  virtual std::type_info const& type() const = 0;

  /**
   * @brief Get actual data for metadata item.
   *
   *
   * @return
   */
  kwiver::vital::any data() const { return this->m_data; }

  // convenience methods
  double as_double() const { return kwiver::vital::any_cast< double >( this->m_data ); }
  std::string as_string() const { return kwiver::vital::any_cast< std::string  >( this->m_data ); }

  // -- MANIPULATORS --
  void set_data( kwiver::vital::any const& data ) { this->m_data = data; }


protected:
  std::string m_name;
  kwiver::vital::any m_data;

  metadata_item(std::string name, kwiver::vital::any const& data )
    : m_name( name ),
      m_data( data ) { }

}; // end class metadata_item


// -----------------------------------------------------------------
/// Class for typed metadata values.
/**
 * This class represents a typed metadata item.
 *
 *
 */
template<vital_metadata_tag TAG, typename TYPE>
class typed_metadata
  : public metadata_item
{
public:
  typed_metadata(std::string name, kwiver::vital::any const& data )
    : metadata_item( name, data )
  {
    if ( data.type() != typeid(TYPE) )
    {
      // throw exception invalid metadata tag data type
    }
  }

  virtual ~typed_metadata() { }

  virtual vital_metadata_tag tag() const { return TAG; }
  virtual std::type_info const& type() const { return typeid( TYPE ); }

}; // end class typed_metadata



// -----------------------------------------------------------------
/// Collection of video metadata.
/**
 * This class represents a set of video metadata items.
 *
 * The concept is to provide a canonical set of useful metadata
 * entries that can be derived from 0104 and 0601 types of KLV
 * data. User specific data can also be added.
 */
class video_metadata
{
public:
  video_metadata();
  ~video_metadata();


  /**
   * @brief Add metadata item to collection.
   *
   * @param item New metadata item to be copied into collection.
   */
  void add( metadata_item const& item );


  /**
   * @brief Determine if metadata collection has tag.
   *
   * This method determines if the specified tag is in this metadata collection.
   *
   * @param tag Check for the presence of this tag.
   *
   * @return \b true if tag is in metadata collection, \b false otherwise.
   */
  bool has( vital_metadata_tag tag ); // needs not-found return value

  metadata_item const& find( vital_metadata_tag tag ); // needs not-found return value

private:
  std::map< vital_metadata_tag, metadata_item > m_metadata_map;

}; // end class video_metadata


/**
 * @brief Convert raw metadata packet into vital metadata entries.
 *
 * @param[in] klv Raw metadata packet containing UDS key
 * @param[in,out] metadata Collection of metadata this updated.
 */
void convert_metadata( klv_data const& klv, video_metadata& metadata );


void convert_0601_metadata( klv_lds_vector_t const& lds, video_metadata& metadata );
  void convert_0104_metadata( klv_uds_vector_t const& uds, video_metadata& metadata );

} } // end namespace

#endif /* KWIVER_VITAL_VIDEO_METADATA_H */
