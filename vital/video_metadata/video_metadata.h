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

#include <vital/vital_export.h>

#include <vital/any.h>

#include <vital/types/timestamp.h>
#include <vital/types/geo_lat_lon.h>

#include <vital/video_metadata/video_metadata_tags.h>

#include <map>
#include <string>
#include <typeinfo>
#include <memory>
#include <ostream>
#include <sstream>
#include <type_traits>

namespace kwiver {
namespace vital {


// -----------------------------------------------------------------
/// Abstract base class for video metadata items
/**
 * This class is the abstract base class for a single metadata
 * item. This mainly provides the interface for the type specific
 * derived classes.
 *
 * All metadata items need a common base class so they can be managed
 * in a collection.
 */
class VITAL_EXPORT metadata_item
{
public:
  virtual ~metadata_item();


  /// Get name of metadata item.
  /**
   * This method returns the descriptive name for this metadata item.
   *
   * @return Descriptive name of this metadata entry.
   */
  std::string const& name() const;


  /// Get vital metadata tag.
  /**
   * This method returns the vital metadata tag enum value.
   *
   * @return Metadata tag value.
   */
  virtual vital_metadata_tag tag() const = 0;


  /// Get metadata data type.
  /**
   * This method returns the type-info for this metadata item.
   *
   * @return Ty[e info for metadata tag.
   */
  virtual std::type_info const& type() const = 0;


  /// Get actual data for metadata item.
  /**
   * This method returns the actual raw data for this metadata item as
   * a "any" object.
   *
   * @return Data for metadata item.
   */
  kwiver::vital::any data() const;


  /// Get metadata value as double.
  /**
   * This method returns the metadata item value as a double or throws
   * an exception if data is not a double.
   *
   * @return Data for metadata item as double.
   * @throws bad_any_cast if data type is not really a double.
   */
  double as_double() const;
  bool has_double() const;


  /// Get metadata value as uint64.
  /**
   * This method returns the metadata item value as a uint64 or throws
   * an exception if data is not a uint64.
   *
   * @return Data for metadata item as uint64.
   * @throws bad_any_cast if data type is not really a uint64.
   */
  uint64_t as_uint64() const;
  bool has_uint64() const;


  /// Get metadata value as string.
  /**
   * This method returns the metadata item value as a string.  If the
   * data is actually a string (as indicated by has_string() method)
   * the native value is returned. If the data is of another type, it
   * is converted to a string.
   *
   * @return Data for metadata item as string.
   */
  virtual std::string as_string() const = 0;
  bool has_string() const;


protected:
  std::string m_name;
  kwiver::vital::any m_data;

  metadata_item(std::string name, kwiver::vital::any const& data );

}; // end class metadata_item


// -----------------------------------------------------------------
/// Class for typed metadata values.
/**
 * This class represents a typed metadata item.
 *
 * tparam TAG Metadata tag value
 * tparam TYPE Metadata value representation type
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
  virtual std::string as_string() const
  {
    if ( this->has_string() )
    {
      return kwiver::vital::any_cast< std::string  > ( m_data );
    }

    // Else convert to a string
    TYPE var = kwiver::vital::any_cast< TYPE > ( m_data );
    std::stringstream ss;

    ss << var;
    return ss.str();
  }

}; // end class typed_metadata


// -----------------------------------------------------------------
/// Collection of video metadata.
/**
 * This class represents a set of video metadata items.
 *
 * The concept is to provide a canonical set of useful metadata
 * entries that can be derived from 0104 and 0601 types of KLV
 * data. User specific data can also be added by manually managing
 * enum values greater than VITAL_META_FIRST_USER_TAG.
 */
class VITAL_EXPORT video_metadata
{
public:
  typedef std::map< vital_metadata_tag, std::unique_ptr< metadata_item > > metadata_map_t;
  typedef metadata_map_t::const_iterator const_iterator_t;

  video_metadata();
  ~video_metadata();

  /// Add metadata item to collection.
  /**
   *This method adds a metadata item to the collection. The collection
   *takes ownership of the item and managed the memory.
   *
   * @param item New metadata item to be copied into collection.
   */
  void add( metadata_item* item );

  /// Determine if metadata collection has tag.
  /**
   * This method determines if the specified tag is in this metadata
   * collection.
   *
   * @param tag Check for the presence of this tag.
   *
   * @return \b true if tag is in metadata collection, \b false otherwise.
   */
  bool has( vital_metadata_tag tag ); // needs not-found return value

  /// Find metadata entry for specified tag.
  /**
   * This method looks for the metadata entrty corresponding to the
   * supplied tag. If the tag is not present in the collection, the
   * results are undefined.
   *
   * @param tag Look for this tag in collection of metadata.
   *
   * @return metadata item object for tag.
   */
  metadata_item const& find( vital_metadata_tag tag ); // needs not-found return value

  /// Get starting iterator for collection of metadata items.
  /**
   * This method returns the const iterator to the first element in
   * the collection of metadata items.
   *
   * Typical usage
   \code
   auto ix = metadata_collection->begin();
   vital_metadata_tag tag = ix->first;
   std::string name = ix->second->name();
   kwiver::vital::any data = ix->second->data();
   \endcode
   *
   * @return Iterator pointing to the first element in the collection.
   */
  const_iterator_t begin() const;

  /// Get ending iterator for collection of video metadata.
  /**
   * This method returns the ending iterator for the collection of
   * video metadata items.
   *
   * Typical usage:
   \code
   auto eix = metadata_collection.end();
   for ( auto ix = metadata_collection.begin(); ix != eix; ix++)
   {
     // process metada items
   }
   \endcode
   * @return Ending iterator for collection
   */
  const_iterator_t end() const;

  /// Set timestamp for this metadata set.
  /**
   * This method sets that time stamp for this metadata
   * collection. This time stamp can be used to relate this metada
   * back to the video image stream.
   *
   * @param ts Time stamp to add to this collection.
   */
  void set_timestamp( kwiver::vital::timestamp const& ts );

  /// Return timestamp associated with these metadata.
  /**
   * This method returns the timestamp associated with this collection
   * of video metadata. The value may not be meaningful if it has not
   * been set by set_timestamp().
   *
   * @return Timestamp value.
   */
  kwiver::vital::timestamp const& timestamp() const;

  /// Get type representation for vital metadata tag. //+ move to convert_metadata
  /**
   * This method returns the type id string for the specified vital
   * metadata tag.
   *
   * @param tag Code for metadata tag.
   *
   * @return Type info for this tag
   */
  static std::type_info const& typeid_for_tag( vital_metadata_tag tag );


  // the corner points are a nested structure so it is in a nested name space.
  struct geo_corner_points
  {
    geo_lat_lon p1;
    geo_lat_lon p2;
    geo_lat_lon p3;
    geo_lat_lon p4;
  };


private:
  metadata_map_t m_metadata_map;
  kwiver::vital::timestamp m_timestamp;

}; // end class video_metadata


VITAL_EXPORT std::ostream& print_metadata( std::ostream& str, video_metadata& metadata );
VITAL_EXPORT std::ostream& operator<<( std::ostream& str, video_metadata::geo_corner_points const& obj );

} } // end namespace

#endif /* KWIVER_VITAL_VIDEO_METADATA_H */
