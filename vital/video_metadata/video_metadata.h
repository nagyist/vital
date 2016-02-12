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

#include <vital/types/geo_lat_lon.h>

#include <vital/video_metadata/video_metadata_tags.h>

#include <map>
#include <string>
#include <typeinfo>
#include <memory>


namespace kwiver {
namespace vital {

  //
  // Canonical metadata tags
  //
  enum vital_metadata_tag {

#define ENUM_ITEM( TAG, NAME, T) VITAL_META_##TAG,

    // Generate enum items
    KWIVER_VITAL_METADATA_TAGS( ENUM_ITEM )

#undef ENUM_ITEM

    // User tags can be generated for a specific application and
    // should start with a value not less than the following.
    VITAL_META_FIRST_USER_TAG
  };


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
class metadata_item
{
public:
  virtual ~metadata_item() { }

  /// Get name of metadata item.
  /**
   * This method returns the descriptive name for this metadata item.
   *
   * @return Descriptive name of this metadata entry.
   */
  std::string const& name() const { return this->m_name; }

  /// Get vital metadata tag.
  /**
   *
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
   * This method returns the actual data for this metadata item as a
   * "any" object.
   *
   * @return Data for metadata item.
   */
  kwiver::vital::any data() const { return this->m_data; }

  /// Get metadat value as double.
  /**
   * This method returns the metadata item value as a double or throws
   * an exception if data is not a double.
   *
   * @return Data for metadata item as double.
   * @throws bad_any_cast if data type is not really a double.
   */
  double as_double() const { return kwiver::vital::any_cast< double >( this->m_data ); }

  /// Get metadat value as string.
  /**
   * This method returns the metadata item value as a double or throws
   * an exception if data is not a string.
   *
   * @return Data for metadata item as string.
   * @throws bad_any_cast if data type is not really a string.
   */
  std::string as_string() const { return kwiver::vital::any_cast< std::string  >( this->m_data ); }


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
 * data. User specific data can also be added by manually managing
 * enum values greater than VITAL_META_FIRST_USER_TAG.
 */
class video_metadata
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
   for ( auto ix = metadata_collection->begin(); ix != eix; ix++)
   {
     // process metada items
   }
   \endcode
   * @return Ending iterator for collection
   */
  const_iterator_t end() const;


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

void print_metadata( video_metadata& metadata );

} } // end namespace

#endif /* KWIVER_VITAL_VIDEO_METADATA_H */
