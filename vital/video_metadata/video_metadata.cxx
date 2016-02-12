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
#include "video_metadata_tags.h"

#include <vital/klv/klv_0601.h>
#include <vital/klv/klv_0104.h>
#include <vital/klv/klv_data.h>

#include <vital/exceptions/klv.h>
#include <vital/logger/logger.h>

#include <vital/util/demangle.h>

namespace kwiver {
namespace vital {

namespace {

std::string
FormatString( std::string const& val )
{
  const char hex_chars[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                               '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  const size_t len( val.size() );
  bool unprintable_found(false);
  std::string ascii;
  std::string hex;

  for (size_t i = 0; i < len; i++)
  {
    char const byte = val[i];
    if ( ! isprint( byte ) )
    {
      ascii.append( 1, '.' );
      unprintable_found = true;
    }
    else
    {
      ascii.append( 1, byte );
    }

    // format as hex
    if (i > 0)
    {
      hex += " ";
    }

    hex += hex_chars[ ( byte & 0xF0 ) >> 4 ];
    hex += hex_chars[ ( byte & 0x0F ) >> 0 ];

  } // end for

  if (unprintable_found)
  {
    ascii += " (" + hex + ")";
  }

  return ascii;
}

} // end namespace


// ----------------------------------------------------------------
/*
 * This class is returned when find can not locate the requested tag.
 *
 */
  class unknown_metadata_item
    : public metadata_item
  {
  public:
    // -- CONSTRUCTORS --
    unknown_metadata_item()
      : metadata_item( "Requested metadata item is not in collection", 0 )
    { }

    virtual ~unknown_metadata_item() {}
    virtual vital_metadata_tag tag() const { return static_cast< vital_metadata_tag >(0); }
    virtual std::type_info const& type() const { return typeid( void ); }
    virtual std::string to_string() const { return "--Unknown metadata item--"; }

  }; // end class unknown_metadata_item

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


// ------------------------------------------------------------------
metadata_item const&
video_metadata
::find( vital_metadata_tag tag )
{
  static unknown_metadata_item unknown_item;

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


// ------------------------------------------------------------------
void
video_metadata
::set_timestamp( kwiver::vital::timestamp const& ts )
{
  this->m_timestamp = ts;
}


kwiver::vital::timestamp const&
video_metadata
::timestamp() const
{
  return this->m_timestamp;
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
std::ostream& print_metadata( std::ostream& str, video_metadata& metadata )
{
  auto eix = metadata.end();
  for ( auto ix = metadata.begin(); ix != eix; ix++)
  {
    // process metada items
   vital_metadata_tag tag = ix->first;
   std::string name = ix->second->name();
   kwiver::vital::any data = ix->second->data();

   str << "Metadata item: "
       << name
       << " (" << tag_to_string( tag )
       << " / <" << demangle( ix->second->type().name() )
       << ">): "
       << FormatString (ix->second->to_string() )
       << std::endl;
  } // end for

  return str;
}


// ------------------------------------------------------------------
std::string tag_to_string( vital_metadata_tag tag )
{
#define TAG_CASE( TAG, NAME, TYPE ) case VITAL_META_##TAG: return "VITAL_META_" #TAG;

  switch (tag)
  {

    KWIVER_VITAL_METADATA_TAGS( TAG_CASE )

  default:
    return "-- unknown tag code --";
    break;
  } // end switch

#undef TAG_CASE

}

// ------------------------------------------------------------------
std::ostream&
operator<<( std::ostream& str, video_metadata::geo_corner_points const& obj )
{
  str << "{ " << obj.p1 << obj.p2 << obj.p3 << obj.p4 << " }";

  return str;
}

} } // end namespace
