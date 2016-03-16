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
 * \brief video_input algorithm definition instantiation
 */

#include <vital/algo/video_input.h>
#include <vital/algo/algorithm.txx>

#include <map>
#include <string>

namespace kwiver {
namespace vital {
namespace algo {

// ==================================================================
// -- video traits implementation --
const video_input_traits::trait_name_t video_input_traits::HAS_EOV( "has-eov" );
const video_input_traits::trait_name_t video_input_traits::HAS_FRAME_NUMBERS( "has-frame-numbers" );
const video_input_traits::trait_name_t video_input_traits::HAS_FRAME_TIME( "has-frame-time" );
const video_input_traits::trait_name_t video_input_traits::HAS_METADATA( "has-metadata" );
const video_input_traits::trait_name_t video_input_traits::HAS_TIMEOUT( "has-timeout" );

// ------------------------------------------------------------------
class video_input_traits::priv
{
public:

  std::map< std::string, bool > m_traits;

};


// ------------------------------------------------------------------
video_input_traits
::video_input_traits()
  : d( new video_input_traits::priv )
{
}


video_input_traits
::video_input_traits( video_input_traits const& other )
  : d( new video_input_traits::priv(*other.d) ) // copy private implementation
{
}


video_input_traits
::~video_input_traits()
{
}


// ------------------------------------------------------------------
video_input_traits&
video_input_traits
::operator=( video_input_traits const& other )
{
  if ( this != &other)
  {
    this->d.reset( new video_input_traits::priv( *other.d ) ); // copy private implementation
  }

  return *this;
}


// ------------------------------------------------------------------
bool
video_input_traits
::has_trait( trait_name_t const& name ) const
{
  return ( d->m_traits.count( name ) > 0 );
}


// ------------------------------------------------------------------
std::vector< video_input_traits::trait_name_t >
video_input_traits
:: trait_list() const
{
  std::vector< video_input_traits::trait_name_t > list;

  std::map< std::string, bool >::const_iterator ix;
  for (ix = d->m_traits.begin(); ix != d->m_traits.end(); ++ix )
  {
    list.push_back( ix->first );
  }

  return list;
}


// ------------------------------------------------------------------
bool
video_input_traits
::trait( trait_name_t const& name ) const
{
  if ( ! has_trait( name ) )
  {
    // TODO: throw something
    return false;
  }

  return d->m_traits[name];
}


// ------------------------------------------------------------------
void
video_input_traits
::set_trait( trait_name_t const& name, bool val )
{
  d->m_traits[name] = val;
}


// ==================================================================
// -- video_input methods --
video_input
::video_input()
{
  attach_logger( "video_input" );
}


video_input
::~video_input()
{
}


// ------------------------------------------------------------------
video_input_traits const&
video_input
::get_implementation_traits() const
{
  return m_traits;
}


// ------------------------------------------------------------------
void
video_input
::set_trait( video_input_traits::trait_name_t const& name, bool val )
{
  m_traits.set_trait( name, val );
}


} } } // end namespace

/// \cond DoxygenSuppress
INSTANTIATE_ALGORITHM_DEF(kwiver::vital::algo::video_input);
/// \endcond
