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
 * \file \brief This file contains the internal interface for
 * converter functions.
 */

#ifndef KWIVER_VITAL_CONVERT_METADATA_H
#define KWIVER_VITAL_CONVERT_METADATA_H

#include "video_metadata_tags.h"

namespace kwiver {
namespace vital {

// usage for creating metadata items
#define NEW_METADATA_ITEM( TAG, DATA )                    \
  new typed_metadata< TAG, vital_meta_trait<TAG>::type >  \
  ( vital_meta_trait<TAG>::name(), DATA )

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
  };

//
// Define all metadata traits
//
  KWIVER_VITAL_METADATA_TAGS( DEFINE_VITAL_META_TRAIT )

#undef DEFINE_VITAL_META_TRAIT

} } // end namespace

#endif /* KWIVER_VITAL_CONVERT_METADATA_H */
