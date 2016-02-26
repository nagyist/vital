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

#ifndef KWIVER_VITAL_GEO_CORNER_POINTS_H
#define KWIVER_VITAL_GEO_CORNER_POINTS_H

#include <vital/vital_export.h>
#include <vital/types/geo_lat_lon.h>

#include <ostream>

namespace kwiver {
namespace vital {

// ----------------------------------------------------------------
/// Four geo lat/lon points defining an image.
/**
 * This class represents the geographic bounds of an image. The corner
 * points are usually stored upper-left, upper-right, lower-right,
 * lower-left.
 */

class VITAL_EXPORT geo_corner_points
{
public:
  geo_lat_lon p1;
  geo_lat_lon p2;
  geo_lat_lon p3;
  geo_lat_lon p4;
};

VITAL_EXPORT std::ostream& operator<<( std::ostream& str, kwiver::vital::geo_corner_points const& obj );

} } // end namespace

#endif /* KWIVER_VITAL_GEO_CORNER_POINTS_H */
