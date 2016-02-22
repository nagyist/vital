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

/// \file
///
/// This program reads a video and extracts all the KLV metadata.

#include <iostream>

#include <vidl/vidl_ffmpeg_istream.h>

#include <vital/klv/klv_data.h>
#include <vital/klv/klv_parse.h>
#include <vital/video_metadata/video_metadata.h>
#include <vital/video_metadata/convert_metadata.h>


// ----------------------------------------------------------------
/** Main entry.
 *
 *
 */
int main( int argc, char** argv )
{

  if( argc == 1 )
  {
    std::cout << "Missing file name.\n"
      << "Usage: " << argv[0] << " video-file-name\n" << std::endl;

      return EXIT_FAILURE;
  }

  char* video_file = argv[1];

  vidl_ffmpeg_istream istr;
  if ( ! istr.open( video_file ) )
  {
    std::cerr << "Couldn't open " << video_file << '\n';
    return EXIT_FAILURE;
  }

  if( !istr.has_metadata() )
  {
    std::cerr << "No metadata stream found in " << video_file << '\n';
    return EXIT_FAILURE;
  }

  std::deque<uint8_t> md_buffer;
  unsigned count = 0;
  kwiver::vital::convert_metadata converter;


  while ( istr.advance() )
  {
    std::cout << "========== Read frame " << istr.frame_number()
              << " (index " << count << ") ==========" <<std::endl;

    std::deque<uint8_t> curr_md = istr.current_metadata();

    // Add new metadata to the end of current metadata stream
    md_buffer.insert(md_buffer.end(), curr_md.begin(), curr_md.end());

    std::cout << "Extracted " << curr_md.size() << " metadata bytes" << std::endl;

    // loop to extract all packets from metadata
    kwiver::vital::klv_data klv_packet;
    while (klv_pop_next_packet( md_buffer, klv_packet ))
    {
      kwiver::vital::print_klv( std::cout, klv_packet );

      kwiver::vital::video_metadata metadata;

      converter.convert( klv_packet, metadata );
      print_metadata( std::cout, metadata );
    }
    ++count;

    if (md_buffer.size() > 0)
    {
      std::cout << "Left " << md_buffer.size()
               << " metadata bytes unprocessed" << std::endl;
    }
  } // end while

  std::cout << "-- End of video --\n";

  return EXIT_SUCCESS;
}
