/*ckwg +29
 * Copyright 2015-2016 by Kitware, Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
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
 * \brief Interface for video_input
 */

#ifndef VITAL_ALGO_VIDEO_INPUT_H_
#define VITAL_ALGO_VIDEO_INPUT_H_

#include <vital/vital_config.h>
#include <vital/vital_export.h>

#include <vital/algo/algorithm.h>
#include <vital/types/image_container.h>
#include <vital/types/timestamp.h>
#include <vital/video_metadata/video_metadata.h>

#include <string>
#include <vector>

namespace kwiver {
namespace vital {
namespace algo {

//+ Is video_input_traits a type and should be found under vital/types?

// ----------------------------------------------------------------
/**
 * \brief Video input trait base class / interface
 *
 * This class represents a collection of traits for the current
 * video input algorithm.
 *
 * Video traits provide a way to flexibly query the concrete
 * implementation and determine the capabilities features and
 * limitations.
 *
 * A note about some traits
 *
 * HAS_EOV - This trait is set to true if the video source can
 *     determine end of video. This is usually the case if the video
 *     is being read from a file, but may not be known if the video is
 *     coming from a streaming source.
 *
 * HAS_FRAME_NUMBERS - This trait is set to true if the video source
 *     supplies frame numbers. If the video source specifies a frame
 *     number, then that number is used when forming a time stamp. If
 *     the video does not supply a frame number, the time stamp will
 *     not have a frame number.
 *
 * HAS_FRAME_TIME - This trait is set to true if the video source
 *     supplies a frame time. If a frame time is supplied, it is made
 *     available in the time stamp for that frame. If the frame time
 *     is not supplied, then the timestamp will hot have the time set.
 *
 * HAS_KLV_METADATA - This trait is set if the video source supplies
 *     KLV style metadata. The metadata could be in 0601 or 0104 data
 *     formats.
 */
class VITAL_EXPORT video_input_traits
{
public:
  typedef std::string trait_name_t;

  // Common traits
  // -- basic traits --
  static const trait_name_t HAS_EOV;         // has end of video indication
  static const trait_name_t HAS_FRAME_NUMBERS;
  static const trait_name_t HAS_FRAME_TIME;
  static const trait_name_t HAS_METADATA;

  video_input_traits();
  video_input_traits( video_input_traits const& other );
  ~video_input_traits();


  /// Indicate if trait is supported.
  /**
   * This method reports if the specified trait is supported by the
   * concrete implementation. If the trait is supported, then the
   * value can be accessed with the trait() method.
   *
   * \param name Trait name
   *
   * \return \b true if trait is supported, \b false otherwise.
   */
  bool has_trait( trait_name_t const& name ) const;


  /// Get list of supported traits.
  /**
   * This method returns a vector of all traits supported by the
   * current algorithm implementation.
   *
   * @return Vector of supported traits.
   */
  std::vector< trait_name_t > trait_list() const;


  /// Return value of trait
  /**
   * This method returns the value of the specified trait.
   * \b false is also returned if the trait does not exist.
   *
   * @param name Trait name.
   *
   * @return Value of trait.
   */
  bool trait( trait_name_t const& name ) const;


  /// Set trait value.
  /**
   * This method creates a trait and sets it to the specified value.
   *
   * @param name Trait name
   * @param val Trait value
   */
  void set_trait( trait_name_t const& name, bool val );


private:
  /// private implementation class
  class priv;
  const std::unique_ptr<priv> d;
};


// ==================================================================
/// An abstract base class for reading videos
/**
 * This class represents an abstract interface for reading
 * videos. Once the video is opened, the frames are returned in order.
 *
 * Use cases:
 * ----------
 *
 * 1) Reading video from a directory of images.
 *
 * 2) Reading video frames from a list of file names.
 *
 * 3) Reading video from mpeg/video file (one of many formats) (e.g. FMV)
 *
 * 4) Reading video from mpeg/video file (one of many formats) with
 *    cropping (e.g. WAMI). This includes Providing geostationary
 *    images by cropping to a specific region from an image. This may
 *    result in no data if the geo region and image do not intersect.
 *
 * 5) Reading video from network stream. (RTSP) This may result in
 *    unexpected end of video conditions and network related
 *    disruptions (e.g. missing frames, connection terminating, ...)
 */
class VITAL_EXPORT video_input
  : public kwiver::vital::algorithm_def<video_input>
{
public:
  virtual ~video_input();

  /// Return the name of this algorithm
  static std::string static_type_name() { return "video_input"; }


  /**
   * \brief Open a video stream.
   *
   * This method opens the specified video stream for reading. The
   * format of the name depends on the concrete implementation. It
   * could be a file name or it could be a URI.
   *
   * \param video_name Identifier of the video stream.
   *
   * \throws exception if open failed
   */
  virtual void open( std::string video_name ) = 0;


  /**
   * \brief Close video stream.
   *
   * Close the currently opened stream and release resources.  Closing
   * a stream that is already closed does not cause a problem.
   */
  virtual void close() = 0;


  /**
   * \brief Return end of video status.
   *
   * This method returns the end-of-video status of the input
   * video. \b true is returned if the last frame has been returned.
   *
   * This method will always return \b false for video streams that have
   * no ability to detect end of video, such as network streams.
   *
   * \return \b true if at end of video, \b false otherwise.
   */
  virtual bool end_of_video() const = 0;


  /**
   * \brief Check whether state of video stream is good.
   *
   * This method checks the current state of the video stream to see
   * if it is good. The definition of \a good depends on the concrete
   * implementation.
   *
   * \return \b true if video stream is good, \b false if not good.
   */
  virtual bool good() const = 0; // like io stream API


  /**
   * \brief Get next frame from video stream.
   *
   * This method returns the next frame and timing information. The
   * timestamp returned may be missing either frame number or time or
   * both, depending on the actual implementation.
   *
   * Calling this method will make new metadata packets available
   * while deleting any old ones.
   *
   * If the video input is already an end, then calling this method
   * will return \b false.
   *
   * \param[out] frame Next frame from source
   * \param[out] ts Time of returned frame
   * \param[in] timeout Number of seconds to wait. 0 = no timeout.
   *
   * \return \b true if frame returned, \b false if timeout or end of
   * video.
   *
   * \throws video_input_timeout_exception when the timeout expires.
   * \throws video_stream_exception when there is an error in the video stream.
   */
  virtual bool next_frame( kwiver::vital::image_container_sptr& frame,
                           kwiver::vital::timestamp& ts,
                           uint32_t timeout = 0 ) = 0;


  /// Get metadata collection for current frame.
  /**
   * This method returns the metadata collection for the current
   * frame. It is best to call this after calling next_frame() to make
   * sure the metadata and video are synchronized and that no metadata
   * collections are lost.
   *
   * Metadata typically occurs less frequently than video frames, so
   * if you call next_frame() and frame_metadata() together while
   * processing a video, the same set of metadata may be returned
   * until new metadata is read from the stream.
   *
   * Also note that the metadata collection has a timestamp that can
   * be used to determine where the metadata fits in the video stream.
   *
   * In video streams without metadata (as determined by the stream
   * traits), this method may return and empty vector, indicating no
   * new metadata has been found.
   *
   * @return Pointer to metadata vector.
   */
  virtual std::vector< kwiver::vital::video_metadata_sptr > frame_metadata() = 0;


  /**
   * \brief Return traits of concrete implementation.
   *
   * This method returns the traits for the currently opened video.
   *
   * \return Reference to supported video traits.
   */
  video_input_traits const& get_implementation_traits() const;


protected:
  video_input(); // CTOR

  void set_trait( video_input_traits::trait_name_t const& name, bool val );

private:
  video_input_traits m_traits;
};

/// Shared pointer type for generic video_input definition type.
typedef std::shared_ptr<video_input> video_input_sptr;

} } } // end namespace

#endif // VITAL_ALGO_VIDEO_INPUT_H_
