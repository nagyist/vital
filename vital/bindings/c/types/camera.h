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
 * \brief C Interface to \p vital::camera class
 */

#ifndef VITAL_C_CAMERA_H_
#define VITAL_C_CAMERA_H_



#ifdef __cplusplus
extern "C"
{
#endif

#include <vital/bindings/c/common.h>
#include <vital/bindings/c/error_handle.h>
#include <vital/bindings/c/types/camera_intrinsics.h>
#include <vital/bindings/c/types/covariance.h>
#include <vital/bindings/c/types/eigen.h>
#include <vital/bindings/c/types/rotation.h>
#include <vital/bindings/c/vital_c_export.h>


/// Opaque structure to a vital::camera class
typedef struct vital_camera_s vital_camera_t;


/// Destroy a vital_camera_t instance
/**
 * The given might not refer to a valid camera instance, causing the error
 * handle to be populated (code -1).
 */
VITAL_C_EXPORT
void vital_camera_destroy( vital_camera_t *cam,
                           vital_error_handle_t *eh );


/// Create a new simple camera
/**
 * \return New reference to the created instance.
 */
VITAL_C_EXPORT
vital_camera_t*
vital_camera_new( vital_eigen_matrix3x1d_t const *center,
                  vital_rotation_d_t const *rotation,
                  vital_camera_intrinsics_t const *intrinsics,
                  vital_error_handle_t *eh );


/// Create a new simple camera instance with default parameters
/**
 * \return New reference to the created instance.
 */
VITAL_C_EXPORT
vital_camera_t*
vital_camera_new_default( vital_error_handle_t *eh );


/// Create a new simple camera from a string
/**
 * String input is expected to be of the format that would be produced by the
 * `vital_camera_to_string` function.
 *
 * \return New reference to the created instance.
 */
VITAL_C_EXPORT
vital_camera_t*
vital_camera_new_from_string( char const *s, vital_error_handle_t *eh );


/// Clone the given camera instance, returning a new camera instance
VITAL_C_EXPORT
vital_camera_t*
vital_camera_clone( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Get the 3D center point of the camera as a new 3x1 matrix (column-vector)
VITAL_C_EXPORT
vital_eigen_matrix3x1d_t*
vital_camera_center( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Get the 3D translation vector of the camera as a new 3x1 matrix (column-vector)
VITAL_C_EXPORT
vital_eigen_matrix3x1d_t*
vital_camera_translation( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Get the covariance of the camera center as a new vital covariance instance
VITAL_C_EXPORT
vital_covariance_3d_t*
vital_camera_center_covar( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Get rotation of the camera as a new vital rotation instance
VITAL_C_EXPORT
vital_rotation_d_t*
vital_camera_rotation( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Get new reference to the shared intrinsics instance of the camera
/**
 * Camera intrinsics are reference counted, so the returned reference should be
 * destroyed using `vital_camera_intrinsics_destroy` despite not being a copy.
 *
 * \param[in] cam
 * \returns New reference to the camera's intrinsics instance.
 */
VITAL_C_EXPORT
vital_camera_intrinsics_t*
vital_camera_intrinsics( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Convert camera to a 3x4 homogeneous projection matrix instance
/**
 * \note This matrix representation does not account for lens distortion
 *  models that may be used in the camera intrinsics.
 *
 * \returns New Eigen 3x4 matrix instance
 */
VITAL_C_EXPORT
vital_eigen_matrix3x4d_t*
vital_camera_as_matrix( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Project a 3D point into a (new) 2D image point via the given camera
VITAL_C_EXPORT
vital_eigen_matrix2x1d_t*
vital_camera_project( vital_camera_t const *cam,
                      vital_eigen_matrix3x1d_t const *pt,
                      vital_error_handle_t *eh );


/// Compute the distance of the 3D point to the image plane
/**
 * Points with negative depth are behind the camera
 */
VITAL_C_EXPORT
double
vital_camera_depth( vital_camera_t const *cam,
                    vital_eigen_matrix3x1d_t const *pt,
                    vital_error_handle_t *eh );


/// Convert the camera into a new string representation
/**
 * \param cam Camera instance
 * \param eh Vital error handle instance
 * \returns New character string
 */
VITAL_C_EXPORT
char*
vital_camera_to_string( vital_camera_t const *cam, vital_error_handle_t *eh );


/// Read in a KRTD file, producing a new vital::camera object
VITAL_C_EXPORT
vital_camera_t*
vital_camera_read_krtd_file( char const *filepath,
                             vital_error_handle_t *eh );


/// Output the given vital_camera_t object to the specified file path
VITAL_C_EXPORT
void
vital_camera_write_krtd_file( vital_camera_t const *cam,
                              char const *filepath,
                              vital_error_handle_t *eh );


#ifdef __cplusplus
}
#endif

#endif // VITAL_C_CAMERA_H_
