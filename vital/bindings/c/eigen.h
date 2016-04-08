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
 * \brief C Interface to Vital use of Eigen vector class use
 */

#ifndef VITAL_C_EIGEN_H_
#define VITAL_C_EIGEN_H_

#include <sys/types.h>


#ifdef __cplusplus
extern "C"
{
#endif

#include <vital/bindings/c/vital_c_export.h>
#include <vital/bindings/c/error_handle.h>


/// Opaque Pointer Types
typedef struct vital_vector_s vital_vector_t;
typedef struct vital_vector2d_s vital_vector2d_t;
// The other types could probably be created by doing all this in a macro
// Probably the same with the matrix types, maybe in the same macro
//typedef struct vital_vector2f_s vital_vector2f_t;
//typedef struct vital_vector3d_s vital_vector3d_t;
//typedef struct vital_vector3f_s vital_vector3f_t;
//typedef struct vital_vector4d_s vital_vector4d_t;
//typedef struct vital_vector4f_s vital_vector4f_t;


/// Create new vector2d instance
/**
 * Initial vector values are uninitialized.
 * \returns New vector of size 2 consisting of double values.
 */
VITAL_C_EXPORT
vital_vector2d_t* vital_vector2d_new();


/// Destroy vital_vector2d_t instance
VITAL_C_EXPORT
void vital_vector2d_destroy( vital_vector2d_t *v, vital_error_handle_t *eh );


/// Get the value at a location
/**
 * \param[in] v Vector instance to get the data of
 * \param[in] row The row of the value to access
 * \param[in] col The column of the value to access
 * \param[in,out] eh Vital C error handle structure
 * \returns The double value at the position specified
 */
VITAL_C_EXPORT
double vital_vector2d_get( vital_vector2d_t *v, int idx,
                           vital_error_handle_t *eh );


/// Set the value at a location
VITAL_C_EXPORT
void vital_vector2d_set( vital_vector2d_t *v, int idx, double value,
                         vital_error_handle_t *eh );


/// Get the pointer to the vector's data array
/**
 * \param[in] v Vector instance to get the data of
 * \param[out] rows Number of rows in the matrix
 * \param[out] cols Number of columns in the matrix
 * \param[out] data Pointer to the matrix data array.
 * \param[in,out] eh Vital C error handle structure
 */
VITAL_C_EXPORT
void vital_vector2d_data( vital_vector2d_t *v,
                          unsigned int *rows,
                          unsigned int *cols,
                          unsigned int *inner_stride,
                          unsigned int *outer_stride,
                          unsigned int *is_row_major,
                          double **data,
                          vital_error_handle_t *eh );


/// === More generic functions === ///


//#define CREATE_FUNCTIONS_FOR_TYPE( T, ROWS, COLS, SUFFIX )
//do {

/// Opaque Pointer Type
typedef struct vital_matrix_2x3d_s vital_matrix_2x3d_t;

/// Create a new Eigen type-based Matrix of the given shape
/**
 * New matrices are column major in storage and uninitialized.
 */
VITAL_C_EXPORT
vital_matrix_2x3d_t* vital_eigen_new_2x3d( unsigned int const rows,
                                      unsigned int const cols );

/// Destroy a given Eigen matrix instance 
VITAL_C_EXPORT 
void vital_eigen_destroy_2x3d( vital_matrix_2x3d_t *m,
                               vital_error_handle_t *eh );

/// Get the value at a location
/**
 * \param[in] m Matrix instance to get the data of
 * \param[in] row The row of the value to access
 * \param[in] col The column of the value to access
 * \param[in,out] eh Vital C error handle structure
 * \returns The value at the position specified
 */
VITAL_C_EXPORT
double vital_eigen_get_2x3d( vital_matrix_2x3d_t *m,
                             unsigned int row, unsigned int col,
                             vital_error_handle_t *eh );

/// Set the value at a location
/**
 * \param[in] m Matrix instance to set the values of
 * \param[in] row The row of the value to set
 * \param[in] col The column of the value to set
 * \param[in] value The value to set
 * \param[in,out] eh Vital C error handle structure
 */
VITAL_C_EXPORT
void vital_eigen_set_2x3d( vital_matrix_2x3d_t *m,
                           unsigned int row, unsigned int col,
                           double value, vital_error_handle_t *eh );

/// Get the pointer to the vector's data array
/**
 * \param[in] v Vector instance to get the data of
 * \param[out] rows Number of rows in the matrix
 * \param[out] cols Number of columns in the matrix
 * \param[out] data Pointer to the matrix data array.
 * \param[in,out] eh Vital C error handle structure
 */
VITAL_C_EXPORT
void vital_eigen_data_2x3d( vital_matrix_2x3d_t *m,
                            unsigned int *rows,
                            unsigned int *cols,
                            unsigned int *inner_stride,
                            unsigned int *outer_stride,
                            unsigned int *is_row_major,
                            double **data,
                            vital_error_handle_t *eh );

//} while(0)

//CREATE_FUNCTIONS_FOR_TYPE( double, 2, 3, d );
//CREATE_FUNCTIONS_FOR_TYPE( float,  2, 3, f );


#ifdef __cplusplus
}
#endif

#endif //VITAL_C_EIGEN_H_
