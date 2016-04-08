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
 * \brief C Interface implementation to Vital use of Eigen vector and matrix
 *        classes
 */

#include "eigen.h"

#include <vital/bindings/c/helpers/c_utils.h>
#include <vital/types/matrix.h>
#include <vital/types/vector.h>


#define REINTERP_TYPE( new_type, c_ptr, var )           \
  new_type *var = reinterpret_cast<new_type*>( c_ptr ); \
  do                                                    \
  {                                                     \
    if( var == 0 )                                      \
    {                                                   \
      throw "Null pointer";                             \
    }                                                   \
  } while(0)



vital_vector2d_t* vital_vector2d_new()
{
  STANDARD_CATCH(
    "vector2d.new", 0,
    return reinterpret_cast<vital_vector2d_t*>(
      new kwiver::vital::vector_2d()
    );
  );
  return 0;
}


void vital_vector2d_destroy( vital_vector2d_t *v, vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "vector2d.desctroy", eh,
    kwiver::vital::vector_2d *v_ptr = reinterpret_cast<kwiver::vital::vector_2d *>( v );
    if( v_ptr )
    {
      delete( v_ptr );
    }
  );
}


double vital_vector2d_get( vital_vector2d_t *v, int idx,
                           vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "vector2d.get", eh,
    REINTERP_TYPE( kwiver::vital::vector_2d, v, v_ptr );
    return (*v_ptr)(idx);
  );
  return 0.0;
}


void vital_vector2d_set( vital_vector2d_t *v, int idx, double value,
                         vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "vector2d.set", eh,
    REINTERP_TYPE( kwiver::vital::vector_2d, v, v_ptr );
    (*v_ptr)(idx) = value;
  );
}


void vital_vector2d_data( vital_vector2d_t *v,
                          unsigned int *rows,
                          unsigned int *cols,
                          unsigned int *inner_stride,
                          unsigned int *outer_stride,
                          unsigned int *is_row_major,
                          double **data,
                          vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "vector2d.data", eh,
    REINTERP_TYPE( kwiver::vital::vector_2d, v, v_ptr );
    // Eigen is Column-major by default, i.e. (rows, cols) indexing

    LOG_DEBUG(m_logger, "v.rows: " << v_ptr->rows());
    *rows = v_ptr->rows();

    LOG_DEBUG(m_logger, "v.cols: " << v_ptr->cols());
    *cols = v_ptr->cols();

    LOG_DEBUG(m_logger, "v.innerSize: " << v_ptr->innerSize());
    LOG_DEBUG(m_logger, "v.innerStride: " << v_ptr->innerStride());
    *inner_stride = v_ptr->innerStride();

    LOG_DEBUG(m_logger, "v.outerSize: " << v_ptr->outerSize());
    LOG_DEBUG(m_logger, "v.outerStride: " << v_ptr->outerStride());
    *outer_stride = v_ptr->outerStride();

    LOG_DEBUG(m_logger, "v.options&RowMajorBit: " << (v_ptr->Flags&Eigen::RowMajorBit));
    *is_row_major = (unsigned int)(v_ptr->Flags & Eigen::RowMajorBit);

    *data = v_ptr->data();
  );
}


/// === More generic functions === ///
// // Eigen is Column-major by default, i.e. (rows, cols) indexing
// TODO: Check that dynamically created instances can be cast to Vital typedef types


/// Create a new Eigen type-based Matrix of the given shape
vital_matrix_2x3d_t* vital_eigen_new_2x3d( unsigned int const rows,
                                           unsigned int const cols )
{
  STANDARD_CATCH(
    "eigen_matrix.new.2x3d", 0,
    return reinterpret_cast<vital_matrix_2x3d_t*>(
      new kwiver::vital::matrix_2x3d()
    );
  );
  return 0;
}

/// Destroy a given Eigen matrix instance
void vital_eigen_destroy_2x3d( vital_matrix_2x3d_t *m,
                               vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "eigen_matrix.destroy.2x3d", eh,
    REINTERP_TYPE( kwiver::vital::matrix_2x3d, m, mp );
    if( mp )
    {
      delete( mp );
    }
  );
}

/// Get the value at a location
double vital_eigen_get_2x3d( vital_matrix_2x3d_t *m,
                             unsigned int row, unsigned int col,
                             vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "eigen_matrix.get.2x3d", eh,
    REINTERP_TYPE( kwiver::vital::matrix_2x3d, m, mp );
    return (*mp)(row, col);
  );
  return 0;
}

/// Set the value at a location
void vital_eigen_set_2x3d( vital_matrix_2x3d_t *m,
                           unsigned int row, unsigned int col,
                           double value, vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "eigen_matrix.set.2x3d", eh,
    REINTERP_TYPE( kwiver::vital::matrix_2x3d, m, mp );
    (*mp)(row, col) = value;
  );
}

/// Get the pointer to the vector's data array
void vital_eigen_data_2x3d( vital_matrix_2x3d_t *m,
                            unsigned int *rows,
                            unsigned int *cols,
                            unsigned int *inner_stride,
                            unsigned int *outer_stride,
                            unsigned int *is_row_major,
                            double **data,
                            vital_error_handle_t *eh )
{
  STANDARD_CATCH(
    "eigen_matrix.data.2x3d", eh,

    REINTERP_TYPE( kwiver::vital::matrix_2x3d, m, mp );

    LOG_DEBUG(m_logger, "m.rows: " << mp->rows());
    *rows = mp->rows();

    LOG_DEBUG(m_logger, "m.cols: " << mp->cols());
    *cols = mp->cols();

    LOG_DEBUG(m_logger, "m.innerSize: " << mp->innerSize());
    LOG_DEBUG(m_logger, "m.innerStride: " << mp->innerStride());
    *inner_stride = mp->innerStride();

    LOG_DEBUG(m_logger, "m.outerSize: " << mp->outerSize());
    LOG_DEBUG(m_logger, "m.outerStride: " << mp->outerStride());
    *outer_stride = mp->outerStride();

    LOG_DEBUG(m_logger, "m.options&RowMajorBit: " << (mp->Flags&Eigen::RowMajorBit));
    *is_row_major = (unsigned int)(mp->Flags & Eigen::RowMajorBit);

    *data = mp->data();
  );
}
