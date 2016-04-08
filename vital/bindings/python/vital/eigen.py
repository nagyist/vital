"""
ckwg +31
Copyright 2016 by Kitware, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 * Neither name of Kitware, Inc. nor the names of any contributors may be used
   to endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

==============================================================================

Interface to VITAL Eigen vector classes through numpy

"""
import ctypes

import numpy

from vital.util import (
    VitalErrorHandle,
    VitalObject,
)


__author__ = 'paul.tunison@kitware.com'


class VitalVector2D (VitalObject):
    """
    Wrapper for Eigen matrix use within Vital. This class coveres both
    "Vector<dim><type>", "Matrix<dim><type>" and
    """

    def __init__(self):
        super(VitalVector2D, self).__init__()

        v_new = self.VITAL_LIB['vital_vector2d_new']
        v_new.restype = self.C_TYPE_PTR
        self._inst_ptr = v_new()
        if not bool(self.c_pointer):
            raise RuntimeError("Failed to construct new vector instance")

        # numpy-wrapper cache
        self._n_cache = None

    def _destroy(self):
        v_del = self.VITAL_LIB['vital_vector2d_destroy']
        v_del.argtypes = [self.C_TYPE_PTR, VitalErrorHandle.C_TYPE_PTR]
        with VitalErrorHandle() as eh:
            v_del(self, eh)
        self._inst_ptr = self.C_TYPE_PTR()

    def __getitem__(self, idx):
        if idx not in (0, 1):
            raise IndexError(idx)
        v_get = self.VITAL_LIB['vital_vector2d_get']
        v_get.argtypes = \
            [self.C_TYPE_PTR, ctypes.c_int, VitalErrorHandle.C_TYPE_PTR]
        v_get.restype = ctypes.c_double
        with VitalErrorHandle() as eh:
            return v_get(self, ctypes.c_int(idx), eh)

    def __setitem__(self, idx, value):
        v_set = self.VITAL_LIB['vital_vector2d_set']
        v_set.argtypes = [self.C_TYPE_PTR, ctypes.c_int, ctypes.c_double,
                          VitalErrorHandle.C_TYPE_PTR]
        with VitalErrorHandle() as eh:
            v_set(self, ctypes.c_int(idx), ctypes.c_double(value), eh)

    def __repr__(self):
        return "%s(%f, %f)" % (self.__class__.__name__, self[0], self[1])

    def as_numpy(self):
        if not self._n_cache:
            v_data = self.VITAL_LIB['vital_vector2d_data']
            v_data.argtypes = [self.C_TYPE_PTR,
                               ctypes.POINTER(ctypes.c_uint),
                               ctypes.POINTER(ctypes.c_uint),
                               ctypes.POINTER(ctypes.c_uint),
                               ctypes.POINTER(ctypes.c_uint),
                               ctypes.POINTER(ctypes.c_uint),
                               ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
                               VitalErrorHandle.C_TYPE_PTR]
            rows = ctypes.c_uint()
            cols = ctypes.c_uint()
            inner_stride = ctypes.c_uint()
            outer_stride = ctypes.c_uint()
            is_row_major = ctypes.c_uint()
            data = ctypes.POINTER(ctypes.c_double)()
            with VitalErrorHandle() as eh:
                v_data(self,
                       ctypes.byref(rows), ctypes.byref(cols),
                       ctypes.byref(inner_stride), ctypes.byref(outer_stride),
                       ctypes.byref(is_row_major),
                       ctypes.byref(data), eh)

            rows = rows.value
            cols = cols.value
            inner_stride = inner_stride.value
            outer_stride = outer_stride.value
            is_row_major = bool(is_row_major.value)

            n = numpy.ctypeslib.as_array(data, (rows, cols))
            if bool(is_row_major):
                print "Reshaping to Fortran byte order"
                n = n.reshape((rows, cols), order='F')
            self._n_cache = n

        return self._n_cache


"""

innerSize ::
For a vector, this is just the size. For a matrix (non-vector), this is the
minor dimension with respect to the storage order, i.e., the number of rows for
a column-major matrix, and the number of columns for a row-major matrix

innerStride ::
The pointer increment between two consecutive elements within a slice in the
inner direction.

outerSize ::
For a vector, this returns just 1. For a matrix (non-vector), this is the major
dimension with respect to the storage order, i.e., the number of columns for a
column-major matrix, and the number of rows for a row-major matrix.

outerStride ::
The pointer increment between two consecutive inner slices (for example, between
two consecutive columns in a column-major matrix).


"""


class VitalEigenMatrix (VitalObject):
    """
    Wrapper for Eigen matrix use within Vital. This class covers both
    "Vector<dim><type>", "Matrix<dim><type>" and custom Eigen matrix types.

    All matrices created and interacted through this class are dense (vs.
    sparse).
    """

    # Valid data type keys.
    MAT_TYPE_KEYS = ('double', 'float')

    FUNC_SUFFIX_TMPL = "_{rows:d}x{cols:d}{type:s}"

    def __init__(self, rows, cols=1, dtype='double'):
        super(VitalEigenMatrix, self).__init__()

        if dtype == self.MAT_TYPE_KEYS[0]:  # double
            self._c_type = ctypes.c_double
            self._func_suffix = self.FUNC_SUFFIX_TMPL.format(
                rows=rows, cols=cols, type='d',
            )
        elif dtype == self.MAT_TYPE_KEYS[1]:  # float
            self._c_type = ctypes.c_float
            self._func_suffix = self.FUNC_SUFFIX_TMPL.format(
                rows=rows, cols=cols, type='f',
            )
        else:
            raise ValueError("Invalid data type given ('%s'). "
                             "Must be one of %s."
                             % (dtype, self.MAT_TYPE_KEYS))

        # numpy-wrapper cache
        self._n_cache = None

        # Creating new eigen matrix of the input shape
        m_new = self.VITAL_LIB['vital_eigen_new' + self._func_suffix]
        m_new.argtypes = [ctypes.c_uint, ctypes.c_uint]
        m_new.restype = self.C_TYPE_PTR
        self._inst_ptr = m_new(int(rows), int(cols))
        if not bool(self.c_pointer):
            raise RuntimeError("Failed to construct new vector instance")

    def _destroy(self):
        if self.c_pointer:
            m_del = self.VITAL_LIB['vital_eigen_destroy' + self._func_suffix]
            m_del.argtypes = [self.C_TYPE_PTR, VitalErrorHandle.C_TYPE_PTR]
            with VitalErrorHandle() as eh:
                m_del(self, eh)
            self._inst_ptr = self.C_TYPE_PTR()

    def __getitem__(self, spec):
        row, col = spec

        v_get = self.VITAL_LIB['vital_eigen_get' + self._func_suffix]
        v_get.argtypes = [self.C_TYPE_PTR, ctypes.c_uint, ctypes.c_uint,
                          VitalErrorHandle.C_TYPE_PTR]
        v_get.restype = self._c_type
        with VitalErrorHandle() as eh:
            return v_get(self, ctypes.c_uint(row), ctypes.c_uint(col), eh)

    def __setitem__(self, spec, value):
        row, col = spec

        v_set = self.VITAL_LIB['vital_eigen_set' + self._func_suffix]
        v_set.argtypes = [self.C_TYPE_PTR, ctypes.c_uint, ctypes.c_uint,
                          self._c_type, VitalErrorHandle.C_TYPE_PTR]
        with VitalErrorHandle() as eh:
            v_set(self, ctypes.c_uint(row), ctypes.c_uint(col),
                  self._c_type(value), eh)

    def as_numpy(self):
        if not self._n_cache:
            v_data = self.VITAL_LIB['vital_eigen_data' + self._func_suffix]
            v_data.argtypes = [self.C_TYPE_PTR,
                               ctypes.POINTER(ctypes.c_uint),  # rows
                               ctypes.POINTER(ctypes.c_uint),  # cols
                               ctypes.POINTER(ctypes.c_uint),  # inner stride
                               ctypes.POINTER(ctypes.c_uint),  # outer stride
                               ctypes.POINTER(ctypes.c_uint),  # is-row-major
                               ctypes.POINTER(ctypes.POINTER(self._c_type)),
                               VitalErrorHandle.C_TYPE_PTR]
            rows = ctypes.c_uint()
            cols = ctypes.c_uint()
            inner_stride = ctypes.c_uint()
            outer_stride = ctypes.c_uint()
            is_row_major = ctypes.c_uint()
            data = ctypes.POINTER(self._c_type)()
            with VitalErrorHandle() as eh:
                v_data(self,
                       ctypes.byref(rows), ctypes.byref(cols),
                       ctypes.byref(inner_stride), ctypes.byref(outer_stride),
                       ctypes.byref(is_row_major),
                       ctypes.byref(data), eh)

            rows = rows.value
            cols = cols.value
            inner_stride = inner_stride.value
            outer_stride = outer_stride.value
            is_row_major = bool(is_row_major.value)

            n = numpy.ctypeslib.as_array(data, (rows * cols,))
            dtype_bytes = numpy.dtype(self._c_type).alignment
            if is_row_major:
                print "col minor, row major"
                strides = (outer_stride * dtype_bytes,
                           inner_stride * dtype_bytes)
            else:
                print 'row minor, col major'
                strides = (inner_stride * dtype_bytes,
                           outer_stride * dtype_bytes)
            print strides
            self._n_cache = numpy.lib.stride_tricks.as_strided(
                n, shape=(rows, cols), strides=strides, subok=True
            )

        return self._n_cache
