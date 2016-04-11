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


class VitalEigenMatrix (VitalObject):
    """
    Wrapper for Eigen matrix use within Vital. This class covers both
    "Vector<dim><type>", "Matrix<dim><type>" and custom Eigen matrix types.

    All matrices created and interacted through this class are dense (vs.
    sparse).
    """

    # Valid data type keys.
    MAT_TYPE_KEYS = ('double', 'float')

    FUNC_SPEC = "{rows:s}x{cols:s}{type:s}"

    @classmethod
    def from_c_pointer(cls, ptr, rows, cols, dtype, shallow_copy_of=None):
        #: :type: VitalEigenMatrix
        m = super(VitalEigenMatrix, cls).from_c_pointer(ptr, shallow_copy_of)

        # Initialize shape and function map into object
        m._init_function_map(rows, cols, dtype)

        return m

    def __init__(self, rows, cols=1, dynamic_rows=False, dynamic_cols=False,
                 dtype='double'):
        """
        Create a new Eigen matrix via the Vital C interface

        **Note:** *Not all sizes are supported. Only the standard vector and
        matrix sizes supported are valid through this constructor. If an invalid
        shape is provided, an exception will be raised stating that the
        correspoding C interface functions cannot be found.*

        :param rows: Number of rows.
        :type rows: int

        :param cols: Number of columns.
        :type cols: int

        :param dtype: Name of the C type to use as the core data representation
            type. See the ``MAT_TYPE_KEYS`` tuple on this class for valid
            options.
        :type dtype: str

        :param dynamic_rows: If the rows of the matrix considered "dynamic" in
            eigen's sense. This allows for creating matrices that

        """
        super(VitalEigenMatrix, self).__init__()
        self._init_function_map(rows, cols, dtype, dynamic_rows, dynamic_cols)

        # Creating new eigen matrix of the input shape
        m_new = self.VITAL_LIB[self._func_map['new_sized']]
        m_new.argtypes = [ctypes.c_size_t, ctypes.c_size_t]
        m_new.restype = self.C_TYPE_PTR
        self._inst_ptr = m_new(rows, cols)
        if not bool(self.c_pointer):
            raise RuntimeError("Failed to construct new vector instance")

        # numpy-wrapper cache
        self._n_cache = None

    def _init_function_map(self, rows, cols, dtype, dynamic_rows, dynamic_cols):
        self._shape = (rows, cols)
        self._dynamic_rows = dynamic_rows
        self._dynamic_cols = dynamic_cols
        if dtype == self.MAT_TYPE_KEYS[0]:  # double
            self._c_type = ctypes.c_double
            type_char = 'd'
        elif dtype == self.MAT_TYPE_KEYS[1]:  # float
            self._c_type = ctypes.c_float
            type_char = 'f'
        else:
            raise ValueError("Invalid data type given ('%s'). "
                             "Must be one of %s."
                             % (dtype, self.MAT_TYPE_KEYS))

        self._func_spec = self.FUNC_SPEC.format(
            rows=(dynamic_rows and 'X') or str(rows),
            cols=(dynamic_cols and 'X') or str(cols),
            type=type_char,
        )

        # Determine function methods to use for the given shape
        self._func_map = {
            'new': 'vital_eigen_matrix{}_new'.format(self._func_spec),
            'new_sized': 'vital_eigen_matrix{}_new_sized'.format(self._func_spec),
            'destroy': 'vital_eigen_matrix{}_destroy'.format(self._func_spec),
            'get': 'vital_eigen_matrix{}_get'.format(self._func_spec),
            'set': 'vital_eigen_matrix{}_set'.format(self._func_spec),
            'data': 'vital_eigen_matrix{}_data'.format(self._func_spec),
        }

    def _destroy(self):
        if self.c_pointer:
            m_del = self.VITAL_LIB[self._func_map['destroy']]
            m_del.argtypes = [self.C_TYPE_PTR, VitalErrorHandle.C_TYPE_PTR]
            with VitalErrorHandle() as eh:
                m_del(self, eh)
            self._inst_ptr = self.C_TYPE_PTR()

    def __getitem__(self, spec):
        if self._shape[1] == 1:
            row = int(spec)
            col = 0
        else:
            row, col = spec
            row = int(row)
            col = int(col)

        v_get = self.VITAL_LIB[self._func_map['get']]
        v_get.argtypes = [self.C_TYPE_PTR, ctypes.c_uint, ctypes.c_uint,
                          VitalErrorHandle.C_TYPE_PTR]
        v_get.restype = self._c_type
        with VitalErrorHandle() as eh:
            return v_get(self, ctypes.c_uint(row), ctypes.c_uint(col), eh)

    def __setitem__(self, spec, value):
        if self._shape[1] == 1:
            row = int(spec)
            col = 1
        else:
            row, col = spec
            row = int(row)
            col = int(col)

        v_set = self.VITAL_LIB[self._func_map['set']]
        v_set.argtypes = [self.C_TYPE_PTR, ctypes.c_uint, ctypes.c_uint,
                          self._c_type, VitalErrorHandle.C_TYPE_PTR]
        with VitalErrorHandle() as eh:
            v_set(self, ctypes.c_uint(row), ctypes.c_uint(col),
                  self._c_type(value), eh)

    def as_numpy(self):
        if self._n_cache is None:
            v_data = self.VITAL_LIB[self._func_map['data']]
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
