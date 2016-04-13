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


class VitalEigenNumpyArray (numpy.ndarray, VitalObject):

    # Valid dtype possibilities
    MAT_TYPE_KEYS = (numpy.double, numpy.float32)

    # C library function component template
    FUNC_SPEC = "{rows:s}x{cols:s}{type:s}"

    def __new__(cls, rows, cols=1, dynamic_rows=False, dynamic_cols=False,
                dtype=numpy.double, c_ptr=None):
        """
        Create a new Vital Eigen matrix and interface

        :param rows: Number of rows in the matrix
        :param cols: Number of columns in the matrix
        :param dynamic_rows: If we should not use compile-time generated types
            in regards to the row specification.
        :param dynamic_cols: If we should not use compile-time generated types
            in regards to the column specification.
        :param dtype: numpy dtype to use
        :param c_ptr: Optional existing C Eigen matrix instance pointer to use
            instead of constructing a new one.

        :return: Interface to a new or existing Eigen matrix instance.

        """
        func_map, c_type = cls._init_func_map(rows, cols,
                                              dynamic_rows, dynamic_cols,
                                              dtype)

        # Create new Eigen matrix
        if c_ptr is None:
            c_new = cls.VITAL_LIB[func_map['new_sized']]
            c_new.argtypes = [ctypes.c_size_t, ctypes.c_size_t]
            c_new.restype = cls.C_TYPE_PTR
            inst_ptr = c_new(ctypes.c_size_t(rows), ctypes.c_size_t(cols))
            if not bool(inst_ptr):
                raise RuntimeError("Failed to construct new Eigen matrix")
        else:
            inst_ptr = c_ptr

        # Get information, data pointer and base transformed array
        rows, cols, inner_stride, outer_stride, is_row_major, data = \
            cls._get_data_components(inst_ptr, c_type, func_map)
        dtype_bytes = numpy.dtype(c_type).alignment
        # Might have to swap out the use of ``dtype_bytes`` for
        # inner/outer size values from Eigen if matrices are ever NOT
        # densely packed.
        if is_row_major:
            strides = (outer_stride * dtype_bytes,
                       inner_stride * dtype_bytes)
        else:
            strides = (inner_stride * dtype_bytes,
                       outer_stride * dtype_bytes)
        if data:
            b = numpy.ctypeslib.as_array(data, (rows * cols,))
        else:
            b = buffer('')

        # args: (subclass, shape, dtype, buffer, offset, strides, order)
        # TODO: Get offset from eigen matrix, too.
        obj = numpy.ndarray.__new__(cls, (rows, cols), dtype, b, 0, strides)

        # local properties
        obj._dynamic_rows = dynamic_rows
        obj._dynamic_cols = dynamic_cols
        obj._func_map = func_map

        VitalObject.__init__(obj)

        obj._inst_ptr = inst_ptr
        obj._parent = None

        return obj

    # noinspection PyMethodOverriding
    @classmethod
    def from_c_pointer(cls, ptr, rows, cols=1,
                       dynamic_rows=False, dynamic_cols=False,
                       dtype=numpy.double, shallow_copy_of=None):
        """
        Special implementation from C pointer due to both needing more
        information and because we are sub-classing numpy.ndarray.

        :param ptr: C API opaque structure pointer type instance
        :type ptr: VitalAlgorithm.C_TYPE_PTR

        :param rows: Number of rows in the matrix
        :param cols: Number of columns in the matrix
        :param dynamic_rows: If we should not use compile-time generated types
            in regards to the row specification.
        :param dynamic_cols: If we should not use compile-time generated types
            in regards to the column specification.
        :param dtype: numpy dtype to use

        :param shallow_copy_of: Optional parent object instance when the ptr
            given is coming from an existing python object.
        :type shallow_copy_of: VitalObject or None

        """
        m = VitalEigenNumpyArray(rows, cols, dynamic_rows, dynamic_cols,
                                 dtype, ptr)
        m._parent = shallow_copy_of
        return m

    # noinspection PyMissingConstructor
    def __init__(self, *args, **kwds):
        # initialization handled in __new__
        pass

    def __array_finalize__(self, obj):
        """
        Where numpy finalized instance properties of an array instance when
        created due to __new__, casting or new-from-template.
        """
        # git here from __new__, nothing to transfer
        if obj is None:
            return

        # copy/move over attributes from parent as necessary
        #   self => New class of this type
        #   obj  => other class MAYBE this type
        if isinstance(obj, VitalEigenNumpyArray):
            self._func_map = obj._func_map
            self._dynamic_rows = obj._dynamic_rows
            self._dynamic_cols = obj._dynamic_cols

            self._inst_ptr = obj._inst_ptr
            if obj._parent is None:
                # obj is the root parent object
                self._parent = obj
            else:
                # transfer parent reference
                self._parent = obj._parent
        else:
            raise RuntimeError("Finalizing VitalEigenNumpyArray whose parent "
                               "is not of the same type: %s" % type(obj))

    def __array_wrap__(self, out_arr, context=None):
        # Don't propagate this class and reference?
        return numpy.asarray(out_arr)

    def _destroy(self):
        if self.c_pointer:
            # print "Destroying"
            m_del = self.VITAL_LIB[self._func_map['destroy']]
            m_del.argtypes = [self.C_TYPE_PTR, VitalErrorHandle.C_TYPE_PTR]
            with VitalErrorHandle() as eh:
                m_del(self, eh)
            self._inst_ptr = self.C_TYPE_PTR()

    @classmethod
    def _init_func_map(cls, rows, cols, d_rows, d_cols, dtype):
        if dtype == cls.MAT_TYPE_KEYS[0]:  # C double
            type_char = 'd'
            c_type = ctypes.c_double
        elif dtype == cls.MAT_TYPE_KEYS[1]:  # C float
            type_char = 'f'
            c_type = ctypes.c_float
        else:
            raise ValueError("Invalid data type given ('%s'). "
                             "Must be one of %s."
                             % (dtype, cls.MAT_TYPE_KEYS))
        func_spec = cls.FUNC_SPEC.format(
            rows=(d_rows and 'X') or str(rows),
            cols=(d_cols and 'X') or str(cols),
            type=type_char,
        )
        func_map = {
            'new': 'vital_eigen_matrix{}_new'.format(func_spec),
            'new_sized': 'vital_eigen_matrix{}_new_sized'.format(func_spec),
            'destroy': 'vital_eigen_matrix{}_destroy'.format(func_spec),
            'get': 'vital_eigen_matrix{}_get'.format(func_spec),
            'set': 'vital_eigen_matrix{}_set'.format(func_spec),
            'data': 'vital_eigen_matrix{}_data'.format(func_spec),
        }
        return func_map, c_type

    @classmethod
    def _get_data_components(cls, ptr, c_type, func_map):
        v_data = cls.VITAL_LIB[func_map['data']]
        v_data.argtypes = [cls.C_TYPE_PTR,
                           ctypes.POINTER(ctypes.c_uint),  # rows
                           ctypes.POINTER(ctypes.c_uint),  # cols
                           ctypes.POINTER(ctypes.c_uint),  # inner stride
                           ctypes.POINTER(ctypes.c_uint),  # outer stride
                           ctypes.POINTER(ctypes.c_uint),  # is-row-major
                           ctypes.POINTER(ctypes.POINTER(c_type)),
                           VitalErrorHandle.C_TYPE_PTR]
        rows = ctypes.c_uint()
        cols = ctypes.c_uint()
        inner_stride = ctypes.c_uint()
        outer_stride = ctypes.c_uint()
        is_row_major = ctypes.c_uint()
        data = ctypes.POINTER(c_type)()
        with VitalErrorHandle() as eh:
            v_data(ptr,
                   ctypes.byref(rows), ctypes.byref(cols),
                   ctypes.byref(inner_stride), ctypes.byref(outer_stride),
                   ctypes.byref(is_row_major),
                   ctypes.byref(data), eh)

        rows = rows.value
        cols = cols.value
        inner_stride = inner_stride.value
        outer_stride = outer_stride.value
        is_row_major = bool(is_row_major.value)
        return rows, cols, inner_stride, outer_stride, is_row_major, data
