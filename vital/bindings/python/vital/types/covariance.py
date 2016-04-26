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

Description here.

"""
import ctypes

import numpy
import scipy.sparse

from vital.types.eigen import EigenArray
from vital.util import OpaqueTypeCache, VitalObject, VitalErrorHandle


class Covariance (VitalObject):

    # Override C opaque pointer type to ones that are dependent on size and type
    TYPE_CACHE = OpaqueTypeCache("Covariance_")
    C_TYPE = TYPE_CACHE.new_type_getter()
    C_TYPE_PTR = TYPE_CACHE.new_ptr_getter()

    SHAPE_SPEC = "{size:d}{type:s}"

    @staticmethod
    def _vector_index(r, c):
        if c > r:
            return c * (c + 1) / 2 + r
        else:
            return r * (r + 1) / 2 + c

    @staticmethod
    def _make_csc_mat(c_ptr, n):
        size = n * (n + 1) / 2
        a = numpy.ctypeslib.as_array(c_ptr, (size,))
        ij = numpy.ndarray((2, size), int)
        for r in xrange(n):
            for c in xrange(r, n):
                i = Covariance._vector_index(r, c)
                ij[:, i] = [[r], [c]]
        scipy.sparse.csc_matrix([a, ij], shape=(n, n))

    # # noinspection PyMethodOverriding
    # @classmethod
    # def from_c_pointer(cls, ptr, size=2, c_type=ctypes.c_double,
    #                    shallow_copy_of=None):
    #     obj = Covariance(size, c_type, ptr)

    def __new__(cls, N=2, c_type=ctypes.c_double, *args, **kwds):
        # noinspection PyProtectedMember
        c_type_char = c_type._type_
        obj = super(Covariance, cls).__new__(cls)
        # Set shape/type specific opaque pointer
        obj.C_TYPE = cls.C_TYPE[str(N) + c_type_char]
        obj.C_TYPE_PTR = cls.C_TYPE_PTR[str(N) + c_type_char]

        # Initialize function map based on shape
        ss = cls.SHAPE_SPEC.format(size=N, type=c_type_char)
        obj._func_map = {
            'new_identity':
                cls.VITAL_LIB['vital_covariance_{}_new'.format(ss)],
            'new_scalar':
                cls.VITAL_LIB['vital_covariance_{}_new_from_scalar'.format(ss)],
            'new_matrix':
                cls.VITAL_LIB['vital_covariance_{}_new_from_matrix'.format(ss)],
            'destroy':
                cls.VITAL_LIB['vital_covariance_{}_destroy'.format(ss)],
            'to_matrix':
                cls.VITAL_LIB['vital_covariance_{}_to_matrix'.format(ss)],
            'get':
                cls.VITAL_LIB['vital_covariance_{}_get'.format(ss)],
            'set':
                cls.VITAL_LIB['vital_covariance_{}_set'.format(ss)],
        }

        return obj

    def __init__(self, N=2, c_type=ctypes.c_double, init_scalar_or_matrix=None):
        """
        Create a new Covariance symmetric matrix instance.

        This object stores the upper triangle portion of a symmetric matrix of
        side-length `N`. Accessing the lower triangle portion of the matrix thus
        yields the same values as the upper portion.

        :param N: Size of the matrix. This determines the side length of the
            symmetric matrix. This is constrained to the sizes declared in C:
            [2, 3]. Default is 2.
        :param c_type: The C data type to represent values in. This may me
            either float or double. Default is double.
        :param init_scalar_or_matrix: By default, we initialize an identity
            matrix. If a scalar value is provided here, we initialize to an
            identity times the given scalar. If it is a square EigenArray
            of size N, we initialize the covariance matrix based on this matrix
            (averages off diagonal elements to enforce symmetry). Input matrix
            data is copied, not shared.

        """
        super(Covariance, self).__init__()

        self._N = N
        self._ctype = c_type

        if init_scalar_or_matrix is None:
            self._log.debug("Initializing identity")
            c_new = self._func_map['new_identity']
            c_new.argtypes = [VitalErrorHandle.C_TYPE_PTR]
            c_new.restype = self.C_TYPE_PTR
            args = ()
        elif isinstance(init_scalar_or_matrix, numpy.ndarray):
            self._log.debug("Initializing with matrix")
            dtype = numpy.dtype(c_type)
            in_mat = init_scalar_or_matrix
            # Checking matrix shape/type properties
            if in_mat.shape != (N, N):
                raise ValueError("Input matrix of non-congruent shape: %s (our "
                                 "shape: %s)" % (in_mat.shape, (N, N)))
            if not isinstance(in_mat, EigenArray) or in_mat.dtype != dtype:
                # create specific shape/type required for construction, and copy
                # in matrix data. This handles type casting required.
                self._log.debug("Creating new EigenArray for type casting")
                mat = EigenArray(N, N, dtype=numpy.dtype(c_type))
                mat[:] = in_mat
            else:
                mat = in_mat
            c_new = self._func_map['new_matrix']
            c_new.argtypes = [mat.C_TYPE_PTR,
                              VitalErrorHandle.C_TYPE_PTR]
            args = (mat,)
        else:
            self._log.debug("Initializing with scalar")
            c_new = self._func_map['new_scalar']
            c_new.argtypes = [self._ctype, VitalErrorHandle.C_TYPE_PTR]
            args = (init_scalar_or_matrix,)

        c_new.restype = self.C_TYPE_PTR
        with VitalErrorHandle() as eh:
            c_args = args + (eh,)
            # self._log.debug("Construction args: %s", c_args)
            self._inst_ptr = c_new(*c_args)
        if not bool(self._inst_ptr):
            raise RuntimeError("C++ Construction failed (null pointer)")

    def _destroy(self):
        c_del = self._func_map['destroy']
        c_del.argtypes = [self.C_TYPE_PTR, VitalErrorHandle.C_TYPE_PTR]
        with VitalErrorHandle() as eh:
            c_del(self, eh)
        self._inst_ptr = self.C_TYPE_PTR()

    def to_matrix(self):
        c_to_mat = self._func_map['to_matrix']
        c_to_mat.argtypes = [self.C_TYPE_PTR, VitalErrorHandle.C_TYPE_PTR]
        mat_spec = '{n:d}x{n:d}{t:s}'.format(n=self._N, t=self._ctype._type_)
        # self._log.debug('mat spec: %s', mat_spec)
        c_to_mat.restype = EigenArray.C_TYPE_PTR[mat_spec]

        with VitalErrorHandle() as eh:
            m_ptr = c_to_mat(self, eh)
            return EigenArray(self._N, self._N, dtype=numpy.dtype(self._ctype),
                              c_ptr=m_ptr, owns_data=True)

    def __getitem__(self, p):
        """
        Get an element of the covariance matrix
        :param p: row, column index pair
        :return: Value at the specified index in if bounds
        :raises IndexError: Index out of bounds
        """
        r, c = p
        if not (0 <= r < self._N and 0 <= c < self._N):
            raise IndexError(p)
        c_get = self._func_map['get']
        c_get.argtypes = [self.C_TYPE_PTR, ctypes.c_uint, ctypes.c_uint,
                          VitalErrorHandle.C_TYPE_PTR]
        c_get.restype = self._ctype
        with VitalErrorHandle() as eh:
            return c_get(self, r, c, eh)

    def __setitem__(self, p, value):
        r, c = p
        if not (0 <= r < self._N and 0 <= c < self._N):
            raise IndexError(p)
        c_set = self._func_map['set']
        c_set.argtypes = [self.C_TYPE_PTR, ctypes.c_uint, ctypes.c_uint,
                          self._ctype, VitalErrorHandle.C_TYPE_PTR]
        with VitalErrorHandle() as eh:
            c_set(self, r, c, value, eh)
