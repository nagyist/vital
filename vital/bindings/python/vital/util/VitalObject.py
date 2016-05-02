"""
ckwg +31
Copyright 2015-2016 by Kitware, Inc.
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

Base class for all VITAL Python interface classes

"""

import abc
import ctypes
import logging

from vital.util.find_vital_library import find_vital_library
from vital.util.string import vital_string_t


class VitalClassMeta (abc.ABCMeta):
    """
    Metaclass for Vital object types.

    Ensures that C_TYPE and C_TYPE_PTR are defined in derived classes.
    """

    def __new__(cls, name, bases, attrs):

        # Create a new structure type for the class if it has not already
        # defined one for itself
        if 'C_TYPE' not in attrs:
            class OpaqueStruct (ctypes.Structure):
                pass
            OpaqueStruct.__name__ = "%sOpaqueStruct" % name
            attrs['C_TYPE'] = OpaqueStruct
            attrs['C_TYPE_PTR'] = ctypes.POINTER(OpaqueStruct)

        return super(VitalClassMeta, cls).__new__(cls, name, bases, attrs)


class VitalObject (object):
    """
    Basic VITAL python interface class.
    """
    __metaclass__ = VitalClassMeta

    VITAL_LIB = find_vital_library()

    # C API opaque structure + pointer
    C_TYPE = None
    C_TYPE_PTR = None

    # Common string structure stuff
    MST_TYPE = vital_string_t
    MST_TYPE_PTR = vital_string_t.PTR_t
    MST_NEW = VITAL_LIB['vital_string_new']
    MST_NEW.argtypes = [ctypes.c_size_t, ctypes.c_char_p]
    MST_NEW.restype = vital_string_t.PTR_t
    MST_FREE = VITAL_LIB['vital_string_free']
    MST_FREE.argtypes = [vital_string_t.PTR_t]

    def __init__(self, from_cptr=None, *args, **kwds):
        """
        Create a new instance of the Python vital type wrapper.

        This initializer should only be called after C_TYPE/C_TYPE_PTR are
        concrete types.

        :param from_cptr: Existing C opaque instance pointer to use, preventing new
            instance construction. This should of course be a valid pointer to
            an instance.

        Optional keyword arguments:

        :param allow_null_pointer: Allow a null pointer to be returned from the
            _new method instead of raising an exceptiong

        """
        if None in (self.C_TYPE, self.C_TYPE_PTR):
            raise RuntimeError("Derived class did not define opaque handle "
                               "structure types.")

        allow_null_pointer = kwds.get('allow_null_pointer', None)
        if allow_null_pointer is not None:
            del kwds['allow_null_pointer']

        if from_cptr is not None:
            # if null pointer and we're not allowing them
            if not (allow_null_pointer or bool(from_cptr)):
                raise RuntimeError("Cannot initialize to a null pointer")
            # if not a valid opaque pointer type
            elif not isinstance(from_cptr, self.C_TYPE_PTR):
                raise RuntimeError("Given C Opaque Pointer is not of the "
                                   "correct type. Given '%s' but expected '%s'."
                                   % type(from_cptr, self.C_TYPE_PTR))
            self._inst_ptr = from_cptr
        else:
            self._inst_ptr = self._new(*args, **kwds)
            # raise if we have a null pointer and we don't allow nulls
            if not (allow_null_pointer or bool(self._inst_ptr)):
                raise RuntimeError("Failed to construct new %s instance: Null "
                                   "pointer returned from construction."
                                   % self.__class__.__name__)

    def __del__(self):
        self._destroy()

    def __nonzero__(self):
        """ bool() operator for 2.x """
        return bool(self.c_pointer)

    def __bool__(self):
        """ bool() operator for 3.x """
        return bool(self.c_pointer)

    @property
    def _as_parameter_(self):
        """
        Ctypes interface attribute for allowing a user to pass the python object
        instance as argument to a C function instead of the opaque pointer.
        This means that when an instance of this class is passed as an argument,
        the underlying opaque pointer is automatically passed in its place.
        """
        return self.c_pointer

    @property
    def _log(self):
        return logging.getLogger('.'.join([self.__module__,
                                           self.__class__.__name__]))

    @property
    def c_pointer(self):
        """
        :return: The ctypes opaque structure pointer
        """
        return self._inst_ptr

    @abc.abstractmethod
    def _new(self, *args, **kwds):
        """
        Construct a new instance, returning new instance opaque C pointer and
        initializing any other necessary object properties

        :returns: New C opaque structure pointer.

        """

    @abc.abstractmethod
    def _destroy(self):
        """
        Call C API destructor for derived class
        """
        raise NotImplementedError("Calling VitalObject class abstract _destroy "
                                  "function.")

    # TODO: Serialization hooks?


class OpaqueTypeCache (object):
    """
    Support structure for VitalObject sub-classes that represent multiple
    C types akin to C++ templating.
    """

    def __init__(self, name_prefix=None):
        # Store pairs of C opaque structure and its pointer type
        #: :type: dict[str, (_ctypes.PyCStructType, _ctypes.PyCPointerType)]
        self._c_type_cache = {}
        self._prefix = name_prefix or ''

    def get_types(self, k):
        """
        Return or generate opaque type and pointer based on shape spec
        """
        if k not in self._c_type_cache:
            # Based on VitalClassMetadata meta-cass
            class OpaqueStruct (ctypes.Structure):
                pass
            OpaqueStruct.__name__ = "%s%s_OpaqueStructure" % (self._prefix, k)
            self._c_type_cache[k] = \
                (OpaqueStruct, ctypes.POINTER(OpaqueStruct))
        return self._c_type_cache[k]

    def new_type_getter(self):
        """
        Returns new simple object  with __getitem__ hook for C Type
        """
        class c_type_manager (object):
            def __getitem__(s2, k):
                return self.get_types(k)[0]
            __contains__ = self._c_type_cache.__contains__
            @property
            def _as_parameter_(self):
                raise RuntimeError("Cannot use type manager as ctypes "
                                   "parameter")
        return c_type_manager()

    def new_ptr_getter(self):
        """
        Returns new simple object with __getitem__ hook for C Type Pointer
        """
        class c_type_ptr_manager (object):
            def __getitem__(s2, k):
                return self.get_types(k)[1]
            __contains__ = self._c_type_cache.__contains__
            @property
            def _as_parameter_(self):
                raise RuntimeError("Cannot use type manager as ctypes "
                                   "parameter")
        return c_type_ptr_manager()
