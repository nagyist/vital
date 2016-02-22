#
# KLV parsing toolkit
#

set( sources
  klv_parse.cxx
  klv_data.cxx
  klv_key.cxx
  klv_0601.cxx
  klv_0104.cxx
  )

set( public_headers
  klv_parse.h
  klv_data.h
  klv_key.h
  klv_0601.h
  klv_0104.h
  )

set( private_headers
  klv_0601_traits.h
)

kwiver_add_library( vital_klv
  ${sources}
  ${public_headers}
  ${private_headers}
  )

kwiver_install_headers(
  ${public_headers}
  SUBDIR   vital/klv
  )

target_link_libraries( vital_klv
  PRIVATE       vital
                vital_logger
  )
