#
# Video metadata processing library
#

set( sources
  video_metadata.cxx
  convert_0601_metadata.cxx
  convert_0104_metadata.cxx
  )

set( public_headers
  video_metadata.h
  )

set( private_headers
  convert_metadata.h
  )

kwiver_add_library( vital_video_metadata
  ${sources}
  ${public_headers}
  ${private_headers}
  )

target_link_libraries( vital_video_metadata
  PRIVATE              vital
                       vital_klv
                       vital_logger
)
