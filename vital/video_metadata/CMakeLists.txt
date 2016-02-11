#
# Video metadata processing library
#

set( sources
  video_metadata.cxx
  )

set( public_headers
  video_metadata.h
  )

kwiver_add_library( vital_video_metadata
  ${sources}
  ${public_headers}
  )

target_link_libraries( vital_video_metadata
  PRIVATE              vital
                       vital_klv
                       vital_logger
)
