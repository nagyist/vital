project(kwiver_klv_tests)

include(vital-test-setup)

set( test_libraries vital vital_apm )

##############################
# KLV tests
##############################

kwiver_discover_tests(KLV             test_libraries test_klv.cxx)
