# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "PHOENIX_2027_Infantry_guodong_up.map"
  )
endif()
