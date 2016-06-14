# - Find IDS
# This module looks for the IDS uEye software package 
# and determines where the binaries and header files are.
# This code sets the following variables:
#
#  IDS_FOUND          - True if uEye API found
#  IDS_PATH:          - Path to the uEye API folder
#  IDS_LIBRARY_DIR    - uEye libraries folder

set(IDS_FOUND)
set(IDS_PATH)
set(IDS_LIBRARY)

if(WIN32)
  find_path(IDS_PATH uEye.h)

  if(EXISTS ${IDS_PATH})
    find_library(IDS_LIBRARY uEye_api.lib)
    if(EXISTS ${IDS_LIBRARY})
      set(IDS_FOUND 1)
    endif()
  endif()
endif()

mark_as_advanced(FORCE IDS_FOUND)
mark_as_advanced(FORCE IDS_PATH)
mark_as_advanced(FORCE IDS_LIBRARY)
