# ****************************************************************************
# CUI
#
# The Advanced Framework for Simulation, Integration, and Modeling (AFSIM)
#
# Copyright 2018 Infoscitex, a DCS Company. All rights reserved.
#
# The use, dissemination or disclosure of data in this file is subject to
# limitation or restriction. See accompanying README and LICENSE for details.
# ****************************************************************************
# *****************************************************************************

if (NOT WIN32)
   # Configure the 3rd_party libraries
   set(SWDEV_FFMPEG_PACKAGE "ffmpeg-4.2.4" CACHE STRING "" FORCE)
   mark_as_advanced(FORCE SWDEV_FFMPEG_PACKAGE)
   swdev_acquire_packages(${CMAKE_CURRENT_SOURCE_DIR}
                          ${SWDEV_FFMPEG_PACKAGE})
   plugin_cmakelists_template(VideoCapture tools/wkf
                              THIRD_PARTY_INCLUDE_DIRS ffmpeg
                              THIRD_PARTY_LINK_LIBS ffmpeg
                              LIBRARIES wkf)
   # install 3rd_party-cmake files that are required to build the plugin not covered elsewhere
   install_source_files(${TOOLS_DIRECTORY}/3rd_party-cmake/config.cmake
                        ${TOOLS_DIRECTORY}/3rd_party-cmake/shared.cmake
                        ${TOOLS_DIRECTORY}/3rd_party-cmake/${SWDEV_FFMPEG_PACKAGE}.cmake
                        tools/3rd_party-cmake)
   install_third_party(${FFMPEGROOT})
else()
   set(winEncoder mfreadwrite mfplat mfuuid)         
   plugin_cmakelists_template(VideoCapture tools/wkf
                              LIBRARIES wkf ${winEncoder})
endif()
