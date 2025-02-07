################################################################################
#                      _____ _____ _____ ______ ______   __                    #
#                     / ____|_   _/ ____|  ____/ __ \ \ / /                    #
#                    | (___   | || |  __| |__ | |  | \ V /                     #
#                     \___ \  | || | |_ |  __|| |  | |> <                      #
#                     ____) |_| || |__| | |   | |__| / . \                     #
#                    |_____/|_____\_____|_|    \____/_/ \_\                    #
#                                  ___   _____                                 #
#                                 / _ \ / ____|                                #
#                                | | | | |  __                                 #
#                                | | | | | |_ |                                #
#                                | |_| | |__| |                                #
#                                 \___/ \_____|                                #
#     _______ ______ _____ _    _ _   _  ____  _      ____   _______     __    #
#    |__   __|  ____/ ____| |  | | \ | |/ __ \| |    / __ \ / ____\ \   / /    #
#       | |  | |__ | |    | |__| |  \| | |  | | |   | |  | | |  __ \ \_/ /     #
#       | |  |  __|| |    |  __  | . ` | |  | | |   | |  | | | |_ | \   /      #
#       | |  | |___| |____| |  | | |\  | |__| | |___| |__| | |__| |  | |       #
#       |_|  |______\_____|_|  |_|_| \_|\____/|______\____/ \_____|  |_|       #
#             ______     __  _    _ _   _          ____ _____ ______           #
#            |  _ \ \   / / | |  | | \ | |   /\   |  _ \_   _|___  /           #
#            | |_) \ \_/ /  | |  | |  \| |  /  \  | |_) || |    / /            #
#            |  _ < \   /   | |  | | . ` | / /\ \ |  _ < | |   / /             #
#            | |_) | | |    | |__| | |\  |/ ____ \| |_) || |_ / /__            #
#            |____/  |_|     \____/|_| \_/_/    \_\____/_____/_____|           #
#                                                                              #
################################################################################
#
# Copyright (c) 2024, UnaBiz SAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1 Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  2 Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

if(DEFINED _initial_CMAKE_TOOLCHAIN_FILE
   AND NOT _initial_CMAKE_TOOLCHAIN_FILE STREQUAL CMAKE_TOOLCHAIN_FILE)
elseif(DEFINED CMAKE_TOOLCHAIN_FILE)
  set(_initial_CMAKE_TOOLCHAIN_FILE "${CMAKE_TOOLCHAIN_FILE}" CACHE INTERNAL "")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
#set(CMAKE_VERBOSE_MAKEFILE True)

#Add OS extention
if(WIN32)
    set(TOOLCHAIN_EXT ".exe" )
else()
    set(TOOLCHAIN_EXT "" )
endif()

#Toolchain name
set(TARGET_TRIPLET "arm-none-eabi-")

#Search toolchain
if(NOT TOOLCHAIN_PATH)
    #Try to find in PATH
    find_program(GCC_COMPILER ${TARGET_TRIPLET}gcc PATH_SUFFIXES bin)
    if(NOT GCC_COMPILER)
        message(STATUS "toolchain not found, Please specify the TOOLCHAIN_PATH\n -DTOOLCHAIN_PATH=<toolchain directory>")
    else()
        get_filename_component(TOOLCHAIN_BIN_PATH ${GCC_COMPILER} DIRECTORY)
        message(STATUS "Found compiler ${TOOLCHAIN_PATH}")
    endif()
else()
    set (TOOLCHAIN_BIN_PATH ${TOOLCHAIN_PATH}/bin)
    message(STATUS "Found compiler ${TOOLCHAIN_PATH}")
endif()

#Configure toolchain
set(CMAKE_C_COMPILER   ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}gcc${TOOLCHAIN_EXT})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}g++${TOOLCHAIN_EXT})
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}gcc${TOOLCHAIN_EXT})
set(CMAKE_LINKER       ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}gcc${TOOLCHAIN_EXT})
set(CMAKE_SIZE_UTIL    ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}size${TOOLCHAIN_EXT})
set(CMAKE_OBJCOPY      ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}objcopy${TOOLCHAIN_EXT})
set(CMAKE_OBJDUMP      ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}objdump${TOOLCHAIN_EXT})
set(CMAKE_NM_UTIL      ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}gcc-nm${TOOLCHAIN_EXT})
set(CMAKE_AR           ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}gcc-ar${TOOLCHAIN_EXT})
set(CMAKE_RANLIB       ${TOOLCHAIN_BIN_PATH}/${TARGET_TRIPLET}gcc-ranlib${TOOLCHAIN_EXT})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_COMMON_FLAGS "-mcpu=cortex-m0plus -Os -mthumb -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wextra -Werror -Wno-unused-parameter -g3")

# Default Flag
set(CMAKE_C_FLAGS "${CMAKE_COMMON_FLAGS} -std=gnu11")
set(CMAKE_CXX_FLAGS "${CMAKE_COMMON_FLAGS} -std=c++11 ")
set(CMAKE_ASM_FLAGS "${CMAKE_COMMON_FLAGS} -x assembler-with-cpp ")

# Release Flag
set(CMAKE_C_FLAGS_RELEASE "-Os")
set(CMAKE_CXX_FLAGS_RELEASE "-Os")
set(CMAKE_ASM_FLAGS_RELEASE "")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "")

# Debug Flag
set(CMAKE_C_FLAGS_DEBUG "-Og -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -DDEBUG")
set(CMAKE_ASM_FLAGS_DEBUG "")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "-nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections ")
