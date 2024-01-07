# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "G:/DevEnv/espidf/components/bootloader/subproject"
  "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader"
  "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader-prefix"
  "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader-prefix/tmp"
  "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader-prefix/src/bootloader-stamp"
  "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader-prefix/src"
  "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "G:/Project/Coding/Hardware/stopplate_refact/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
