# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1"
  "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/1"
  "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1"
  "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1/tmp"
  "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1/src/CyberryPotter+Target_1-stamp"
  "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1/src"
  "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1/src/CyberryPotter+Target_1-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1/src/CyberryPotter+Target_1-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/13087/Desktop/Cyberry_Potter_Electromagic_Wand/Software/tmp/CyberryPotter+Target_1/src/CyberryPotter+Target_1-stamp${cfgdir}") # cfgdir has leading slash
endif()
