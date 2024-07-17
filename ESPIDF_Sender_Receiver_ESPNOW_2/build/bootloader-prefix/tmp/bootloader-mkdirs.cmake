# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/samue/esp/v5.2.2/esp-idf/components/bootloader/subproject"
  "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader"
  "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader-prefix"
  "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader-prefix/tmp"
  "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader-prefix/src"
  "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/samue/Desktop/Uni_Work/Student_Team/Got_in/ESDA_Aesterius/ESPIDF_Sender_Receiver_ESPNOW_2/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
