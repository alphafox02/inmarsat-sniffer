cmake_minimum_required(VERSION 3.9)
project(inmarsat-sniffer C CXX)
enable_language(CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

# Homebrew include/link paths on MacOS (Intel and Apple Silicon)
if(APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64|AARCH64")
        link_directories(/opt/homebrew/lib)
        include_directories(/opt/homebrew/include)
    else()
        link_directories(/usr/local/lib)
        include_directories(/usr/local/include)
    endif()
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-common -Wall -Wextra -Wno-unused-parameter -std=c99 -Werror=implicit-function-declaration")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -std=c++11")

set(CMAKE_C_FLAGS_DEBUG "-ggdb3 -g3 -O0 -fsanitize=address")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -ggdb3 -g3")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "-fsanitize=address")

# Build RelWithDebInfo by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
      "Choose the type of build: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

# Required dependencies
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# SDR backends (all optional, need at least one for live capture)
find_package(SoapySDR)
find_package(SDRplay)
find_package(RTLSDR)
find_library(HACKRF_LIB hackrf)
find_path(HACKRF_INC libhackrf/hackrf.h)
if(HACKRF_LIB AND HACKRF_INC)
    set(HACKRF_FOUND TRUE)
endif()
find_library(BLADERF_LIB bladeRF)
find_path(BLADERF_INC libbladeRF.h)
if(BLADERF_LIB AND BLADERF_INC)
    set(BLADERF_FOUND TRUE)
endif()
find_library(UHD_LIB uhd)
find_path(UHD_INC uhd.h)
if(UHD_LIB AND UHD_INC)
    set(UHD_FOUND TRUE)
endif()
find_library(AIRSPY_LIB airspy)
find_path(AIRSPY_INC libairspy/airspy.h)
if(AIRSPY_LIB AND AIRSPY_INC)
    set(AIRSPY_FOUND TRUE)
endif()

# Optional libacars-2 for ARINC-622/ADS-C/CPDLC decoding in ACARS messages
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBACARS QUIET libacars-2)
endif()
if(NOT LIBACARS_FOUND)
    find_library(LIBACARS_LIB acars-2)
    find_path(LIBACARS_INC libacars/libacars.h PATH_SUFFIXES libacars-2)
    if(LIBACARS_LIB AND LIBACARS_INC)
        set(LIBACARS_FOUND TRUE)
        set(LIBACARS_LIBRARIES ${LIBACARS_LIB})
        set(LIBACARS_INCLUDE_DIRS ${LIBACARS_INC})
    endif()
endif()

set(HAVE_ANY_SDR FALSE)
if(SOAPYSDR_FOUND)
    set(HAVE_ANY_SDR TRUE)
endif()
if(SDRPLAY_FOUND)
    set(HAVE_ANY_SDR TRUE)
endif()
if(RTLSDR_FOUND)
    set(HAVE_ANY_SDR TRUE)
endif()
if(HACKRF_FOUND)
    set(HAVE_ANY_SDR TRUE)
endif()
if(BLADERF_FOUND)
    set(HAVE_ANY_SDR TRUE)
endif()
if(UHD_FOUND)
    set(HAVE_ANY_SDR TRUE)
endif()
if(AIRSPY_FOUND)
    set(HAVE_ANY_SDR TRUE)
endif()

if(NOT HAVE_ANY_SDR)
    message(WARNING "No SDR backend found. Live capture will not be available.\n"
                    "Install SoapySDR: sudo apt install libsoapysdr-dev")
endif()

# Sources
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/jaero_dsp)

set(SOURCES
    ${PROJECT_SOURCE_DIR}/main.c
    ${PROJECT_SOURCE_DIR}/options.c
    ${PROJECT_SOURCE_DIR}/satellites.c
    ${PROJECT_SOURCE_DIR}/channelizer.c
    ${PROJECT_SOURCE_DIR}/demod_dbpsk.c
    ${PROJECT_SOURCE_DIR}/viterbi.c
    ${PROJECT_SOURCE_DIR}/stdc_decode.c
    ${PROJECT_SOURCE_DIR}/feed.c
    ${PROJECT_SOURCE_DIR}/web.c
    ${PROJECT_SOURCE_DIR}/vita49.c
    ${PROJECT_SOURCE_DIR}/basestation.c
    ${PROJECT_SOURCE_DIR}/aircraft_db.c
    ${PROJECT_SOURCE_DIR}/acars_position.c
    ${PROJECT_SOURCE_DIR}/waypoint_db.c
    ${PROJECT_SOURCE_DIR}/learned_waypoints.c
    ${PROJECT_SOURCE_DIR}/simd_generic.c
)

# SIMD kernels — platform-specific, runtime-detected (ported from iridium-sniffer)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64|x86|i[3-6]86")
    set_source_files_properties(${PROJECT_SOURCE_DIR}/simd_sse42.c
        PROPERTIES COMPILE_FLAGS "-msse4.2")
    set_source_files_properties(${PROJECT_SOURCE_DIR}/simd_avx2.c
        PROPERTIES COMPILE_FLAGS "-mavx2 -mfma")
    list(APPEND SOURCES
        ${PROJECT_SOURCE_DIR}/simd_sse42.c
        ${PROJECT_SOURCE_DIR}/simd_avx2.c)
    message(STATUS "SIMD: SSE4.2 + AVX2+FMA kernels enabled (runtime-detected)")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64|AARCH64")
    list(APPEND SOURCES
        ${PROJECT_SOURCE_DIR}/simd_neon.c)
    message(STATUS "SIMD: NEON kernels enabled (AArch64)")
else()
    message(STATUS "SIMD: scalar only (unrecognised platform)")
endif()

# Vendored libcorrect (BSD 3-clause, github.com/quiet/libcorrect)
# Used by JAERO's AeroL Viterbi + Reed-Solomon decoder
set(LIBCORRECT_DIR ${PROJECT_SOURCE_DIR}/jaero_dsp/libcorrect)
file(GLOB LIBCORRECT_SOURCES
    ${LIBCORRECT_DIR}/src/convolutional/*.c
    ${LIBCORRECT_DIR}/src/reed-solomon/*.c
    ${LIBCORRECT_DIR}/src/fec_shim.c)
include_directories(${LIBCORRECT_DIR}/include)

# JAERO DSP library (C++ with C API, stripped of Qt)
set(JAERO_DSP_SOURCES
    ${PROJECT_SOURCE_DIR}/jaero_dsp/DSP.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/burstmskdemodulator.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/burstoqpskdemodulator.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/mskdemodulator.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/oqpskdemodulator.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/coarsefreqestimate.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/fftrwrapper.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/fftwrapper.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/jfft.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/jconvolutionalcodec.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/aerol.cpp
    ${PROJECT_SOURCE_DIR}/jaero_dsp/jaero_demod.cpp
)
list(APPEND SOURCES ${JAERO_DSP_SOURCES} ${LIBCORRECT_SOURCES})

# SDR backends
if(SOAPYSDR_FOUND)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/soapysdr.c)
    include_directories(${SOAPYSDR_INCLUDE_DIR})
endif()

if(SDRPLAY_FOUND)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/sdrplay.c)
    include_directories(${SDRPLAY_INCLUDE_DIR})
endif()

if(RTLSDR_FOUND)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/rtlsdr.c)
    include_directories(${RTLSDR_INCLUDE_DIR})
endif()

if(HACKRF_FOUND)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/hackrf.c)
    include_directories(${HACKRF_INC})
endif()

if(BLADERF_FOUND)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/bladerf.c)
    include_directories(${BLADERF_INC})
endif()

if(AIRSPY_FOUND)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/airspy.c)
    include_directories(${AIRSPY_INC})
endif()

if(UHD_FOUND)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/usrp.c)
    include_directories(${UHD_INC})
endif()

# Optional MQTT for publishing ACARS messages
find_library(MOSQUITTO_LIB mosquitto)
find_path(MOSQUITTO_INC mosquitto.h)
if(MOSQUITTO_LIB AND MOSQUITTO_INC)
    set(MOSQUITTO_FOUND TRUE)
    list(APPEND SOURCES ${PROJECT_SOURCE_DIR}/mqtt.c)
    include_directories(${MOSQUITTO_INC})
endif()

# Optional ZMQ for JAERO-compatible audio output and raw IQ output
find_library(ZMQ_LIB zmq)
find_path(ZMQ_INC zmq.h)
if(ZMQ_LIB AND ZMQ_INC)
    set(ZMQ_FOUND TRUE)
    list(APPEND SOURCES
        ${PROJECT_SOURCE_DIR}/zmq_audio.c
        ${PROJECT_SOURCE_DIR}/zmq_iq.c
    )
    include_directories(${ZMQ_INC})
endif()

add_executable(inmarsat-sniffer ${SOURCES})

# Test tools
add_executable(test_demod ${PROJECT_SOURCE_DIR}/tools/test_demod.c ${JAERO_DSP_SOURCES} ${LIBCORRECT_SOURCES})
target_link_libraries(test_demod PRIVATE Threads::Threads m stdc++)
target_include_directories(test_demod PRIVATE ${PROJECT_SOURCE_DIR}/jaero_dsp ${PROJECT_SOURCE_DIR}/jaero_dsp/JFFT ${PROJECT_SOURCE_DIR}/jaero_dsp/libcorrect/include)

# Link required libraries
target_link_libraries(inmarsat-sniffer PRIVATE
    Threads::Threads
    m
    stdc++
   )

# Link SDR backends
if(SOAPYSDR_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${SOAPYSDR_LIBRARIES})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_SOAPYSDR)
    message(STATUS "SoapySDR: enabled")
else()
    message(STATUS "SoapySDR: not found")
endif()

if(SDRPLAY_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${SDRPLAY_LIBRARIES})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_SDRPLAY)
    message(STATUS "SDRplay: enabled")
else()
    message(STATUS "SDRplay: not found")
endif()

if(ZMQ_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${ZMQ_LIB})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_ZMQ)
    message(STATUS "ZMQ: enabled (JAERO audio + raw IQ output)")
else()
    message(STATUS "ZMQ: not found (ZMQ outputs disabled)")
endif()

if(RTLSDR_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${RTLSDR_LIBRARIES})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_RTLSDR)
    message(STATUS "RTL-SDR: enabled")
else()
    message(STATUS "RTL-SDR: not found")
endif()

if(HACKRF_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${HACKRF_LIB})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_HACKRF)
    message(STATUS "HackRF: enabled")
else()
    message(STATUS "HackRF: not found")
endif()

if(BLADERF_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${BLADERF_LIB})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_BLADERF)
    message(STATUS "BladeRF: enabled")
else()
    message(STATUS "BladeRF: not found")
endif()

if(UHD_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${UHD_LIB})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_UHD)
    message(STATUS "USRP (UHD): enabled")
else()
    message(STATUS "USRP (UHD): not found")
endif()

if(AIRSPY_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${AIRSPY_LIB})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_AIRSPY)
    message(STATUS "Airspy: enabled")
else()
    message(STATUS "Airspy: not found")
endif()

if(MOSQUITTO_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${MOSQUITTO_LIB})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_MQTT)
    message(STATUS "MQTT: enabled")
else()
    message(STATUS "MQTT: not found (install libmosquitto-dev)")
endif()

if(LIBACARS_FOUND)
    target_link_libraries(inmarsat-sniffer PRIVATE ${LIBACARS_LIBRARIES})
    target_include_directories(inmarsat-sniffer PRIVATE ${LIBACARS_INCLUDE_DIRS})
    target_compile_definitions(inmarsat-sniffer PRIVATE HAVE_LIBACARS)
    message(STATUS "libacars: enabled")
else()
    message(STATUS "libacars: not found (ACARS sublayer decoding disabled)")
endif()

set_property(TARGET inmarsat-sniffer PROPERTY C_STANDARD 99)
set_property(TARGET inmarsat-sniffer PROPERTY CXX_STANDARD 11)
set_property(TARGET inmarsat-sniffer PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

install(TARGETS inmarsat-sniffer DESTINATION bin)

# CPack packaging
set(CPACK_PACKAGE_NAME "inmarsat-sniffer")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Inmarsat L-band STD-C and Aero decoder")
set(CPACK_PACKAGE_VENDOR "CEMAXECUTER LLC")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "CEMAXECUTER LLC")
set(CPACK_DEBIAN_PACKAGE_SECTION "hamradio")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
include(CPack)
