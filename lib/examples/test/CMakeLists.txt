#
# Copyright 2013-2023 Software Radio Systems Limited
#
# This file is part of srsRAN
#
# srsRAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsRAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

########################################################################
# NPDSCH_ENODEB/UE TESTS 
########################################################################

SET(CTEST_OUTPUT_ON_FAILURE TRUE)

# Run eNB and generate DL signal
# - we generate 512 DL frames
# - schedulingInfoSIB1-NB-r13 is 2, so 16 SIB repetitions are used
# - each frame contains a NPDCCH and NPDSCH with TBS=24 for RNTI 0x1234
set(ARG "-o;/tmp/enbtest1.bin;-n;512;-t;2")
add_test(NAME enb1
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_enodeb> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)

# Run npdsch_npdcch file tests

# Try to decode SIB (-k)
set(ARG "-i;/tmp/enbtest1.bin;-m;512;-t;2;-w;0;-k")
add_test(NAME npdsch_npdcch_file1
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_npdcch_file_test> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
# Specify test, SIB1 should be decoded once
set_property(TEST npdsch_npdcch_file1 PROPERTY PASS_REGULAR_EXPRESSION "pkt_ok=1")
set_property(TEST npdsch_npdcch_file1 APPEND PROPERTY DEPENDS enb1)


# Try to decode NPDCCH+NPDSCH for user
set(ARG "-i;/tmp/enbtest1.bin;-m;512;-t;2;-w;0;-r;0x1234")
add_test(NAME npdsch_npdcch_file2
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_npdcch_file_test> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
# Specify test
set_property(TEST npdsch_npdcch_file2 PROPERTY PASS_REGULAR_EXPRESSION "pkt_ok=512")
set_property(TEST npdsch_npdcch_file2 APPEND PROPERTY DEPENDS enb1)

# Run eNB and generate DL signal
# - we generate 10 DL frames
# - schedulingInfoSIB1-NB-r13 is 0, so 4 SIB repetitions are used
# - i_sf is 1, so two subframes
# - i_rep is 2, so four repetitions
# - i_mcs is 4, so TBS of 120
# - each NPDSCH lasts over 8 subframes in total, starting in sf_idx,
#   so in total, two frames are needed for NPDCCH+NPDSCH, hence, 10/2=5 frames
#   should be received
set(ARG "-o;/tmp/enbtest2.bin;-n;10;-t;0;-i;1;-m;4;-r;2")
add_test(NAME enb2
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_enodeb> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)

# Try to decode NPDCCH+NPDSCH for user
set(ARG "-i;/tmp/enbtest2.bin;-m;512;-w;0;-r;0x1234")
add_test(NAME npdsch_npdcch_file3
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_npdcch_file_test> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
# Specify test
set_property(TEST npdsch_npdcch_file3 PROPERTY PASS_REGULAR_EXPRESSION "pkt_ok=5")
set_property(TEST npdsch_npdcch_file3 APPEND PROPERTY DEPENDS enb2)

########################################################################
# IN-BAND NB-IoT: NB-IoT anchor PRB 17 inside a 25 PRB LTE carrier (PCI 1),
# standard LTE sample rates (7.68 MS/s), as in a real srsenb deployment.
#  -N 25 LTE PRBs, -M 0 inband-samePCI, -c 1 PCI, -p 17 anchor PRB, -S standard rates
########################################################################

set(ARG "-o;/tmp/enbtest_inband.bin;-n;64;-t;2;-N;25;-M;0;-c;1;-p;17;-S")
add_test(NAME enb_inband
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_enodeb> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)

# SIB1 must be decoded once
set(ARG "-i;/tmp/enbtest_inband.bin;-m;64;-t;2;-w;0;-S;-n;25;-M;0;-I;1;-P;17;-k")
add_test(NAME npdsch_npdcch_file_inband_sib1
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_npdcch_file_test> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
set_property(TEST npdsch_npdcch_file_inband_sib1 PROPERTY PASS_REGULAR_EXPRESSION "pkt_ok=1")
set_property(TEST npdsch_npdcch_file_inband_sib1 APPEND PROPERTY DEPENDS enb_inband)

# NPDCCH + NPDSCH for the user in every one of the 64 frames
set(ARG "-i;/tmp/enbtest_inband.bin;-m;64;-t;2;-w;0;-S;-n;25;-M;0;-I;1;-P;17;-r;0x1234")
add_test(NAME npdsch_npdcch_file_inband_user
         COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_npdcch_file_test> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
set_property(TEST npdsch_npdcch_file_inband_user PROPERTY PASS_REGULAR_EXPRESSION "pkt_ok=64")
set_property(TEST npdsch_npdcch_file_inband_user APPEND PROPERTY DEPENDS enb_inband)

# Negative controls: a decoder looking at the wrong PRB (a far one, and the immediate neighbour, which is where in-band
# data used to leak) or the wrong PCI must decode nothing. These prove the round trip really depends on the placement.
foreach(NEG "wrongprb7;-P;7;-I;1" "wrongprb18;-P;18;-I;1" "wrongpci;-P;17;-I;2")
  list(GET NEG 0 NEG_NAME)
  list(REMOVE_AT NEG 0)
  set(ARG "-i;/tmp/enbtest_inband.bin;-m;64;-t;2;-w;0;-S;-n;25;-M;0;-r;0x1234;${NEG}")
  add_test(NAME npdsch_npdcch_file_inband_neg_${NEG_NAME}
           COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_npdcch_file_test> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
  set_property(TEST npdsch_npdcch_file_inband_neg_${NEG_NAME} PROPERTY PASS_REGULAR_EXPRESSION "pkt_ok=0")
  set_property(TEST npdsch_npdcch_file_inband_neg_${NEG_NAME} APPEND PROPERTY DEPENDS enb_inband)
endforeach()

########################################################################
# NB-IoT system information builder: MIB-NB / SIB1-NB from the carrier config
########################################################################
if(LIBCONFIGPP_FOUND)
  add_executable(nbiot_sib_test nbiot_sib_test.cc)
  target_link_libraries(nbiot_sib_test nbiot_sib_builder)
  add_test(NAME nbiot_sib COMMAND nbiot_sib_test)
  set_property(TEST nbiot_sib PROPERTY PASS_REGULAR_EXPRESSION "0 failed")

  # End to end: build SIB1-NB from the carrier config -> broadcast it in-band as IQ -> decode it from the IQ ->
  # the recovered transport block must be byte-identical to the one that was built.
  # Remove any leftovers first so the comparison at the end can never pass on a stale file from an earlier run.
  add_test(NAME nbiot_sib1_clean
           COMMAND ${CMAKE_COMMAND} -E remove -f /tmp/nbiot_sib1_test.bin /tmp/nbiot_sib2_test.bin /tmp/nbiot_sib1_rx.bin /tmp/enbtest_inband_sib1.bin)
  add_test(NAME nbiot_sib1_build
           COMMAND nbiot_sib_pack -c ${CMAKE_CURRENT_SOURCE_DIR}/nbiot_inband.conf -o /tmp/nbiot_sib1_test.bin -s /tmp/nbiot_sib2_test.bin)
  set_property(TEST nbiot_sib1_build PROPERTY PASS_REGULAR_EXPRESSION "SIB2-NB: [0-9]+ bytes.*wrote 26 bytes.*wrote 26 bytes")
  set_property(TEST nbiot_sib1_build APPEND PROPERTY DEPENDS nbiot_sib1_clean)

  # The standalone decoder must read back what was written: SIB2-NB inside a SystemInformation-NB message.
  add_test(NAME nbiot_sib2_decode COMMAND nbiot_sib_pack -d /tmp/nbiot_sib2_test.bin)
  set_property(TEST nbiot_sib2_decode PROPERTY PASS_REGULAR_EXPRESSION "systemInformation-r13.*sib2-r13.*nprach-ParametersList-r13")
  set_property(TEST nbiot_sib2_decode APPEND PROPERTY DEPENDS nbiot_sib1_build)

  set(ARG "-o;/tmp/enbtest_inband_sib1.bin;-n;64;-t;0;-N;25;-M;0;-c;1;-p;17;-S;-B;/tmp/nbiot_sib1_test.bin")
  add_test(NAME enb_inband_sib1
           COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_enodeb> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
  set_property(TEST enb_inband_sib1 APPEND PROPERTY DEPENDS nbiot_sib1_build)

  set(ARG "-i;/tmp/enbtest_inband_sib1.bin;-m;64;-t;0;-w;0;-S;-n;25;-M;0;-I;1;-P;17;-k;-D;/tmp/nbiot_sib1_rx.bin")
  add_test(NAME npdsch_npdcch_file_inband_sib1_dump
           COMMAND ${CMAKE_COMMAND} -DCMD=$<TARGET_FILE:npdsch_npdcch_file_test> "-DARG=${ARG}" -V -P ${CMAKE_CURRENT_SOURCE_DIR}/iqtests.cmake)
  set_property(TEST npdsch_npdcch_file_inband_sib1_dump PROPERTY PASS_REGULAR_EXPRESSION "sib1_dumped=1")
  set_property(TEST npdsch_npdcch_file_inband_sib1_dump APPEND PROPERTY DEPENDS enb_inband_sib1)

  add_test(NAME nbiot_sib1_content_match
           COMMAND ${CMAKE_COMMAND} -E compare_files /tmp/nbiot_sib1_test.bin /tmp/nbiot_sib1_rx.bin)
  set_property(TEST nbiot_sib1_content_match APPEND PROPERTY DEPENDS npdsch_npdcch_file_inband_sib1_dump)

  # A SIB1-NB file whose size does not match schedulingInfoSIB1 must be refused, not transmitted truncated/padded.
  add_test(NAME enb_inband_sib1_wrong_size
           COMMAND npdsch_enodeb -o /tmp/enbtest_inband_sib1_bad.bin -n 4 -t 3 -N 25 -M 0 -c 1 -p 17 -S -B /tmp/nbiot_sib1_test.bin)
  set_property(TEST enb_inband_sib1_wrong_size PROPERTY PASS_REGULAR_EXPRESSION "carries 41 bytes")
  set_property(TEST enb_inband_sib1_wrong_size APPEND PROPERTY DEPENDS nbiot_sib1_build)
endif(LIBCONFIGPP_FOUND)

# The encoder and decoder share their RE mapping, so the round trip above cannot detect a symmetrically wrong mapping.
# This check re-derives the expected placement from the specifications with an independent FFT (needs python3+numpy).
find_program(NBIOT_PYTHON NAMES python3 python)
set(NBIOT_HAVE_NUMPY FALSE)
if(NBIOT_PYTHON)
  execute_process(COMMAND ${NBIOT_PYTHON} -c "import numpy" RESULT_VARIABLE NBIOT_NUMPY_RC OUTPUT_QUIET ERROR_QUIET)
  if(NBIOT_NUMPY_RC EQUAL 0)
    set(NBIOT_HAVE_NUMPY TRUE)
  endif()
endif()
if(NBIOT_HAVE_NUMPY)
  add_test(NAME nbiot_inband_geometry
           COMMAND ${NBIOT_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/nbiot_inband_geometry.py /tmp/enbtest_inband.bin --prb 17 --pci 1)
  set_property(TEST nbiot_inband_geometry PROPERTY PASS_REGULAR_EXPRESSION "RESULT: PASS")
  set_property(TEST nbiot_inband_geometry APPEND PROPERTY DEPENDS enb_inband)
else()
  message(WARNING "python3 with numpy not found: the independent in-band geometry test (nbiot_inband_geometry) is NOT "
                  "registered. The in-band decode round trips alone cannot detect a symmetric RE-mapping error.")
endif()