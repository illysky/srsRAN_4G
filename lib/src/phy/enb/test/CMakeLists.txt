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

# NB-IoT downlink composer on top of a real LTE downlink, against an independent transmitter written from the spec
add_executable(enb_dl_nbiot_dump enb_dl_nbiot_dump.c)
target_link_libraries(enb_dl_nbiot_dump srsran_phy)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  add_test(NAME enb_dl_nbiot_python
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/enb_dl_nbiot_check.py
                   $<TARGET_FILE:enb_dl_nbiot_dump> ${CMAKE_CURRENT_SOURCE_DIR}/../../sync/test
                   ${CMAKE_CURRENT_SOURCE_DIR}/../../phch/test)
  set_tests_properties(enb_dl_nbiot_python PROPERTIES LABELS "lib;phy;enb")
endif()
