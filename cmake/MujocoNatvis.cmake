# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if(MSVC)
  set(MUJOCO_NATVIS_FILES
    ${CMAKE_SOURCE_DIR}/mjdata.natvis
    ${CMAKE_SOURCE_DIR}/mjmodel.natvis
    ${CMAKE_SOURCE_DIR}/mjoption.natvis
    ${CMAKE_SOURCE_DIR}/mjCGContext.natvis
  )

  function(mujoco_target_add_natvis target)
    if(CMAKE_GENERATOR MATCHES "Visual Studio")
      target_sources(${target} PRIVATE ${MUJOCO_NATVIS_FILES})
    else()
      foreach(_natvis IN LISTS MUJOCO_NATVIS_FILES)
        target_link_options(${target} PRIVATE "/NATVIS:${_natvis}")
      endforeach()
    endif()
  endfunction()
endif()
