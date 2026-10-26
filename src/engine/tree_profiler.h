// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing the License.

#ifndef MUJOCO_SRC_ENGINE_TREE_PROFILER_H_
#define MUJOCO_SRC_ENGINE_TREE_PROFILER_H_

#include <mujoco/mjexport.h>

#ifdef __cplusplus
extern "C" {
#endif

// Hierarchical tree profiler C ABI. No-ops when MJ_ENABLE_TREE_PROFILER is 0.
MJAPI void mj_treeProfilerStart(const char* name);
MJAPI void mj_treeProfilerEnd(void);
MJAPI void mj_treeProfilerDump(void);
MJAPI void mj_treeProfilerReset(void);

#ifdef __cplusplus
}  // extern "C"
#endif

// Default off. Set to 1 and rebuild mujoco.dll to enable hierarchical timing.
#define MJ_ENABLE_TREE_PROFILER 1

#if MJ_ENABLE_TREE_PROFILER

#define PROFILE_START(name) mj_treeProfilerStart(#name)
#define PROFILE_END() mj_treeProfilerEnd()

#ifdef __cplusplus
class TreeProfileScope {
 public:
  explicit TreeProfileScope(const char* name) { mj_treeProfilerStart(name); }
  ~TreeProfileScope() { mj_treeProfilerEnd(); }

  TreeProfileScope(const TreeProfileScope&) = delete;
  TreeProfileScope& operator=(const TreeProfileScope&) = delete;
};

#define PROFILE_SCOPE(name) TreeProfileScope profile_scope_##name(#name)
#else
#define PROFILE_SCOPE(name) ((void)0)
#endif

#else  // !MJ_ENABLE_TREE_PROFILER

#define PROFILE_START(name) ((void)0)
#define PROFILE_END() ((void)0)
#define PROFILE_SCOPE(name) ((void)0)

#endif  // MJ_ENABLE_TREE_PROFILER

#endif  // MUJOCO_SRC_ENGINE_TREE_PROFILER_H_
