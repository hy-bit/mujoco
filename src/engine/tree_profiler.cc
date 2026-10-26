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

#include "engine/tree_profiler.h"

#if MJ_ENABLE_TREE_PROFILER

#include <chrono>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>

namespace {

struct TreeProfileNode {
  const char* name{nullptr};
  double totalMs{0.0};
  int callCount{0};

  TreeProfileNode* parent{nullptr};
  std::vector<std::unique_ptr<TreeProfileNode>> children;

  TreeProfileNode* FindOrAddChild(const char* childName) {
    for (const auto& child : children) {
      if (child->name == childName ||
          (child->name && childName && std::strcmp(child->name, childName) == 0)) {
        return child.get();
      }
    }

    auto node = std::make_unique<TreeProfileNode>();
    node->name = childName;
    node->parent = this;
    TreeProfileNode* raw = node.get();
    children.push_back(std::move(node));
    return raw;
  }
};

class TreeProfiler {
 public:
  TreeProfiler() { m_root.name = "root"; }

  TreeProfiler(const TreeProfiler&) = delete;
  TreeProfiler& operator=(const TreeProfiler&) = delete;

  static TreeProfiler& Instance() {
    if (!g_instance) {
      g_instance = new TreeProfiler();
      g_instance->enabled = true;
    }
    return *g_instance;
  }

  static TreeProfiler* TryInstance() { return g_instance; }

  void Start(const char* name) {
    if (!enabled) {
      return;
    }
    TreeProfileNode* parent = m_stack.empty() ? &m_root : m_stack.back().node;
    TreeProfileNode* child = parent->FindOrAddChild(name);
    m_stack.push_back(ActiveFrame{child, NowMs()});
  }

  void End() {
    if (!enabled || m_stack.empty()) {
      return;
    }
    ActiveFrame& frame = m_stack.back();
    const double elapsed = NowMs() - frame.startMs;
    frame.node->totalMs += elapsed;
    frame.node->callCount += 1;
    m_stack.pop_back();
  }

  void Reset() {
    m_stack.clear();
    m_root.children.clear();
    m_root.totalMs = 0.0;
    m_root.callCount = 0;
  }

  std::string FormatReport() const {
    constexpr int kNameWidth = 36;

    std::ostringstream oss;
    oss << "=== TreeProfiler report ===\n";
    oss << std::left << std::setw(kNameWidth) << "name"
        << std::right << std::setw(14) << "total_ms"
        << std::setw(10) << "calls"
        << std::setw(14) << "avg_ms"
        << std::setw(10) << "pct"
        << '\n';

    double baseTotalMs = 0.0;
    for (const auto& child : m_root.children) {
      if (child->callCount > 0) {
        baseTotalMs += child->totalMs;
      }
    }

    std::string body;
    for (const auto& child : m_root.children) {
      if (child->callCount > 0) {
        FormatNode(body, *child, 0, baseTotalMs, 0.0, kNameWidth);
      }
    }

    return oss.str() + body;
  }

  void DumpReport(FILE* out) const {
    if (!out) {
      out = stdout;
    }
    const std::string report = FormatReport();
    std::fputs(report.c_str(), out);
    std::fflush(out);
  }

  bool enabled{false};

 private:
  static double NowMs() {
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration<double, std::milli>(now).count();
  }

  struct ActiveFrame {
    TreeProfileNode* node{nullptr};
    double startMs{0.0};
  };

  void FormatNode(std::string& out, const TreeProfileNode& node, int depth,
                  double baseTotalMs, double parentTotalMs, int nameWidth) const {
    if (node.callCount <= 0) {
      return;
    }

    const double totalMs = node.totalMs;
    const double avgMs = totalMs / static_cast<double>(node.callCount);
    const double pct = (baseTotalMs > 0.0) ? 100.0 * totalMs / baseTotalMs : 0.0;

    std::ostringstream line;
    std::string displayName =
        std::string(static_cast<size_t>(2 * depth), ' ') + (node.name ? node.name : "");
    if (depth > 0 && totalMs > parentTotalMs + 1e-9) {
      displayName += " [!]";
    }
    line << std::left << std::setw(nameWidth) << displayName
         << std::right << std::fixed << std::setprecision(3) << std::setw(14) << totalMs
         << std::setw(10) << node.callCount
         << std::setprecision(5) << std::setw(14) << avgMs
         << std::setprecision(2) << std::setw(9) << pct << '%'
         << '\n';
    out += line.str();

    for (const auto& child : node.children) {
      if (child->callCount > 0) {
        FormatNode(out, *child, depth + 1, baseTotalMs, totalMs, nameWidth);
      }
    }
  }

  TreeProfileNode m_root;
  std::vector<ActiveFrame> m_stack;

  static TreeProfiler* g_instance;
};

TreeProfiler* TreeProfiler::g_instance{nullptr};

}  // namespace

extern "C" {

void mj_treeProfilerStart(const char* name) {
  TreeProfiler::Instance().Start(name);
}

void mj_treeProfilerEnd(void) {
  if (TreeProfiler* p = TreeProfiler::TryInstance()) {
    p->End();
  }
}

void mj_treeProfilerDump(void) {
  if (TreeProfiler* p = TreeProfiler::TryInstance()) {
    p->DumpReport(stdout);
  }
}

void mj_treeProfilerReset(void) {
  if (TreeProfiler* p = TreeProfiler::TryInstance()) {
    p->Reset();
  }
}

}  // extern "C"

#else  // !MJ_ENABLE_TREE_PROFILER

extern "C" {

void mj_treeProfilerStart(const char* name) { (void)name; }
void mj_treeProfilerEnd(void) {}
void mj_treeProfilerDump(void) {}
void mj_treeProfilerReset(void) {}

}  // extern "C"

#endif  // MJ_ENABLE_TREE_PROFILER
