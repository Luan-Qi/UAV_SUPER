-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

--- @file mid360_3d_localization.lua
--- @brief Cartographer 3D 纯定位配置文件（Livox Mid-360 LiDAR）。
---
--- 该文件继承 mid360_3d.lua 的全部建图参数，并在此基础上增加
--- 纯定位模式（pure localization）专用配置。纯定位模式下，
--- Cartographer 加载预先建立的 .pbstream 地图文件，不再创建新子图，
--- 而是将当前激光扫描与已有地图进行匹配以估计位姿。
---
--- @section localization_params 纯定位特有参数
--- - pure_localization_trimmer.max_submaps_to_keep = 3 :
---   保持活跃的子图数量上限为 3 个。纯定位不需要保留大量历史子图，
---   限制子图数量可以显著降低内存占用和计算开销。
--- - POSE_GRAPH.optimize_every_n_nodes = 100 :
---   相比建图模式（320），定位模式下更频繁地触发位姿图优化
---   （每 100 个节点一次），以获得更平滑的定位轨迹。
---
--- @see mid360_3d.lua 基础建图配置。
---
--- @author 鸾棋
--- @date   2025

include "mid360_3d.lua"

--- 纯定位修整器：仅保留最近 3 个子图以降低内存和计算开销。
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}

--- 定位模式下每 100 个节点触发一次优化，比建图模式更频繁。
POSE_GRAPH.optimize_every_n_nodes = 100

return options
