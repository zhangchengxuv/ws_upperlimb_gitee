// Copyright 2024 Zhang Chenxu
// Licensed under the Apache License, Version 2.0 (the "License");      
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  * 负责实现各控制模式的算法逻辑

#include "upperlimb_robot/control_algorithms.hpp"
#include <iostream>

// ===================== 各控制模式实现 =====================

void runLeftPresetPosition(const MotorStateCache &state)
{
    (void)state;
    // TODO: 左侧回归预设位置
}

void runRightPresetPosition(const MotorStateCache &state)
{
    (void)state;
    // TODO: 右侧回归预设位置
}
void runIdle(const MotorStateCache &state)
{
    (void)state;
    // TODO: 待机模式
}

void runPassiveTraining(const MotorStateCache &state)
{
    (void)state;
    // TODO: 被动训练算法
}

void runZeroForceMode(const MotorStateCache &state)
{
    (void)state;
    // TODO: 零力控制
}

void runBilateralMode(const MotorStateCache &state)
{
    (void)state;
    // TODO: 双侧协同控制
}

void runSlidingModeControl(const MotorStateCache &state)
{
    (void)state;
    // TODO: 滑模控制
}

