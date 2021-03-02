// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "core/core_timing.h"

struct ARMul_State;

unsigned InterpreterMainLoop(ARMul_State* state, Core::Timing::Timer* timer);
