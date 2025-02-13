/**
 * @file ChVisualJawSystemVSG.h
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 16.08.2024
 */

/*
 * Copyright (c) 2025 Paul-Otto Müller
 *
 * https://github.com/paulotto/exosim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef EXOSIM_BIOMECHANICS_CH_VISUAL_JAW_SYSTEM_VSG_H
#define EXOSIM_BIOMECHANICS_CH_VISUAL_JAW_SYSTEM_VSG_H

#include <chrono_vsg/ChVisualSystemVSG.h>


namespace chrono::vsg3d {
    class ChVisualJawSystemVSG : public ChVisualSystemVSG {
        public:
            explicit ChVisualJawSystemVSG(int num_divs = 24)
                : ChVisualSystemVSG(num_divs) {
            }

            ~ChVisualJawSystemVSG() override = default;

            void BindAll(ChSystem* sys);

            void UnbindItem(std::shared_ptr<ChPhysicsItem> item) override;
    };
} // namespace chrono::vsg3d

#endif //EXOSIM_BIOMECHANICS_CH_VISUAL_JAW_SYSTEM_VSG_H
