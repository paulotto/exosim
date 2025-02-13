/**
 * @file ChJawVSG.h
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

#ifndef EXOSIM_BIOMECHANICS_CH_JAW_VSG_H
#define EXOSIM_BIOMECHANICS_CH_JAW_VSG_H

#include "exosim/biomechanics/ChJaw.h"
#include "exosim/biomechanics/vis/ChVisualJawSystemVSG.h"
#include "exosim/biomechanics/vis/ImPlotUtils.h"

#include <chrono_vsg/ChGuiComponentVSG.h>
#include <chrono_vsg/ChVisualSystemVSG.h>


namespace chrono::vsg3d {
    class ChResetJawModelVSG : public ChGuiComponentVSG {
        public:
            explicit ChResetJawModelVSG(biomechanics::ChJaw* jaw, ChVisualJawSystemVSG* app)
                : jaw_(jaw), app_(app) {
            }

            ~ChResetJawModelVSG() override = default;

            /**
             * Specify the ImGui elements to be rendered for this GUI component.
             */
            void render() override;

        private:
            biomechanics::ChJaw* jaw_;
            ChVisualJawSystemVSG* app_;
    };

    class ChStartStopJawSimVSG : public ChGuiComponentVSG {
        public:
            explicit ChStartStopJawSimVSG(ChVisualJawSystemVSG* app, bool update_simulation = false)
                : update_simulation_(update_simulation), app_(app) {
            }

            ~ChStartStopJawSimVSG() override = default;

            /**
             * Specify the ImGui elements to be rendered for this GUI component.
             */
            void render() override;

            bool UpdateSimulation() const { return update_simulation_; }

        private:
            bool update_simulation_;

            ChVisualJawSystemVSG* app_;
    };

    class ChRewindJawSimVSG : public ChGuiComponentVSG {
        public:
            explicit ChRewindJawSimVSG(biomechanics::ChJaw* jaw, ChVisualJawSystemVSG* app)
                : jaw_(jaw), app_(app) {
            }

            ~ChRewindJawSimVSG() override = default;

            /**
             * Specify the ImGui elements to be rendered for this GUI component.
             */
            void render() override;

        private:
            float rewind_time_previous_{0.0f};

            biomechanics::ChJaw* jaw_;
            ChVisualJawSystemVSG* app_;
    };

    class ChToggleJawVisibilityVSG : public ChGuiComponentVSG {
        public:
            explicit ChToggleJawVisibilityVSG(ChVisualJawSystemVSG* app)
                : app_(app) {
            }

            ~ChToggleJawVisibilityVSG() override = default;

            /**
             * Specify the ImGui elements to be rendered for this GUI component.
             */
            void render() override;

            void ToggleVisibility(const std::string& component_name, bool is_visible) const;

        private:
            ChVisualJawSystemVSG* app_;
    };

    class ChJawMuscleExcActVSG : public ChGuiComponentVSG {
        public:
            ChJawMuscleExcActVSG(biomechanics::ChJaw* jaw, ChVisualJawSystemVSG* app, bool rolling = true)
                : rolling_(rolling), jaw_(jaw), app_(app) {
                num_plots_ = jaw_->MuscleMap().size();
                show_plots_.resize(num_plots_, true);

                for (const auto& [name, muscle]: jaw_->MuscleMap()) {
                    rolling_buffers_.emplace(std::piecewise_construct,
                                             std::forward_as_tuple(name),
                                             std::forward_as_tuple(x_span_roll_, 10000));

                    scrolling_buffers_.emplace(name, x_max_size_scroll);
                }
            }

            ~ChJawMuscleExcActVSG() override = default;

            /**
             * Specify the ImGui elements to be rendered for this GUI component.
             */
            void render() override;

        private:
            bool rolling_{false}; ///< Flag indicating whether to use a rolling or scrolling buffer.

            float x_span_roll_{10.0f};
            int x_max_size_scroll{60000};
            int num_plots_{0}; ///< Number of plots to display.
            int history_min_{1}; ///< Minimum history length (in seconds).
            int history_max_{60}; ///< Maximum history length (in seconds).

            biomechanics::ChJaw* jaw_;
            ChVisualJawSystemVSG* app_;

            std::vector<bool> show_plots_{};

            exosim::ImPlot::RollingVecBuffer time_roll_buffer_{x_span_roll_};
            exosim::ImPlot::ScrollingVecBuffer time_scroll_buffer_{x_max_size_scroll};

            std::map<std::string, exosim::ImPlot::RollingBuffer<3> > rolling_buffers_;
            std::map<std::string, exosim::ImPlot::ScrollingBuffer<3> > scrolling_buffers_;
    };
} // namespace chrono::vsg3d

#endif //EXOSIM_BIOMECHANICS_CH_JAW_VSG_H
