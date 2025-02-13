/**
 * @file ChJawVSG.cpp
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

#include "exosim/biomechanics/vis/ChJawVSG.h"

#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>


namespace chrono::vsg3d {
    void ChResetJawModelVSG::render() {
        ImGui::SetNextWindowPos(ImVec2(64, 264), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(244, 196), ImGuiCond_FirstUseEver);

        ImGui::Begin("Tools");
        if (ImGui::Button("Reset")) {
            // // const auto vis_sys = jaw_->System()->GetVisualSystem();
            // jaw_->Build();
            // // AttachSystem needs to be invoked as ChSystem::visual_system is protected.
            // app_->AttachSystem(jaw_->System().get());
            // app_->GetSystems()[0] = jaw_->System().get();
            // // app_->Initialize();
            // app_->BindAll(jaw_->System().get());
            // // app_->GetGuiComponent(0)->render();
            jaw_->ResetSimulation();
        }
        ImGui::End();
    }

    void ChStartStopJawSimVSG::render() {
        ImGui::Begin("Tools");
        if (ImGui::Button(update_simulation_ ? "Pause" : "Start")) {
            update_simulation_ = !update_simulation_;
        }
        ImGui::End();
    }

    void ChRewindJawSimVSG::render() {
        const double buffer_duration = jaw_->TimeStep() * static_cast<double>(jaw_->ReplayBuffer().size());

        ImGui::Begin("Tools");

        static float rewind_time{0.0f};
        ImGui::SliderFloat("Rewind", &rewind_time, -static_cast<float>(buffer_duration), 0, "%.3f s",
                           ImGuiSliderFlags_AlwaysClamp);

        if (!jaw_->UpdateSimulation() && rewind_time_previous_ != rewind_time) {
            double rewind_percentage = -rewind_time / buffer_duration * 100.0;
            rewind_percentage = std::max(0.0, std::min(100.0, rewind_percentage));
            jaw_->RewindSimulation(rewind_percentage);
        }

        if (jaw_->UpdateSimulation()) rewind_time = 0.0f;

        rewind_time_previous_ = rewind_time;

        ImGui::End();
    }

    void ChToggleJawVisibilityVSG::render() {
        ImGui::Begin("Tools");

        if (ImGui::TreeNode("Model Components")) {
            static bool skull_visible = true;
            static bool maxilla_visible = true;
            static bool mandible_visible = true;
            static bool hyoid_visible = true;

            if (ImGui::Checkbox("Skull", &skull_visible)) {
                ToggleVisibility("skull", skull_visible);
            }
            if (ImGui::Checkbox("Maxilla", &maxilla_visible)) {
                ToggleVisibility("maxilla", maxilla_visible);
            }
            if (ImGui::Checkbox("Mandible", &mandible_visible)) {
                ToggleVisibility("mandible", mandible_visible);
            }
            if (ImGui::Checkbox("Hyoid", &hyoid_visible)) {
                ToggleVisibility("hyoid", hyoid_visible);
            }

            ImGui::TreePop();
        }
        ImGui::End();
    }

    void ChToggleJawVisibilityVSG::ToggleVisibility(const std::string& component_name, bool is_visible) const {
        const auto& sys = app_->GetSystem(0);
        const auto& bodies = sys.GetBodies();
        const auto it = std::find_if(bodies.begin(), bodies.end(), [&component_name](const auto& body) {
            return body->GetName() == component_name;
        });

        if (it == bodies.end()) {
            std::cerr << "[ERROR] Component not found: " << component_name << std::endl;
            return;
        }

        if (is_visible) {
            app_->BindItem(*it);
        } else {
            app_->UnbindItem(*it);
        }
    }

    void ChJawMuscleExcActVSG::render() {
        ImGui::SetNextWindowPos(ImVec2(1240, 686), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(670, 380), ImGuiCond_FirstUseEver);

        ImGui::Begin("Muscle Excitation and Activation");

        static float history{10.0f};
        ImGui::SliderFloat("History", &history, 1, 60, "%.1f s");
        // ImGui::SameLine();
        // ImGui::InputFloat("##HistoryInput", &history, 0.0f, 0.0f, "%.1f s");

        x_span_roll_ = history;
        // x_max_size_scroll = static_cast<int>(history / jaw_->TimeStep());

        if (ImGui::Button(rolling_ ? "Scrolling" : "Rolling")) {
            rolling_ = !rolling_;
        }

        static float t{0.0f};
        t = static_cast<float>(jaw_->System()->GetChTime());

        const auto& muscle_map = jaw_->MuscleMap();
        int i{0};

        for (auto& [name, buffer]: rolling_buffers_) {
            buffer.span = x_span_roll_;
            std::array<float, 3> elem{
                t, muscle_map.at(name)->GetCurrentExcitation(), muscle_map.at(name)->GetCurrentActivation()
            };
            // buffer.AddPoint(elem);

            // if (buffer.FindElement(elem) == -1) {
            //     buffer.AddPoint(elem);
            // }
            if (auto latest = buffer.GetLatestValue(); latest.has_value()) {
                if (latest.value()[0] < elem[0]) {
                    buffer.AddPoint(elem);
                } else {
                    buffer.RemoveElementsFrom(elem);
                    buffer.AddPoint(elem);
                }
            } else if (buffer.data.empty()) {
                buffer.AddPoint(elem);
            }

            std::string exc_id = "Excitation##" + std::to_string(i);
            std::string act_id = "Activation##" + std::to_string(i++);

            if (rolling_ && ImPlot::BeginPlot(name.c_str(), ImVec2(-1, 150))) {
                ImPlot::SetupAxes("Time [s]", nullptr, 0, 0);
                ImPlot::SetupAxisLimits(ImAxis_X1, buffer.data[0][0], buffer.data[0][0] + history,
                                        ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 1.0);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
                ImPlot::PlotShaded(act_id.c_str(), &buffer.data[0][0], &buffer.data[0][2],
                                   static_cast<int>(buffer.data.size()), 0, 0, 0, 3 * sizeof(float));
                ImPlot::PlotLine(exc_id.c_str(), &buffer.data[0][0], &buffer.data[0][1],
                                 static_cast<int>(buffer.data.size()), 0, 0, 3 * sizeof(float));
                ImPlot::EndPlot();
            }
        }

        i = 0;
        for (auto& [name, buffer]: scrolling_buffers_) {
            std::array<float, 3> elem{
                t, muscle_map.at(name)->GetCurrentExcitation(), muscle_map.at(name)->GetCurrentActivation()
            };
            // buffer.AddPoint(elem);

            // if (buffer.FindElement(elem) == -1) {
            //     buffer.AddPoint(elem);
            // }

            if (auto latest = buffer.GetLatestValue(); latest.has_value()) {
                if (latest.value()[0] < elem[0]) {
                    buffer.AddPoint(elem);
                }
            } else if (buffer.data.empty()) {
                buffer.AddPoint(elem);
            }

            std::string exc_id = "Excitation##" + std::to_string(i);
            std::string act_id = "Activation##" + std::to_string(i++);

            if (!rolling_ && ImPlot::BeginPlot(name.c_str(), ImVec2(-1, 150))) {
                ImPlot::SetupAxes("Time [s]", nullptr, 0, 0);
                ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 1.0);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
                ImPlot::PlotShaded(act_id.c_str(), &buffer.data[0][0], &buffer.data[0][2],
                                   static_cast<int>(buffer.data.size()), 0, 0, buffer.offset,
                                   3 * sizeof(float));
                ImPlot::PlotLine(exc_id.c_str(), &buffer.data[0][0], &buffer.data[0][1],
                                 static_cast<int>(buffer.data.size()), 0, buffer.offset, 3 * sizeof(float));
                ImPlot::EndPlot();
            }
        }

        ImGui::End();
    }
} // namespace chrono::vsg3d
