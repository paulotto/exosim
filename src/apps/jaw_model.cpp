/**
 * @file jaw_model.cpp
 * @brief Biomechanical model of the human jaw.
 * @author Paul-Otto Müller
 * @date 01.02.2025
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

#include <chrono>
#include <iostream>

#include "ExoSimConfig.h"
#include "exosim/biomechanics/ChJaw.h"
#include "exosim/biomechanics/utils/utils.h"

#include <chrono/functions/ChFunctionSine.h>
#include <chrono/functions/ChFunctionConst.h>

#include <chrono/serialization/ChArchive.h>
#include <chrono/serialization/ChArchiveJSON.h>

using namespace chrono;

const std::string EXOSIM_RESOURCES_DIR_STR{EXOSIM_RESOURCES_DIR};

int main(int argc, char* argv[]) {
    const auto jaw_json = std::make_shared<std::string>(EXOSIM_RESOURCES_DIR_STR + "/json/jaw_2/jaw.json");

    // Rigid model: thread=~1 --> best performance.
    biomechanics::ChJaw jaw(1);
    // Run Build() before LoadFromJSON() to set all parameters to a valid state.
    jaw.Build();
    jaw.LoadFromJSON(*jaw_json);
    jaw.Build();

    std::string info;
    jaw.InfoString(info);
    std::cout << info;

    // Set muscle excitations
    for (const auto& m_prop: jaw.MuscleProperties()) {
        const auto muscle = jaw.MuscleMap().at(m_prop.name);

        // Opening
        if (m_prop.name == "geniohyoid_left" || m_prop.name == "geniohyoid_right" ||
            m_prop.name == "anterior_digastric_left" || m_prop.name == "anterior_digastric_right" ||
            m_prop.name == "posterior_mylohyoid_left" || m_prop.name == "posterior_mylohyoid_right" ||
            m_prop.name == "anterior_mylohyoid_left" || m_prop.name == "anterior_mylohyoid_right") {
            muscle->SetExcitation(chrono_types::make_shared<ChFunctionSine>(0.5, 2.0, 0.0)); // Peck: 0.5, Millard: 0.5
            // muscle->SetExcitation(chrono_types::make_shared<ChFunctionConst>(0.0));
        }

        if ( //m_prop.name == "superior_lateral_pterygoid_left" ||
            //m_prop.name == "superior_lateral_pterygoid_right" ||
            m_prop.name == "inferior_lateral_pterygoid_left" ||
            m_prop.name == "inferior_lateral_pterygoid_right") {
            muscle->SetExcitation(chrono_types::make_shared<ChFunctionSine>(1.0, 2.0, 0.0)); // Peck: 0.85, Millard: 1.0
            // muscle->SetExcitation(chrono_types::make_shared<ChFunctionConst>(0.0));
        }

        // Closing
        if (m_prop.name == "superficial_masseter_left" || m_prop.name == "superficial_masseter_right" ||
            m_prop.name == "deep_masseter_left" || m_prop.name == "deep_masseter_right" ||
            m_prop.name == "posterior_temporalis_left" || m_prop.name == "posterior_temporalis_right" ||
            m_prop.name == "medial_temporalis_left" || m_prop.name == "medial_temporalis_right" ||
            m_prop.name == "anterior_temporalis_left" || m_prop.name == "anterior_temporalis_right") {
            muscle->SetExcitation(chrono_types::make_shared<ChFunctionSine>(0.5, 2.0, -CH_PI));
            // Peck: 0.15, Millard: 0.5
            // muscle->SetExcitation(chrono_types::make_shared<ChFunctionConst>(0.0));
        }
    }

    const ChFrameMoving mandible_init{
        {-0.000115127, -0.13448, 0.0868957},
        {0.560642, 0.826194, 0.0475799, -0.0286252}
    };
    const ChFrameMoving T_incisal_mand_ref{
        {0.000013162, -0.110933, 0.100209},
        {0.69105421, 0.72057171, 0.04304498, -0.03698142}
    };

    // jaw.SetMandibleState(mandible_init, T_incisal_mand_ref);

    // Start the Simulation
    jaw.StartSimulation();

    // Access mandible position during simulation.
    for (int i = 0; i < jaw.SimulationTimeLimit() * 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const auto kin = jaw.GetMandibleKinematics();
        // biomechanics::ChJaw::read_lock lock(mtx);
        std::cout << "Time: " << kin.time << '\n';
        std::cout << "Mandible position [x, y, z]: " << kin.position[0] << ", " << kin.position[1] << ", "
                << kin.position[2] << '\n';
        std::cout << "Mandible orientation [w, x, y, z]: " << kin.rotation[0] << ", " << kin.rotation[1] << ", "
                << kin.rotation[2] << ", " << kin.rotation[3] << '\n';
        // In the same way: kin.linear_velocity, kin.angular_velocity, kin.linear_acceleration, kin.angular_acceleration
    }

    // Stop the simulation (wait until the simulation thread has finished)
    jaw.StopSimulation(true);

    return 0;
}
