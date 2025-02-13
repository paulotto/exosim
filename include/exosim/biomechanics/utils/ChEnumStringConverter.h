/**
* @file ChEnumStringConverter.h
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 25.06.2024
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

#ifndef EXOSIM_CH_ENUMSTRINGCONVERTER_H
#define EXOSIM_CH_ENUMSTRINGCONVERTER_H

#include <map>
#include <string>

#include "exosim/biomechanics/utils/utils.h"

#include <chrono/physics/ChContactMaterial.h>
#include <chrono/collision/ChCollisionSystem.h>
#include <chrono/collision/multicore/ChNarrowphase.h>
#include <chrono/solver/ChSolverADMM.h>
#include <chrono/timestepper/ChTimestepper.h>
#include <chrono/assets/ChVisualShapeFEA.h>

namespace chrono::utils {
    // -----------------------------------------------------
    // Maps for simple conversions between enums and strings
    // -----------------------------------------------------

    inline std::map<std::string, ChContactMethod> contact_method_enum_conversion_map{
        {"NSC", ChContactMethod::NSC},
        {"SMC", ChContactMethod::SMC}
    };

    inline std::map<std::string, ChCollisionSystem::Type> collision_system_enum_conversion_map{
        {"MULTICORE", ChCollisionSystem::Type::MULTICORE},
        {"BULLET", ChCollisionSystem::Type::BULLET}
    };

    inline std::map<std::string, ChNarrowphase::Algorithm> narrowphase_algorithm_enum_conversion_map{
        {"HYBRID", ChNarrowphase::Algorithm::HYBRID},
        {"PRIMS", ChNarrowphase::Algorithm::PRIMS},
        {"MPR", ChNarrowphase::Algorithm::MPR}
    };

    inline std::map<std::string, ChSolverADMM::AdmmAcceleration> admm_acceleration_enum_conversion_map{
        {"BASIC", ChSolverADMM::AdmmAcceleration::BASIC},
        {"NESTEROV", ChSolverADMM::AdmmAcceleration::NESTEROV}
    };

    inline std::map<std::string, ChSolverADMM::AdmmStepType> admm_step_enum_conversion_map{
        {"NONE", ChSolverADMM::AdmmStepType::NONE},
        {"BALANCED_FAST", ChSolverADMM::AdmmStepType::BALANCED_FAST},
        {"BALANCED_RANGE", ChSolverADMM::AdmmStepType::BALANCED_RANGE},
        {"BALANCED_UNSCALED", ChSolverADMM::AdmmStepType::BALANCED_UNSCALED}
    };

    inline std::map<std::string, ChSolver::Type> solver_enum_conversion_map{
        {"ADMM", ChSolver::Type::ADMM},
        {"PSOR", ChSolver::Type::PSOR},
        {"PSSOR", ChSolver::Type::PSSOR},
        {"PJACOBI", ChSolver::Type::PJACOBI},
        {"PMINRES", ChSolver::Type::PMINRES},
        {"BARZILAIBORWEIN", ChSolver::Type::BARZILAIBORWEIN},
        {"APGD", ChSolver::Type::APGD},
        {"SPARSE_LU", ChSolver::Type::SPARSE_LU},
        {"SPARSE_QR", ChSolver::Type::SPARSE_QR},
        {"PARDISO_MKL", ChSolver::Type::PARDISO_MKL},
        {"MUMPS", ChSolver::Type::MUMPS},
        {"GMRES", ChSolver::Type::GMRES},
        {"MINRES", ChSolver::Type::MINRES},
        {"BICGSTAB", ChSolver::Type::BICGSTAB},
        {"CUSTOM", ChSolver::Type::CUSTOM}
    };

    inline std::map<std::string, ChTimestepper::Type> time_stepper_enum_conversion_map{
        {"EULER_IMPLICIT_LINEARIZED", ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED},
        {"EULER_IMPLICIT_PROJECTED", ChTimestepper::Type::EULER_IMPLICIT_PROJECTED},
        {"EULER_IMPLICIT", ChTimestepper::Type::EULER_IMPLICIT},
        {"TRAPEZOIDAL", ChTimestepper::Type::TRAPEZOIDAL},
        {"TRAPEZOIDAL_LINEARIZED", ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED},
        {"HHT", ChTimestepper::Type::HHT},
        {"HEUN", ChTimestepper::Type::HEUN},
        {"RUNGEKUTTA45", ChTimestepper::Type::RUNGEKUTTA45},
        {"EULER_EXPLICIT", ChTimestepper::Type::EULER_EXPLICIT},
        {"LEAPFROG", ChTimestepper::Type::LEAPFROG},
        {"NEWMARK", ChTimestepper::Type::NEWMARK},
        {"CUSTOM", ChTimestepper::Type::CUSTOM},
    };

    inline std::map<std::string, ChVisualShapeFEA::DataType> visual_shape_fea_enum_conversion_map{
        {"NONE", ChVisualShapeFEA::DataType::NONE},
        {"SURFACE", ChVisualShapeFEA::DataType::SURFACE},
        {"CONTACTSURFACES", ChVisualShapeFEA::DataType::CONTACTSURFACES},
        {"LOADSURFACES", ChVisualShapeFEA::DataType::LOADSURFACES},
        {"NODE_DISP_NORM", ChVisualShapeFEA::DataType::NODE_DISP_NORM},
        {"NODE_DISP_X", ChVisualShapeFEA::DataType::NODE_DISP_X},
        {"NODE_DISP_Y", ChVisualShapeFEA::DataType::NODE_DISP_Y},
        {"NODE_DISP_Z", ChVisualShapeFEA::DataType::NODE_DISP_Z},
        {"NODE_SPEED_NORM", ChVisualShapeFEA::DataType::NODE_SPEED_NORM},
        {"NODE_SPEED_X", ChVisualShapeFEA::DataType::NODE_SPEED_X},
        {"NODE_SPEED_Y", ChVisualShapeFEA::DataType::NODE_SPEED_Y},
        {"NODE_SPEED_Z", ChVisualShapeFEA::DataType::NODE_SPEED_Z},
        {"NODE_ACCEL_NORM", ChVisualShapeFEA::DataType::NODE_ACCEL_NORM},
        {"NODE_ACCEL_X", ChVisualShapeFEA::DataType::NODE_ACCEL_X},
        {"NODE_ACCEL_Y", ChVisualShapeFEA::DataType::NODE_ACCEL_Y},
        {"NODE_ACCEL_Z", ChVisualShapeFEA::DataType::NODE_ACCEL_Z},
        {"ELEM_STRAIN_VONMISES", ChVisualShapeFEA::DataType::ELEM_STRAIN_VONMISES},
        {"ELEM_STRESS_VONMISES", ChVisualShapeFEA::DataType::ELEM_STRESS_VONMISES},
        {"ELEM_STRAIN_HYDROSTATIC", ChVisualShapeFEA::DataType::ELEM_STRAIN_HYDROSTATIC},
        {"ELEM_STRESS_HYDROSTATIC", ChVisualShapeFEA::DataType::ELEM_STRESS_HYDROSTATIC},
        {"ELEM_BEAM_MX", ChVisualShapeFEA::DataType::ELEM_BEAM_MX},
        {"ELEM_BEAM_MY", ChVisualShapeFEA::DataType::ELEM_BEAM_MY},
        {"ELEM_BEAM_MZ", ChVisualShapeFEA::DataType::ELEM_BEAM_MZ},
        {"ELEM_BEAM_TX", ChVisualShapeFEA::DataType::ELEM_BEAM_TX},
        {"ELEM_BEAM_TY", ChVisualShapeFEA::DataType::ELEM_BEAM_TY},
        {"ELEM_BEAM_TZ", ChVisualShapeFEA::DataType::ELEM_BEAM_TZ},
        {"NODE_FIELD_VALUE", ChVisualShapeFEA::DataType::NODE_FIELD_VALUE},
        {"ANCF_BEAM_AX", ChVisualShapeFEA::DataType::ANCF_BEAM_AX},
        {"ANCF_BEAM_BD", ChVisualShapeFEA::DataType::ANCF_BEAM_BD}
    };

    /**
     * Converts a string to an enum.
     * @tparam T The enum type.
     * @param str The string to convert.
     * @param map The map to use for the conversion.
     * @return The converted enum.
     */
    template<typename T>
    T StringToEnum(const std::string& str, const std::map<std::string, T>& map) {
        auto it = map.find(str);
        biomechanics::utils::throw_invalid_argument(it == map.end(),
                                                    "[ERROR] [ChEnumStringConverter] Invalid string: {}", str);
        return it->second;
    }

    /**
     * Converts an enum to a string.
     * @tparam T The enum type.
     * @param value The enum to convert.
     * @param map The map to use for the conversion.
     * @return The converted string.
     */
    template<typename T>
    std::string EnumToString(T value, const std::map<std::string, T>& map) {
        for (const auto& pair: map) {
            if (pair.second == value) {
                return pair.first;
            }
        }
        biomechanics::utils::throw_invalid_argument(true, "[ERROR] [ChEnumStringConverter] Invalid enum value");
        return "";
    }
} // namespace chrono

#endif //EXOSIM_CH_ENUMSTRINGCONVERTER_H
