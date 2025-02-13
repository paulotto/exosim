/**
 * @file ChMuscleExcitation.h
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 19.07.2024
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

#ifndef EXOSIM_CH_MUSCLE_EXCITATION_H
#define EXOSIM_CH_MUSCLE_EXCITATION_H

#include <vector>

#include <Eigen/Dense>

#include <spline/src/spline.h>

#include <chrono/functions/ChFunction.h>
#include <chrono/functions/ChFunctionBSpline.h>

#include "exosim/biomechanics/utils/utils.h"


namespace chrono::biomechanics {
    DEFINE_HAS_MEMBER_FUNCTION(has_member_GetVal, GetVal)

    DEFINE_HAS_MEMBER_FUNCTION(has_member_GetDer, GetDer)

    DEFINE_HAS_MEMBER_FUNCTION(has_member_deriv, deriv)

    DEFINE_HAS_MEMBER_FUNCTION(has_member_call_operator, operator())

    template<class T = tk::spline>
    class ChMuscleExcitation final : public ChFunction {
        public:
            explicit ChMuscleExcitation(std::shared_ptr<T> representation)
                : representation_(std::move(representation)) {
                static_assert(has_member_GetVal<T, double, double>::value
                              || has_member_call_operator<T, double, double>::value,
                              "Representation must have a 'double GetVal(double)' or "
                              "'double operator()(double)' member function!");
                static_assert(has_member_GetDer<T, double, double>::value
                              || has_member_deriv<T, double, int, double>::value,
                              "Representation must have a 'double GetDer(double)' or "
                              "'double deriv(int, double)' member function!");
            };

            ~ChMuscleExcitation() override = default;

            ChFunction* Clone() const override { return new ChMuscleExcitation(representation_); }

            const std::shared_ptr<T>& GetRepresentation() const { return representation_; }

            double GetVal(double x) const override {
                if constexpr (has_member_GetVal<T, double, double>::value) {
                    return representation_->GetVal(x);
                } else if constexpr (has_member_call_operator<T, double, double>::value) {
                    return (*representation_)(x);
                }
                return 0.0;
            }

            double GetDer(double x) const override {
                if constexpr (has_member_GetDer<T, double, double>::value) {
                    return representation_->GetDer(x);
                } else if constexpr (has_member_deriv<T, double, int, double>::value) {
                    return representation_->deriv(1, x);
                }
                return 0.0;
            }

            void GetVal(const std::vector<double>& timestamps, std::vector<double>& values) const {
                values.resize(timestamps.size());
                for (size_t i = 0; i < timestamps.size(); ++i) {
                    values[i] = GetVal(timestamps[i]);
                }
            }

            void GetVal(const Eigen::VectorXd& timestamps, Eigen::VectorXd& values) const {
                values.resize(timestamps.size());
                for (Eigen::Index i = 0; i < timestamps.size(); ++i) {
                    values[i] = GetVal(timestamps[i]);
                }
            }

            void GetDer(const std::vector<double>& timestamps, std::vector<double>& values) const {
                values.resize(timestamps.size());
                for (size_t i = 0; i < timestamps.size(); ++i) {
                    values[i] = GetDer(timestamps[i]);
                }
            }

            void GetDer(const Eigen::VectorXd& timestamps, Eigen::VectorXd& values) const {
                values.resize(timestamps.size());
                for (Eigen::Index i = 0; i < timestamps.size(); ++i) {
                    values[i] = GetDer(timestamps[i]);
                }
            }

        private:
            std::shared_ptr<T> representation_;
    };
} // namespace chrono::biomechanics

#endif // EXOSIM_CH_MUSCLE_EXCITATION_H
