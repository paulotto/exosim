/**
 * @file ChMuscle.cpp
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 20.07.2023
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

#include "exosim/biomechanics/ChMuscle.h"

#include <valarray>

namespace chrono::biomechanics {
    // Register into the object factory, to enable run-time dynamic creation and persistence
    CH_FACTORY_REGISTER(ChMuscleTSDA)

    CH_FACTORY_REGISTER(ChMuscleConstant)

    CH_FACTORY_REGISTER(ChMuscleLinear)

    CH_FACTORY_REGISTER(ChMusclePeck)

    CH_FACTORY_REGISTER(ChMuscleMillard)

    /// ===================================ChMuscle===================================

    ChMuscle::ChMuscleProperties::ChMuscleProperties(
        std::string name,
        ChMuscleParameterVector parameter,
        const std::shared_ptr<ChBody>& body_1,
        const std::shared_ptr<ChBody>& body_2,
        const ChVector3d& loc_1,
        const ChVector3d& loc_2,
        bool local_coordinates,
        bool rel_to_ref,
        const ChColor& color/*,
        const std::shared_ptr<ChVisualShapePointPoint>& visual_shape*/) : name(std::move(name)),
                                                                        params(std::move(parameter)),
                                                                        body_1(body_1),
                                                                        body_2(body_2),
                                                                        loc_1(loc_1),
                                                                        loc_2(loc_2),
                                                                        local_coordinates(local_coordinates),
                                                                        rel_to_ref(rel_to_ref),
                                                                        color(color)/*,
                                                                        shape(visual_shape)*/ {
    }

    void ChMuscle::SetMaxContractileForceLength(double l_max_active_force) {
        write_lock lock(mtx_);
        l_max_active_force_ = std::max(0.0, l_max_active_force);
        h_fiber_ = l_max_active_force_ * std::sin(alpha_opt_);
        utils::throw_invalid_argument(l_max_active_force < 0.0,
                                      "[ERROR] [ChMuscle] l_max_active_force ({}) mustn't be smaller than 0!",
                                      l_max_active_force);
    }

    void ChMuscle::SetMaxLength(double l_max) {
        write_lock lock(mtx_);
        if (l_max > l_zero_passive_force_ && l_max > 0.0) {
            l_max_ = l_max;
        } else {
            utils::throw_invalid_argument(true, "[ERROR] [ChMuscle] 0 <= l_zero_passive_force ({}) < l_max ({}) "
                                          "not fulfilled! l_max set to {}!",
                                          l_zero_passive_force_, l_max, l_max_active_force_);
        }
    }

    void ChMuscle::SetZeroPassiveForceLength(double l_zero_passive_force) {
        write_lock lock(mtx_);
        if (l_zero_passive_force < l_max_ && l_zero_passive_force >= 0.0) {
            l_zero_passive_force_ = l_zero_passive_force;
        } else {
            utils::throw_invalid_argument(true, "[ERROR] [ChMuscle] 0 <= l_zero_passive_force ({}) < l_max ({}) "
                                          "not fulfilled! l_zero_passive_force set to {}!",
                                          l_zero_passive_force, l_max_, l_zero_passive_force_);
        }
    }

    void ChMuscle::SetTendonSlackLength(double l_tendon_slack) {
        write_lock lock(mtx_);
        // TODO
        l_tendon_slack_ = l_tendon_slack;
    }

    void ChMuscle::SetMaxContractionVelocity(double vel_max_contraction) {
        write_lock lock(mtx_);
        // TODO
        vel_max_contraction_ = std::abs(vel_max_contraction);
    }

    void ChMuscle::SetFractionPassiveTension(double passive_tension_frac) {
        write_lock lock(mtx_);
        passive_tension_frac_ = std::max(0.0, passive_tension_frac);
        utils::throw_invalid_argument(passive_tension_frac < 0.0,
                                      "[ERROR] [ChMuscle] passive_tension_frac ({}) mustn't be smaller than 0!",
                                      passive_tension_frac);
    }

    void ChMuscle::SetTendonRatio(double r_tendon) {
        write_lock lock(mtx_);
        tendon_ratio_ = r_tendon;
        utils::throw_invalid_argument(r_tendon == 1.0,
                                      "[ERROR] [ChMuscle] r_tendon mustn't be 1!");
    }

    void ChMuscle::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChMuscle>();

        // Serialize parent class
        ChLinkTSDA::ArchiveOut(archive_out);

        // Serialize enums
        ChMuscleEnumMapper::ChMuscleType_mapper type_mapper;
        archive_out << CHNVP(type_mapper(type_), "type_");

        // Serialize all member data
        archive_out << CHNVP(rigid_tendon_);
        archive_out << CHNVP(ignore_f_vel_);
        archive_out << CHNVP(activation_dynamics_);
        archive_out << CHNVP(e_current_);
        archive_out << CHNVP(a_current_);
        archive_out << CHNVP(a_min_);
        archive_out << CHNVP(tau_act_);
        archive_out << CHNVP(tau_deact_);
        archive_out << CHNVP(alpha_opt_);
        archive_out << CHNVP(f_max_active_);
        archive_out << CHNVP(l_max_);
        archive_out << CHNVP(l_max_active_force_);
        archive_out << CHNVP(l_zero_passive_force_);
        archive_out << CHNVP(l_tendon_slack_);
        archive_out << CHNVP(vel_max_contraction_);
        archive_out << CHNVP(passive_tension_frac_);
        archive_out << CHNVP(force_scale_);
        archive_out << CHNVP(tendon_ratio_);
        archive_out << CHNVP(h_fiber_);
        archive_out << CHNVP(excitation_);
        // archive_out << CHNVP(muscle_force_);
        // TODO: Serialize curves
        // archive_out << CHNVP(active_f_l_curve_);
        // archive_out << CHNVP(passive_f_l_curve_);
        // archive_out << CHNVP(tendon_f_l_curve_);
        // archive_out << CHNVP(f_vel_curve_);
    }

    void ChMuscle::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChMuscle>();

        // Deserialize parent class
        ChLinkTSDA::ArchiveIn(archive_in);

        // Deserialize enums
        ChMuscleEnumMapper::ChMuscleType_mapper type_mapper;
        archive_in >> CHNVP(type_mapper(type_), "type_");

        // Deserialize all member data
        archive_in >> CHNVP(rigid_tendon_);
        archive_in >> CHNVP(ignore_f_vel_);
        archive_in >> CHNVP(activation_dynamics_);
        archive_in >> CHNVP(e_current_);
        archive_in >> CHNVP(a_current_);
        archive_in >> CHNVP(a_min_);
        archive_in >> CHNVP(tau_act_);
        archive_in >> CHNVP(tau_deact_);
        archive_in >> CHNVP(alpha_opt_);
        archive_in >> CHNVP(f_max_active_);
        archive_in >> CHNVP(l_max_);
        archive_in >> CHNVP(l_max_active_force_);
        archive_in >> CHNVP(l_zero_passive_force_);
        archive_in >> CHNVP(l_tendon_slack_);
        archive_in >> CHNVP(vel_max_contraction_);
        archive_in >> CHNVP(passive_tension_frac_);
        archive_in >> CHNVP(force_scale_);
        archive_in >> CHNVP(tendon_ratio_);
        archive_in >> CHNVP(h_fiber_);
        archive_in >> CHNVP(excitation_);
        // archive_in >> CHNVP(muscle_force_);
        // TODO: Deserialize curves
        // archive_in >> CHNVP(active_f_l_curve_);
        // archive_in >> CHNVP(passive_f_l_curve_);
        // archive_in >> CHNVP(tendon_f_l_curve_);
        // archive_in >> CHNVP(f_vel_curve_);
    }

    void ChMuscle::CreateActiveForceLengthCurve(double l_min_active_norm_fiber, double l_transition_norm_fiber,
                                                double l_max_active_norm_fiber, double slope_shallow_ascending,
                                                double f_min) {
        utils::throw_invalid_argument(l_min_active_norm_fiber <= 0,
                                      "[ERROR] [ChMuscle] l_min_active_norm_fiber must be > 0");
        utils::throw_invalid_argument(l_transition_norm_fiber >= 1,
                                      "[ERROR] [ChMuscle] l_transition_norm_fiber must be < 1");
        utils::throw_invalid_argument(l_min_active_norm_fiber >= l_transition_norm_fiber,
                                      "[ERROR] [ChMuscle] l_min_active_norm_fiber must be < l_transition_norm_fiber");
        utils::throw_invalid_argument(l_max_active_norm_fiber <= 1,
                                      "[ERROR] [ChMuscle] l_max_active_norm_fiber must be > 1");
        utils::throw_invalid_argument(slope_shallow_ascending < 0,
                                      "[ERROR] [ChMuscle] slope_shallow_ascending must be >= 0");
        utils::throw_invalid_argument(slope_shallow_ascending >= 1 / (1 - l_transition_norm_fiber),
                                      "[ERROR] [ChMuscle] slope_shallow_ascending must be < "
                                      "1/(1-l_transition_norm_fiber)");
        utils::throw_invalid_argument(f_min < 0, "[ERROR] [ChMuscle] f_min must be >= 0");

        // Start point - transition point - f_max point - end point
        std::vector<double> x{l_min_active_norm_fiber, l_transition_norm_fiber, 1.0, l_max_active_norm_fiber};
        std::vector<double> y{f_min, 1.0 - slope_shallow_ascending * (1.0 - l_transition_norm_fiber), 1.0, f_min};

        // fmt::print(utils::DEBUG_MSG, "[DEBUG] [ChMuscle] [CreateActiveForceLengthCurve] "
        //            "x: {} {} {} {}, y: {} {} {} {}\n", x[0], x[1], x[2], x[3], y[0], y[1], y[2], y[3]);

        write_lock lock(mtx_);
        active_f_l_curve_ = std::make_shared<tk::spline>(x, y);
    }

    void ChMuscle::CreatePassiveForceLengthCurve(double strain_at_zero_force,
                                                 double strain_at_one_norm_force,
                                                 double stiffness_at_low_force,
                                                 double stiffness_at_one_norm_force,
                                                 double curviness) {
        utils::throw_invalid_argument(strain_at_zero_force >= strain_at_one_norm_force,
                                      "[ERROR] [ChMuscle] strain_at_zero_force must be < strain_at_one_norm_force");
        utils::throw_invalid_argument(
            stiffness_at_one_norm_force <= 1 / (strain_at_one_norm_force - strain_at_zero_force),
            "[ERROR] [ChMuscle] stiffness_at_one_norm_force must be > "
            "1/(strain_at_one_norm_force - strain_at_zero_force)");
        utils::throw_invalid_argument(stiffness_at_low_force <= 0,
                                      "[ERROR] [ChMuscle] stiffness_at_low_force must be > 0");
        utils::throw_invalid_argument(stiffness_at_low_force >= stiffness_at_one_norm_force,
                                      "[ERROR] [ChMuscle] stiffness_at_low_force must be < "
                                      "stiffness_at_one_norm_force");
        utils::throw_invalid_argument(curviness < 0 || curviness > 1,
                                      "[ERROR] [ChMuscle] curviness must be in the range [0,1]");

        // Start point
        const double x_start = 1.0 + strain_at_zero_force;

        // ISO point
        const double x_iso = 1.0 + strain_at_one_norm_force;

        // Intermediate point
        const double x_delta = std::min(0.1 / stiffness_at_one_norm_force, 0.1 * (x_iso - x_start));
        const double x_low = x_start + x_delta;
        const double x_foot = x_start + 0.5 * (x_low - x_start);
        const double y_intermediate = stiffness_at_low_force * (x_low - x_foot);

        std::vector<double> x{x_start, x_start + x_delta, x_iso};
        std::vector<double> y{0.0, y_intermediate, 1.0};

        // fmt::print(utils::DEBUG_MSG, "[DEBUG] [ChMuscle] [CreatePassiveForceLengthCurve] "
        //            "x: {} {} {}, y: {} {} {}\n", x[0], x[1], x[2], y[0], y[1], y[2]);

        write_lock lock(mtx_);
        // Start point - intermediate point - ISO point
        passive_f_l_curve_ = std::make_shared<tk::spline>(x, y);
    }

    void ChMuscle::CreateTendonForceLengthCurve(double strain_at_one_norm_force,
                                                double stiffness_at_one_norm_force,
                                                double f_norm_at_toe_end,
                                                double curviness) {
        utils::throw_invalid_argument(strain_at_one_norm_force <= 0,
                                      "[ERROR] [ChMuscle] strain_at_one_norm_force must be > 0");
        utils::throw_invalid_argument(stiffness_at_one_norm_force <= 1 / strain_at_one_norm_force,
                                      "[ERROR] [ChMuscle] stiffness_at_one_norm_force must be > "
                                      "1/strain_at_one_norm_force");
        utils::throw_invalid_argument(f_norm_at_toe_end <= 0 || f_norm_at_toe_end >= 1,
                                      "[ERROR] [ChMuscle] f_norm_at_toe_end must be in the range (0,1)");
        utils::throw_invalid_argument(curviness < 0 || curviness > 1,
                                      "[ERROR] [ChMuscle] curviness must be in the range [0,1]");

        // ISO point
        const double x_iso = 1.0 + strain_at_one_norm_force;

        // Intermediate point at which curve becomes linear
        const double x_intermediate = (f_norm_at_toe_end - 1.0) / stiffness_at_one_norm_force + x_iso;

        std::vector<double> x{1.0, x_intermediate, x_iso};
        std::vector<double> y{0.0, f_norm_at_toe_end, 1.0};

        // fmt::print(utils::DEBUG_MSG, "[DEBUG] [ChMuscle] [CreateTendonForceLengthCurve] "
        //            "x: {} {} {}, y: {} {} {}\n", x[0], x[1], x[2], y[0], y[1], y[2]);

        write_lock lock(mtx_);
        // Start point - intermediate point - ISO point
        tendon_f_l_curve_ = std::make_shared<tk::spline>(x, y);
    }

    void ChMuscle::CreateForceVelocityCurve(double slope_concentric_at_vmax,
                                            double slope_concentric_near_vmax,
                                            double slope_isometric,
                                            double slope_eccentric_at_vmax,
                                            double slope_eccentric_near_vmax,
                                            double max_eccentric_vel_f_multiplier,
                                            double concentric_curviness,
                                            double eccentric_curviness) {
        utils::throw_invalid_argument(slope_concentric_at_vmax < 0 || slope_concentric_at_vmax >= 1,
                                      "[ERROR] [ChMuscle] slope_concentric_at_vmax must be in the range [0,1)");
        utils::throw_invalid_argument(slope_concentric_near_vmax <= slope_concentric_at_vmax ||
                                      slope_concentric_near_vmax >= slope_isometric,
                                      "[ERROR] [ChMuscle] slope_concentric_near_vmax must be > "
                                      "slope_concentric_at_vmax and < slope_isometric");
        utils::throw_invalid_argument(slope_isometric <= 1, "slope_isometric must be > 1");
        utils::throw_invalid_argument(slope_eccentric_near_vmax >= slope_isometric ||
                                      slope_eccentric_near_vmax <= slope_eccentric_at_vmax,
                                      "[ERROR] [ChMuscle] slope_eccentric_near_vmax must be < slope_isometric "
                                      "and > slope_eccentric_at_vmax");
        utils::throw_invalid_argument(slope_isometric <= max_eccentric_vel_f_multiplier - 1,
                                      "[ERROR] [ChMuscle] slope_isometric must exceed "
                                      "(max_eccentric_vel_f_multiplier-1)");
        utils::throw_invalid_argument(slope_eccentric_at_vmax < 0, "slope_eccentric_at_vmax must be >= 0");
        utils::throw_invalid_argument(slope_eccentric_at_vmax >= max_eccentric_vel_f_multiplier - 1,
                                      "[ERROR] [ChMuscle] slope_eccentric_at_vmax must be less than "
                                      "(max_eccentric_vel_f_multiplier-1)");
        utils::throw_invalid_argument(max_eccentric_vel_f_multiplier <= 1,
                                      "[ERROR] [ChMuscle] max_eccentric_vel_f_multiplier must be > 1");
        utils::throw_invalid_argument(concentric_curviness < 0 || concentric_curviness > 1,
                                      "[ERROR] [ChMuscle] concentric_curviness must be in the range [0,1]");
        utils::throw_invalid_argument(eccentric_curviness < 0 || eccentric_curviness > 1,
                                      "[ERROR] [ChMuscle] eccentric_curviness must be in the range [0,1]");

        // Need to make end points have small non-zero slopes to ensure curve is invertible.

        // Start point at C
        const double x_c = -1.0;
        const double y_c = 0.0;

        // Point near C
        const double x_near_c = -0.9;
        const double y_near_c = y_c + 0.5 * slope_concentric_near_vmax * (x_near_c - x_c)
                                + 0.5 * slope_concentric_at_vmax * (x_near_c - x_c);

        // End point E
        const double x_e = 1.0;
        const double y_e = max_eccentric_vel_f_multiplier;

        // Point near E
        const double x_near_e = 0.9;
        const double y_near_e = y_e + 0.5 * slope_eccentric_near_vmax * (x_near_e - x_e)
                                + 0.5 * slope_eccentric_at_vmax * (x_near_e - x_e);

        std::vector<double> x{x_c, x_near_c, 0.0, x_near_e, x_e};
        std::vector<double> y{y_c, y_near_c, 1.0, y_near_e, y_e};

        // fmt::print(utils::DEBUG_MSG, "[DEBUG] [ChMuscle] [CreateForceVelocityCurve] "
        //            "x: {} {} {} {} {}, y: {} {} {} {} {}\n", x[0], x[1], x[2], x[3], x[4], y[0], y[1], y[2], y[3], y[4]);

        write_lock lock(mtx_);
        f_vel_curve_ = std::make_shared<tk::spline>(x, y);
        // f_vel_curve_->make_monotonic();
    }

    /// ===================================ChMuscleTSDA===================================

    ChMuscleTSDA::ChMuscleTSDA(const std::shared_ptr<ChFunction>& excitation,
                               double k,
                               double d,
                               double f_max_active)
        : ChMuscle(excitation, d, f_max_active) {
        type_ = ChMuscleType::TSDA;
        this->SetSpringCoefficient(k);
    }

    double ChMuscleTSDA::ComputeMuscleForce(double time,
                                            double rest_length,
                                            double length,
                                            double vel,
                                            const ChMuscle& muscle) {
        read_lock lock(mtx_);
        a_current_ = excitation_ == nullptr ? 0.0 : static_cast<float>(excitation_->GetVal(time));
        a_current_ = std::max<float>(std::min<float>(a_current_, 1.0), 0.0);

        return -GetSpringCoefficient() * (length - rest_length) - GetDampingCoefficient() * vel - f_max_active_ *
               a_current_;
    }

    void ChMuscleTSDA::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChMuscleTSDA>();

        // Serialize parent class
        ChMuscle::ArchiveOut(archive_out);

        // Serialize all member data
    }

    void ChMuscleTSDA::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChMuscleTSDA>();

        // Deserialize parent class
        ChMuscle::ArchiveIn(archive_in);

        // Deserialize all member data
    }

    /// ===================================ChMuscleConstant===================================

    ChMuscleConstant::ChMuscleConstant(const std::shared_ptr<ChFunction>& excitation,
                                       double d,
                                       double f_max_active,
                                       double passive_tension_frac)
        : ChMuscle(excitation, d, f_max_active, passive_tension_frac) {
        type_ = ChMuscleType::CONSTANT;
    }

    double ChMuscleConstant::ComputeMuscleForce(double time,
                                                double rest_length,
                                                double length,
                                                double vel,
                                                const ChMuscle& muscle) {
        read_lock lock(mtx_);
        a_current_ = excitation_ == nullptr ? 0.0 : static_cast<float>(excitation_->GetVal(time));
        a_current_ = std::max<float>(std::min<float>(a_current_, 1.0), 0.0);

        return -f_max_active_ * a_current_ - passive_tension_frac_ * f_max_active_ - GetDampingCoefficient() * vel;
    }

    void ChMuscleConstant::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChMuscleConstant>();

        // Serialize parent class
        ChMuscle::ArchiveOut(archive_out);

        // Serialize all member data
    }

    void ChMuscleConstant::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChMuscleConstant>();

        // Deserialize parent class
        ChMuscle::ArchiveIn(archive_in);

        // Deserialize all member data
    }

    /// ===================================ChMuscleLinear===================================

    ChMuscleLinear::ChMuscleLinear(const std::shared_ptr<ChFunction>& excitation,
                                   double d,
                                   double f_max_active,
                                   double passive_tension_frac,
                                   double l_max_force,
                                   double l_zero_passive_force)
        : ChMuscle(excitation, d, f_max_active, passive_tension_frac, l_max_force,
                   l_max_force, l_zero_passive_force) {
        type_ = ChMuscleType::LINEAR;
    }

    double ChMuscleLinear::ComputeMuscleForce(double time,
                                              double rest_length,
                                              double length,
                                              double vel,
                                              const ChMuscle& muscle) {
        double l_dash = 0.0;
        read_lock lock(mtx_);
        a_current_ = excitation_ == nullptr ? 0.0 : static_cast<float>(excitation_->GetVal(time));
        a_current_ = std::max<float>(std::min<float>(a_current_, 1.0), 0.0);
        //        length = std::abs(length);

        if (length > l_max_) {
            l_dash = 1.0;
        } else if (length > l_zero_passive_force_) {
            l_dash = (length - l_zero_passive_force_) / (l_max_ - l_zero_passive_force_);
        }

        return -f_max_active_ * l_dash * a_current_ - passive_tension_frac_ * f_max_active_ * l_dash
               - GetDampingCoefficient() * vel;
    }

    void ChMuscleLinear::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChMuscleLinear>();

        // Serialize parent class
        ChMuscle::ArchiveOut(archive_out);

        // Serialize all member data
    }

    void ChMuscleLinear::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChMuscleLinear>();

        // Deserialize parent class
        ChMuscle::ArchiveIn(archive_in);

        // Deserialize all member data
    }

    /// ===================================ChMusclePeck===================================

    ChMusclePeck::ChMusclePeck(const std::shared_ptr<ChFunction>& excitation,
                               double d,
                               double f_max_active,
                               double passive_tension_frac,
                               double l_max_active_force,
                               double l_max,
                               double l_zero_passive_force,
                               double force_scale,
                               double tendon_ratio)
        : ChMuscle(excitation, d, f_max_active, passive_tension_frac, l_max_active_force, l_max,
                   l_zero_passive_force, 0.0, 1.0, force_scale, tendon_ratio) {
        type_ = ChMuscleType::PECK;

        // this->SetMinimumActivation(0.001);
        // activation_dynamics_ = true;
        // if (activation_dynamics_) {
        //     ode_ = chrono_types::make_shared<MuscleActivationODE>(*this);
        //     this->RegisterODE(ode_.get());
        // }
    }

    double ChMusclePeck::ComputeMuscleForce(double time,
                                            double rest_length,
                                            double length,
                                            double vel,
                                            const ChMuscle& muscle) {
        read_lock lock(mtx_);
        e_current_ = excitation_ == nullptr ? 0.0f : static_cast<float>(excitation_->GetVal(time));
        double f = 0.0, f_passive = 0.0, l_dash = 0.0, l_norm_fibre = 0.0, x_act = 0.0;

        e_current_ = std::max<float>(std::min<float>(e_current_, 1.0), 0.0);
        // length = std::abs(length);

        if (activation_dynamics_) {
            a_current_ = static_cast<float>(muscle.GetStates().tail(1)(0));
        } else {
            a_current_ = e_current_;
        }

        // Passive part
        if (linear_) {
            if (length > l_zero_passive_force_) {
                l_dash = (length - l_zero_passive_force_) / (l_max_ - l_zero_passive_force_);
            }
            f_passive = f_max_active_ * passive_tension_frac_ * l_dash;
        } else {
            if (length > l_zero_passive_force_) {
                l_dash = length - l_max_active_force_;
            }
            f_passive = (std::exp(l_dash / l_max_ * std::exp(1)) - 1)
                        * f_max_active_ * passive_tension_frac_ / (std::exp(std::exp(1)) - 1);
        }

        // Active part
        l_norm_fibre = (length - l_max_active_force_ * tendon_ratio_) / (l_max_active_force_ * (1 - tendon_ratio_));
        if (MIN_STRETCH_ < l_norm_fibre && l_norm_fibre < MAX_STRETCH_) {
            x_act = 0.5 * (1 + cos(2 * CH_PI * l_norm_fibre));
        }

        f = force_scale_ * (a_current_ * f_max_active_ * x_act + f_passive + GetDampingCoefficient() * vel);
        // f = std::max<double>(std::min<double>(f, 1.5 * f_max_active_), -1.5 * f_max_active_);
        f = std::max<double>(std::min<double>(f, 1.5 * f_max_active_), 0.0); // Limit max. muscle force

        if (verbose_ && time > time_prev_) {
            fmt::print(utils::DEBUG_MSG, "[DEBUG] [{:^9.6}] [ChMusclePeck] {:<55s} "
                       "| force: {:>10.6f} | passive force: {:>10.6f} | length: {:>10.6f} | vel: {:>10.6f} | excitation: {:>8.4f} | activation: {:>8.4f}\n",
                       time, muscle.GetName(), -f, -f_passive, length, vel, e_current_, a_current_);
        }
        time_prev_ = time;

        return -f;
    }

    void ChMusclePeck::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChMusclePeck>();

        // Serialize parent class
        ChMuscle::ArchiveOut(archive_out);

        // Serialize all member data
        archive_out << CHNVP(MIN_STRETCH_);
        archive_out << CHNVP(MAX_STRETCH_);
        archive_out << CHNVP(linear_);
    }

    void ChMusclePeck::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChMusclePeck>();

        // Deserialize parent class
        ChMuscle::ArchiveIn(archive_in);

        // Deserialize all member data
        archive_in >> CHNVP(MIN_STRETCH_);
        archive_in >> CHNVP(MAX_STRETCH_);
        archive_in >> CHNVP(linear_);
    }

    /// ===================================ChMuscleMillard===================================

    ChMuscleMillard::ChMuscleMillard(const std::shared_ptr<ChFunction>& excitation,
                                     double d,
                                     double f_max_active,
                                     double l_max_active_force,
                                     double vel_max_contraction,
                                     double tendon_ratio,
                                     double alpha_opt,
                                     bool rigid_tendon,
                                     bool ignore_f_vel)
        : ChMuscle(excitation, d, f_max_active, 0.0, l_max_active_force, 1.5,
                   1.0, tendon_ratio * l_max_active_force, vel_max_contraction, 1.0,
                   tendon_ratio, alpha_opt) {
        type_ = ChMuscleType::MILLARD;
        rigid_tendon_ = rigid_tendon;
        ignore_f_vel_ = ignore_f_vel;
        tau_act_ = 10.0e-3;
        tau_deact_ = 40.0e-3;

        CreateActiveForceLengthCurve(0.4441, 0.73, 1.8123,
                                     0.8616, 0.1);

        CreateForceVelocityCurve(0.01, 0.25, 5.0,
                                 0.01, 0.15, 1.4,
                                 0.6, 0.9);

        const double strain_at_zero_force{0.0};
        double strain_at_one_norm_force{0.7};
        CreatePassiveForceLengthCurve(strain_at_zero_force, strain_at_one_norm_force, 0.2,
                                      2.0 / (strain_at_one_norm_force - strain_at_zero_force),
                                      0.75);

        strain_at_one_norm_force = 0.049;
        CreateTendonForceLengthCurve(strain_at_one_norm_force,
                                     1.375 / strain_at_one_norm_force,
                                     2.0 / 3.0, 0.5);
    }

    double ChMuscleMillard::ComputeMuscleForce(double time,
                                               double rest_length,
                                               double length,
                                               double vel,
                                               const ChMuscle& muscle) {
        double f_m{0.0}, f_a{0.0}, f_p{0.0};
        read_lock lock(mtx_);
        e_current_ = excitation_ == nullptr ? 0.0f : static_cast<float>(excitation_->GetVal(time));
        e_current_ = std::max<float>(std::min<float>(e_current_, 1.0), 0.0);

        if (activation_dynamics_) {
            a_current_ = static_cast<float>(muscle.GetStates().tail(1)(0));
        } else {
            a_current_ = e_current_;
        }

        if (rigid_tendon_) {
            const double cos_alpha_l_m = length - l_tendon_slack_; // = l_m * cos(alpha)
            const double l_m = std::sqrt(h_fiber_ * h_fiber_ + cos_alpha_l_m * cos_alpha_l_m); // Muscle length
            double cos_alpha = cos_alpha_l_m / l_m;

            if (cos_alpha < 0.0) return 0.0;
            cos_alpha = std::min(cos_alpha, 1.47); // ~84°

            const double l_n = l_m / l_max_active_force_; // Normalized muscle length
            double v_n = vel / (cos_alpha * (1 + std::tan(alpha_opt_) * std::tan(alpha_opt_)));
            v_n /= vel_max_contraction_; // Normalized muscle velocity
            f_a = (*active_f_l_curve_)(l_n);
            f_p = (*passive_f_l_curve_)(l_n);

            if (ignore_f_vel_) {
                f_m = f_max_active_ * (f_a * a_current_ + f_p + this->GetDampingCoefficient() * v_n) * cos_alpha;
            } else {
                const double f_v = (*f_vel_curve_)(v_n);
                f_m = f_max_active_ * (f_a * f_v * a_current_ + f_p + this->GetDampingCoefficient() * v_n) * cos_alpha;
            }
        } else {
            // TODO: non-rigid tendon case
            utils::throw_invalid_argument(true, "[ERROR] [ChMuscle] Non-rigid-tendon muscle not implemented yet!\n");
        }

        f_m = std::max<double>(0.0, f_m);

        if (verbose_ && time > time_prev_) {
            fmt::print(utils::DEBUG_MSG, "[DEBUG] [{:^9.6}] [ChMuscleMillard] {:<55s} "
                       "| force: {:>10.6f} | passive force: {:>10.6f} | length: {:>10.6f} | vel: {:>10.6f} | excitation: {:>8.4f} | activation: {:>8.4f}\n",
                       time, muscle.GetName(), -f_m, -f_p, length, vel, e_current_, a_current_);
        }
        time_prev_ = time;

        return -f_m;
    }

    void ChMuscleMillard::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChMuscleMillard>();

        // Serialize parent class
        ChMuscle::ArchiveOut(archive_out);

        // Serialize all member data
    }

    void ChMuscleMillard::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChMuscleMillard>();

        // Deserialize parent class
        ChMuscle::ArchiveIn(archive_in);

        // Deserialize all member data
    }
} // namespace chrono::biomechanics
