/**
 * @file ChMuscle.h
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

#ifndef EXOSIM_CH_MUSCLE_H
#define EXOSIM_CH_MUSCLE_H

#include <tuple>
#include <utility>
#include <vector>
#include <variant>
#include <atomic>
#include <shared_mutex>
//
#include "exosim/biomechanics/utils/utils.h"
//
#include "spline/src/spline.h"
//
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChBody.h>
//
#include <chrono/functions/ChFunction.h>
//
#include <chrono/assets/ChVisualShapePointPoint.h>
//
#include <chrono/serialization/ChArchive.h>


namespace chrono::biomechanics {
    /**
     * @class ChMuscle
     * @inherit \p ChLinkTSDA
     * @brief TODO
     */
    class ChApi ChMuscle : public ChLinkTSDA {
        public:
            /// {damping, f_max_active, passive_tension_frac, l_max_active_force,
            /// l_max, l_zero_passive_force, force_scale, r_tendon}
            using ChMuscleParameterVector = std::vector<std::variant<double, bool> >;

            using mutex_type = std::shared_mutex;
            using read_lock  = std::shared_lock<mutex_type>;
            using write_lock = std::unique_lock<mutex_type>;

            /**
             * @struct ChMuscleProperties
             * @brief A struct for creating a larger amount of muscles more conveniently.
             */
            struct ChMuscleProperties {
                std::string name; ///< Muscle name

                /// {damping, f_max_active, passive_tension_frac, l_max_active_force,
                /// l_max, l_zero_passive_force, force_scale, r_tendon}
                ChMuscle::ChMuscleParameterVector params;

                std::shared_ptr<ChBody> body_1; ///< 1st body for the attachment
                std::shared_ptr<ChBody> body_2; ///< 2nd body for the attachment

                ChVector3d loc_1; ///< Coordinates specifying location on 1st body
                ChVector3d loc_2; ///< Coordinates specifying location on 2nd body

                bool local_coordinates; ///< True if attachment points defined relative to bodies
                bool rel_to_ref; ///< True if attachment points defined relative to REF frame of bodies

                ChColor color; ///< Muscle color
                // std::shared_ptr<ChVisualShapePointPoint> shape; ///< Visual model

                ChMuscleProperties()
                    : ChMuscleProperties("",
                                         {},
                                         nullptr,
                                         nullptr,
                                         ChVector3d(0, 0, 0),
                                         ChVector3d(0, 0, 0),
                                         true,
                                         true,
                                         ChColor(0.6f, 0.17f, 0.17f)/*,
                                         chrono_types::make_shared<ChVisualShapeSpring>(0.0025, 150, 20)*/) {
                }

                ChMuscleProperties(
                    std::string name,
                    ChMuscleParameterVector parameter,
                    const std::shared_ptr<ChBody>& body_1,
                    const std::shared_ptr<ChBody>& body_2,
                    const ChVector3d& loc_1,
                    const ChVector3d& loc_2,
                    bool local_coordinates = true,
                    bool rel_to_ref = true,
                    const ChColor& color = ChColor(0.6f, 0.17f, 0.17f)/*,
                    const std::shared_ptr<ChVisualShapePointPoint>& visual_shape = chrono_types::make_shared<
                        ChVisualShapeSpring>(0.0025, 150, 20)*/
                );

                virtual ~ChMuscleProperties() = default;

                virtual void ArchiveOut(ChArchiveOut& archive_out) {
                    archive_out << CHNVP(name);
                    archive_out << CHNVP(params);
                    archive_out << CHNVP(body_1);
                    archive_out << CHNVP(body_2);
                    archive_out << CHNVP(loc_1);
                    archive_out << CHNVP(loc_2);
                    archive_out << CHNVP(local_coordinates);
                    archive_out << CHNVP(rel_to_ref);
                    archive_out << CHNVP(color);
                    // archive_out << CHNVP(shape);
                }

                virtual void ArchiveIn(ChArchiveIn& archive_in) {
                    archive_in >> CHNVP(name);
                    archive_in >> CHNVP(params);
                    archive_in >> CHNVP(body_1);
                    archive_in >> CHNVP(body_2);
                    archive_in >> CHNVP(loc_1);
                    archive_in >> CHNVP(loc_2);
                    archive_in >> CHNVP(local_coordinates);
                    archive_in >> CHNVP(rel_to_ref);
                    archive_in >> CHNVP(color);
                    // archive_in >> CHNVP(shape);
                }
            };

            /**
             * @enum ChMuscleType
             * @brief Supported types of muscle models.
             */
            enum ChMuscleType {
                TSDA, ///< Simple linear spring-damper-mass model
                CONSTANT,
                ///< Model with contractile force proportional to its excitation,
                                         ///< a constant passive tension and linear damping
                LINEAR, ///< Model with a linear relationship between length and tension and linear damping
                PECK, ///< Model after Peck et al.
                MILLARD ///< Model after Millard et al.
            };

            /**
             * @class ChMuscleEnumMapper
             * @brief Can be used to convert between enums and strings.
             */
            class ChMuscleEnumMapper final {
                public:
                    CH_ENUM_MAPPER_BEGIN(ChMuscleType)
                                CH_ENUM_VAL(TSDA)
                                CH_ENUM_VAL(CONSTANT)
                                CH_ENUM_VAL(LINEAR)
                                CH_ENUM_VAL(PECK)
                                CH_ENUM_VAL(MILLARD)
                    CH_ENUM_MAPPER_END(ChMuscleType)
            };

            ChMuscle() : ChMuscle(nullptr) {
            }

            ChMuscle(const std::shared_ptr<ChFunction>& excitation,
                     const ChMuscleParameterVector& params)
                : ChMuscle(excitation,
                           std::get<double>(params[0]),
                           std::get<double>(params[1]),
                           std::get<double>(params[2]),
                           std::get<double>(params[3]),
                           std::get<double>(params[4]),
                           std::get<double>(params[5]),
                           std::get<double>(params[6]),
                           std::get<double>(params[7]),
                           std::get<double>(params[8]),
                           std::get<double>(params[9]),
                           std::get<double>(params[10])) {
            }

            /**
             * TODO
             * @param excitation
             * @param d
             * @param f_max_active
             * @param passive_tension_frac
             * @param l_max_active_force
             * @param l_max
             * @param l_zero_passive_force
             * @param l_tendon_slack
             * @param vel_max_contraction
             * @param force_scale
             * @param tendon_ratio
             * @param alpha_opt
             */
            explicit ChMuscle(const std::shared_ptr<ChFunction>& excitation,
                              double d = 0.0,
                              double f_max_active = 1.0,
                              double passive_tension_frac = 0.0,
                              double l_max_active_force = 1.0,
                              double l_max = 1.5,
                              double l_zero_passive_force = 1.0,
                              double l_tendon_slack = 0.0,
                              double vel_max_contraction = 1.0,
                              double force_scale = 1.0,
                              double tendon_ratio = 0.0,
                              double alpha_opt = 0.0)
                : f_max_active_(f_max_active), l_max_(l_max), l_tendon_slack_(l_tendon_slack),
                  vel_max_contraction_(vel_max_contraction), force_scale_(force_scale), excitation_(excitation) {
                this->SetDampingCoefficient(d);

                // TODO: for all parameters
                SetMaxContractileForceLength(l_max_active_force);
                SetZeroPassiveForceLength(l_zero_passive_force);
                SetFractionPassiveTension(passive_tension_frac);
                SetTendonRatio(tendon_ratio);
                SetOptPennationAngle(alpha_opt);

                h_fiber_ = l_max_active_force_ * std::sin(alpha_opt_);

                muscle_force_ = chrono_types::make_shared<MuscleForceFunctor>(*this);
                this->RegisterForceFunctor(muscle_force_);
            }

            /**
             * Computes and returns the general muscle force. If the muscle has internal \p ODE states,
             * the current states can be accessed with
             * @code{.unparsed}
             * muscle->GetStates();
             * @endcode
             * @param time Current time.
             * @param rest_length Undeformed length of the muscle.
             * @param length Current length.
             * @param vel Current velocity. Positive when extending.
             * @param muscle Associated muscle.
             * @return General muscle force.
             */
            virtual double ComputeMuscleForce(double time,
                                              double rest_length,
                                              double length,
                                              double vel,
                                              const ChMuscle& muscle
            ) = 0;

            /**
             * By default, the muscle excitations are directly used as muscle activations. If this function is called,
             * activation dynamics are enabled and the muscle activation is calculated by an ODE. The activation
             * time constant can be set with \p SetActivationTimeConstant and the deactivation time constant with
             * \p SetDeactivationTimeConstant. \n
             * \b NOTE: Not all muscles make use of the activation dynamics ODE.
             */
            virtual void EnableActivationDynamics() {
                write_lock lock(mtx_);
                activation_dynamics_ = true;
                ode_ = chrono_types::make_shared<MuscleActivationODE>(*this);
                this->RegisterODE(ode_.get());
            }

            // ------
            // Setter
            // ------

            void SetVerbose(bool verbose) {
                write_lock lock(mtx_);
                verbose_ = verbose;
            }

            void SetRigidTendon(bool rigid_tendon) {
                write_lock lock(mtx_);
                rigid_tendon_ = rigid_tendon;
            }
            void SetIgnoreForceVelocity(bool ignore_f_vel) {
                write_lock lock(mtx_);
                ignore_f_vel_ = ignore_f_vel;
            }
            void SetMinimumActivation(float a_min) {
                write_lock lock(mtx_);
                a_min_ = a_min;
            }

            void SetActivationTimeConstant(float tau_act) {
                write_lock lock(mtx_);
                tau_act_ = tau_act;
            }
            void SetDeactivationTimeConstant(float tau_deact) {
                write_lock lock(mtx_);
                tau_deact_ = tau_deact;
            }

            void SetOptPennationAngle(double alpha) {
                write_lock lock(mtx_);
                alpha_opt_ = alpha;
                h_fiber_ = l_max_active_force_ * std::sin(alpha_opt_);
            }

            void SetMaxContractileForce(double max_f) {
                write_lock lock(mtx_);
                f_max_active_ = std::abs(max_f);
            }

            void SetMaxContractileForceLength(double l_max_active_force);

            void SetMaxLength(double l_max);

            void SetZeroPassiveForceLength(double l_zero_passive_force);

            void SetTendonSlackLength(double l_tendon_slack); // TODO
            void SetMaxContractionVelocity(double vel_max_contraction); // TODO
            void SetFractionPassiveTension(double passive_tension_frac);

            void SetForceScale(double force_scale) {
                write_lock lock(mtx_);
                force_scale_ = force_scale;
            }

            void SetTendonRatio(double r_tendon);

            void SetMuscleFiberHeight(double h_fiber) {
                write_lock lock(mtx_);
                h_fiber_ = h_fiber;
                alpha_opt_ = std::asin(h_fiber_ / l_max_active_force_);
            }

            void SetExcitation(std::shared_ptr<ChFunction> excitation) {
                write_lock lock(mtx_);
                excitation_ = std::move(excitation);
            }

            // ------
            // Getter
            // ------

            ChMuscleType GetMuscleType() const { read_lock lock(mtx_); return type_; }
            bool GetRigidTendon() const { read_lock lock(mtx_); return rigid_tendon_; }
            bool GetIgnoreForceVelocity() const { read_lock lock(mtx_); return ignore_f_vel_; }
            float GetMinimumActivation() const { read_lock lock(mtx_); return a_min_; }
            bool GetHasActivationDynamics() const { read_lock lock(mtx_); return activation_dynamics_; }
            float GetCurrentExcitation() const { read_lock lock(mtx_); return e_current_; }
            float GetCurrentActivation() const { read_lock lock(mtx_); return a_current_; }
            float GetActivationTimeConstant() const { read_lock lock(mtx_); return tau_act_; }
            float GetDeactivationTimeConstant() const { read_lock lock(mtx_); return tau_deact_; }
            double GetOptPennationAngle() const { read_lock lock(mtx_); return alpha_opt_; }
            double GetMaxContractileForce() const { read_lock lock(mtx_); return f_max_active_; }
            double GetMaxContractileForceLength() const { read_lock lock(mtx_); return l_max_active_force_; }
            double GetMaxPassiveForceLength() const { read_lock lock(mtx_); return l_max_; }
            double GetZeroPassiveForceLength() const { read_lock lock(mtx_); return l_zero_passive_force_; }
            double GetTendonSlackLength() const { read_lock lock(mtx_); return l_tendon_slack_; }
            double GetMaxContractionVelocity() const { read_lock lock(mtx_); return vel_max_contraction_; };
            double GetFractionPassiveTension() const { read_lock lock(mtx_); return passive_tension_frac_; }
            double GetForceScale() const { read_lock lock(mtx_); return force_scale_; }
            double GetTendonRatio() const { read_lock lock(mtx_); return tendon_ratio_; }

            /**
             * Get the excitation function. If no excitation function is set, a nullptr is returned.
             * This function per se is thread-safe, but if the ChFunction object is changed, the user has to
             * ensure thread-safety. When only the current excitation value is sufficient, use \p GetCurrentExcitation.
             * @return A shared pointer to a ChFunction object representing the excitation.
             */
            std::shared_ptr<ChFunction> GetExcitation() {
                read_lock lock(mtx_);
                return excitation_;
            }

            tk::spline GetActiveForceLengthCurve() const { read_lock lock(mtx_); return *active_f_l_curve_; }
            tk::spline GetPassiveForceLengthCurve() const { read_lock lock(mtx_); return *passive_f_l_curve_; }
            tk::spline GetTendonForceLengthCurve() const { read_lock lock(mtx_); return *tendon_f_l_curve_; }
            tk::spline GetForceVelocityCurve() const { read_lock lock(mtx_); return *f_vel_curve_; }

            // -------------
            // Serialization
            // -------------

            /**
             * Method to allow serialization of transient data to archives.
             * @param archive_out ChArchiveOut.
             */
            void ArchiveOut(ChArchiveOut& archive_out) override;

            /**
             * Method to allow serialization of transient data from archives.
             * @param archive_in ChArchiveIn.
             */
            void ArchiveIn(ChArchiveIn& archive_in) override;

        protected:
            // -----------------
            // Protected classes
            // -----------------

            class ChApi MuscleForceFunctor : public ForceFunctor {
                public:
                    explicit MuscleForceFunctor(ChMuscle& muscle) : muscle_{muscle} {
                    }

                    double evaluate(double time,
                                    double rest_length,
                                    double length,
                                    double vel,
                                    const ChLinkTSDA& link) override {
                        return muscle_.ComputeMuscleForce(time, rest_length, length, vel, muscle_);
                    }

                private:
                    ChMuscle& muscle_;
            };

            class ChApi MuscleActivationODE : public ODE {
                public:
                    explicit MuscleActivationODE(ChMuscle& muscle) : muscle_(muscle) {
                    }

                    /**
                     * Calculate and return the \p ODE right-hand side at the provided time and states.
                     * Must load f(t,y). \n\n
                     * \f{equation*}{
                     * \dot{a} = f(a, u) = \frac{u - \hat{a}}{\tau}\quad \text{with} \quad \tau = \begin{cases}
                     * \tau_{\text{act}} \cdot (0.5 + 1.5\, \hat{a}) & \text{if} \quad u > \hat{a} \\
                     * \frac{\tau_{\text{deact}}}{0.5 + 1.5\, \hat{a}} & \text{otherwise}
                     * \end{cases},\quad
                     * \hat{a} = \frac{a - a_{\text{min}}}{1 - a_{\text{min}}}.
                     * \f}
                     * @param time Current time.
                     * @param states Current ODE states.
                     * @param rhs Current ODE right-hand side vector.
                     * @param link Associated muscle.
                     */
                    void CalculateRHS(double time,
                                      const ChVectorDynamic<>& states,
                                      ChVectorDynamic<>& rhs,
                                      const ChLinkTSDA& link) override {
                        double tau;
                        const float a_min = muscle_.GetMinimumActivation();
                        const double a_current = states(0);
                        const float e_current = muscle_.GetCurrentExcitation();
                        const double a_hat = (a_current - a_min) / (1.0 - a_min);
                        const double c_1 = 0.5 + 1.5 * a_hat;

                        if (e_current > a_hat) {
                            tau = muscle_.GetActivationTimeConstant() * c_1;
                        } else {
                            tau = muscle_.GetDeactivationTimeConstant() / c_1;
                        }

                        rhs(0) = (e_current - a_hat) / tau;
                    }

                    /**
                     * Calculate the Jacobian of the \p ODE right-hand side with respect to the \p ODE states.
                     * Only used if the link force is declared as stiff. If provided, load df/dy into the
                     * provided matrix \p jac (already set to zero before the call) and return \p true. In that case,
                     * the user-provided Jacobian will overwrite the default finite-difference approximation. \n\n
                     * \f{equation*}{
                     * \frac{\partial f(a, u)}{\partial a} = \frac{-1}{\tau (1 - a_{\text{min}})}.
                     * \f}
                     * @param time Current time.
                     * @param states Current ODE states.
                     * @param rhs Current ODE right-hand side vector.
                     * @param jac Output Jacobian matrix.
                     * @param link Associated muscle.
                     * @return True if finite-difference approximation is to be overwritten with user definition.
                     */
                    bool CalculateJac(double time,
                                      const ChVectorDynamic<>& states,
                                      const ChVectorDynamic<>& rhs,
                                      ChMatrixDynamic<>& jac,
                                      const ChLinkTSDA& link) override {
                        const float tau_a = muscle_.GetActivationTimeConstant();
                        const float tau_d = muscle_.GetDeactivationTimeConstant();
                        const float e_current = muscle_.GetCurrentExcitation();
                        const float a_min = muscle_.GetMinimumActivation();
                        const double a_hat = (states(0) - a_min) / (1.0 - a_min);

                        if (e_current > a_hat) {
                            const double den = -1.5 * a_hat - 0.5;
                            jac(0, 0) = 1.5 * (e_current - a_hat) / (tau_a * (a_min - 1) * den * den) - 1 / (
                                            tau_a * (a_min - 1) * den);
                        } else {
                            jac(0, 0) = (1.5 * (a_hat - e_current) + (0.5 + 1.5 * a_hat)) / ((a_min - 1) * tau_d);
                        }

                        return true;
                    }

                    /**
                     * Set initial conditions. Must load y0 = y(0).
                     * @param states Output initial conditions vector.
                     * @param link Associated muscle.
                     */
                    void SetInitialConditions(ChVectorDynamic<>& states, const ChLinkTSDA& link) override {
                        states = ChVectorDynamic<>(GetNumStates());
                        states(0) = 0.0;
                    }

                    /**
                     * Specify number of states (dimension of y).
                     * @return Number of states.
                     */
                    unsigned int GetNumStates() const override { return 1; }

                private:
                    ChMuscle& muscle_;
            };

            // -------------------
            // Protected functions
            // -------------------

            /**
             * TODO \n\n
             * Reference: ArtiSynth \p AxialMuscleMaterialBase
             * (\link https://www.artisynth.org/doc/javadocs/artisynth/core/materials/AxialMuscleMaterialBase.html \endlink )
             * @param l_min_active_norm_fiber
             * @param l_transition_norm_fiber
             * @param l_max_active_norm_fiber
             * @param slope_shallow_ascending
             * @param f_min
             */
            void CreateActiveForceLengthCurve(double l_min_active_norm_fiber,
                                              double l_transition_norm_fiber,
                                              double l_max_active_norm_fiber,
                                              double slope_shallow_ascending,
                                              double f_min);

            void CreatePassiveForceLengthCurve(double strain_at_zero_force,
                                               double strain_at_one_norm_force,
                                               double stiffness_at_low_force,
                                               double stiffness_at_one_norm_force,
                                               double curviness);

            void CreateTendonForceLengthCurve(double strain_at_one_norm_force,
                                              double stiffness_at_one_norm_force,
                                              double f_norm_at_toe_end,
                                              double curviness);

            void CreateForceVelocityCurve(double slope_concentric_at_vmax,
                                          double slope_concentric_near_vmax,
                                          double slope_isometric,
                                          double slope_eccentric_at_vmax,
                                          double slope_eccentric_near_vmax,
                                          double max_eccentric_vel_f_multiplier,
                                          double concentric_curviness,
                                          double eccentric_curviness);

            // -------------------
            // Protected variables
            // -------------------

            bool verbose_{false};

            mutable mutex_type mtx_;

            ChMuscleType type_{}; ///< Muscle type

            bool rigid_tendon_{false}; ///< True if tendons are to be modeled inextensible
            bool ignore_f_vel_{false}; ///< True if force-velocity relationship is to be ignored
            bool activation_dynamics_{false}; ///< True if activation dynamics are represented by an ODE

            float e_current_{0.0}; ///< Current excitation value, element of [0, 1]
            float a_current_{0.0}; ///< Current activation value, element of [0, 1]
            float a_min_{0.0}; ///< Minimum activation value >= 0.0
            float tau_act_{10.0e-3}; ///< Activation time constant
            float tau_deact_{40.0e-3}; ///< Deactivation time constant

            double alpha_opt_{0.0}; ///< Optimal pennation angle in radians at \p l_max_active_force_

            double f_max_active_{1.0}; ///< Max. contractile force (element of R⁺)
            double l_max_{1.5}; ///< Max. length of the muscle
            double l_max_active_force_{1.0}; ///< Length at which maximum contractile force is generated
            double l_zero_passive_force_{1.0};
            ///< Length at which zero passive force is generated
                              ///< (typically: \p l_max_active_force_ <= \p l_zero_passive_force_ < \p l_max_)
            double l_tendon_slack_{0.1}; ///< Tendon slack length

            double vel_max_contraction_{10.0}; ///< Max. contraction velocity

            double passive_tension_frac_{0.0}; ///< Fraction of \p f_max_active_ applied as passive tension
            double force_scale_{1.0}; ///< Factor for scaling the muscle force
            double tendon_ratio_{0.0}; ///< Tendon to fibre length ratio < 1.0
            double h_fiber_{0.0}; ///< Height of muscle fibers

            double time_prev_{-1.0}; ///< Previous time

            std::shared_ptr<ChFunction> excitation_{nullptr}; ///< Muscle excitation element of [0, 1]
            std::shared_ptr<ODE> ode_{nullptr}; ///< Muscle activation dynamics
            std::shared_ptr<MuscleForceFunctor> muscle_force_{nullptr};

            std::shared_ptr<tk::spline> active_f_l_curve_{nullptr};
            std::shared_ptr<tk::spline> passive_f_l_curve_{nullptr};
            std::shared_ptr<tk::spline> tendon_f_l_curve_{nullptr};
            std::shared_ptr<tk::spline> f_vel_curve_{nullptr};
    };

    /**
     * @class ChMuscleTSDA
     * @brief TODO
     */
    class ChApi ChMuscleTSDA : public ChMuscle {
        public:
            ChMuscleTSDA() : ChMuscleTSDA(nullptr) {
            }

            ChMuscleTSDA(const std::shared_ptr<ChFunction>& excitation,
                         const ChMuscleParameterVector& params)
                : ChMuscleTSDA(excitation,
                               std::get<double>(params[0]),
                               std::get<double>(params[1]),
                               std::get<double>(params[2])) {
            }

            explicit ChMuscleTSDA(const std::shared_ptr<ChFunction>& excitation,
                                  double k = 0.0,
                                  double d = 0.0,
                                  double f_max_active = 1.0);

            double ComputeMuscleForce(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChMuscle& muscle) override;

            // -------------
            // Serialization
            // -------------

            /**
             * Method to allow serialization of transient data to archives.
             * @param archive_out ChArchiveOut
             */
            void ArchiveOut(ChArchiveOut& archive_out) override;

            /**
             * Method to allow serialization of transient data from archives.
             * @param archive_in ChArchiveIn
             */
            void ArchiveIn(ChArchiveIn& archive_in) override;
    };

    /**
     * @class ChMuscleConstant
     * @brief TODO
     */
    class ChApi ChMuscleConstant : public ChMuscle {
        public:
            ChMuscleConstant() : ChMuscleConstant(nullptr) {
            }

            ChMuscleConstant(const std::shared_ptr<ChFunction>& excitation,
                             const ChMuscleParameterVector& params)
                : ChMuscleConstant(excitation,
                                   std::get<double>(params[0]),
                                   std::get<double>(params[1]),
                                   std::get<double>(params[2])) {
            }

            explicit ChMuscleConstant(const std::shared_ptr<ChFunction>& excitation,
                                      double d = 0.0,
                                      double f_max_active = 1.0,
                                      double passive_tension_frac = 0.0);

            double ComputeMuscleForce(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChMuscle& muscle) override;

            // -------------
            // Serialization
            // -------------

            /**
             * Method to allow serialization of transient data to archives.
             * @param archive_out ChArchiveOut
             */
            void ArchiveOut(ChArchiveOut& archive_out) override;

            /**
             * Method to allow serialization of transient data from archives.
             * @param archive_in ChArchiveIn
             */
            void ArchiveIn(ChArchiveIn& archive_in) override;
    };

    /**
     * @class ChMuscleLinear
     * @brief TODO
     */
    class ChApi ChMuscleLinear : public ChMuscle {
        public:
            ChMuscleLinear() : ChMuscleLinear(nullptr) {
            }

            ChMuscleLinear(const std::shared_ptr<ChFunction>& excitation,
                           const ChMuscleParameterVector& params)
                : ChMuscleLinear(excitation,
                                 std::get<double>(params[0]),
                                 std::get<double>(params[1]),
                                 std::get<double>(params[2]),
                                 std::get<double>(params[3]),
                                 std::get<double>(params[4])) {
            }

            explicit ChMuscleLinear(const std::shared_ptr<ChFunction>& excitation,
                                    double d = 0.0,
                                    double f_max_active = 1.0,
                                    double passive_tension_frac = 0.0,
                                    double l_max_force = 1.0,
                                    double l_zero_passive_force = 0.0);

            double ComputeMuscleForce(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChMuscle& muscle) override;

            // -------------
            // Serialization
            // -------------

            /**
             * Method to allow serialization of transient data to archives.
             * @param archive_out ChArchiveOut
             */
            void ArchiveOut(ChArchiveOut& archive_out) override;

            /**
             * Method to allow serialization of transient data from archives.
             * @param archive_in ChArchiveIn
             */
            void ArchiveIn(ChArchiveIn& archive_in) override;
    };

    /**
     * @class ChMusclePeck
     * @brief Typical Hill-type active force-length relationship (modeled as a cosine), but the passive force-length
     * properties are linear. This muscle model was empirically verified for jaw muscles during wide jaw opening.\n\n
     * Implementation similar to ArtiSynth's \p PeckAxialMuscle
     * (\link https://www.artisynth.org/doc/artisynth_core_3.3/javadocs/artisynth/core/materials/PeckAxialMuscle.html \endlink ).
     */
    class ChApi ChMusclePeck : public ChMuscle {
        public:
            ChMusclePeck() : ChMusclePeck(nullptr) {
            }

            ChMusclePeck(const std::shared_ptr<ChFunction>& excitation,
                         const ChMuscleParameterVector& params)
                : ChMusclePeck(excitation,
                               std::get<double>(params[0]),
                               std::get<double>(params[1]),
                               std::get<double>(params[2]),
                               std::get<double>(params[3]),
                               std::get<double>(params[4]),
                               std::get<double>(params[5]),
                               std::get<double>(params[6]),
                               std::get<double>(params[7])) {
            }

            explicit ChMusclePeck(const std::shared_ptr<ChFunction>& excitation,
                                  double d = 0.0,
                                  double f_max_active = 1.0,
                                  double passive_tension_frac = 0.0,
                                  double l_max_active_force = 1.0,
                                  double l_max = 1.5,
                                  double l_zero_passive_force = 1.0,
                                  double force_scale = 1.0,
                                  double tendon_ratio = 0.0);

            double ComputeMuscleForce(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChMuscle& muscle) override;

            void SetLinearPassiveForce(bool linear) { linear_ = linear; }

            // -------------
            // Serialization
            // -------------

            /**
             * Method to allow serialization of transient data to archives.
             * @param archive_out ChArchiveOut
             */
            void ArchiveOut(ChArchiveOut& archive_out) override;

            /**
             * Method to allow serialization of transient data from archives.
             * @param archive_in ChArchiveIn
             */
            void ArchiveIn(ChArchiveIn& archive_in) override;

        private:
            const double MIN_STRETCH_{0.5}; ///< Percentage of optimal fibre length (\p l_max_active_force_)
            const double MAX_STRETCH_{1.5}; ///< Percentage of optimal fibre length (\p l_max_active_force_)

            bool linear_{false}; ///< True if passive force shall be linear. Exponential, otherwise.
    };

    /**
     * @class ChMuscleMillard
     * @brief Implementation of a Hill-type muscle model described by Millard et al.
     * For now, only the rigid-tendon case is implemented. \n\n
     * Reference: \n
     *  - OpenSim (\link https://simtk-confluence.stanford.edu:8443/display/OpenSim/Millard+2012+Muscle+Models \endlink) \n
     *  - Paper (\link https://asmedigitalcollection.asme.org/biomechanical/article/135/2/021005/371394/Flexing-Computational-Muscle-Modeling-and \endlink)
     */
    class ChApi ChMuscleMillard : public ChMuscle {
        public:
            ChMuscleMillard() : ChMuscleMillard(nullptr) {
            }

            ChMuscleMillard(const std::shared_ptr<ChFunction>& excitation,
                            const ChMuscleParameterVector& params)
                : ChMuscleMillard(excitation,
                                  std::get<double>(params[0]),
                                  std::get<double>(params[1]),
                                  std::get<double>(params[2]),
                                  std::get<double>(params[3]),
                                  std::get<double>(params[4]),
                                  std::get<double>(params[5]),
                                  std::get<bool>(params[6]),
                                  std::get<bool>(params[7])) {
            }

            /**
             * Constructor for the Millard muscle model.
             * @param excitation The excitation function.
             * @param d Damping factor.
             * @param f_max_active Peak isometric force.
             * @param l_max_active_force Length of muscle fibers at which muscle develops peak isometric active force.
             * @param vel_max_contraction Maximum contraction velocity.
             * @param tendon_ratio Tendon slack to optimal muscle fiber length (l_tendon_slack/l_max_active_force).
             * Slack length: length at which the tendon begins to develop tensile force.
             * @param alpha_opt Pennation angle between tendon and fibers at optimal fiber length expressed in radians.
             * Limited to 1.47 rad (~84°).
             * @param rigid_tendon True if tendon is rigid.
             * @param ignore_f_vel True if force-velocity relationship is to be ignored.
             */
            explicit ChMuscleMillard(const std::shared_ptr<ChFunction>& excitation,
                                     double d = 0.0,
                                     double f_max_active = 1.0,
                                     double l_max_active_force = 1.0,
                                     double vel_max_contraction = 1.0,
                                     double tendon_ratio = 0.1,
                                     double alpha_opt = 0.0,
                                     bool rigid_tendon = true,
                                     bool ignore_f_vel = false);

            double ComputeMuscleForce(double time,
                                      double rest_length,
                                      double length,
                                      double vel,
                                      const ChMuscle& muscle) override;

            // -------------
            // Serialization
            // -------------

            /**
             * Method to allow serialization of transient data to archives.
             * @param archive_out ChArchiveOut
             */
            void ArchiveOut(ChArchiveOut& archive_out) override;

            /**
             * Method to allow serialization of transient data from archives.
             * @param archive_in ChArchiveIn
             */
            void ArchiveIn(ChArchiveIn& archive_in) override;
    };
} // namespace chrono::biomechanics

namespace chrono {
    CH_CLASS_VERSION(biomechanics::ChMuscle, 0)

    CH_CLASS_VERSION(biomechanics::ChMuscleTSDA, 0)

    CH_CLASS_VERSION(biomechanics::ChMuscleConstant, 0)

    CH_CLASS_VERSION(biomechanics::ChMuscleLinear, 0)

    CH_CLASS_VERSION(biomechanics::ChMusclePeck, 0)

    CH_CLASS_VERSION(biomechanics::ChMuscleMillard, 0)
}


#endif // EXOSIM_CH_MUSCLE_H
