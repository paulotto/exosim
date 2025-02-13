/**
 * @file ChLigament.h
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 24.08.2023
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

#ifndef EXOSIM_CH_LIGAMENT_H
#define EXOSIM_CH_LIGAMENT_H

#include <vector>
#include <variant>
#include <shared_mutex>
//
#include "exosim/biomechanics/utils/utils.h"
//
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChBody.h>
//
#include <chrono/fea/ChMesh.h>
//
#include <chrono/assets/ChVisualShapePointPoint.h>
//
#include <chrono/serialization/ChArchive.h>

namespace chrono::biomechanics {
    /**
     * @class ChLigament
     * @inherit \p ChLinkTSDA
     * @brief TODO
     */
    class ChApi ChLigament : public ChLinkTSDA {
        public:
            using ChLigamentParameterVector = std::vector<std::variant<double, bool> >;

            using mutex_type = std::shared_mutex;
            using read_lock = std::shared_lock<mutex_type>;
            using write_lock = std::unique_lock<mutex_type>;

            /**
             * @struct ChLigamentProperties
             * @brief A struct for creating a larger amount of ligaments more conveniently.
             */
            struct ChLigamentProperties {
                std::string name; ///< Muscle name

                ChLigament::ChLigamentParameterVector params; ///< Parameters to initialize a \p ChLigament

                std::shared_ptr<ChBody> body_1; ///< 1st body for the attachment
                std::shared_ptr<ChBody> body_2; ///< 2nd body for the attachment

                ChVector3d loc_1; ///< Coordinates specifying location on 1st body
                ChVector3d loc_2; ///< Coordinates specifying location on 2nd body

                bool local_coordinates; ///< True if attachment points defined relative to bodies
                bool rel_to_ref; ///< True if attachment points defined relative to REF frame of bodies

                ChColor color; ///< Ligament color
                std::shared_ptr<ChVisualShapePointPoint> shape; ///< Visual model

                ChLigamentProperties()
                    : ChLigamentProperties("",
                                           {},
                                           nullptr,
                                           nullptr,
                                           ChVector3d(0, 0, 0),
                                           ChVector3d(0, 0, 0),
                                           true,
                                           true,
                                           ChColor(0.13f, 0.54f, 0.13f),
                                           chrono_types::make_shared<ChVisualShapeSpring>(0.0025, 100, 20)) {
                }

                ChLigamentProperties(
                    std::string name,
                    ChLigamentParameterVector params,
                    const std::shared_ptr<ChBody>& body_1,
                    const std::shared_ptr<ChBody>& body_2,
                    const ChVector3d& loc_1,
                    const ChVector3d& loc_2,
                    bool local_coordinates = true,
                    bool rel_to_ref = true,
                    const ChColor& color = ChColor(0.13f, 0.54f, 0.13f),
                    const std::shared_ptr<ChVisualShapePointPoint>& shape = chrono_types::make_shared<
                        ChVisualShapeSpring>(0.0025, 100, 20)
                );

                virtual ~ChLigamentProperties() = default;

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
                    archive_out << CHNVP(shape);
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
                    archive_in >> CHNVP(shape);
                }
            };

            /**
             * @enum ChLigamentType
             * @brief Supported types of ligament models.
             */
            enum ChLigamentType {
                DEFAULT, ///< Default ligament
                BLANKERVOORT ///< Blankevoort et al. (1991)
            };

            /**
             * @class ChLigamentEnumMapper
             * @brief Can be used to convert between enums and strings.
             */
            class ChLigamentEnumMapper final {
                public:
                    CH_ENUM_MAPPER_BEGIN(ChLigamentType)
                                CH_ENUM_VAL(DEFAULT)
                                CH_ENUM_VAL(BLANKERVOORT)
                    CH_ENUM_MAPPER_END(ChLigamentType)
            };

            explicit ChLigament(const ChLigamentParameterVector& params)
                : ChLigament(std::get<double>(params[0]),
                             std::get<double>(params[1]),
                             std::get<double>(params[2]),
                             std::get<double>(params[3]),
                             std::get<double>(params[4])) {
            }

            explicit ChLigament(double d = 0.0,
                                double k_c = 0.0,
                                double k_e = 0.0,
                                double l_rest = 0.0,
                                double l_rest_percentage = 0.01)
                : k_c_(k_c), k_e_(k_e), l_rest_percentage_(l_rest_percentage) {
                this->SetDampingCoefficient(d);
                this->SetRestLength(l_rest);

                ligament_force_ = chrono_types::make_shared<LigamentForceFunctor>(*this);
                this->RegisterForceFunctor(ligament_force_);
            }

            /**
             * Computes and returns the general ligament force. If the ligament has internal \p ODE states,
             * the current states can be accessed with
             * @code{.unparsed}
             * ligament->GetStates();
             * @endcode
             * @param time Current time.
             * @param rest_length Undeformed length of the ligament.
             * @param length Current length.
             * @param vel Current velocity. Positive when extending.
             * @param ligament Associated ligament.
             * @return General ligament force.
             */
            virtual double ComputeLigamentForce(double time,
                                                double rest_length,
                                                double length,
                                                double vel,
                                                const ChLigament& ligament
            ) = 0;

            // ------
            // Setter
            // ------

            void SetVerbose(bool verbose) {
                write_lock lock(mtx_);
                verbose_ = verbose;
            }

            // ------
            // Getter
            // ------

            ChLigamentType GetLigamentType() const {
                read_lock lock(mtx_);
                return type_;
            }

            double GetCompressionStiffness() const {
                read_lock lock(mtx_);
                return k_c_;
            }

            double GetElongationStiffness() const {
                read_lock lock(mtx_);
                return k_e_;
            }

            double GetRestLengthPercentage() const {
                read_lock lock(mtx_);
                return l_rest_percentage_;
            }

            void SetCompressionStiffness(double k_c) {
                read_lock lock(mtx_);
                k_c_ = k_c;
            }

            void SetElongationStiffness(double k_e) {
                read_lock lock(mtx_);
                k_e_ = k_e;
            }

            void SetRestLengthPercentage(double l_0_percentage) {
                read_lock lock(mtx_);
                l_rest_percentage_ = l_0_percentage;
            }

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

        protected:
            // -----------------
            // Protected classes
            // -----------------

            class LigamentForceFunctor : public ForceFunctor {
                public:
                    explicit LigamentForceFunctor(ChLigament& ligament) : ligament_{ligament} {
                    }

                    double evaluate(double time,
                                    double rest_length,
                                    double length,
                                    double vel,
                                    const ChLinkTSDA& link) override {
                        return ligament_.ComputeLigamentForce(time, rest_length, length, vel, ligament_);
                    }

                    virtual void ArchiveOut(ChArchiveOut& archive_out) {
                        archive_out << CHNVP(ligament_);
                    }

                    virtual void ArchiveIn(ChArchiveIn& archive_in) {
                        archive_in >> CHNVP(ligament_);
                    }

                private:
                    ChLigament& ligament_;
            };

            // -------------------
            // Protected variables
            // -------------------

            bool verbose_{false};

            mutable mutex_type mtx_;

            ChLigamentType type_{}; ///< Ligament type

            double k_c_{0.0}; ///< Compression stiffness
            double k_e_{0.0}; ///< Elongation stiffness
            double l_rest_percentage_{0.01}; ///< Percentage of rest length
            double time_prev_{-1.0};

            std::shared_ptr<LigamentForceFunctor> ligament_force_{nullptr};
    };

    /**
     * @class ChLigamentDefault
     * @inherit ChLigament
     * @brief Reference: ArtiSynth: \p LigamentAxialMaterial \n
     * (\link https://www.artisynth.org/doc/javadocs/artisynth/core/materials/LigamentAxialMaterial.html \endlink )
     */
    class ChApi ChLigamentDefault : public ChLigament {
        public:
            explicit ChLigamentDefault(const ChLigamentParameterVector& params)
                : ChLigamentDefault(std::get<double>(params[0]),
                                    std::get<double>(params[1]),
                                    std::get<double>(params[2]),
                                    std::get<double>(params[3]),
                                    std::get<double>(params[4])) {
            }

            explicit ChLigamentDefault(double d = 0.0,
                                       double k_c = 0.0,
                                       double k_e = 0.0,
                                       double l_rest = 0.0,
                                       double l_rest_percentage = 0.01)
                : ChLigament(d, k_c, k_e, l_rest, l_rest_percentage) {
                type_ = ChLigamentType::DEFAULT;
            }

            double ComputeLigamentForce(double time,
                                        double rest_length,
                                        double length,
                                        double vel,
                                        const ChLigament& ligament) override;
    };

    /**
     * @class ChLigamentBlankevoort
     * @inherit ChLigament
     * @brief The ligament model used in the University of Wisconsin knee model.
     * Based on: Blankevoort, L., and Huiskes, R. (1991). Ligament-bone
     * interaction in a three-dimensional model of the knee.
     * Journal of Biomechanical Engineering, 113(3), 263-269. \n\n
     * \f{align*}{
     * f_s &= \begin{cases}
     * k_e\, (\epsilon - 0.5\, \epsilon_{\text{transition}}) & \text{if} \quad \epsilon \ge \epsilon_{\text{transition}} \\
     * 0.25\, k_e\, \frac{\epsilon^2}{0.5\, \epsilon_{\text{transition}}} & \text{if} \quad \epsilon > 0 \land
     * \epsilon < \epsilon_{\text{transition}} \\
     * 0 & \text{otherwise}
     * \end{cases}\quad \text{with} \quad \epsilon = \frac{l - l_{\text{slack}}}{l_{\text{slack}}}, \quad
     * l_{\text{slack}} = \frac{l_{\text{ref}}}{1 + \epsilon_{\text{ref}}} \\
     * f_d &= d\, \dot{\epsilon} \quad \text{with} \quad \dot{\epsilon} = \frac{\dot{l}}{l_{\text{slack}}} \\
     * f &= f_s + f_d.
     * \f}
     */
    class ChApi ChLigamentBlankevoort : public ChLigament {
        public:
            explicit ChLigamentBlankevoort(const ChLigamentParameterVector& params)
                : ChLigamentBlankevoort(std::get<double>(params[0]),
                                        std::get<double>(params[1]),
                                        std::get<double>(params[2]),
                                        std::get<double>(params[3]),
                                        std::get<double>(params[4])) {
            }

            /**
             * Constructor.
             * @param l_ref The length of the ligament when the joint is in its arbitrary reference position.
             * @param e_ref The strain at the reference length.
             * @param k_e The stiffness.
             * @param e_transition The strain at the transition from toe to linear.
             * @param d The damping factor.
             */
            explicit ChLigamentBlankevoort(double l_ref = 1.0,
                                           double e_ref = 0.25,
                                           double k_e = 100.0,
                                           double e_transition = 0.06,
                                           double d = 0.003)
                : ChLigament(d, 0.0, k_e, 0.0, 0.0),
                  l_ref_(l_ref), e_ref_(e_ref), e_transition_(e_transition) {
                type_ = ChLigamentType::BLANKERVOORT;

                this->SetRestLength(l_ref / (e_ref + 1));
                utils::throw_invalid_argument(this->GetRestLength() == 0.0,
                                              "[ERROR] [ChLigamentBlankevoort] l_ref cannot be zero!\n");
                utils::throw_invalid_argument(e_transition == 0.0,
                                              "[ERROR] [ChLigamentBlankevoort] e_transition cannot be zero!\n");
            }

            double ComputeLigamentForce(double time,
                                        double rest_length,
                                        double length,
                                        double vel,
                                        const ChLigament& ligament) override;

            double GetReferenceLength() const { read_lock lock(mtx_); return l_ref_; }

            double GetReferenceStrain() const { read_lock lock(mtx_); return e_ref_; }

            double GetTransitionStrain() const { read_lock lock(mtx_); return e_transition_; }

            void SetReferenceLength(double l_reference) {
                write_lock lock(mtx_);
                l_ref_ = l_reference;
                // Slack length / Zero load length
                this->SetRestLength(l_ref_ / (e_ref_ + 1));
            }

            void SetReferenceStrain(double e_reference) {
                write_lock lock(mtx_);
                e_ref_ = e_reference;
                // Slack length / Zero load length
                this->SetRestLength(l_ref_ / (e_ref_ + 1));
            }

            void SetTransitionStrain(double e_transition) {
                write_lock lock(mtx_);
                e_transition_ = e_transition;
            }

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
            double l_ref_{0.0}; ///< Length of the ligament when the joint is in its arbitrary reference position
            double e_ref_{0.0}; ///< Strain at l_ref, defines the slack length indirectly
            double e_transition_{0.06}; ///< Strain at transition from toe to linear
    };
} // namespace chrono::biomechanics

namespace chrono {
    CH_CLASS_VERSION(biomechanics::ChLigament, 0)

    CH_CLASS_VERSION(biomechanics::ChLigamentDefault, 0)

    CH_CLASS_VERSION(biomechanics::ChLigamentBlankevoort, 0)
}


#endif // EXOSIM_CH_LIGAMENT_H
