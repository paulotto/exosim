/**
 * @file ChContactForceTorqueJawSMC.h
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 01.09.2023
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

// MODIFIED FROM PROJECT CHRONO: ChContactSMC.h
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef EXOSIM_CH_CONTACT_JAW_SMC_H
#define EXOSIM_CH_CONTACT_JAW_SMC_H

#include <map>
#include <cmath>
#include <cassert>
#include <algorithm>
//
#include "exosim/biomechanics/utils/utils.h"
//
#include "chrono/collision/ChCollisionModel.h"
//
#include "chrono/core/ChMatrix.h"
//
#include "chrono/solver/ChKRMBlock.h"
//
#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactTuple.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChSystemSMC.h"
//
#include <chrono/serialization/ChArchive.h>


namespace chrono::biomechanics {
    /**
     * @struct ContactBehaviorMapComp
     * @brief Used as comparison operator to sort the elements in the ContactBehaviorMap.
     * Otherwise, std::pair<ChContactable*, ChContactable*> won't work as map key.
     * This implementation was chosen to make the order of the elements in the pair used as key irrelevant.
     */
    struct ContactBehaviorMapComp {
        bool operator()(std::pair<ChContactable*, ChContactable*> const& lhs,
                        std::pair<ChContactable*, ChContactable*> const& rhs) const {
            const auto lhs_mass_first = lhs.first->GetContactableMass();
            const auto lhs_mass_second = lhs.second->GetContactableMass();
            const auto rhs_mass_first = rhs.first->GetContactableMass();
            const auto rhs_mass_second = rhs.second->GetContactableMass();

            const auto lhs_min = std::min(lhs_mass_first, lhs_mass_second);
            const auto lhs_max = std::max(lhs_mass_first, lhs_mass_second);
            const auto rhs_min = std::min(rhs_mass_first, rhs_mass_second);
            const auto rhs_max = std::max(rhs_mass_first, rhs_mass_second);

            return lhs_min < rhs_min || (!(rhs_min < lhs_min) && lhs_max < rhs_max);
        }
    };

    /**
     * @class ChContactForceTorqueJawSMC
     * @brief TODO
     * @inherit ChDefaultContactForceSMC
     */
    class ChApi ChContactForceTorqueJawSMC : public ChSystemSMC::ChContactForceTorqueSMC {
        public:
            /**
             * @enum ContactForceTorqueModel
             * @brief Contact types supported by the jaw model.
             */
            enum ContactForceTorqueModel {
                Hooke, ///< linear Hookean model
                Hertz, ///< nonlinear Hertzian model
                PlainCoulomb, ///< basic tangential force definition for non-granular bodies
                Flores, ///< not implemented yet
                EFC ///< elastic foundation contact model after Bey and Fregly (2004)
            };

            /**
            * @enum AdhesionForceTorqueModel
            * @brief Adhesion force models supported by the jaw model.
            */
            enum AdhesionForceTorqueModel {
                NoneAdhesion, ///< no adhesion force
                Constant, ///< constant adhesion force
                DMT, ///< Derjagin-Muller-Toropov model
                Perko ///< Perko et al. (2001) model. Not implemented yet
            };

            /**
            * @enum TangentialDisplacementModel
            * @brief Tangential displacement models supported by the jaw model.
            */
            enum TangentialDisplacementModel {
                NoneTangential, ///< no tangential force
                OneStep, ///< use only current relative tangential velocity
                MultiStep ///< use contact history (from contact initiation). Not implemented yet
            };

            /**
             * @class ChContactForceTorqueJawSMCEnumMapper
             * @brief Can be used to convert between enums and strings.
             */
            class ChContactForceTorqueJawSMCEnumMapper final {
                public:
                    CH_ENUM_MAPPER_BEGIN(ContactForceTorqueModel)
                                CH_ENUM_VAL(Hooke)
                                CH_ENUM_VAL(Hertz)
                                CH_ENUM_VAL(PlainCoulomb)
                                CH_ENUM_VAL(EFC)
                                CH_ENUM_VAL(Flores)
                    CH_ENUM_MAPPER_END(ContactForceTorqueModel)

                    CH_ENUM_MAPPER_BEGIN(AdhesionForceTorqueModel)
                                CH_ENUM_VAL(NoneAdhesion)
                                CH_ENUM_VAL(Constant)
                                CH_ENUM_VAL(DMT)
                                CH_ENUM_VAL(Perko)
                    CH_ENUM_MAPPER_END(AdhesionForceTorqueModel)

                    CH_ENUM_MAPPER_BEGIN(TangentialDisplacementModel)
                                CH_ENUM_VAL(NoneTangential)
                                CH_ENUM_VAL(OneStep)
                                CH_ENUM_VAL(MultiStep)
                    CH_ENUM_MAPPER_END(TangentialDisplacementModel)
            };

            /**
             * @struct ContactModelsSpecifications
             * @brief TODO
             */
            struct ContactModelsSpecifications {
                ContactForceTorqueModel contact_force_model{ContactForceTorqueModel::Hertz};
                AdhesionForceTorqueModel adhesion_force_model{AdhesionForceTorqueModel::Constant};
                TangentialDisplacementModel tangential_displacement_model{TangentialDisplacementModel::OneStep};
            };

            using ContactBehaviorMap = std::map<std::pair<ChContactable*, ChContactable*>,
                ContactModelsSpecifications, ContactBehaviorMapComp>;

            ChContactForceTorqueJawSMC(ChSystemSMC* sys,
                                       ContactBehaviorMap* contact_behavior)
                : sys_(sys),
                  contact_behavior_(contact_behavior) {
            }

            ~ChContactForceTorqueJawSMC() override = default;

            /**
             * Calculate the contact force (resultant of both normal and tangential components) for a contact between two
             * objects, obj1 and obj2. Note that this function is always called with delta > 0.
             * @param sys Containing system
             * @param normal_dir Normal contact direction (expressed in global frame)
             * @param p1 Most penetrated point on obj1 (expressed in global frame)
             * @param p2 Most penetrated point on obj2 (expressed in global frame)
             * @param vel1 Velocity of contact point on obj1 (expressed in global frame)
             * @param vel2 Velocity of contact point on obj2 (expressed in global frame)
             * @param mat Composite material for contact pair
             * @param delta Overlap in normal direction
             * @param eff_radius Effective radius of curvature at contact
             * @param mass1 Mass of obj1
             * @param mass2 Mass of obj2
             * @param objA Pointer to contactable obj1
             * @param objB Pointer to contactable obj2
             * @return The contact force
             */
            ChWrenchd CalculateForceTorque(
                const ChSystemSMC& sys,
                const ChVector3d& normal_dir,
                const ChVector3d& p1,
                const ChVector3d& p2,
                const ChVector3d& vel1,
                const ChVector3d& vel2,
                const ChContactMaterialCompositeSMC& mat,
                double delta,
                double eff_radius,
                double mass1,
                double mass2,
                ChContactable* objA,
                ChContactable* objB
            ) const override;

            void SetContactBehaviorMap(ContactBehaviorMap* contact_behavior) {
                assert(contact_behavior);
                contact_behavior_ = contact_behavior;
            }

            // /**
            //  * Method to allow serialization of transient data to archives.
            //  * @param archive_out ChArchiveOut
            //  */
            // virtual void ArchiveOut(ChArchiveOut& archive_out);
            //
            // /**
            //  * Method to allow serialization of transient data from archives.
            //  * @param archive_in ChArchiveIn
            //  */
            // virtual void ArchiveIn(ChArchiveIn& archive_in);

        protected:
            ChSystemSMC* sys_{nullptr};

            ContactBehaviorMap* contact_behavior_{nullptr};
    };
} // namespace chrono::biomechanics


#endif // EXOSIM_CH_CONTACT_JAW_SMC_H
