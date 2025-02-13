/**
 * @file ChContactForceTorqueJawSMC.cpp
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

// MODIFIED FROM PROJECT CHRONO: ChContactSMC.cpp
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

#include "exosim/biomechanics/physics/ChContactForceTorqueJawSMC.h"


namespace chrono::biomechanics {
    // Register into the object factory, to enable run-time dynamic creation and persistence
    CH_FACTORY_REGISTER(ChContactForceTorqueJawSMC)

    ChWrenchd
    ChContactForceTorqueJawSMC::CalculateForceTorque(const ChSystemSMC& sys,
                                                     const ChVector3d& normal_dir,
                                                     const ChVector3d& p1,
                                                     const ChVector3d& p2,
                                                     const ChVector3d& vel1,
                                                     const ChVector3d& vel2,
                                                     const ChContactMaterialCompositeSMC& mat,
                                                     double delta,
                                                     double eff_radius,
                                                     double mass1, double mass2,
                                                     ChContactable* objA, ChContactable* objB) const {
        ContactModelsSpecifications contact_model_specs;

        /// Reference: ChDefaultContactForceSMC::CalculateForce in ChContactSMC.h
        // Set contact force to zero if no penetration.
        if (delta <= 0.0) {
            return {VNULL, VNULL};
        }

        try {
            contact_model_specs = contact_behavior_->at({objA, objB});
        } catch (const std::out_of_range& e) {
            std::cout << fmt::format(
                utils::WARNING_MSG,
                "[WARNING] [ChContactForceTorqueJawSMC] Contact behavior not found! Proceeding with standard behavior...\n");
            contact_model_specs = ContactModelsSpecifications();
        }

        // Extract parameters from containing system
        double dT = sys.GetStep();
        bool use_mat_props = sys.UsingMaterialProperties();
        ContactForceTorqueModel contact_model = contact_model_specs.contact_force_model;
        AdhesionForceTorqueModel adhesion_model = contact_model_specs.adhesion_force_model;
        TangentialDisplacementModel tangential_model = contact_model_specs.tangential_displacement_model;

        // Relative velocity at contact
        ChVector3d relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector3d relvel_n = relvel_n_mag * normal_dir;
        ChVector3d relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();

        //TODO: limit vel & delta to avoid instability
        // double vel_limit{0.05};
        // double delta_limit{5.0e-3};
        // relvel_n_mag = std::max(std::min(relvel_n_mag, vel_limit), -vel_limit);
        // relvel_t_mag = std::max(std::min(relvel_t_mag, vel_limit), -vel_limit);
        // delta = std::min(delta, delta_limit);

        // Calculate effective mass
        double eff_mass = mass1 * mass2 / (mass1 + mass2);

        // Calculate stiffness and viscous damping coefficients.
        // All models use the following formulas for normal and tangential forces:
        //     Fn = kn * delta_n - gn * v_n
        //     Ft = kt * delta_t - gt * v_t
        double kn = 0;
        double kt = 0;
        double gn = 0;
        double gt = 0;

        constexpr double eps = std::numeric_limits<double>::epsilon();

        switch (contact_model) {
            case ChContactForceTorqueJawSMC::ContactForceTorqueModel::Flores:
            // Currently not implemented. Fall through to Hooke.
            case ChContactForceTorqueJawSMC::ContactForceTorqueModel::Hooke:
                if (use_mat_props) {
                    double tmp_k = (16.0 / 15) * std::sqrt(eff_radius) * mat.E_eff;
                    double v2 = sys.GetCharacteristicImpactVelocity() * sys.GetCharacteristicImpactVelocity();
                    double loge = (mat.cr_eff < eps) ? std::log(eps) : std::log(mat.cr_eff);
                    loge = (mat.cr_eff > 1 - eps) ? std::log(1 - eps) : loge;
                    double tmp_g = 1 + std::pow(CH_PI / loge, 2);
                    kn = tmp_k * std::pow(eff_mass * v2 / tmp_k, 1.0 / 5);
                    kt = kn;
                    gn = std::sqrt(4 * eff_mass * kn / tmp_g);
                    gt = gn;
                } else {
                    kn = mat.kn;
                    kt = mat.kt;
                    gn = eff_mass * mat.gn;
                    gt = eff_mass * mat.gt;
                }

                break;

            case ChContactForceTorqueJawSMC::ContactForceTorqueModel::Hertz:
                if (use_mat_props) {
                    double sqrt_Rd = std::sqrt(eff_radius * delta);
                    double Sn = 2 * mat.E_eff * sqrt_Rd;
                    double St = 8 * mat.G_eff * sqrt_Rd;
                    double loge = (mat.cr_eff < eps) ? std::log(eps) : std::log(mat.cr_eff);
                    double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
                    kn = (2.0 / 3) * Sn;
                    kt = St;
                    gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);
                    gt = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(St * eff_mass);
                } else {
                    double tmp = eff_radius * std::sqrt(delta);
                    kn = tmp * mat.kn;
                    kt = tmp * mat.kt;
                    gn = tmp * eff_mass * mat.gn;
                    gt = tmp * eff_mass * mat.gt;
                }

                break;

            case ChContactForceTorqueJawSMC::ContactForceTorqueModel::PlainCoulomb:
                if (use_mat_props) {
                    double sqrt_Rd = std::sqrt(delta);
                    double Sn = 2 * mat.E_eff * sqrt_Rd;
                    double loge = (mat.cr_eff < eps) ? std::log(eps) : std::log(mat.cr_eff);
                    double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
                    kn = (2.0 / 3) * Sn;
                    gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);
                } else {
                    double tmp = std::sqrt(delta);
                    kn = tmp * mat.kn;
                    gn = tmp * mat.gn;
                }

                kt = 0;
                gt = 0; {
                    double forceN = kn * delta - gn * relvel_n_mag;
                    if (forceN < 0)
                        forceN = 0;
                    double forceT = mat.mu_eff * std::tanh(5.0 * relvel_t_mag) * forceN;
                    switch (adhesion_model) {
                        case AdhesionForceTorqueModel::NoneAdhesion:
                            break;
                        case AdhesionForceTorqueModel::Perko:
                            std::cout << fmt::format(
                                utils::WARNING_MSG,
                                "[WARNING] [ChContactForceTorqueJawSMC] Perko adhesion model not implemented yet! "
                                "Falling back to constant adhesion model...");
                        case AdhesionForceTorqueModel::Constant:
                            forceN -= mat.adhesion_eff;
                            break;
                        case AdhesionForceTorqueModel::DMT:
                            forceN -= mat.adhesionMultDMT_eff * sqrt(eff_radius);
                            break;
                    }
                    ChVector3d force = forceN * normal_dir;
                    if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
                        force -= (forceT / relvel_t_mag) * relvel_t;

                    return {force, VNULL}; // Zero torque anyway
                }

            case ChContactForceTorqueJawSMC::ContactForceTorqueModel::EFC:
                // TODO: check which object has mesh
                // TODO: compute contribution of both meshes if applicable and half the force of each
                // TODO: collision shapes seem not to be defined with respect to REF but COG?! -> check this (visualize)
                auto* body_a = dynamic_cast<ChBody*>(objA);
                auto* body_b = dynamic_cast<ChBody*>(objB);

            // auto p1_ref = p1 >> body_a->GetFrame_REF_to_abs().GetInverse();
                auto p1_ref_norm = body_a->GetFrameRefToAbs().TransformPointParentToLocal(p1).GetNormalized();
                auto p2_ref_norm = body_a->GetFrameRefToAbs().TransformPointParentToLocal(p2).GetNormalized();

                int idx_closest{0};
                double cos_angle{0.0}, cos_angle_best{-2.0};
                ChVector3d tri_shape_pos_ref_norm;

            // Search for closest shape to contact point (TODO: works only if only one shape defined)
                const auto& [shape, frame] = body_a->GetCollisionModel()->GetShapeInstance(0);
                const auto& shape_tri_mesh = std::static_pointer_cast<ChCollisionShapeTriangleMesh>(
                    shape);

            // TODO: better metric
                for (int i = 0; i < body_a->GetCollisionModel()->GetNumShapes(); i++) {
                    // tri_shape_pos_ref_norm = body_a->GetCollisionModel()->GetShapePos(i).pos.GetNormalized();
                    // cos_angle = p1_ref_norm.Dot(tri_shape_pos_ref_norm);

                    const auto triangle = shape_tri_mesh->GetMesh()->GetTriangle(i);
                    ChVector3d barycenter;
                    barycenter = (triangle.p1 + triangle.p2 + triangle.p3) / 3.0;

                    cos_angle = p1_ref_norm.Dot(barycenter.GetNormalized());

                    if ((cos_angle - 1.0) > cos_angle_best) {
                        cos_angle_best = cos_angle;
                        // std::cout << fmt::format(WARNING_MSG, "[WARNING] [ChContactJawSMC] {}:{}", i, cos_angle);
                        idx_closest = i;
                    }
                }

                auto shape_tri_mesh_closest = std::static_pointer_cast<ChCollisionShapeTriangleMesh>(
                    body_a->GetCollisionModel()->GetShapeInstance(0).first);
                auto tri_closest = shape_tri_mesh_closest->GetMesh()->GetTriangle(idx_closest);
                auto edge_1 = (tri_closest.p2 - tri_closest.p1).Length();
                auto edge_2 = (tri_closest.p3 - tri_closest.p1).Length();
                auto edge_3 = (tri_closest.p3 - tri_closest.p2).Length();
                auto s = 0.5 * (edge_1 + edge_2 + edge_3);
                auto area = std::sqrt(s * (s - edge_1) * (s - edge_2) * (s - edge_3));

            // TODO: implement new material (ChMaterialSurface) type for EFC model
                double E_efc{2.7e6}, p_efc{0.49}, h{4.0e-4}, c{0.2}, u_s{0.0}, u_d{0.0}, u_v{0.3}, vel_t{0.01};
                auto vel_rel = vel2 - vel1;
                auto vel_norm = vel_rel.Dot(normal_dir);
                auto vel_tang = vel_rel - vel_norm * normal_dir;

                ChVector3d force;

            // Spring stiffness
                auto k = (1.0 - p_efc) * E_efc / ((1 + p_efc) * (1 - 2 * p_efc) * h);
            // Normal force
                auto f_n = k * area * delta * (1 + c * vel_norm);

            // Include adhesion force
                switch (adhesion_model) {
                    case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::NoneAdhesion:
                        break;
                    case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::Perko:
                        std::cout << fmt::format(
                            utils::WARNING_MSG,
                            "[WARNING] [ChContactForceTorqueJawSMC] Perko adhesion model not implemented yet! "
                            "Falling back to constant adhesion model...");
                    case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::Constant:
                        f_n -= mat.adhesion_eff;
                        break;
                    case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::DMT:
                        f_n -= mat.adhesionMultDMT_eff * sqrt(eff_radius);
                        break;
                }

                force = (f_n > 0.0) ? f_n * normal_dir : ChVector3d(0.0, 0.0, 0.0);

            // Friction force
                if (auto vel_slip = vel_tang.Length(); f_n > 0.0 && vel_slip != 0.0) {
                    auto vel_ratio = vel_slip / vel_t;
                    auto f_f = f_n * (std::min(vel_ratio, 1.0) * (u_d + 2 * (u_s - u_d) / (1 + vel_ratio * vel_ratio))
                                      + u_v * vel_slip);
                    force -= f_f * vel_tang / vel_slip;
                }

            // std::cout << force << '\n';
                return {force, VNULL};
        }

        // Tangential displacement (magnitude)
        double delta_t = 0;
        switch (tangential_model) {
            case ChContactForceTorqueJawSMC::TangentialDisplacementModel::NoneTangential:
                break;
            case ChContactForceTorqueJawSMC::TangentialDisplacementModel::MultiStep:
                std::cout << fmt::format(
                    utils::WARNING_MSG,
                    "[WARNING] [ChContactForceTorqueJawSMC] MultiStep tangential displacement model not "
                    "implemented yet! Falling back to OneStep tangential displacement model...");
            case ChContactForceTorqueJawSMC::TangentialDisplacementModel::OneStep:
                delta_t = relvel_t_mag * dT;
                break;
            default:
                break;
        }

        // Calculate the magnitudes of the normal and tangential contact forces
        double forceN = kn * delta - gn * relvel_n_mag;
        double forceT = kt * delta_t + gt * relvel_t_mag;

        // If the resulting normal contact force is negative, the two shapes are moving
        // away from each other so fast that no contact force is generated.
        if (forceN < 0) {
            forceN = 0;
            forceT = 0;
        }

        // Include adhesion force
        switch (adhesion_model) {
            case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::NoneAdhesion:
                break;
            case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::Perko:
                std::cout << fmt::format(
                    utils::WARNING_MSG,
                    "[WARNING] [ChContactForceTorqueJawSMC] Perko adhesion model not implemented yet! "
                    "Falling back to constant adhesion model...");
            case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::Constant:
                forceN -= mat.adhesion_eff;
                break;
            case ChContactForceTorqueJawSMC::AdhesionForceTorqueModel::DMT:
                forceN -= mat.adhesionMultDMT_eff * sqrt(eff_radius);
                break;
        }

        // Coulomb law
        forceT = std::min<double>(forceT, mat.mu_eff * std::abs(forceN));

        // Accumulate normal and tangential forces
        ChVector3d force = forceN * normal_dir;
        if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
            force -= (forceT / relvel_t_mag) * relvel_t;

        // std::cout << "Contact force: " << force << '\n';
        // std::cout << "Contact kn: " << kn << '\n';
        // std::cout << "Contact gn: " << gn << '\n';
        // std::cout << "Contact delta: " << delta << '\n';
        // std::cout << "Contact relvel_n_mag: " << relvel_n_mag << '\n';
        // std::cout << "Contact relvel: " << relvel << '\n';
        // std::cout << "Contact mat.E_eff: " << mat.E_eff << '\n';
        // std::cout << "Contact eff_radius: " << eff_radius << '\n';
        // std::cout << "Collision default margin: " << ChCollisionModel::GetDefaultSuggestedMargin() << '\n';
        // auto* body_a = dynamic_cast<ChBody*>(objA);
        // std::cout << "Collision margin: " << body_a->GetCollisionModel()->GetSafeMargin() << '\n';

        return {force, VNULL}; // Zero torque anyway
    }
} // namespace chrono::biomechanics
