/**
 * @file ChVisualShapeBiomechanics.cpp
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 07.01.2025
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

#include <omp.h>
#include <Eigen/Core>

#include <chrono/physics/ChLinkMarkers.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChLinkDistance.h>
#include <chrono/physics/ChLinkRevoluteSpherical.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChLinkRSDA.h>
#include <chrono/physics/ChHydraulicActuator.h>

#include "exosim/biomechanics/ChMuscle.h"
#include "exosim/biomechanics/vis/ChVisualShapeBiomechanics.h"


namespace chrono::biomechanics {
    // Register into the object factory, to enable run-time dynamic creation and persistence
    CH_FACTORY_REGISTER(ChVisualShapeSurfaceBiomechanics)

    CH_FACTORY_REGISTER(ChVisualShapeMuscle)

    void ChVisualShapeSurfaceBiomechanics::Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        // ChVisualShapeSurface::Update(updater, frame);
        // auto muscle = static_cast<ChMuscle*>(updater);

        if (const auto link_markers = dynamic_cast<ChLinkMarkers*>(updater)) {
            point_1_ = link_markers->GetMarker1()->GetAbsCoordsys().pos;
            point_2_ = link_markers->GetMarker2()->GetAbsCoordsys().pos;
        } else if (const auto link_dist = dynamic_cast<ChLinkDistance*>(updater)) {
            point_1_ = link_dist->GetEndPoint1Abs();
            point_2_ = link_dist->GetEndPoint2Abs();
        } else if (const auto link_rs = dynamic_cast<ChLinkRevoluteSpherical*>(updater)) {
            point_1_ = link_rs->GetPoint1Abs();
            point_2_ = link_rs->GetPoint2Abs();
        } else if (const auto link_tsda = dynamic_cast<ChLinkTSDA*>(updater)) {
            point_1_ = link_tsda->GetPoint1Abs();
            point_2_ = link_tsda->GetPoint2Abs();
        } else if (const auto link_mate = dynamic_cast<ChLinkMateGeneric*>(updater)) {
            point_1_ = link_mate->GetBody1()->TransformPointLocalToParent(link_mate->GetFrame1Rel().GetPos());
            point_2_ = link_mate->GetBody2()->TransformPointLocalToParent(link_mate->GetFrame2Rel().GetPos());
        } else if (const auto link = dynamic_cast<ChLink*>(updater)) {
            point_1_ = link->GetBody1()->GetPos();
            point_2_ = link->GetBody2()->GetPos();
        } else if (const auto actuator = dynamic_cast<ChHydraulicActuatorBase*>(updater)) {
            point_1_ = actuator->GetPoint1Abs();
            point_2_ = actuator->GetPoint2Abs();
        } else {
            return;
        }

        shape_frame_ = frame;

        point_local_1_ = frame.TransformPointParentToLocal(point_1_);
        point_local_2_ = frame.TransformPointParentToLocal(point_2_);

        UpdateSurface(point_local_1_, point_local_2_);
    }

    ChVisualShapeMuscle::ChVisualShapeMuscle(int ctrl_pts_length,
                                             int ctrl_pts_radial,
                                             std::shared_ptr<ChMuscle> muscle)
        : ChVisualShapeSurfaceBiomechanics(ctrl_pts_length, ctrl_pts_radial),
          muscle_(std::move(muscle)) {
        this->SetMutable(true);

        l_0_ = muscle_->GetMaxContractileForceLength();
        utils::throw_runtime_error(l_0_ <= 0.0,
                                   "[ERROR] [ChVisualShapeMuscle] Optimal muscle length must be positive.");

        this->SetSurfaceGeometry(chrono_types::make_shared<ChSurfaceNurbs>());
    }

    void ChVisualShapeMuscle::Update(ChPhysicsItem* updater, const ChFrame<>& frame) {
        // ChVisualShapeBiomechanics::Update(updater, frame);

        // if (const auto link_tsda = dynamic_cast<ChLinkTSDA*>(updater)) {
        //     point_1 = link_tsda->GetPoint1Abs();
        //     point_2 = link_tsda->GetPoint2Abs();
        // }

        shape_frame_ = frame;

        point_1_ = muscle_->GetPoint1Abs();
        point_2_ = muscle_->GetPoint2Abs();

        point_local_1_ = frame.TransformPointParentToLocal(point_1_);
        point_local_2_ = frame.TransformPointParentToLocal(point_2_);

        UpdateSurface(point_local_1_, point_local_2_);
    }

    void ChVisualShapeMuscle::UpdateSurface(const ChVector3d& point_1, const ChVector3d& point_2) {
        const double current_muscle_length = muscle_->GetLength();
        const auto& num_ctrl_pts_length = this->GetNumCtrlPtsLength();
        const auto& num_ctrl_pts_radial = this->GetNumCtrlPtsRadial();

        // Compute the direction vector and length
        const ChVector3d direction = point_2 - point_1;
        // double length = direction.Length();
        const ChVector3d z_axis = direction.GetNormalized();

        // Rotation matrix
        ChMatrix33<> rotation;
        rotation.SetFromAxisZ(z_axis, VECT_X);

        ChCoordsys<> m_pos(point_1, rotation.GetQuaternion());

        // Create control points for the NURBS surface
#if EXOSIM_VIS_SHAPE_MUSCLE_USE_OPENMP
#pragma omp parallel for collapse(2) // Collapse outer and inner loops for parallel execution
        for (int i = 0; i < num_ctrl_pts_length; ++i) {
            for (int j = 0; j < num_ctrl_pts_radial; ++j) {
                const double u = static_cast<double>(i) / (num_ctrl_pts_length - 1); // Normalize length
                const double z = current_muscle_length * u; // Position along the length of the muscle

                // Compute the radius for this length position
                double local_radius =
                        ref_radius_ * (1 - contraction_coefficient_ * (current_muscle_length / l_0_ - 1.0)) * (
                            1 - tapering_factor_ * std::abs(2 * u - 1));
                local_radius = std::max(local_radius, min_radius_);

                const double v = 2 * CH_PI * j / (num_ctrl_pts_radial - 1); // Angle around the circumference
                const double x = local_radius * std::cos(v); // X-coordinate
                const double y = local_radius * std::sin(v); // Y-coordinate

                // Local control point
                ChVector3d local_point(x, y, z);

                // Transform to global coordinates
                control_points_(i, j) = m_pos.TransformPointLocalToParent(local_point);
            }
        }
#else
        for (int i = 0; i < num_ctrl_pts_length; ++i) {
            const double u = static_cast<double>(i) / (num_ctrl_pts_length - 1); // Normalize length
            const double z = current_muscle_length * u; // Position along the length of the muscle

            // Compute the radius for this length position
            double local_radius =
                    ref_radius_ * (1 - contraction_coefficient_ * (current_muscle_length / l_0_ - 1.0)) * (
                        1 - tapering_factor_ * std::abs(2 * u - 1));
            local_radius = std::max(local_radius, min_radius_);

            for (int j = 0; j < num_ctrl_pts_radial; ++j) {
                const double v = 2 * CH_PI * j / (num_ctrl_pts_radial - 1); // Angle around the circumference
                const double x = local_radius * std::cos(v); // X-coordinate
                const double y = local_radius * std::sin(v); // Y-coordinate

                // Local control point
                ChVector3d local_point(x, y, z);

                // Transform to global coordinates
                control_points_(i, j) = m_pos.TransformPointLocalToParent(local_point);
            }
        }
#endif

        // Knot vectors for the NURBS surface (uniform in both directions)
        // ChVectorDynamic<double> knots_u(nr_ctrl_pts_length + degree_u_ + 1);
        // ChVectorDynamic<double> knots_v(nr_ctrl_pts_radial + degree_v_ + 1);
        // for (size_t i = 0; i < knots_u.size(); ++i) knots_u[i] = static_cast<double>(i) / (knots_u.size() - 1);
        // for (size_t i = 0; i < knots_v.size(); ++i) knots_v[i] = static_cast<double>(i) / (knots_v.size() - 1);

        // Create the NURBS surface
        // const auto nurbs_surface = chrono_types::make_shared<ChSurfaceNurbs>();
        const auto nurbs_surface = std::static_pointer_cast<ChSurfaceNurbs>(this->GetSurfaceGeometry());
        // nurbs_surface->Setup(degree_u_, degree_v_, control_points, &knots_u, &knots_v);
        nurbs_surface->Setup(this->GetDegreeU(), this->GetDegreeV(), control_points_);

        // this->SetSurfaceGeometry(nurbs_surface);
    }
} // namespace chrono::biomechanics
