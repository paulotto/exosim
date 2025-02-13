/**
 * @file ChVisualShapeBiomechanics.h
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

#ifndef EXOSIM_CH_VISUAL_SHAPE_MUSCLE_H
#define EXOSIM_CH_VISUAL_SHAPE_MUSCLE_H

#define EXOSIM_VIS_SHAPE_MUSCLE_USE_OPENMP false

#include <chrono/core/ChApiCE.h>
#include <chrono/geometry/ChSurface.h>
#include <chrono/geometry/ChSurfaceNurbs.h>
#include <chrono/assets/ChVisualShapeSurface.h>

#include "exosim/biomechanics/utils/utils.h"


namespace chrono::biomechanics {
    class ChMuscle; // Forward declaration

    class ChApi ChVisualShapeSurfaceBiomechanics : public ChVisualShapeSurface {
        public:
            ChVisualShapeSurfaceBiomechanics(int num_ctrl_pts_length,
                                             int num_ctrl_pts_radial)
                : ChVisualShapeSurface(),
                  num_ctrl_pts_length_(num_ctrl_pts_length),
                  num_ctrl_pts_radial_(num_ctrl_pts_radial) {
                utils::throw_invalid_argument(num_ctrl_pts_length <= 0 || num_ctrl_pts_radial <= 0,
                                              "[ERROR] [ChVisualShapeMuscle] ctrl_pts_length and ctrl_pts_radial "
                                              "must be > 0!");
                if (num_ctrl_pts_length < degree_u_ + 1) {
                    fmt::print("[WARNING] [ChVisualShapeMuscle] ctrl_pts_length must be at least degree_u_ + 1! "
                        "Value will be adjusted.\n");
                    num_ctrl_pts_length_ = degree_u_ + 1;
                }
                if (num_ctrl_pts_radial < degree_v_ + 1) {
                    fmt::print("[WARNING] [ChVisualShapeMuscle] ctrl_pts_radial must be at least degree_v_ + 1! "
                        "Value will be adjusted.\n");
                    num_ctrl_pts_radial_ = degree_v_ + 1;
                }

                // Initialize a matrix for the control points for the NURBS surface
                control_points_ = ChMatrixDynamic<ChVector3d>(num_ctrl_pts_length_, num_ctrl_pts_radial_);
            }

            void SetName(const std::string& name) { name_ = name; }
            void SetDegreeU(int degree) { degree_u_ = degree; }
            void SetDegreeV(int degree) { degree_v_ = degree; }
            void SetNumCtrlPtsLength(int num_ctrl_pts) { num_ctrl_pts_length_ = num_ctrl_pts; }
            void SetNumCtrlPtsRadial(int num_ctrl_pts) { num_ctrl_pts_radial_ = num_ctrl_pts; }

            std::string GetName() const { return name_; }
            const ChFrame<>& GetShapeFrame() const { return shape_frame_; }
            const ChVector3d& GetPoint1Abs() const { return point_1_; }
            const ChVector3d& GetPoint2Abs() const { return point_2_; }
            const ChVector3d& GetPoint1Local() const { return point_local_1_; }
            const ChVector3d& GetPoint2Local() const { return point_local_2_; }
            const int& GetDegreeU() const { return degree_u_; }
            const int& GetDegreeV() const { return degree_v_; }
            const int& GetNumCtrlPtsLength() const { return num_ctrl_pts_length_; }
            const int& GetNumCtrlPtsRadial() const { return num_ctrl_pts_radial_; }

        protected:
            void Update(ChPhysicsItem* updater, const ChFrame<>& frame) override;

            virtual void UpdateSurface(const ChVector3d& point_1, const ChVector3d& point_2) = 0;

            ChFrame<> shape_frame_; ///< Frame of the shape (in global frame)

            ChVector3d point_1_; ///< Location of 1st end point (in global frame)
            ChVector3d point_2_; ///< Location of 2nd end point (in global frame)

            ChVector3d point_local_1_; ///< Location of 1st end point (in local frame)
            ChVector3d point_local_2_; ///< Location of 2nd end point (in local frame)

            ChMatrixDynamic<ChVector3d> control_points_;

        private:
            std::string name_;

            int degree_u_{2}; ///< Degree of the surface representation in length direction
            int degree_v_{2}; ///< Degree of the surface representation in radial direction

            int num_ctrl_pts_length_{3}; ///< Number of control points in length direction
            int num_ctrl_pts_radial_{8}; ///< Number of control points in radial direction
    };

    class ChApi ChVisualShapeMuscle : public ChVisualShapeSurfaceBiomechanics {
        public:
            ChVisualShapeMuscle(int ctrl_pts_length,
                                int ctrl_pts_radial,
                                std::shared_ptr<ChMuscle> muscle);

            void SetReferenceRadius(double radius) { ref_radius_ = radius; }
            void SetTaperingFactor(double factor) { tapering_factor_ = factor; }
            void SetContractionCoefficient(double coefficient) { contraction_coefficient_ = coefficient; }

            const std::shared_ptr<ChMuscle>& GetMuscle() const { return muscle_; }
            const double& GetReferenceRadius() const { return ref_radius_; }
            const double& GetTaperingFactor() const { return tapering_factor_; }
            const double& GetContractionCoefficient() const { return contraction_coefficient_; }

        protected:
            void Update(ChPhysicsItem* updater, const ChFrame<>& frame) override;

            void UpdateSurface(const ChVector3d& point_1, const ChVector3d& point_2) override;

            std::shared_ptr<ChMuscle> muscle_;

        private:
            /// Minimum radius
            double min_radius_{1.0e-3};
            /// Reference radius (at reference muscle length l_0)
            double ref_radius_{3.0e-3};
            /// Reference muscle length
            double l_0_{1.0};
            /// Tapering factor for narrowing at the ends
            double tapering_factor_{0.6};
            /// Scaling factor for radius shrinkage/expansion as the muscle stretches or contracts
            double contraction_coefficient_{1.2};
    };
} // namespace chrono::biomechanics

namespace chrono {
    CH_CLASS_VERSION(biomechanics::ChVisualShapeSurfaceBiomechanics, 0)

    CH_CLASS_VERSION(biomechanics::ChVisualShapeMuscle, 0)
}


#endif // EXOSIM_CH_VISUAL_SHAPE_MUSCLE_H
