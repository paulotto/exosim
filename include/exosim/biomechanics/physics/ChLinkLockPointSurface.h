/**
 * @file ChLinkLockPointSurface.h
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 17.11.2023
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

#ifndef EXOSIM_CH_LINKPOINTSURFACE_H
#define EXOSIM_CH_LINKPOINTSURFACE_H

#include "exosim/biomechanics/utils/utils.h"
//
#include <chrono/geometry/ChGeometry.h>
#include <chrono/geometry/ChLine.h>
#include <chrono/geometry/ChLinePoly.h>
#include <chrono/geometry/ChLineNurbs.h>
#include <chrono/geometry/ChLineBSpline.h>
#include <chrono/geometry/ChSurface.h>
#include <chrono/geometry/ChSurfaceNurbs.h>
#include <chrono/geometry/ChBasisToolsNurbs.h>
#include "chrono/geometry/ChBasisToolsBSpline.h"
//
#include <chrono/physics/ChLinkLock.h>
//
#include <chrono/serialization/ChArchive.h>


/**
 * TODO:
 * [x] Create constraint from ChSurfaceNurbs
 * [] Create constraint from ChLinePoly/ChLineNurbs/ChLineBspline (extrude to 2D)
 * [] Support SMC contact
 */


namespace chrono::biomechanics {
    /**
     * @class ChLinkLockPointSurface
     * @inherit ChLinkLock
     * @brief TODO
     */
    class ChApi ChLinkLockPointSurface : public ChLinkLock {
        public:
            /**
             * @enum ChSurfaceType
             * @brief Supported surface types. 1D line types are extended linearly to 2D.
             */
            enum ChSurfaceType {
                SURFACE, ///< ChSurface (2D)
                LINE ///< ChLine (extended to 2D)
            };

            /**
             * @class ChLinkLockPointSurfaceEnumMapper
             * @brief Can be used to convert between enums and strings.
             */
            class ChLinkLockPointSurfaceEnumMapper final {
                public:
                    CH_ENUM_MAPPER_BEGIN(ChSurfaceType)
                                CH_ENUM_VAL(SURFACE)
                                CH_ENUM_VAL(LINE)
                    CH_ENUM_MAPPER_END(ChSurfaceType)
            };

            ChLinkLockPointSurface()
                : ChLinkLockPointSurface(chrono_types::make_shared<ChSurfaceNurbs>()) {
            }

            explicit ChLinkLockPointSurface(const std::shared_ptr<ChGeometry>& surface,
                                            bool verbose = false);

            ~ChLinkLockPointSurface() override = default;

            /**
             * "Virtual" copy constructor (covariant return type)
             * @return ChLinkLockPointSurface*
             */
            ChLinkLockPointSurface* Clone() const override { return new ChLinkLockPointSurface(*this); }

            /**
             * Gets the number of test points used in the 1st approximation of the nearest point search.
             * @return Number of samples
             */
            int GetSamples() const { return samples_; }

            /**
             * Gets the maximum number of iterations for the refinement step of the nearest point search.
             * @return Max. iterations
             */
            int GetMaxIterations() const { return max_iters_; }

            /**
             * Gets the tolerance controlling the accuracy of the nearest point search.
             * @return Tolerance
             */
            double GetTolerance() const { return tolerance_; }

            /**
             * Gets the perturbation used for calculating the tangent space with finite differences.
             * @return Perturbation
             */
            double GetBDF() const { return bdf_; }

            /**
             * Gets the surface geometry object.
             * @return A ChGeometry reference
             */
            const ChGeometry& GetSurfaceGeometry() const { return *surface_geometry_; }

            /**
             * Get the number of unilateral constraints for this link.
             * @return Number of unilateral constraints
             */
            unsigned int GetNumConstraintsUnilateral() override;

            /**
             * Sets the number of test points used in the 1st approximation of the nearest point search.
             * @param samples Number of samples
             */
            void SetSamples(int samples) { samples_ = samples; }

            /**
             * Sets the maximum number of iterations for the refinement step of the nearest point search.
             * @param max_iters Max. iterations
             */
            void SetMaxIterations(int max_iters) { max_iters_ = max_iters; }

            /**
             * Sets the tolerance controlling the accuracy of the nearest point search.
             * @param tol Tolerance (default: 1e-6)
             */
            void SetTolerance(double tol = 1e-6) { tolerance_ = tol; }

            /**
             * Sets the perturbation used for calculating the tangent space with finite differences.
             * @param bdf Perturbation (default: 10e-9)
             */
            void SetBDF(double bdf = 10e-9) { bdf_ = bdf; }

            /**
             * Sets the geometry object used for the surface constraint.
             * Only the following geometries are supported: \n
             *  - ChLine (and derived classes) \n
             *  - ChSurface (so far: ChSurfaceNurbs) \n
             * 1D line types are extended linearly to 2D.
             * @param surface_geometry Surface geometry
             */
            void SetSurfaceGeometry(const std::shared_ptr<ChGeometry>& surface_geometry);

            /**
             * Activates/Deactivates limits in u and v direction. For now, limits are defined as the contour of
             * the surface object. With other words, if u or v is 0 or 1, respectively.
             * @param u True if limits in u-direction is to be activated
             * @param v True if limits in v-direction is to be activated
             */
            void SetLimitActive(bool u, bool v) {
                limit_u_active_ = u;
                limit_v_active_ = v;
            }

            //        void SetLimitMaxX(bool active, double limit_x_max = 10.0) {
            //            limit_x_init_.second.x() = limit_x_max;
            //            this->GetLimit_X().SetMax(limit_x_max);
            //            this->GetLimit_X().constr_upper.SetActive(active);
            //        }
            //
            //        void SetLimitMaxY(bool active, double limit_y_max = 10.0) {
            //            limit_y_init_.second.y() = limit_y_max;
            //            this->GetLimit_Y().SetMax(limit_y_max);
            //            this->GetLimit_Y().constr_upper.SetActive(active);
            //        }
            //
            //        void SetLimitMinX(bool active, double limit_x_min = -10.0) {
            //            limit_x_init_.first.x() = limit_x_min;
            //            this->GetLimit_X().SetMin(limit_x_min);
            //            this->GetLimit_X().constr_lower.SetActive(active);
            //        }
            //
            //        void SetLimitMinY(bool active, double limit_y_min = -10.0) {
            //            limit_y_init_.first.y() = limit_y_min;
            //            this->GetLimit_Y().SetMin(limit_y_min);
            //            this->GetLimit_Y().constr_lower.SetActive(active);
            //        }

            /**
             * Use this function after link creation, to initialize the link from two markers to join.
             * Each marker must belong to a rigid body, and both rigid bodies must belong to the same ChSystem.
             * The position of mark2 is used as link's position and main reference.
             * @param mark_1 First  marker to join
             * @param mark_2 Second marker to join (master)
             */
            void Initialize(std::shared_ptr<ChMarker> mark_1, std::shared_ptr<ChMarker> mark_2) override;

            /**
             * Use this function after link creation, to initialize the link from two joined rigid bodies.
             * Both rigid bodies must belong to the same ChSystem.
             * Two markers will be created and added to the rigid bodies (later, you can use GetMarker1()
             * and GetMarker2() to access them. To specify the (absolute) position of link and markers, use 'pos'.
             * @param body_1 First  body to join
             * @param body_2 Second body to join
             * @param pos The current absolute pos. & alignment
             */
            void Initialize(std::shared_ptr<ChBody> body_1,
                            std::shared_ptr<ChBody> body_2,
                            const ChFrame<>& pos) override;

            /**
             * Use this function after link creation, to initialize the link from two joined rigid bodies.
             * Both rigid bodies must belong to the same ChSystem.
             * Two markers will be created and added to the rigid bodies (later, you can use GetMarker1()
             * and GetMarker2() to access them. To specify the (absolute) position of link and markers, use 'pos'.
             * @param body_1 First  body to join
             * @param body_2 Second body to join
             * @param pos_are_relative If = true, following two positions are relative to bodies. If false, are absolute
             * @param pos_1 The position & alignment of 1st marker (relative to body1 cords, or absolute)
             * @param pos_2 The position & alignment of 2nd marker (relative to body2 cords, or absolute)
             */
            void Initialize(std::shared_ptr<ChBody> body_1,
                            std::shared_ptr<ChBody> body_2,
                            bool pos_are_relative,
                            const ChFrame<>& pos_1,
                            const ChFrame<>& pos_2) override;

            /**
             * Given new time, current body state, update time-dependent quantities in
             * link state, for example motion laws, moving markers, etc. Default: do nothing except setting new time.
             * @param time Time
             */
            void UpdateTime(double time) override;

            /**
             * Finds the surface parameters U and V of the closest point of a given point in 3D space.
             * Reference: ChLine::FindNearestLinePoint
             * @param surface Needs to be defined with respect to absolute coordinates
             * @param point
             * @param U_res
             * @param V_res
             * @param tol
             * @param samples
             * @param max_iters
             * @param verbose
             * @param name Name of the ChLinkLockPointSurface object. Used for debugging.
             * @return True if search "converged" with respect to the tolerance, false otherwise
             */
            static bool FindNearestSurfacePoint(const std::shared_ptr<ChSurface>& surface,
                                                const ChVector3d& point,
                                                double& U_res,
                                                double& V_res,
                                                double tol,
                                                double max_iters = 11,
                                                int samples = 10,
                                                bool verbose = false,
                                                std::string name = "");

            /**
             * TODO
             * @param dir_u
             * @param dir_v
             * @param surface
             * @param par_U
             * @param par_V
             */
            static void DeriveSurfaceNurbs(ChVector3d& dir_u, ChVector3d& dir_v,
                                           const std::shared_ptr<ChSurfaceNurbs>& surface,
                                           double par_U, double par_V);

            /**
             * TODO
             * @param dir_u
             * @param dir_v
             * @param surface
             * @param U
             * @param V
             * @param bdf
             */
            static void DerivativeSurface(ChVector3d& dir_u, ChVector3d& dir_v,
                                          const std::shared_ptr<ChSurface>& surface,
                                          double U,
                                          double V,
                                          double bdf = 10e-9);

            /**
             * TODO
             * @param J
             * @param surface
             * @param U
             * @param V
             * @param bdf
             */
            static void DerivativeSurface(ChMatrixNM<double, 3, 2>& J,
                                          const std::shared_ptr<ChSurface>& surface,
                                          double U,
                                          double V,
                                          double bdf = 10e-9);

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
            /**
             * Checks if limits are activated and triggered, and controls the limits if necessary.
             * For now, limits are defined as the contour of the surface object.
             * TODO: needs testing when tangent vectors may change direction!
             * @param U Parameter U of curve/surface
             * @param V Parameter V of surface
             */
            virtual void CheckLimits_(double U, double V);

            bool verbose_{false};
            bool limit_u_active_{false};
            bool limit_v_active_{false};

            double current_U_{0.0};
            double current_V_{0.0};

            int samples_{50}; ///< Number of test points used in the 1st approx. of the nearest point search
            int max_iters_{11}; ///< Max. iterations for refinement step of the nearest point search
            double tolerance_{1e-4}; ///< Tolerance for nearest point search
            double bdf_{10e-9}; ///< Perturbation used for calculating the tangent space with finite differences

            std::pair<ChVector3d, ChVector3d> limit_x_init_{{-10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
            std::pair<ChVector3d, ChVector3d> limit_y_init_{{0.0, -10.0, 0.0}, {0.0, 10.0, 0.0}};

            ChSurfaceType surface_type_{SURFACE};

            ChCoordsys<> m_1_init_pose_;
            ChCoordsys<> m_2_init_pose_;

            std::shared_ptr<ChGeometry> surface_geometry_{nullptr};
    };
} // namespace chrono::biomechanics

namespace chrono {
    CH_CLASS_VERSION(biomechanics::ChLinkLockPointSurface, 0)
}


#endif // EXOSIM_CH_LINKPOINTSURFACE_H
