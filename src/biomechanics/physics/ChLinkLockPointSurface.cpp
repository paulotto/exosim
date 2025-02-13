/**
 * @file ChLinkLockPointSurface.cpp
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

#include <typeinfo>

#include "exosim/biomechanics/physics/ChLinkLockPointSurface.h"


namespace chrono::biomechanics {
    // Register into the object factory, to enable run-time dynamic creation and persistence
    CH_FACTORY_REGISTER(ChLinkLockPointSurface)

    ChLinkLockPointSurface::ChLinkLockPointSurface(const std::shared_ptr<ChGeometry>& surface, bool verbose)
        : verbose_(verbose) {
        SetSurfaceGeometry(surface);

        // Mask: Restrict movement only in normal (z).
        // Translation tangentially (x, y) and rotation is permitted.
        mask.SetLockMask(false, false, true, false, false, false, false);
        BuildLink();
    }

    unsigned int ChLinkLockPointSurface::GetNumConstraintsUnilateral() {
        // CheckLimits_(current_U_, current_U_);
        // return ChLinkLock::GetNumConstraintsUnilateral();
        /* TODO: The issue is that GetNumConstraintsUnilateral() is called before UpdateTime(), which means that the
            number of constraints is not yet updated so that the corresponding constraint matrix in
            ChTimeStepper::Advance has a wrong size. This leads to an Eigen assertion failure. */
        return 4;
    }

    void ChLinkLockPointSurface::SetSurfaceGeometry(const std::shared_ptr<ChGeometry>& surface_geometry) {
        using GT = ChGeometry::Type;

        auto& s = *surface_geometry;
        const GT geometry_type = surface_geometry->GetType();

        // Unfortunately, not all classes implement the GetClassType method so that some have NONE as type
        if (typeid(s) == typeid(ChSurfaceNurbs)) {
            surface_type_ = SURFACE;
        } else if (geometry_type == GT::LINE || geometry_type == GT::LINE_POLY || geometry_type == GT::LINE_ARC
                   || geometry_type == GT::LINE_BEZIER || geometry_type == GT::LINE_CAM ||
                   geometry_type == GT::LINE_SEGMENT || geometry_type == GT::LINE_PATH) {
            surface_type_ = LINE;
        } else {
            utils::throw_invalid_argument(true, "[ERROR] [ChLinkLockPointSurface] No supported geometry type!\n");
        }

        surface_geometry_ = surface_geometry;
    }

    void ChLinkLockPointSurface::Initialize(std::shared_ptr<ChMarker> mark_1, std::shared_ptr<ChMarker> mark_2) {
        ChLinkMarkers::Initialize(mark_1, mark_2);
        m_1_init_pose_ = marker1->GetAbsCoordsys();
        m_2_init_pose_ = marker2->GetAbsCoordsys();
    }

    void ChLinkLockPointSurface::Initialize(std::shared_ptr<ChBody> body_1,
                                            std::shared_ptr<ChBody> body_2,
                                            const ChFrame<>& pos) {
        ChLinkMarkers::Initialize(body_1, body_2, pos);
        m_1_init_pose_ = marker1->GetAbsCoordsys();
        m_2_init_pose_ = marker2->GetAbsCoordsys();
    }

    void ChLinkLockPointSurface::Initialize(std::shared_ptr<ChBody> body_1,
                                            std::shared_ptr<ChBody> body_2,
                                            bool pos_are_relative,
                                            const ChFrame<>& pos_1,
                                            const ChFrame<>& pos_2) {
        ChLinkMarkers::Initialize(body_1, body_2, pos_are_relative, pos_1, pos_2);
        m_1_init_pose_ = marker1->GetAbsCoordsys();
        m_2_init_pose_ = marker2->GetAbsCoordsys();
    }

    void ChLinkLockPointSurface::UpdateTime(double time) {
        ChTime = time;

        if (surface_geometry_) {
            if (surface_type_ == SURFACE) {
                ChVector3d vec_du, vec_dv;

                const auto surface_nurbs_ = std::static_pointer_cast<ChSurfaceNurbs>(surface_geometry_);

                // Find the point on the surface which is closest to the origin of marker 1
                const ChVector3d m_1_origin = marker1->GetAbsCoordsys().pos;
                FindNearestSurfacePoint(surface_nurbs_, m_1_origin, current_U_, current_V_,
                                        tolerance_, max_iters_, samples_, verbose_, this->GetName());

                // Compute tangent vectors
                // DeriveSurfaceNurbs(vec_du, vec_dv, surface_nurbs_, U, V);
                DerivativeSurface(vec_du, vec_dv, surface_nurbs_, current_U_, current_V_, bdf_);

                // Define missing axis with respect to tangent vectors
                vec_du = Vnorm(vec_du);
                vec_dv = Vnorm(vec_dv);
                ChVector3d vec_norm = Vcross(vec_du, vec_dv);

                // Define new marker 2 pose
                ChVector3d m_2_pos_new = surface_nurbs_->Evaluate(current_U_, current_V_);

                ChMatrix33<> m_abs;
                m_abs.SetFromDirectionAxes(vec_du, vec_dv, vec_norm);
                ChQuaternion m_2_q_new = m_abs.GetQuaternion();

                ChFrame<> m_2_pose_new;
                m_2_pose_new.SetPos(m_2_pos_new);
                m_2_pose_new.SetRot(m_2_q_new);

                // Move (main) marker 2 to new location (tangent pose)
                marker2->ImposeAbsoluteTransform(m_2_pose_new);
                // BDF routine won't handle speed and acc.calculus of the moved marker!
                marker2->SetMotionType(ChMarker::MotionType::EXTERNAL);

                CheckLimits_(current_U_, current_V_);
            } else {
                // surface_type_ == LINE
                double U;
                ChQuaternion m_2_q_new;
                ChVector3d m_2_pos_new, vec_du_1, vec_du_2, vec_norm;

                // Find the point on the surface which is closest to the origin of marker 1
                ChVector3d m_1_origin = marker1->GetAbsCoordsys().pos;
                // TODO

                utils::throw_invalid_argument(
                    true, "[ERROR] [ChLinkLockPointSurface] Surface type LINE not implemented yet!\n");
            }
        } else {
            utils::throw_invalid_argument(true, "[ERROR] [ChLinkLockPointSurface] Surface geometry not defined!\n");
        }
    }

    bool ChLinkLockPointSurface::FindNearestSurfacePoint(const std::shared_ptr<ChSurface>& surface,
                                                         const ChVector3d& point,
                                                         double& U_res,
                                                         double& V_res,
                                                         double tol,
                                                         double max_iters,
                                                         int samples,
                                                         bool verbose,
                                                         std::string name) {
        bool closed_U, closed_V;
        double U, V;
        double U_best = 0.0, V_best = 0.0;
        double dist_best = 9999999;
        double dist, d_u_1, d_u_2, d_v_1, d_v_2;
        int iters = 0;

        ChVector3d vec_res, vec_p_1, vec_p_2, vec_p_3, vec_p_4;

        closed_U = surface->IsClosedU();
        closed_V = surface->IslosedV();

        // 1st approximation
        for (int u = 0; u <= samples; u++) {
            U = static_cast<double>(u) / samples;

            for (int v = 0; v <= samples; v++) {
                V = static_cast<double>(v) / samples;
                vec_res = surface->Evaluate(U, V);
                dist = Vlength(Vsub(vec_res, point));

                if (dist < dist_best) {
                    dist_best = dist;
                    U_best = U;
                    V_best = V;
                }
            }
        }

        if (verbose) {
            fmt::print(utils::DEBUG_MSG, "[DEBUG] [ChLinkLockPointSurface] [{:^30s}] 1st approximation: "
                       "{:>4} | U_best = {:>7.6f} | V_best = {:>7.6f} | dist = {:>9.7f}\n", name, samples, U_best,
                       V_best,
                       dist);
        }

        // Refine position with pseudo-NR
        double step = 1.0 / samples;
        double U_1, U_2, V_1, V_2;
        double U_nr = U_best, V_nr = V_best;

        U_1 = U_nr - step;
        V_1 = V_nr - step;

        U_2 = U_nr + step;
        V_2 = V_nr + step;

        if (U_1 < 0.0) {
            if (!closed_U)
                U_1 = 0;
            else
                U_1 += 1.0;
        }
        if (V_1 < 0.0) {
            if (!closed_V)
                V_1 = 0;
            else
                V_1 += 1.0;
        }

        if (U_2 > 1.0) {
            if (!closed_U)
                U_2 = 1.0;
            else
                U_2 -= 1.0;
        }
        if (V_2 > 1.0) {
            if (!closed_V)
                V_2 = 1.0;
            else
                V_2 -= 1.0;
        }

        vec_res = surface->Evaluate(U_1, V_best);
        d_u_1 = Vlength(Vsub(vec_res, point));
        vec_p_1 = vec_res;

        vec_res = surface->Evaluate(U_2, V_best);
        d_u_2 = Vlength(Vsub(vec_res, point));
        vec_p_2 = vec_res;

        vec_res = surface->Evaluate(U_best, V_1);
        d_v_1 = Vlength(Vsub(vec_res, point));
        vec_p_3 = vec_res;

        vec_res = surface->Evaluate(U_best, V_2);
        d_v_2 = Vlength(Vsub(vec_res, point));
        vec_p_4 = vec_res;

        while (true) {
            iters++;

            if (U_nr < 0.0) {
                if (!closed_U)
                    U_nr = 0;
                else
                    U_nr += 1.0;
            }
            if (V_nr < 0.0) {
                if (!closed_V)
                    V_nr = 0;
                else
                    V_nr += 1.0;
            }

            if (U_nr > 1.0) {
                if (!closed_U)
                    U_nr = 1.0;
                else
                    U_nr -= 1.0;
            }
            if (V_nr > 1.0) {
                if (!closed_V)
                    V_nr = 1.0;
                else
                    V_nr -= 1.0;
            }

            vec_res = surface->Evaluate(U_nr, V_nr);
            dist = Vlength(Vsub(vec_res, point));

            //            auto d_min = std::min({d_u_1, d_u_2, d_v_1, d_v_2}, std::less<>());

            if (d_u_1 < d_u_2) {
                U_2 = U_nr;
                d_u_2 = dist;
                vec_p_2 = vec_res; // Move point 2 to U_nr
                step /= 2;
                U_nr -= step; // Move U_nr to the middle
                if (d_u_1 < dist)
                    U_best = U_1;
            } else {
                U_1 = U_nr;
                d_u_1 = dist;
                vec_p_1 = vec_res;
                step /= 2;
                U_nr += step;
                if (d_u_2 < dist)
                    U_best = U_2;
            }

            if (d_v_1 < d_v_2) {
                V_2 = V_nr;
                d_v_2 = dist;
                vec_p_4 = vec_res; // Move point 4 to V_nr
                step /= 2;
                V_nr -= step;
                if (d_v_1 < dist)
                    V_best = V_1;
            } else {
                V_1 = V_nr;
                d_v_1 = dist;
                vec_p_3 = vec_res;
                step /= 2;
                V_nr += step;
                if (d_v_2 < dist)
                    V_best = V_2;
            }

            if (verbose) {
                fmt::print(utils::DEBUG_MSG, "[DEBUG] [ChLinkLockPointSurface] [{:^30s}] Refinement: "
                           "{:>4} | U_res = {:>7.6f} | V_res = {:>7.6f} | dist = {:>9.7f}\n", name, iters, U_best,
                           V_best, dist);
            }

            if ((Vlength(Vsub(vec_p_1, vec_p_2)) <= tol)
                || (Vlength(Vsub(vec_p_3, vec_p_4)) <= tol)
                || (dist <= tol)) {
                U_res = U_best;
                V_res = V_best;

                if (verbose) {
                    fmt::print(utils::INFO_MSG, "[INFO] [ChLinkLockPointSurface] [{:^30s}] Search converged: "
                               "{:>4} | U_res = {:>7.6f} | V_res = {:>7.6f} | dist = {:>9.7f}\n\n", name, iters, U_res,
                               V_res, dist);
                }

                return true;
            }

            if (iters >= max_iters) {
                U_res = U_best;
                V_res = V_best;

                if (verbose) {
                    fmt::print(utils::WARNING_MSG,
                               "[WARNING] [ChLinkLockPointSurface] [{:^30s}] Search reached max iteration: "
                               "{:>4} | U_res = {:>7.6f} | V_res = {:>7.6f} | dist = {:>9.7f}\n\n", name, iters, U_res,
                               V_res, dist);
                }

                return false;
            }
        }
    }

    void ChLinkLockPointSurface::DeriveSurfaceNurbs(ChVector3d& dir_u, ChVector3d& dir_v,
                                                    const std::shared_ptr<ChSurfaceNurbs>& surface,
                                                    double par_U, double par_V) {
        double u = surface->ComputeKnotUfromU(par_U);
        double v = surface->ComputeKnotVfromV(par_V);

        ChMatrixDynamic<> R(surface->p_u + 1, surface->p_v + 1);
        ChMatrixDynamic<> dRdu(surface->p_u + 1, surface->p_v + 1);
        ChMatrixDynamic<> dRdv(surface->p_u + 1, surface->p_v + 1);
        ChBasisToolsNurbsSurfaces::BasisEvaluateDeriv(surface->p_u, surface->p_v, u, v,
                                                      surface->weights, surface->knots_u, surface->knots_v,
                                                      R, dRdu, dRdv);

        int span_U = ChBasisToolsBSpline::FindSpan(surface->p_u, u, surface->knots_u);
        int span_V = ChBasisToolsBSpline::FindSpan(surface->p_v, v, surface->knots_v);

        dir_u = VNULL;
        dir_v = VNULL;
        int u_ind = span_U - surface->p_u;
        int v_ind = span_V - surface->p_v;

        for (int iu = 0; iu <= surface->p_u; iu++) {
            for (int iv = 0; iv <= surface->p_v; iv++) {
                dir_u += surface->points(u_ind + iu, v_ind + iv) * dRdu(iu, iv);
                dir_v += surface->points(u_ind + iu, v_ind + iv) * dRdv(iu, iv);
            }
        }
    }

    void ChLinkLockPointSurface::DerivativeSurface(ChVector3d& dir_u, ChVector3d& dir_v,
                                                   const std::shared_ptr<ChSurface>& surface,
                                                   double U,
                                                   double V,
                                                   double bdf) {
        double u_1, u_2, v_1, v_2;

        // Derivative in direction of U
        if (U > 0.5) {
            u_2 = U;
            u_1 = U - bdf;
        } else {
            u_2 = U + bdf;
            u_1 = U;
        }

        ChVector3d vec_1 = surface->Evaluate(u_1, V);
        ChVector3d vec_2 = surface->Evaluate(u_2, V);
        dir_u = (vec_2.eigen() - vec_1.eigen()) / bdf;

        // Derivative in direction of V
        if (V > 0.5) {
            v_2 = V;
            v_1 = V - bdf;
        } else {
            v_2 = V + bdf;
            v_1 = V;
        }

        vec_1 = surface->Evaluate(U, v_1);
        vec_2 = surface->Evaluate(U, v_2);
        dir_v = (vec_2.eigen() - vec_1.eigen()) / bdf;
    }

    void ChLinkLockPointSurface::DerivativeSurface(ChMatrixNM<double, 3, 2>& J,
                                                   const std::shared_ptr<ChSurface>& surface,
                                                   double U,
                                                   double V,
                                                   double bdf) {
        ChVector3d dir_u, dir_v;

        DerivativeSurface(dir_u, dir_v, surface, U, V, bdf);

        J.block(0, 0, 3, 1) = dir_u.eigen();
        J.block(0, 1, 3, 1) = dir_v.eigen();
    }

    void ChLinkLockPointSurface::ArchiveOut(ChArchiveOut& archive_out) {
        // Version number
        archive_out.VersionWrite<ChLinkLockPointSurface>();

        // Serialize parent class
        ChLinkLock::ArchiveOut(archive_out);

        // Serialize enums
        ChLinkLockPointSurfaceEnumMapper::ChSurfaceType_mapper type_mapper;
        archive_out << CHNVP(type_mapper(surface_type_), "surface_type_");

        // Serialize all member data
        archive_out << CHNVP(verbose_);
        archive_out << CHNVP(limit_u_active_);
        archive_out << CHNVP(limit_v_active_);
        archive_out << CHNVP(current_U_);
        archive_out << CHNVP(current_V_);
        archive_out << CHNVP(samples_);
        archive_out << CHNVP(max_iters_);
        archive_out << CHNVP(tolerance_);
        archive_out << CHNVP(bdf_);
        archive_out << CHNVP(limit_x_init_);
        archive_out << CHNVP(limit_y_init_);
        archive_out << CHNVP(m_1_init_pose_);
        archive_out << CHNVP(m_2_init_pose_);
        archive_out << CHNVP(surface_geometry_);
    }

    void ChLinkLockPointSurface::ArchiveIn(ChArchiveIn& archive_in) {
        // Version number
        int version = archive_in.VersionRead<ChLinkLockPointSurface>();

        // Deserialize parent class
        ChLinkLock::ArchiveIn(archive_in);

        // Deserialize enums
        ChLinkLockPointSurfaceEnumMapper::ChSurfaceType_mapper type_mapper;
        archive_in >> CHNVP(type_mapper(surface_type_), "surface_type_");

        // Deserialize all member data:
        archive_in >> CHNVP(verbose_);
        archive_in >> CHNVP(limit_u_active_);
        archive_in >> CHNVP(limit_v_active_);
        archive_in >> CHNVP(current_U_);
        archive_in >> CHNVP(current_V_);
        archive_in >> CHNVP(samples_);
        archive_in >> CHNVP(max_iters_);
        archive_in >> CHNVP(tolerance_);
        archive_in >> CHNVP(bdf_);
        archive_in >> CHNVP(limit_x_init_);
        archive_in >> CHNVP(limit_y_init_);
        archive_in >> CHNVP(m_1_init_pose_);
        archive_in >> CHNVP(m_2_init_pose_);
        archive_in >> CHNVP(surface_geometry_);
    }

    void ChLinkLockPointSurface::CheckLimits_(double U, double V) {
        if (!limit_u_active_ && !limit_v_active_) return;

        constexpr double tol{10e-9};

        if (limit_u_active_) {
            if (U <= 0.0 + tol) {
                this->LimitX().SetMin(0.0);
                this->LimitX().SetActive(true);
                this->LimitX().constr_upper.SetActive(false);
            } else if (U >= 1.0 - tol) {
                this->LimitX().SetMax(0.0);
                this->LimitX().SetActive(true);
                this->LimitX().constr_lower.SetActive(false);
            } else {
                this->LimitX().SetActive(false);
            }
        }

        if (limit_v_active_) {
            if (V <= 0.0 + tol) {
                this->LimitY().SetMin(0.0);
                this->LimitY().SetActive(true);
                this->LimitY().constr_upper.SetActive(false);
            } else if (V >= 1.0 - tol) {
                this->LimitY().SetMax(0.0);
                this->LimitY().SetActive(true);
                this->LimitY().constr_lower.SetActive(false);
            } else {
                this->LimitY().SetActive(false);
            }
        }
    }
} // namespace chrono::biomechanics
