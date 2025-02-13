/**
 * @file ChJaw.cpp
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 01.08.2023
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

#include <list>
#include <tuple>
#include <limits>
#include <sstream>
//
#include "exosim/biomechanics/ChJaw.h"
#include "exosim/biomechanics/utils/ChUtilsJSON.h"
#include "exosim/biomechanics/utils/ChEnumStringConverter.h"
#include "exosim/biomechanics/vis/ChVisualShapeBiomechanics.h"
#ifdef CHRONO_VSG
#include "exosim/biomechanics/vis/ChJawVSG.h"
#endif
//
#include <chrono/core/ChRealtimeStep.h>
//
#include <chrono/solver/ChIterativeSolverLS.h>
#include <chrono/solver/ChIterativeSolverVI.h>
#include <chrono/solver/ChSolverBB.h>
#include <chrono/solver/ChSolverAPGD.h>
#include <chrono/solver/ChSolverPJacobi.h>
#include <chrono/solver/ChSolverPSOR.h>
#include <chrono/solver/ChSolverPSSOR.h>
//
#include <chrono/timestepper/ChAssemblyAnalysis.h>
//
#include <chrono/assets/ChVisualShapes.h>
#include <chrono/assets/ChVisualShapeFEA.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
// #include <chrono/assets/ChVisualShapePointPoint.h>
//
#include <chrono/functions/ChFunction.h>

#include "fmt/chrono.h"


namespace chrono::biomechanics {
    // Register into the object factory, to enable run-time dynamic creation and persistence
    CH_FACTORY_REGISTER(ChJaw)

    ChJaw::ChJaw(unsigned int threads)
        : threads_(threads) {
    }

    // ChJaw::ChJaw(const ChJaw& other)
    //     : enable_rigid_body_model_(other.enable_rigid_body_model_),
    //       enable_solver_debugging_(other.enable_solver_debugging_),
    //       enable_system_debugging_(other.enable_system_debugging_),
    //       threads_(other.threads_),
    //       gravity_(other.gravity_),
    //       time_step_(other.time_step_),
    //       simulation_time_limit_(other.simulation_time_limit_),
    //       reset_simulation_state_(other.reset_simulation_state_),
    //       time_stepper_verbose_(other.time_stepper_verbose_),
    //       hht_step_control_(other.hht_step_control_),
    //       euler_i_max_iters_(other.euler_i_max_iters_),
    //       trapezoidal_lin_max_iters_(other.trapezoidal_lin_max_iters_),
    //       newmark_max_iters_(other.newmark_max_iters_),
    //       hht_max_iters_(other.hht_max_iters_),
    //       hht_alpha_(other.hht_alpha_),
    //       time_stepper_type_(other.time_stepper_type_),
    //       solver_verbose_(other.solver_verbose_),
    //       solver_tolerance_(other.solver_tolerance_),
    //       max_solver_iterations_(other.max_solver_iterations_),
    //       solver_type_(other.solver_type_),
    //       admm_warm_start_(other.admm_warm_start_),
    //       admm_rho_(other.admm_rho_),
    //       admm_tolerance_dual_(other.admm_tolerance_dual_),
    //       admm_tolerance_primal_(other.admm_tolerance_primal_),
    //       admm_step_policy_(other.admm_step_policy_),
    //       admm_acceleration_(other.admm_acceleration_),
    //       mkl_lock_sparsity_pattern_(other.mkl_lock_sparsity_pattern_),
    //       mkl_use_sparsity_pattern_learner_(other.mkl_use_sparsity_pattern_learner_),
    //       run_sim_at_startup_(other.run_sim_at_startup_),
    //       window_width_(other.window_width_),
    //       window_height_(other.window_height_),
    //       window_title_(other.window_title_),
    //       vis_mode_(other.vis_mode_),
    //       render_mode_(other.render_mode_),
    //       bone_color_(other.bone_color_),
    //       background_color_(other.background_color_),
    //       collision_margin_(other.collision_margin_),
    //       collision_envelope_(other.collision_envelope_),
    //       bone_mesh_swept_sphere_thickness_(other.bone_mesh_swept_sphere_thickness_),
    //       tmj_disc_mesh_swept_sphere_thickness_(other.tmj_disc_mesh_swept_sphere_thickness_),
    //       contact_method_(other.contact_method_),
    //       collision_type_(other.collision_type_),
    //       narrowphase_algorithm_multicore_(other.narrowphase_algorithm_multicore_),
    //       broadphase_grid_resolution_multicore_(other.broadphase_grid_resolution_multicore_),
    //       contact_breaking_threshold_bullet_(other.contact_breaking_threshold_bullet_),
    //       default_effective_curvature_radius_smc_(other.default_effective_curvature_radius_smc_),
    //       contact_force_model_smc_(other.contact_force_model_smc_),
    //       adhesion_force_model_smc_(other.adhesion_force_model_smc_),
    //       tangential_displ_model_smc_(other.tangential_displ_model_smc_),
    //       min_bounce_speed_nsc_(other.min_bounce_speed_nsc_),
    //       contact_recovery_speed_nsc_(other.contact_recovery_speed_nsc_),
    //       bone_density_(other.bone_density_),
    //       use_material_properties_smc_(other.use_material_properties_smc_),
    //       bone_properties_smc_(other.bone_properties_smc_),
    //       tmj_disc_properties_smc_(other.tmj_disc_properties_smc_),
    //       bone_properties_nsc_(other.bone_properties_nsc_),
    //       tmj_disc_properties_nsc_(other.tmj_disc_properties_nsc_),
    //       tmj_disc_fea_use_mooney_rivlin_(other.tmj_disc_fea_use_mooney_rivlin_),
    //       tmj_disc_fea_mr_c_1_(other.tmj_disc_fea_mr_c_1_),
    //       tmj_disc_fea_mr_c_2_(other.tmj_disc_fea_mr_c_2_),
    //       tmj_disc_fea_density_(other.tmj_disc_fea_density_),
    //       tmj_disc_fea_young_modulus_(other.tmj_disc_fea_young_modulus_),
    //       tmj_disc_fea_poisson_ratio_(other.tmj_disc_fea_poisson_ratio_),
    //       tmj_disc_fea_rayleigh_damping_alpha_(other.tmj_disc_fea_rayleigh_damping_alpha_),
    //       tmj_disc_fea_rayleigh_damping_beta_(other.tmj_disc_fea_rayleigh_damping_beta_),
    //       tmj_disc_fea_aux_body_radius_(other.tmj_disc_fea_aux_body_radius_),
    //       tmj_disc_fea_element_type_(other.tmj_disc_fea_element_type_),
    //       tmj_disc_fea_implementation_(other.tmj_disc_fea_implementation_),
    //       mandible_kinematics_(other.mandible_kinematics_),
    //       incisal_frame_abs_(other.incisal_frame_abs_),
    //       stiff_ligaments_(other.stiff_ligaments_),
    //       ligament_type_(other.ligament_type_),
    //       k_e_(other.k_e_),
    //       slack_length_factor_(other.slack_length_factor_),
    //       tmj_capsule_k_e_(other.tmj_capsule_k_e_),
    //       slack_length_factor_capsule_(other.slack_length_factor_capsule_),
    //       tmj_capsule_ligament_color_(other.tmj_capsule_ligament_color_),
    //       stiff_muscles_(other.stiff_muscles_),
    //       muscle_activation_dynamics_(other.muscle_activation_dynamics_),
    //       muscle_type_(other.muscle_type_),
    //       f_frac_(other.f_frac_),
    //       f_frac_open_(other.f_frac_open_),
    //       l_max_p_scale_(other.l_max_p_scale_),
    //       mesh_dir_(other.mesh_dir_),
    //       skull_obj_file_vis_(other.skull_obj_file_vis_),
    //       maxilla_obj_file_vis_(other.maxilla_obj_file_vis_),
    //       mandible_obj_file_vis_(other.mandible_obj_file_vis_),
    //       hyoid_obj_file_vis_(other.hyoid_obj_file_vis_),
    //       skull_obj_file_coll_(other.skull_obj_file_coll_),
    //       maxilla_obj_file_coll_(other.maxilla_obj_file_coll_),
    //       mandible_obj_file_coll_(other.mandible_obj_file_coll_),
    //       skull_obj_file_rigid_coll_(other.skull_obj_file_rigid_coll_),
    //       maxilla_obj_file_rigid_coll_(other.maxilla_obj_file_rigid_coll_),
    //       mandible_obj_file_rigid_coll_(other.mandible_obj_file_rigid_coll_),
    //       tmj_disc_obj_file_coll_(other.tmj_disc_obj_file_coll_),
    //       fea_wireframe_(other.fea_wireframe_),
    //       fea_smooth_faces_(other.fea_smooth_faces_),
    //       color_scale_min_max_(other.color_scale_min_max_),
    //       fea_vis_data_type_(other.fea_vis_data_type_),
    //       tmj_disc_left_msh_file_fea_(other.tmj_disc_left_msh_file_fea_),
    //       tmj_disc_right_msh_file_fea_(other.tmj_disc_right_msh_file_fea_) {
    //     // contact_behavior_map_ = other.contact_behavior_map_;
    //     bone_contact_material_ = nullptr;
    //     tmj_disc_contact_material_ = nullptr;
    //     tmj_disc_fea_material_ = chrono_types::make_shared<fea::ChContinuumMaterial>(*other.tmj_disc_fea_material_);
    //     sys_ = nullptr;
    //
    //     // Deep copy of shared pointers
    //     for (size_t i = 0; i < other.tmj_disc_left_fea_aux_bodies_.size(); ++i) {
    //         tmj_disc_left_fea_aux_bodies_[i] = std::make_shared<ChBody>(*other.tmj_disc_left_fea_aux_bodies_[i]);
    //     }
    //     for (size_t i = 0; i < other.tmj_disc_right_fea_aux_bodies_.size(); ++i) {
    //         tmj_disc_right_fea_aux_bodies_[i] = std::make_shared<ChBody>(*other.tmj_disc_right_fea_aux_bodies_[i]);
    //     }
    //
    //     tmj_disc_left_fea_aux_links_.clear();
    //     tmj_disc_right_fea_aux_links_.clear();
    //     tmj_disc_left_fea_ = nullptr;
    //     tmj_disc_right_fea_ = nullptr;
    //
    //     skull_ = chrono_types::make_shared<ChBodyAuxRef>();
    //     maxilla_ = chrono_types::make_shared<ChBodyAuxRef>();
    //     mandible_ = chrono_types::make_shared<ChBodyAuxRef>();
    //     hyoid_ = chrono_types::make_shared<ChBodyAuxRef>();
    //
    //     incisal_point_aux_body_ = chrono_types::make_shared<ChBody>();
    //     incisal_point_link_ = chrono_types::make_shared<ChLinkMateFix>();
    //
    //     tmj_constr_left_ = nullptr;
    //     tmj_constr_right_ = nullptr;
    //
    //     ligament_map_.clear();
    //     tmj_capsule_ligament_map_.clear();
    //
    //     ligament_init_properties_ = other.ligament_init_properties_;
    //     tmj_capsule_ligament_init_properties_ = other.tmj_capsule_ligament_init_properties_;
    //
    //     // Update attachment bodies in ligament_init_properties_
    //     for (auto& prop: ligament_init_properties_) {
    //         prop.body_1 = this->GetBoneBody(prop.body_1->GetName());
    //         prop.body_2 = this->GetBoneBody(prop.body_2->GetName());
    //         prop.shape = std::make_shared<ChVisualShapeSpring>(0.0025, 100, 20);
    //     }
    //
    //     // Update attachment bodies in tmj_capsule_ligament_init_properties_
    //     for (auto& prop: tmj_capsule_ligament_init_properties_) {
    //         try {
    //             prop.body_1 = this->GetBoneBody(prop.body_1->GetName());
    //         } catch (std::invalid_argument& e) {
    //             const auto aux_body_left = std::find_if(tmj_disc_left_fea_aux_bodies_.begin(),
    //                                                     tmj_disc_left_fea_aux_bodies_.end(),
    //                                                     [&prop](const auto& body) {
    //                                                         return body->GetName() == prop.body_1->GetName();
    //                                                     });
    //             const auto aux_body_right = std::find_if(tmj_disc_right_fea_aux_bodies_.begin(),
    //                                                      tmj_disc_right_fea_aux_bodies_.end(),
    //                                                      [&prop](const auto& body) {
    //                                                          return body->GetName() == prop.body_1->GetName();
    //                                                      });
    //             if (aux_body_left != tmj_disc_left_fea_aux_bodies_.end()) {
    //                 const char index = (*aux_body_left)->GetName().back();
    //                 prop.body_1 = tmj_disc_left_fea_aux_bodies_[index - '0'];
    //             } else if (aux_body_right != tmj_disc_right_fea_aux_bodies_.end()) {
    //                 const char index = (*aux_body_right)->GetName().back();
    //                 prop.body_1 = tmj_disc_right_fea_aux_bodies_[index - '0'];
    //             } else {
    //                 utils::throw_invalid_argument(true, "[ERROR] [ChJaw] Invalid body name in "
    //                                               "'tmj_capsule_ligament_init_properties_'!\n");
    //             }
    //         }
    //
    //         try {
    //             prop.body_2 = this->GetBoneBody(prop.body_2->GetName());
    //         } catch (std::invalid_argument& e) {
    //             const auto aux_body_left = std::find_if(tmj_disc_left_fea_aux_bodies_.begin(),
    //                                                     tmj_disc_left_fea_aux_bodies_.end(),
    //                                                     [&prop](const auto& body) {
    //                                                         return body->GetName() == prop.body_2->GetName();
    //                                                     });
    //             const auto aux_body_right = std::find_if(tmj_disc_right_fea_aux_bodies_.begin(),
    //                                                      tmj_disc_right_fea_aux_bodies_.end(),
    //                                                      [&prop](const auto& body) {
    //                                                          return body->GetName() == prop.body_2->GetName();
    //                                                      });
    //             if (aux_body_left != tmj_disc_left_fea_aux_bodies_.end()) {
    //                 const char index = (*aux_body_left)->GetName().back();
    //                 prop.body_2 = tmj_disc_left_fea_aux_bodies_[index - '0'];
    //             } else if (aux_body_right != tmj_disc_right_fea_aux_bodies_.end()) {
    //                 const char index = (*aux_body_right)->GetName().back();
    //                 prop.body_2 = tmj_disc_right_fea_aux_bodies_[index - '0'];
    //             } else {
    //                 utils::throw_invalid_argument(true, "[ERROR] [ChJaw] Invalid body name in "
    //                                               "'tmj_capsule_ligament_init_properties_'!\n");
    //             }
    //         }
    //
    //         prop.shape = std::make_shared<ChVisualShapeSpring>(0.0025, 100, 20);
    //     }
    //
    //     muscle_map_.clear();
    //     muscle_init_properties_ = other.muscle_init_properties_;
    //
    //     for (auto& prop: muscle_init_properties_) {
    //         prop.body_1 = this->GetBoneBody(prop.body_1->GetName());
    //         prop.body_2 = this->GetBoneBody(prop.body_2->GetName());
    //         // prop.shape = std::make_shared<ChVisualShapeSpring>(0.0025, 150, 20);
    //     }
    //
    //     build_called_ = false;
    // }
    //
    // ChJaw::ChJaw(ChJaw&& other) noexcept
    //     : build_called_(std::move(other.build_called_)),
    //       enable_rigid_body_model_(std::move(other.enable_rigid_body_model_)),
    //       enable_solver_debugging_(std::move(other.enable_solver_debugging_)),
    //       enable_system_debugging_(std::move(other.enable_system_debugging_)),
    //       threads_(std::move(other.threads_)),
    //       gravity_(std::move(other.gravity_)),
    //       time_step_(std::move(other.time_step_)),
    //       simulation_time_limit_(std::move(other.simulation_time_limit_)),
    //       reset_simulation_state_(std::move(other.reset_simulation_state_)),
    //       time_stepper_verbose_(std::move(other.time_stepper_verbose_)),
    //       hht_step_control_(std::move(other.hht_step_control_)),
    //       euler_i_max_iters_(std::move(other.euler_i_max_iters_)),
    //       trapezoidal_lin_max_iters_(std::move(other.trapezoidal_lin_max_iters_)),
    //       newmark_max_iters_(std::move(other.newmark_max_iters_)),
    //       hht_max_iters_(std::move(other.hht_max_iters_)),
    //       hht_alpha_(std::move(other.hht_alpha_)),
    //       time_stepper_type_(std::move(other.time_stepper_type_)),
    //       solver_verbose_(std::move(other.solver_verbose_)),
    //       solver_tolerance_(std::move(other.solver_tolerance_)),
    //       max_solver_iterations_(std::move(other.max_solver_iterations_)),
    //       solver_type_(std::move(other.solver_type_)),
    //       admm_warm_start_(std::move(other.admm_warm_start_)),
    //       admm_rho_(std::move(other.admm_rho_)),
    //       admm_tolerance_dual_(std::move(other.admm_tolerance_dual_)),
    //       admm_tolerance_primal_(std::move(other.admm_tolerance_primal_)),
    //       admm_step_policy_(std::move(other.admm_step_policy_)),
    //       admm_acceleration_(std::move(other.admm_acceleration_)),
    //       mkl_lock_sparsity_pattern_(std::move(other.mkl_lock_sparsity_pattern_)),
    //       mkl_use_sparsity_pattern_learner_(std::move(other.mkl_use_sparsity_pattern_learner_)),
    //       run_sim_at_startup_(std::move(other.run_sim_at_startup_)),
    //       window_width_(std::move(other.window_width_)),
    //       window_height_(std::move(other.window_height_)),
    //       window_title_(std::move(other.window_title_)),
    //       vis_mode_(std::move(other.vis_mode_)),
    //       render_mode_(std::move(other.render_mode_)),
    //       bone_color_(std::move(other.bone_color_)),
    //       background_color_(std::move(other.background_color_)),
    //       collision_margin_(std::move(other.collision_margin_)),
    //       collision_envelope_(std::move(other.collision_envelope_)),
    //       bone_mesh_swept_sphere_thickness_(std::move(other.bone_mesh_swept_sphere_thickness_)),
    //       tmj_disc_mesh_swept_sphere_thickness_(std::move(other.tmj_disc_mesh_swept_sphere_thickness_)),
    //       contact_method_(std::move(other.contact_method_)),
    //       collision_type_(std::move(other.collision_type_)),
    //       narrowphase_algorithm_multicore_(std::move(other.narrowphase_algorithm_multicore_)),
    //       broadphase_grid_resolution_multicore_(std::move(other.broadphase_grid_resolution_multicore_)),
    //       contact_breaking_threshold_bullet_(std::move(other.contact_breaking_threshold_bullet_)),
    //       default_effective_curvature_radius_smc_(std::move(other.default_effective_curvature_radius_smc_)),
    //       contact_force_model_smc_(std::move(other.contact_force_model_smc_)),
    //       adhesion_force_model_smc_(std::move(other.adhesion_force_model_smc_)),
    //       tangential_displ_model_smc_(std::move(other.tangential_displ_model_smc_)),
    //       min_bounce_speed_nsc_(std::move(other.min_bounce_speed_nsc_)),
    //       contact_recovery_speed_nsc_(std::move(other.contact_recovery_speed_nsc_)),
    //       bone_density_(std::move(other.bone_density_)),
    //       bone_contact_material_(std::move(other.bone_contact_material_)),
    //       tmj_disc_contact_material_(std::move(other.tmj_disc_contact_material_)),
    //       use_material_properties_smc_(std::move(other.use_material_properties_smc_)),
    //       bone_properties_smc_(std::move(other.bone_properties_smc_)),
    //       tmj_disc_properties_smc_(std::move(other.tmj_disc_properties_smc_)),
    //       bone_properties_nsc_(std::move(other.bone_properties_nsc_)),
    //       tmj_disc_properties_nsc_(std::move(other.tmj_disc_properties_nsc_)),
    //       tmj_disc_fea_use_mooney_rivlin_(std::move(other.tmj_disc_fea_use_mooney_rivlin_)),
    //       tmj_disc_fea_mr_c_1_(std::move(other.tmj_disc_fea_mr_c_1_)),
    //       tmj_disc_fea_mr_c_2_(std::move(other.tmj_disc_fea_mr_c_2_)),
    //       tmj_disc_fea_density_(std::move(other.tmj_disc_fea_density_)),
    //       tmj_disc_fea_young_modulus_(std::move(other.tmj_disc_fea_young_modulus_)),
    //       tmj_disc_fea_poisson_ratio_(std::move(other.tmj_disc_fea_poisson_ratio_)),
    //       tmj_disc_fea_rayleigh_damping_alpha_(std::move(other.tmj_disc_fea_rayleigh_damping_alpha_)),
    //       tmj_disc_fea_rayleigh_damping_beta_(std::move(other.tmj_disc_fea_rayleigh_damping_beta_)),
    //       tmj_disc_fea_material_(std::move(other.tmj_disc_fea_material_)),
    //       sys_(std::move(other.sys_)),
    //       tmj_disc_fea_aux_body_radius_(std::move(other.tmj_disc_fea_aux_body_radius_)),
    //       tmj_disc_left_fea_aux_bodies_(std::move(other.tmj_disc_left_fea_aux_bodies_)),
    //       tmj_disc_right_fea_aux_bodies_(std::move(other.tmj_disc_right_fea_aux_bodies_)),
    //       tmj_disc_left_fea_aux_links_(std::move(other.tmj_disc_left_fea_aux_links_)),
    //       tmj_disc_right_fea_aux_links_(std::move(other.tmj_disc_right_fea_aux_links_)),
    //       tmj_disc_fea_element_type_(std::move(other.tmj_disc_fea_element_type_)),
    //       tmj_disc_fea_implementation_(std::move(other.tmj_disc_fea_implementation_)),
    //       tmj_disc_left_fea_(std::move(other.tmj_disc_left_fea_)),
    //       tmj_disc_right_fea_(std::move(other.tmj_disc_right_fea_)),
    //       skull_(std::move(other.skull_)),
    //       maxilla_(std::move(other.maxilla_)),
    //       mandible_(std::move(other.mandible_)),
    //       hyoid_(std::move(other.hyoid_)),
    //       mandible_kinematics_(std::move(other.mandible_kinematics_)),
    //       incisal_frame_abs_(std::move(incisal_frame_abs_)),
    //       incisal_point_aux_body_(std::move(other.incisal_point_aux_body_)),
    //       incisal_point_link_(std::move(other.incisal_point_link_)),
    //       tmj_constr_left_(std::move(other.tmj_constr_left_)),
    //       tmj_constr_right_(std::move(other.tmj_constr_right_)),
    //       stiff_ligaments_(std::move(other.stiff_ligaments_)),
    //       ligament_type_(std::move(other.ligament_type_)),
    //       ligament_map_(std::move(other.ligament_map_)),
    //       tmj_capsule_ligament_map_(std::move(other.tmj_capsule_ligament_map_)),
    //       k_e_(std::move(other.k_e_)),
    //       slack_length_factor_(std::move(other.slack_length_factor_)),
    //       ligament_init_properties_(std::move(other.ligament_init_properties_)),
    //       tmj_capsule_k_e_(std::move(other.tmj_capsule_k_e_)),
    //       slack_length_factor_capsule_(std::move(other.slack_length_factor_capsule_)),
    //       tmj_capsule_ligament_color_(std::move(other.tmj_capsule_ligament_color_)),
    //       tmj_capsule_ligament_init_properties_(std::move(other.tmj_capsule_ligament_init_properties_)),
    //       stiff_muscles_(std::move(other.stiff_muscles_)),
    //       muscle_activation_dynamics_(std::move(other.muscle_activation_dynamics_)),
    //       muscle_type_(std::move(other.muscle_type_)),
    //       muscle_map_(std::move(other.muscle_map_)),
    //       f_frac_(std::move(other.f_frac_)),
    //       f_frac_open_(std::move(other.f_frac_open_)),
    //       l_max_p_scale_(std::move(other.l_max_p_scale_)),
    //       muscle_init_properties_(std::move(other.muscle_init_properties_)),
    //       mesh_dir_(std::move(other.mesh_dir_)),
    //       skull_obj_file_vis_(std::move(other.skull_obj_file_vis_)),
    //       maxilla_obj_file_vis_(std::move(other.maxilla_obj_file_vis_)),
    //       mandible_obj_file_vis_(std::move(other.mandible_obj_file_vis_)),
    //       hyoid_obj_file_vis_(std::move(other.hyoid_obj_file_vis_)),
    //       skull_obj_file_coll_(std::move(other.skull_obj_file_coll_)),
    //       maxilla_obj_file_coll_(std::move(other.maxilla_obj_file_coll_)),
    //       mandible_obj_file_coll_(std::move(other.mandible_obj_file_coll_)),
    //       skull_obj_file_rigid_coll_(std::move(other.skull_obj_file_rigid_coll_)),
    //       maxilla_obj_file_rigid_coll_(std::move(other.maxilla_obj_file_rigid_coll_)),
    //       mandible_obj_file_rigid_coll_(std::move(other.mandible_obj_file_rigid_coll_)),
    //       tmj_disc_obj_file_coll_(std::move(other.tmj_disc_obj_file_coll_)),
    //       fea_wireframe_(std::move(other.fea_wireframe_)),
    //       fea_smooth_faces_(std::move(other.fea_smooth_faces_)),
    //       color_scale_min_max_(std::move(other.color_scale_min_max_)),
    //       fea_vis_data_type_(std::move(other.fea_vis_data_type_)),
    //       tmj_disc_left_msh_file_fea_(std::move(other.tmj_disc_left_msh_file_fea_)),
    //       tmj_disc_right_msh_file_fea_(std::move(other.tmj_disc_right_msh_file_fea_)) {
    //     other.sys_ = nullptr;
    //     other.skull_ = nullptr;
    //     other.maxilla_ = nullptr;
    //     other.mandible_ = nullptr;
    //     other.hyoid_ = nullptr;
    //     other.incisal_point_aux_body_ = nullptr;
    //     other.incisal_point_link_ = nullptr;
    //     other.tmj_constr_left_ = nullptr;
    //     other.tmj_constr_right_ = nullptr;
    //     other.tmj_disc_left_fea_ = nullptr;
    //     other.tmj_disc_right_fea_ = nullptr;
    // }
    //
    // ChJaw& ChJaw::operator=(const ChJaw& other) {
    //     if (this != &other) {
    //         utils::throw_runtime_error(!other.build_called_,
    //                                    "[ERROR] [ChJaw] Cannot copy if 'Build' wasn't called for the source object!\n");
    //
    //         // Copy simple members
    //         enable_rigid_body_model_ = other.enable_rigid_body_model_;
    //         enable_solver_debugging_ = other.enable_solver_debugging_;
    //         enable_system_debugging_ = other.enable_system_debugging_;
    //         threads_ = other.threads_;
    //         gravity_ = other.gravity_;
    //         time_step_ = other.time_step_;
    //         simulation_time_limit_ = other.simulation_time_limit_;
    //         reset_simulation_state_ = other.reset_simulation_state_;
    //         time_stepper_verbose_ = other.time_stepper_verbose_;
    //         hht_step_control_ = other.hht_step_control_;
    //         euler_i_max_iters_ = other.euler_i_max_iters_;
    //         trapezoidal_lin_max_iters_ = other.trapezoidal_lin_max_iters_;
    //         newmark_max_iters_ = other.newmark_max_iters_;
    //         hht_max_iters_ = other.hht_max_iters_;
    //         hht_alpha_ = other.hht_alpha_;
    //         time_stepper_type_ = other.time_stepper_type_;
    //         solver_verbose_ = other.solver_verbose_;
    //         solver_tolerance_ = other.solver_tolerance_;
    //         max_solver_iterations_ = other.max_solver_iterations_;
    //         solver_type_ = other.solver_type_;
    //         admm_warm_start_ = other.admm_warm_start_;
    //         admm_rho_ = other.admm_rho_;
    //         admm_tolerance_dual_ = other.admm_tolerance_dual_;
    //         admm_tolerance_primal_ = other.admm_tolerance_primal_;
    //         admm_step_policy_ = other.admm_step_policy_;
    //         admm_acceleration_ = other.admm_acceleration_;
    //         mkl_lock_sparsity_pattern_ = other.mkl_lock_sparsity_pattern_;
    //         mkl_use_sparsity_pattern_learner_ = other.mkl_use_sparsity_pattern_learner_;
    //         run_sim_at_startup_ = other.run_sim_at_startup_;
    //         window_width_ = other.window_width_;
    //         window_height_ = other.window_height_;
    //         window_title_ = other.window_title_;
    //         vis_mode_ = other.vis_mode_;
    //         render_mode_ = other.render_mode_;
    //         bone_color_ = other.bone_color_;
    //         background_color_ = other.background_color_;
    //         collision_margin_ = other.collision_margin_;
    //         collision_envelope_ = other.collision_envelope_;
    //         bone_mesh_swept_sphere_thickness_ = other.bone_mesh_swept_sphere_thickness_;
    //         tmj_disc_mesh_swept_sphere_thickness_ = other.tmj_disc_mesh_swept_sphere_thickness_;
    //         contact_method_ = other.contact_method_;
    //         collision_type_ = other.collision_type_;
    //         narrowphase_algorithm_multicore_ = other.narrowphase_algorithm_multicore_;
    //         broadphase_grid_resolution_multicore_ = other.broadphase_grid_resolution_multicore_;
    //         contact_breaking_threshold_bullet_ = other.contact_breaking_threshold_bullet_;
    //         default_effective_curvature_radius_smc_ = other.default_effective_curvature_radius_smc_;
    //         contact_force_model_smc_ = other.contact_force_model_smc_;
    //         adhesion_force_model_smc_ = other.adhesion_force_model_smc_;
    //         tangential_displ_model_smc_ = other.tangential_displ_model_smc_;
    //         min_bounce_speed_nsc_ = other.min_bounce_speed_nsc_;
    //         contact_recovery_speed_nsc_ = other.contact_recovery_speed_nsc_;
    //         bone_density_ = other.bone_density_;
    //         use_material_properties_smc_ = other.use_material_properties_smc_;
    //         bone_properties_smc_ = other.bone_properties_smc_;
    //         tmj_disc_properties_smc_ = other.tmj_disc_properties_smc_;
    //         bone_properties_nsc_ = other.bone_properties_nsc_;
    //         tmj_disc_properties_nsc_ = other.tmj_disc_properties_nsc_;
    //         tmj_disc_fea_use_mooney_rivlin_ = other.tmj_disc_fea_use_mooney_rivlin_;
    //         tmj_disc_fea_mr_c_1_ = other.tmj_disc_fea_mr_c_1_;
    //         tmj_disc_fea_mr_c_2_ = other.tmj_disc_fea_mr_c_2_;
    //         tmj_disc_fea_density_ = other.tmj_disc_fea_density_;
    //         tmj_disc_fea_young_modulus_ = other.tmj_disc_fea_young_modulus_;
    //         tmj_disc_fea_poisson_ratio_ = other.tmj_disc_fea_poisson_ratio_;
    //         tmj_disc_fea_rayleigh_damping_alpha_ = other.tmj_disc_fea_rayleigh_damping_alpha_;
    //         tmj_disc_fea_rayleigh_damping_beta_ = other.tmj_disc_fea_rayleigh_damping_beta_;
    //         tmj_disc_fea_aux_body_radius_ = other.tmj_disc_fea_aux_body_radius_;
    //         tmj_disc_fea_element_type_ = other.tmj_disc_fea_element_type_;
    //         tmj_disc_fea_implementation_ = other.tmj_disc_fea_implementation_;
    //         mandible_kinematics_ = other.mandible_kinematics_;
    //         incisal_frame_abs_ = other.incisal_frame_abs_;
    //         stiff_ligaments_ = other.stiff_ligaments_;
    //         ligament_type_ = other.ligament_type_;
    //         k_e_ = other.k_e_;
    //         slack_length_factor_ = other.slack_length_factor_;
    //         tmj_capsule_k_e_ = other.tmj_capsule_k_e_;
    //         slack_length_factor_capsule_ = other.slack_length_factor_capsule_;
    //         tmj_capsule_ligament_color_ = other.tmj_capsule_ligament_color_;
    //         stiff_muscles_ = other.stiff_muscles_;
    //         muscle_activation_dynamics_ = other.muscle_activation_dynamics_;
    //         muscle_type_ = other.muscle_type_;
    //         f_frac_ = other.f_frac_;
    //         f_frac_open_ = other.f_frac_open_;
    //         l_max_p_scale_ = other.l_max_p_scale_;
    //         mesh_dir_ = other.mesh_dir_;
    //         skull_obj_file_vis_ = other.skull_obj_file_vis_;
    //         maxilla_obj_file_vis_ = other.maxilla_obj_file_vis_;
    //         mandible_obj_file_vis_ = other.mandible_obj_file_vis_;
    //         hyoid_obj_file_vis_ = other.hyoid_obj_file_vis_;
    //         skull_obj_file_coll_ = other.skull_obj_file_coll_;
    //         maxilla_obj_file_coll_ = other.maxilla_obj_file_coll_;
    //         mandible_obj_file_coll_ = other.mandible_obj_file_coll_;
    //         skull_obj_file_rigid_coll_ = other.skull_obj_file_rigid_coll_;
    //         maxilla_obj_file_rigid_coll_ = other.maxilla_obj_file_rigid_coll_;
    //         mandible_obj_file_rigid_coll_ = other.mandible_obj_file_rigid_coll_;
    //         tmj_disc_obj_file_coll_ = other.tmj_disc_obj_file_coll_;
    //         fea_wireframe_ = other.fea_wireframe_;
    //         fea_smooth_faces_ = other.fea_smooth_faces_;
    //         color_scale_min_max_ = other.color_scale_min_max_;
    //         fea_vis_data_type_ = other.fea_vis_data_type_;
    //         tmj_disc_left_msh_file_fea_ = other.tmj_disc_left_msh_file_fea_;
    //         tmj_disc_right_msh_file_fea_ = other.tmj_disc_right_msh_file_fea_;
    //
    //
    //         // contact_behavior_map_ = other.contact_behavior_map_;
    //         bone_contact_material_ = nullptr;
    //         tmj_disc_contact_material_ = nullptr;
    //         tmj_disc_fea_material_ = chrono_types::make_shared<fea::ChContinuumMaterial>(*other.tmj_disc_fea_material_);
    //         sys_ = nullptr;
    //
    //         // Deep copy of shared pointers
    //         for (size_t i = 0; i < other.tmj_disc_left_fea_aux_bodies_.size(); ++i) {
    //             tmj_disc_left_fea_aux_bodies_[i] = std::make_shared<ChBody>(*other.tmj_disc_left_fea_aux_bodies_[i]);
    //         }
    //         for (size_t i = 0; i < other.tmj_disc_right_fea_aux_bodies_.size(); ++i) {
    //             tmj_disc_right_fea_aux_bodies_[i] = std::make_shared<ChBody>(*other.tmj_disc_right_fea_aux_bodies_[i]);
    //         }
    //
    //         tmj_disc_left_fea_aux_links_.clear();
    //         tmj_disc_right_fea_aux_links_.clear();
    //         tmj_disc_left_fea_ = nullptr;
    //         tmj_disc_right_fea_ = nullptr;
    //
    //         skull_ = chrono_types::make_shared<ChBodyAuxRef>();
    //         maxilla_ = chrono_types::make_shared<ChBodyAuxRef>();
    //         mandible_ = chrono_types::make_shared<ChBodyAuxRef>();
    //         hyoid_ = chrono_types::make_shared<ChBodyAuxRef>();
    //
    //         incisal_point_aux_body_ = chrono_types::make_shared<ChBody>();
    //         incisal_point_link_ = chrono_types::make_shared<ChLinkMateFix>();
    //
    //         tmj_constr_left_ = nullptr;
    //         tmj_constr_right_ = nullptr;
    //
    //         ligament_map_.clear();
    //         tmj_capsule_ligament_map_.clear();
    //
    //         ligament_init_properties_ = other.ligament_init_properties_;
    //         tmj_capsule_ligament_init_properties_ = other.tmj_capsule_ligament_init_properties_;
    //
    //         // Update attachment bodies in ligament_init_properties_
    //         for (auto& prop: ligament_init_properties_) {
    //             prop.body_1 = this->GetBoneBody(prop.body_1->GetName());
    //             prop.body_2 = this->GetBoneBody(prop.body_2->GetName());
    //             prop.shape = std::make_shared<ChVisualShapeSpring>(0.0025, 100, 20);
    //         }
    //
    //         // Update attachment bodies in tmj_capsule_ligament_init_properties_
    //         for (auto& prop: tmj_capsule_ligament_init_properties_) {
    //             try {
    //                 prop.body_1 = this->GetBoneBody(prop.body_1->GetName());
    //             } catch (std::invalid_argument& e) {
    //                 const auto aux_body_left = std::find_if(tmj_disc_left_fea_aux_bodies_.begin(),
    //                                                         tmj_disc_left_fea_aux_bodies_.end(),
    //                                                         [&prop](const auto& body) {
    //                                                             return body->GetName() == prop.body_1->GetName();
    //                                                         });
    //                 const auto aux_body_right = std::find_if(tmj_disc_right_fea_aux_bodies_.begin(),
    //                                                          tmj_disc_right_fea_aux_bodies_.end(),
    //                                                          [&prop](const auto& body) {
    //                                                              return body->GetName() == prop.body_1->GetName();
    //                                                          });
    //                 if (aux_body_left != tmj_disc_left_fea_aux_bodies_.end()) {
    //                     const char index = (*aux_body_left)->GetName().back();
    //                     prop.body_1 = tmj_disc_left_fea_aux_bodies_[index - '0'];
    //                 } else if (aux_body_right != tmj_disc_right_fea_aux_bodies_.end()) {
    //                     const char index = (*aux_body_right)->GetName().back();
    //                     prop.body_1 = tmj_disc_right_fea_aux_bodies_[index - '0'];
    //                 } else {
    //                     utils::throw_invalid_argument(true, "[ERROR] [ChJaw] Invalid body name in "
    //                                                   "'tmj_capsule_ligament_init_properties_'!\n");
    //                 }
    //             }
    //
    //             try {
    //                 prop.body_2 = this->GetBoneBody(prop.body_2->GetName());
    //             } catch (std::invalid_argument& e) {
    //                 const auto aux_body_left = std::find_if(tmj_disc_left_fea_aux_bodies_.begin(),
    //                                                         tmj_disc_left_fea_aux_bodies_.end(),
    //                                                         [&prop](const auto& body) {
    //                                                             return body->GetName() == prop.body_2->GetName();
    //                                                         });
    //                 const auto aux_body_right = std::find_if(tmj_disc_right_fea_aux_bodies_.begin(),
    //                                                          tmj_disc_right_fea_aux_bodies_.end(),
    //                                                          [&prop](const auto& body) {
    //                                                              return body->GetName() == prop.body_2->GetName();
    //                                                          });
    //                 if (aux_body_left != tmj_disc_left_fea_aux_bodies_.end()) {
    //                     const char index = (*aux_body_left)->GetName().back();
    //                     prop.body_2 = tmj_disc_left_fea_aux_bodies_[index - '0'];
    //                 } else if (aux_body_right != tmj_disc_right_fea_aux_bodies_.end()) {
    //                     const char index = (*aux_body_right)->GetName().back();
    //                     prop.body_2 = tmj_disc_right_fea_aux_bodies_[index - '0'];
    //                 } else {
    //                     utils::throw_invalid_argument(true, "[ERROR] [ChJaw] Invalid body name in "
    //                                                   "'tmj_capsule_ligament_init_properties_'!\n");
    //                 }
    //             }
    //
    //             prop.shape = std::make_shared<ChVisualShapeSpring>(0.0025, 100, 20);
    //         }
    //
    //         muscle_map_.clear();
    //         muscle_init_properties_ = other.muscle_init_properties_;
    //
    //         for (auto& prop: muscle_init_properties_) {
    //             prop.body_1 = this->GetBoneBody(prop.body_1->GetName());
    //             prop.body_2 = this->GetBoneBody(prop.body_2->GetName());
    //             // prop.shape = std::make_shared<ChVisualShapeSpring>(0.0025, 150, 20);
    //         }
    //
    //         this->Build();
    //     }
    //     return *this;
    // }
    //
    // ChJaw& ChJaw::operator=(ChJaw&& other) noexcept {
    //     if (this != &other) {
    //         // Transfer ownership of resources from `other` to `this`
    //         build_called_ = other.build_called_;
    //         enable_rigid_body_model_ = other.enable_rigid_body_model_;
    //         enable_solver_debugging_ = other.enable_solver_debugging_;
    //         enable_system_debugging_ = other.enable_system_debugging_;
    //         threads_ = other.threads_;
    //         gravity_ = other.gravity_;
    //         time_step_ = other.time_step_;
    //         simulation_time_limit_ = other.simulation_time_limit_;
    //         reset_simulation_state_ = other.reset_simulation_state_;
    //         time_stepper_verbose_ = other.time_stepper_verbose_;
    //         hht_step_control_ = other.hht_step_control_;
    //         euler_i_max_iters_ = other.euler_i_max_iters_;
    //         trapezoidal_lin_max_iters_ = other.trapezoidal_lin_max_iters_;
    //         newmark_max_iters_ = other.newmark_max_iters_;
    //         hht_max_iters_ = other.hht_max_iters_;
    //         hht_alpha_ = other.hht_alpha_;
    //         time_stepper_type_ = other.time_stepper_type_;
    //         solver_verbose_ = other.solver_verbose_;
    //         solver_tolerance_ = other.solver_tolerance_;
    //         max_solver_iterations_ = other.max_solver_iterations_;
    //         solver_type_ = other.solver_type_;
    //         admm_warm_start_ = other.admm_warm_start_;
    //         admm_rho_ = other.admm_rho_;
    //         admm_tolerance_dual_ = other.admm_tolerance_dual_;
    //         admm_tolerance_primal_ = other.admm_tolerance_primal_;
    //         admm_step_policy_ = other.admm_step_policy_;
    //         admm_acceleration_ = other.admm_acceleration_;
    //         mkl_lock_sparsity_pattern_ = other.mkl_lock_sparsity_pattern_;
    //         mkl_use_sparsity_pattern_learner_ = other.mkl_use_sparsity_pattern_learner_;
    //         run_sim_at_startup_ = other.run_sim_at_startup_;
    //         window_width_ = other.window_width_;
    //         window_height_ = other.window_height_;
    //         window_title_ = other.window_title_;
    //         vis_mode_ = other.vis_mode_;
    //         render_mode_ = other.render_mode_;
    //         bone_color_ = other.bone_color_;
    //         background_color_ = other.background_color_;
    //         collision_margin_ = other.collision_margin_;
    //         collision_envelope_ = other.collision_envelope_;
    //         bone_mesh_swept_sphere_thickness_ = other.bone_mesh_swept_sphere_thickness_;
    //         tmj_disc_mesh_swept_sphere_thickness_ = other.tmj_disc_mesh_swept_sphere_thickness_;
    //         contact_method_ = other.contact_method_;
    //         collision_type_ = other.collision_type_;
    //         narrowphase_algorithm_multicore_ = other.narrowphase_algorithm_multicore_;
    //         broadphase_grid_resolution_multicore_ = other.broadphase_grid_resolution_multicore_;
    //         contact_breaking_threshold_bullet_ = other.contact_breaking_threshold_bullet_;
    //         default_effective_curvature_radius_smc_ = other.default_effective_curvature_radius_smc_;
    //         contact_force_model_smc_ = other.contact_force_model_smc_;
    //         adhesion_force_model_smc_ = other.adhesion_force_model_smc_;
    //         tangential_displ_model_smc_ = other.tangential_displ_model_smc_;
    //         min_bounce_speed_nsc_ = other.min_bounce_speed_nsc_;
    //         contact_recovery_speed_nsc_ = other.contact_recovery_speed_nsc_;
    //         bone_density_ = other.bone_density_;
    //         use_material_properties_smc_ = other.use_material_properties_smc_;
    //         bone_properties_smc_ = std::move(other.bone_properties_smc_);
    //         tmj_disc_properties_smc_ = std::move(other.tmj_disc_properties_smc_);
    //         bone_properties_nsc_ = std::move(other.bone_properties_nsc_);
    //         tmj_disc_properties_nsc_ = std::move(other.tmj_disc_properties_nsc_);
    //         tmj_disc_fea_use_mooney_rivlin_ = other.tmj_disc_fea_use_mooney_rivlin_;
    //         tmj_disc_fea_mr_c_1_ = other.tmj_disc_fea_mr_c_1_;
    //         tmj_disc_fea_mr_c_2_ = other.tmj_disc_fea_mr_c_2_;
    //         tmj_disc_fea_density_ = other.tmj_disc_fea_density_;
    //         tmj_disc_fea_young_modulus_ = other.tmj_disc_fea_young_modulus_;
    //         tmj_disc_fea_poisson_ratio_ = other.tmj_disc_fea_poisson_ratio_;
    //         tmj_disc_fea_rayleigh_damping_alpha_ = other.tmj_disc_fea_rayleigh_damping_alpha_;
    //         tmj_disc_fea_rayleigh_damping_beta_ = other.tmj_disc_fea_rayleigh_damping_beta_;
    //         tmj_disc_fea_aux_body_radius_ = other.tmj_disc_fea_aux_body_radius_;
    //         tmj_disc_fea_element_type_ = other.tmj_disc_fea_element_type_;
    //         tmj_disc_fea_implementation_ = other.tmj_disc_fea_implementation_;
    //         mandible_kinematics_ = other.mandible_kinematics_;
    //         incisal_frame_abs_ = other.incisal_frame_abs_;
    //         stiff_ligaments_ = other.stiff_ligaments_;
    //         ligament_type_ = other.ligament_type_;
    //         k_e_ = other.k_e_;
    //         slack_length_factor_ = other.slack_length_factor_;
    //         tmj_capsule_k_e_ = other.tmj_capsule_k_e_;
    //         slack_length_factor_capsule_ = other.slack_length_factor_capsule_;
    //         tmj_capsule_ligament_color_ = other.tmj_capsule_ligament_color_;
    //         stiff_muscles_ = other.stiff_muscles_;
    //         muscle_activation_dynamics_ = other.muscle_activation_dynamics_;
    //         muscle_type_ = other.muscle_type_;
    //         f_frac_ = other.f_frac_;
    //         f_frac_open_ = other.f_frac_open_;
    //         l_max_p_scale_ = other.l_max_p_scale_;
    //         mesh_dir_ = other.mesh_dir_;
    //         skull_obj_file_vis_ = other.skull_obj_file_vis_;
    //         maxilla_obj_file_vis_ = other.maxilla_obj_file_vis_;
    //         mandible_obj_file_vis_ = other.mandible_obj_file_vis_;
    //         hyoid_obj_file_vis_ = other.hyoid_obj_file_vis_;
    //         skull_obj_file_coll_ = other.skull_obj_file_coll_;
    //         maxilla_obj_file_coll_ = other.maxilla_obj_file_coll_;
    //         mandible_obj_file_coll_ = other.mandible_obj_file_coll_;
    //         skull_obj_file_rigid_coll_ = other.skull_obj_file_rigid_coll_;
    //         maxilla_obj_file_rigid_coll_ = other.maxilla_obj_file_rigid_coll_;
    //         mandible_obj_file_rigid_coll_ = other.mandible_obj_file_rigid_coll_;
    //         tmj_disc_obj_file_coll_ = other.tmj_disc_obj_file_coll_;
    //         fea_wireframe_ = other.fea_wireframe_;
    //         fea_smooth_faces_ = other.fea_smooth_faces_;
    //         color_scale_min_max_ = other.color_scale_min_max_;
    //         fea_vis_data_type_ = other.fea_vis_data_type_;
    //         tmj_disc_left_msh_file_fea_ = other.tmj_disc_left_msh_file_fea_;
    //         tmj_disc_right_msh_file_fea_ = other.tmj_disc_right_msh_file_fea_;
    //
    //         // Transfer ownership of shared pointers
    //         bone_contact_material_ = std::move(other.bone_contact_material_);
    //         tmj_disc_contact_material_ = std::move(other.tmj_disc_contact_material_);
    //         tmj_disc_fea_material_ = std::move(other.tmj_disc_fea_material_);
    //         sys_ = std::move(other.sys_);
    //
    //         skull_ = std::move(other.skull_);
    //         maxilla_ = std::move(other.maxilla_);
    //         mandible_ = std::move(other.mandible_);
    //         hyoid_ = std::move(other.hyoid_);
    //
    //         incisal_point_aux_body_ = std::move(other.incisal_point_aux_body_);
    //         incisal_point_link_ = std::move(other.incisal_point_link_);
    //
    //         tmj_constr_left_ = std::move(other.tmj_constr_left_);
    //         tmj_constr_right_ = std::move(other.tmj_constr_right_);
    //
    //         ligament_map_ = std::move(other.ligament_map_);
    //         tmj_capsule_ligament_map_ = std::move(other.tmj_capsule_ligament_map_);
    //         ligament_init_properties_ = std::move(other.ligament_init_properties_);
    //         tmj_capsule_ligament_init_properties_ = std::move(other.tmj_capsule_ligament_init_properties_);
    //         muscle_map_ = std::move(other.muscle_map_);
    //         muscle_init_properties_ = std::move(other.muscle_init_properties_);
    //
    //         tmj_disc_left_fea_aux_bodies_ = std::move(other.tmj_disc_left_fea_aux_bodies_);
    //         tmj_disc_right_fea_aux_bodies_ = std::move(other.tmj_disc_right_fea_aux_bodies_);
    //         tmj_disc_left_fea_aux_links_ = std::move(other.tmj_disc_left_fea_aux_links_);
    //         tmj_disc_right_fea_aux_links_ = std::move(other.tmj_disc_right_fea_aux_links_);
    //         tmj_disc_left_fea_ = std::move(other.tmj_disc_left_fea_);
    //         tmj_disc_right_fea_ = std::move(other.tmj_disc_right_fea_);
    //
    //         // Reset the source object
    //         other.sys_ = nullptr;
    //         other.skull_ = nullptr;
    //         other.maxilla_ = nullptr;
    //         other.mandible_ = nullptr;
    //         other.hyoid_ = nullptr;
    //         other.incisal_point_aux_body_ = nullptr;
    //         other.incisal_point_link_ = nullptr;
    //         other.tmj_constr_left_ = nullptr;
    //         other.tmj_constr_right_ = nullptr;
    //         other.tmj_disc_left_fea_ = nullptr;
    //         other.tmj_disc_right_fea_ = nullptr;
    //     }
    //
    //     return *this;
    // }

    void ChJaw::Build() {
        // Set the Chrono data directory path
        SetChronoDataPath(CHRONO_DATA_DIR);

        // -------------------------------------
        // Global collision model specifications
        // -------------------------------------

        // ChCollisionModel::SetDefaultSuggestedEnvelope(collision_envelope_);
        // ChCollisionModel::SetDefaultSuggestedMargin(collision_margin_);
        ChCollisionSystemBullet::SetContactBreakingThreshold(contact_breaking_threshold_bullet_);
        ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(default_effective_curvature_radius_smc_);

        // -----------------
        // Create the system
        // -----------------

        InitSystem();

        // --------------------
        // Initialize the model
        // --------------------

        InitModel();

        // ----------------------------------------------------------
        // Define object-object contact behavior for each object pair
        // ----------------------------------------------------------

        DefineObjectObjectContactBehavior();

        // -----------------------------------------------------------------------------------------------------
        // Assemble the system.
        // The assembling is performed by satisfying constraints at position, velocity, and acceleration levels.
        // -----------------------------------------------------------------------------------------------------

        const auto assembly_flag = sys_->DoAssembly(AssemblyAnalysis::Level::FULL, 10);
        utils::throw_logic_error(assembly_flag == AssemblyAnalysis::ExitFlag::NOT_CONVERGED,
                                 "[ERROR] [ChJaw] Impossible assembly?!\n");

        // Set initial mandible kinematics
        CaptureMandibleKinematics();

        // ----------------
        // Set reset state
        // ---------------

        reset_simulation_state_ = ChSimulationState{
            sys_->GetChTime(),
            ChState(sys_->GetNumCoordsPosLevel(), sys_.get()),
            ChStateDelta(sys_->GetNumCoordsVelLevel(), sys_.get())
        };
        sys_->StateGather(reset_simulation_state_.state, reset_simulation_state_.state_dt,
                          reset_simulation_state_.time);

        replay_buffer_.push(reset_simulation_state_);

        build_called_ = true;
    }

    void ChJaw::InitSystem() {
        switch (contact_method_) {
            case ChContactMethod::NSC: {
                auto sys_NSC = chrono_types::make_shared<ChSystemNSC>();
                sys_NSC->SetMinBounceSpeed(min_bounce_speed_nsc_);
                sys_ = sys_NSC;
                break;
            }
            case ChContactMethod::SMC: {
                auto sys_SMC = chrono_types::make_shared<ChSystemSMC>();
                // TODO: enums
                sys_SMC->SetContactForceModel(static_cast<ChSystemSMC::ContactForceModel>(contact_force_model_smc_));
                sys_SMC->SetAdhesionForceModel(static_cast<ChSystemSMC::AdhesionForceModel>(adhesion_force_model_smc_));
                sys_SMC->SetTangentialDisplacementModel(
                    static_cast<ChSystemSMC::TangentialDisplacementModel>(tangential_displ_model_smc_));
                sys_SMC->UseMaterialProperties(use_material_properties_smc_);
                // sys_SMC->SetContactStiff(true);

                // -----------------------------------------------------------------------------------
                // Set custom contact force algorithm for distinguished object-object contact behavior
                // -----------------------------------------------------------------------------------

                sys_SMC->SetContactForceTorqueAlgorithm(std::make_unique<ChContactForceTorqueJawSMC>(
                    sys_SMC.get(), contact_behavior_map_.get()));

                sys_ = sys_SMC;
                break;
            }
            default: {
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No valid contact model!\n");
            }
        }

        // -------------------------------
        // Collision system specifications
        // -------------------------------

        SetCollisionSystemType(collision_type_);

        // ---------------------------
        // Time stepper specifications
        // ---------------------------

        SetTimeStepperType(time_stepper_type_);

        // ---------------------
        // Solver specifications
        // ---------------------

        SetSolverType(solver_type_);

        // ----------------
        // General settings
        // ----------------

        // Set gravity vector
        sys_->SetGravitationalAcceleration(ChVector3d(0.0, -gravity_, 0.0));

        // Set number of threads
        sys_->SetNumThreads(static_cast<int>(threads_), 0, 0);

        // Debug
        if (enable_solver_debugging_) {
            const std::string debug_path{std::string(EXOSIM_RESOURCES_DIR) + "/debug/"};
            sys_->EnableSolverMatrixWrite(true, debug_path + "solver");
        }
    }

    void ChJaw::InitModel() {
        // ------------------------
        // Create contact materials
        // ------------------------

        DefineContactMaterials();

        // -------------------
        // Create rigid bodies
        // -------------------

        // skull_ = chrono_types::make_shared<ChBodyAuxRef>();
        // maxilla_ = chrono_types::make_shared<ChBodyAuxRef>();
        // mandible_ = chrono_types::make_shared<ChBodyAuxRef>();
        // hyoid_ = chrono_types::make_shared<ChBodyAuxRef>();

        // const auto skull = std::dynamic_pointer_cast<ChBodyAuxRef>(skull_);
        // const auto maxilla = std::dynamic_pointer_cast<ChBodyAuxRef>(maxilla_);
        // const auto mandible = std::dynamic_pointer_cast<ChBodyAuxRef>(mandible_);
        // const auto hyoid = std::dynamic_pointer_cast<ChBodyAuxRef>(hyoid_);
        //
        // utils::throw_runtime_error(!skull || !maxilla || !mandible || !hyoid,
        //                            "[ERROR] [ChJaw] Failed to cast to ChBodyAuxRef in 'InitModel'!\n");

        *skull_ = ChBodyAuxRef();
        *maxilla_ = ChBodyAuxRef();
        *mandible_ = ChBodyAuxRef();
        *hyoid_ = ChBodyAuxRef();

        skull_->SetName(SKULL_NAME_);
        maxilla_->SetName(MAXILLA_NAME_);
        mandible_->SetName(MANDIBLE_NAME_);
        hyoid_->SetName(HYOID_NAME_);

        bool skull_coll{true};
        std::string skull_obj_file_coll;
        std::string maxilla_obj_file_coll;
        std::string mandible_obj_file_coll;

        if (model_type_ == ChModelType::RIGID) {
            skull_coll = false;

            skull_obj_file_coll = skull_obj_file_rigid_coll_;
            maxilla_obj_file_coll = maxilla_obj_file_rigid_coll_;
            mandible_obj_file_coll = mandible_obj_file_rigid_coll_;
        } else {
            skull_obj_file_coll = skull_obj_file_coll_;
            maxilla_obj_file_coll = maxilla_obj_file_coll_;
            mandible_obj_file_coll = mandible_obj_file_coll_;
        }

        // Load collision meshes
        DefineBodyCollisionMeshes(skull_, bone_density_, bone_contact_material_, skull_obj_file_vis_,
                                  skull_obj_file_coll, skull_coll, true, false,
                                  bone_mesh_swept_sphere_thickness_, bone_color_);
        DefineBodyCollisionMeshes(maxilla_, bone_density_, bone_contact_material_, maxilla_obj_file_vis_,
                                  maxilla_obj_file_coll, true, true, false,
                                  bone_mesh_swept_sphere_thickness_, bone_color_);
        DefineBodyCollisionMeshes(mandible_, bone_density_, bone_contact_material_, mandible_obj_file_vis_,
                                  mandible_obj_file_coll, true, true, false,
                                  bone_mesh_swept_sphere_thickness_, bone_color_);
        DefineBodyCollisionMeshes(hyoid_, bone_density_, bone_contact_material_, hyoid_obj_file_vis_,
                                  "NONE", false, true, false,
                                  bone_mesh_swept_sphere_thickness_, bone_color_);

        /// Attention: This might invalidate the defined ligament and muscle attachments if not defined in local coord.!
        const ChVector3d mandible_pos_offset = {0.0, -0.000, -0.00}; // TODO: Remove offset
        mandible_->SetFrameRefToAbs(ChFrame(mandible_pos_offset));

        // Fix skull, maxilla and hyoid in place
        skull_->SetFixed(true);
        maxilla_->SetFixed(true);
        mandible_->SetFixed(false);
        hyoid_->SetFixed(true);

        // maxilla_->GetCollisionModel()->SetEnvelope(static_cast<float>(collision_envelope_));
        // mandible_->GetCollisionModel()->SetEnvelope(static_cast<float>(collision_envelope_));
        //
        // maxilla_->GetCollisionModel()->SetSafeMargin(static_cast<float>(collision_margin_));
        // mandible_->GetCollisionModel()->SetSafeMargin(static_cast<float>(collision_margin_));

        // Set collision families
        //        skull_->GetCollisionModel()->SetFamilyGroup(0b0);
        //        maxilla_->GetCollisionModel()->SetFamilyGroup(0b0);
        //        mandible_->GetCollisionModel()->SetFamilyGroup(0b1);
        //        maxilla_->GetCollisionModel()->SetFamilyMaskDoCollisionWithFamily(0b1);

        // Define incisal point
        incisal_point_aux_body_->SetName("incisal_point_aux_body");
        // incisal_point_aux_body_->SetPos(incisal_pos_abs_ + mandible_pos_offset);
        incisal_point_aux_body_->SetCoordsys({
            incisal_frame_abs_.GetPos(), incisal_frame_abs_.GetRot().GetNormalized()
        });
        incisal_point_link_->Initialize(incisal_point_aux_body_, mandible_);
        incisal_point_aux_body_->SetMass(0.0);
        incisal_point_aux_body_->SetInertiaXX({0, 0, 0});

        sys_->AddBody(incisal_point_aux_body_);
        sys_->AddLink(incisal_point_link_);

        // const auto T_mand_ref_abs = mandible_->GetFrameRefToAbs();
        // const auto T_in_abs = incisal_point_aux_body_->GetFrameRefToAbs();
        // const auto T_mand_ref_in = T_in_abs >> T_mand_ref_abs.GetInverse();
        // std::cout << "T_mand_ref_incisal:\n" << T_mand_ref_in << std::endl;

        // ------------------------
        // Add bodies to the system
        // ------------------------

        for (auto& body: {skull_, maxilla_, mandible_, hyoid_}) {
            sys_->AddBody(body);
        }

        // ----------------------------
        // Add FEA meshes to the system
        // ----------------------------

        if (model_type_ == ChModelType::FEM) {
            DefineFEAMeshes();
            InitFEAAuxiliaryBodies();

            for (auto& fea_mesh: {tmj_disc_left_fea_, tmj_disc_right_fea_}) {
                sys_->AddMesh(fea_mesh);
            }

            for (int i = 0; i < 4; i++) {
                sys_->AddBody(tmj_disc_left_fea_aux_bodies_[i]);
                sys_->AddBody(tmj_disc_right_fea_aux_bodies_[i]);
            }

            for (auto& constr: tmj_disc_left_fea_aux_links_) {
                sys_->AddLink(constr);
            }
            for (auto& constr: tmj_disc_right_fea_aux_links_) {
                sys_->AddLink(constr);
            }
        }

        // --------------
        // Attach muscles
        // --------------

        DefineMuscles(muscle_type_);

        for (auto& [fst, snd]: muscle_map_) {
            sys_->AddLink(snd);
        }

        // ----------------
        // Attach ligaments
        // ----------------

        DefineLigaments(ligament_type_);

        for (auto& [l_name, l]: ligament_map_) {
            sys_->AddLink(l);
        }

        if (model_type_ == ChModelType::FEM) {
            for (auto& [l_name, l]: tmj_capsule_ligament_map_) {
                sys_->AddLink(l);
            }
        }

        // --------------------------------------------------
        // Impose TMJ constraints (only for rigid-body model)
        // --------------------------------------------------

        if (model_type_ == ChModelType::RIGID) {
            DefineRigidTMJConstraints();

            for (auto& tmj_constraint: {tmj_constr_left_, tmj_constr_right_}) {
                sys_->AddLink(tmj_constraint);
            }
        }
    }

    void ChJaw::StartSimulation() {
        run_simulation_ = true;
        simulation_thread_ = std::thread(static_cast<void(ChJaw::*)()>(&ChJaw::Simulate), this);
    }

    void ChJaw::StopSimulation(bool wait) {
        run_simulation_ = wait;
        if (simulation_thread_.joinable()) {
            simulation_thread_.join();
        }
    }

    void ChJaw::ResetSimulation() {
        sys_->SetChTime(reset_simulation_state_.time);
        sys_->StateScatter(reset_simulation_state_.state, reset_simulation_state_.state_dt,
                           reset_simulation_state_.time, true);

        CaptureMandibleKinematics();

        replay_buffer_.clear();
        replay_buffer_.push(reset_simulation_state_);
    }

    void ChJaw::ResetSimulation(const ChSimulationState& reset_state) {
        sys_->SetChTime(reset_state.time);
        sys_->StateScatter(reset_state.state, reset_state.state_dt, reset_state.time, true);

        CaptureMandibleKinematics();

        replay_buffer_.clear();
        replay_buffer_.push(reset_state);
    }

    void ChJaw::RewindSimulation(double time_percentage) {
        utils::throw_invalid_argument(time_percentage < 0.0 || time_percentage > 100.0,
                                      "[ERROR] [ChJaw] Invalid time percentage!\n");

        if (replay_buffer_.empty()) {
            fmt::print(utils::WARNING_MSG, "[WARNING] [ChJaw] Replay buffer is empty!\n");
            return;
        }

        const auto index = static_cast<unsigned int>(std::round(
            static_cast<double>(replay_buffer_.size() - 1) * (100.0 - time_percentage) * 1e-2));

        const auto& [time, state, state_dt] = replay_buffer_[index];
        if (update_simulation_) replay_buffer_.move_to_index(index);

        // Reset the system to the state at the specified time
        if (time != sys_->GetChTime()) {
            sys_->SetChTime(time);
            sys_->StateScatter(state, state_dt, time, true);
        }
    }

    void ChJaw::CaptureSimulationState(ChSimulationState& state) {
        state.time = sys_->GetChTime();
        state.state = ChState(sys_->GetNumCoordsPosLevel(), sys_.get());
        state.state_dt = ChStateDelta(sys_->GetNumCoordsVelLevel(), sys_.get());

        sys_->StateGather(state.state, state.state_dt, state.time);
    }

    void ChJaw::LoadFromJSON(const std::string& filename) {
        utils::ReadJawJSON(*this, filename);
        build_called_ = false;
    }

    void ChJaw::WriteToJSON(const std::string& filename) {
        utils::WriteJawJSON(*this, filename);
    }

    void ChJaw::SetCollisionSystemType(ChCollisionSystem::Type type) {
        sys_->SetCollisionSystemType(type);
        sys_->SetMaxPenetrationRecoverySpeed(contact_recovery_speed_nsc_);

        auto coll_sys = sys_->GetCollisionSystem();

        switch (type) {
#ifdef CHRONO_COLLISION
            case ChCollisionSystem::Type::MULTICORE: {
                const auto coll_chrono = std::static_pointer_cast<ChCollisionSystemMulticore>(coll_sys);
                coll_chrono->SetEnvelope(collision_envelope_);
                coll_chrono->SetNarrowphaseAlgorithm(narrowphase_algorithm_multicore_);
                coll_chrono->SetBroadphaseGridResolution(broadphase_grid_resolution_multicore_);
                // coll_chrono->EnableActiveBoundingBox(ChVector3d(-0.1, -0.1, -0.1), ChVector3d(+0.1, +0.1, +0.1));
                // Set number of threads used by the collision detection system
                coll_chrono->SetNumThreads(static_cast<int>(threads_));

                collision_type_ = ChCollisionSystem::Type::MULTICORE;
                break;
            }
#else
            case ChCollisionSystem::Type::MULTICORE: {
                std::cout << fmt::format(utils::WARNING_MSG, "[WARNING] [ChJaw] Chrono::Multicore not available! "
                                         "Falling back to Bullet...\n");
                // Fall through to BULLET case
            }
#endif
            case ChCollisionSystem::Type::BULLET: {
                auto coll_bullet = std::static_pointer_cast<ChCollisionSystemBullet>(coll_sys);
                coll_bullet->SetNumThreads(static_cast<int>(threads_));

                collision_type_ = ChCollisionSystem::Type::BULLET;
                break;
            }
            default: {
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No supported collision system type!\n");
            }
        }
    }

    void ChJaw::SetTimeStepperType(ChTimestepper::Type type) {
        switch (type) {
            case ChTimestepper::Type::HHT: {
                sys_->SetTimestepperType(type);
                const auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys_->GetTimestepper());
                hht->SetAlpha(hht_alpha_);
                hht->SetMaxIters(hht_max_iters_);
                hht->SetStepControl(hht_step_control_);
                break;
            }
            case ChTimestepper::Type::NEWMARK: {
                sys_->SetTimestepperType(type);
                const auto newmark = std::dynamic_pointer_cast<ChTimestepperNewmark>(sys_->GetTimestepper());
                newmark->SetMaxIters(newmark_max_iters_);
                break;
            }
            case ChTimestepper::Type::EULER_IMPLICIT: {
                sys_->SetTimestepperType(type);
                const auto e_i = std::dynamic_pointer_cast<ChTimestepperEulerImplicit>(sys_->GetTimestepper());
                e_i->SetMaxIters(euler_i_max_iters_);
                break;
            }
            case ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED: {
                sys_->SetTimestepperType(type);
                auto e_i_l = std::dynamic_pointer_cast<ChTimestepperEulerImplicitLinearized>(
                    sys_->GetTimestepper());
                break;
            }
            case ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED: {
                sys_->SetTimestepperType(type);
                const auto t_l = std::dynamic_pointer_cast<ChTimestepperTrapezoidalLinearized>(
                    sys_->GetTimestepper());
                t_l->SetMaxIters(trapezoidal_lin_max_iters_);
                break;
            }
            default: {
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No supported time stepper type!\n");
            }
        }

        sys_->GetTimestepper()->SetVerbose(time_stepper_verbose_);
        time_stepper_type_ = type;
    }

    void ChJaw::SetSolverType(ChSolver::Type type) {
        switch (type) {
            case ChSolver::Type::ADMM: {
#ifdef CHRONO_PARDISO_MKL
                const auto mkl = chrono_types::make_shared<ChSolverPardisoMKL>(threads_);
                mkl->LockSparsityPattern(mkl_lock_sparsity_pattern_);
                mkl->UseSparsityPatternLearner(mkl_use_sparsity_pattern_learner_);
                mkl->SetVerbose(solver_verbose_);
                const auto admm = chrono_types::make_shared<ChSolverADMM>(mkl);
#else
                const auto admm = chrono_types::make_shared<ChSolverADMM>();
#endif
                admm->SetStepAdjustPolicy(admm_step_policy_);
                admm->SetAcceleration(admm_acceleration_);
                admm->SetMaxIterations(max_solver_iterations_);
                admm->SetTolerance(solver_tolerance_);
                admm->SetToleranceDual(admm_tolerance_dual_);
                admm->SetTolerancePrimal(admm_tolerance_primal_);
                admm->SetRho(admm_rho_);
                admm->SetVerbose(solver_verbose_);

                if (time_stepper_type_ == ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED) {
                    admm->EnableWarmStart(admm_warm_start_);
                }

                sys_->SetSolver(admm);
                break;
            }
            case ChSolver::Type::APGD: {
                const auto apgd = chrono_types::make_shared<ChSolverAPGD>();
                apgd->SetMaxIterations(max_solver_iterations_);
                apgd->SetTolerance(solver_tolerance_);
                apgd->SetVerbose(solver_verbose_);
                sys_->SetSolver(apgd);
                break;
            }
            case ChSolver::Type::BARZILAIBORWEIN: {
                const auto bb = chrono_types::make_shared<ChSolverBB>();
                bb->SetMaxIterations(max_solver_iterations_);
                bb->SetTolerance(solver_tolerance_);
                bb->SetVerbose(solver_verbose_);
                sys_->SetSolver(bb);
                break;
            }
            case ChSolver::Type::PSOR: {
                const auto psor = chrono_types::make_shared<ChSolverPSOR>();
                psor->SetMaxIterations(max_solver_iterations_);
                psor->SetTolerance(solver_tolerance_);
                psor->SetVerbose(solver_verbose_);
                sys_->SetSolver(psor);
                break;
            }
            case ChSolver::Type::MINRES: {
                const auto minres = chrono_types::make_shared<ChSolverMINRES>();
                minres->SetMaxIterations(max_solver_iterations_);
                minres->SetVerbose(solver_verbose_);
                sys_->SetSolver(minres);
                break;
            }
#ifdef CHRONO_PARDISO_MKL
            case ChSolver::Type::PARDISO_MKL: {
                const auto mkl = chrono_types::make_shared<ChSolverPardisoMKL>(threads_);
                mkl->LockSparsityPattern(mkl_lock_sparsity_pattern_);
                mkl->UseSparsityPatternLearner(mkl_use_sparsity_pattern_learner_);
                mkl->SetVerbose(solver_verbose_);
                sys_->SetSolver(mkl);
                break;
            }
#endif
            default: {
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No supported solver type!\n");
            }
        }

        solver_type_ = type;
    }

    void ChJaw::SetMandibleState(const ChFrameMoving<>& state_abs, const ChFrameMoving<>& ref_frame_local) {
        ChFrameMoving<> mandible_ref_abs_new{state_abs};

        // Get the new pose of the mandible's reference frame in the absolute frame if a local reference frame is
        // provided
        if (ref_frame_local != ChFrameMoving()) {
            mandible_ref_abs_new = ref_frame_local.GetInverse() >> state_abs;
        }

        std::cout << "state_abs:\n" << state_abs << std::endl;
        std::cout << "ref_frame_local:\n" << ref_frame_local << std::endl;
        std::cout << "Mandible ref abs:\n" << mandible_->GetFrameRefToAbs() << std::endl;
        std::cout << "Mandible ref abs new:\n" << mandible_ref_abs_new << std::endl;

        mandible_->SetFrameRefToAbs(mandible_ref_abs_new);
        // const_cast<ChCoordsysd&>(mandible_->GetFrameRefToAbs().GetCoordsysDt()) = mandible_ref_abs_new.GetCoordsysDt();
        // const_cast<ChCoordsysd&>(mandible_->GetFrameRefToAbs().GetCoordsysDt2()) = mandible_ref_abs_new.
        //         GetCoordsysDt2();
        // mandible_->SetCoordsysDt(state.GetCoordsysDt());
        // mandible_->SetCoordsysDt2(state.GetCoordsysDt2());

        std::cout << "Mandible ref abs after:\n" << mandible_->GetFrameRefToAbs() << std::endl;

        if (const auto assembly_flag = sys_->DoAssembly(AssemblyAnalysis::Level::FULL, 20);
            assembly_flag == AssemblyAnalysis::ExitFlag::NOT_CONVERGED) {
            ResetSimulation();
            utils::throw_runtime_error(assembly_flag == AssemblyAnalysis::ExitFlag::NOT_CONVERGED,
                                       "[ERROR] [ChJaw] Mandible state not feasible ('SetMandibleState')!\n");
        }
    }

    //    void ChJaw::EnableThreadTuning(uint8_t min_threads, uint8_t max_threads) {
    //
    //        threads_ = max_threads;
    //        sys_->SetNumThreads(threads_, threads_, threads_);
    //        sys_->EnableThreadTuning(min_threads, max_threads);
    //    }

    void ChJaw::Simulate(double end_time) {
        ChRealtimeStepTimer rt;

        utils::throw_logic_error(!build_called_, "[ERROR] [ChJaw] 'Build' must be called before 'Simulate'!\n");

        const auto assembly_flag = sys_->DoAssembly(AssemblyAnalysis::Level::FULL, 10);
        utils::throw_logic_error(assembly_flag == AssemblyAnalysis::ExitFlag::NOT_CONVERGED,
                                 "[ERROR] [ChJaw] Impossible assembly?!\n");

        const double simulation_start_time = sys_->GetChTime();

        std::function<void(double, double, double, bool&)> stop_simulation_fct;

        if (end_time < 0.0) {
            stop_simulation_fct = [](double current_t, double end_t, double start_t, bool& run_sim) -> void {
            };
        } else {
            stop_simulation_fct = [](double current_t, double end_t, double start_t, bool& run_sim) -> void {
                if (const auto elapsed_time = current_t - start_t;
                    elapsed_time >= end_t) {
                    fmt::print(utils::INFO_MSG, "[INFO] [ChJaw] Time limit ({}s) exceeded!\n", end_t);
                    run_sim = false;
                }
            };
        }

        switch (vis_mode_) {
            case ChVisualizationMode::OPENGL: {
#ifdef CHRONO_OPENGL
                opengl::ChVisualSystemOpenGL vis;
                vis.AttachSystem(sys_.get());
                vis.SetWindowTitle(window_title_);
                vis.SetWindowSize(window_width_, window_height_);

                switch (render_mode_) {
                    case ChRenderMode::SOLID: {
                        vis.SetRenderMode(opengl::SOLID);
                        break;
                    }
                    case ChRenderMode::WIREFRAME: {
                        vis.SetRenderMode(opengl::WIREFRAME);
                        break;
                    }
                    case ChRenderMode::POINTS: {
                        vis.SetRenderMode(opengl::POINTS);
                        break;
                    }
                }

                vis.Initialize();
                vis.SetCameraPosition(ChVector3d(0.5, -0.05, 0.5));
                vis.SetCameraVertical(CameraVerticalDir::Y);

                // Simulation loop
                while (vis.Run() && run_simulation_) {
                    vis.Render();

                    // CaptureMandibleKinematics();

                    stop_simulation_fct(sys_->GetChTime(), end_time, simulation_start_time, run_simulation_);

                    sys_->DoStepDynamics(time_step_);

                    // rt.Spin(time_step_);
                }

                break;
#else
                std::cout << fmt::format(utils::WARNING_MSG, "[WARNING] [ChJaw] OpenGL not available! "
                                                      "Falling back to Irrlicht...\n");
#endif
            }
            case ChVisualizationMode::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                // Create the Irrlicht visualization system
                irrlicht::ChVisualSystemIrrlicht vis;
                vis.AttachSystem(sys_.get());
                vis.SetWindowSize(window_width_, window_height_);
                vis.SetWindowTitle(window_title_);
                vis.Initialize();
                vis.ShowInfoPanel(true);
                vis.ShowExplorer(false);
                vis.ShowProfiler(false);
                vis.SetCameraVertical(CameraVerticalDir::Y);

                // vis.EnableBodyFrameDrawing(true);
                // vis.AddLogo();
                // vis.AddSkyBox();
                vis.AddCamera(ChVector3d(0.5, -0.05, 0.5));
                // vis.AddTypicalLights();

                double r_point_light{0.75};
                ChColor color_point_light{0.7f, 0.7f, 0.7f};
                vis.AddLight(ChVector3d(0.5, 0.0, 0.0), r_point_light, color_point_light);
                vis.AddLight(ChVector3d(-0.5, 0.0, 0.0), r_point_light, color_point_light);
                vis.AddLight(ChVector3d(0, 0.0, 0.5), r_point_light, color_point_light);
                vis.AddLight(ChVector3d(0.0, 0.0, -0.5), r_point_light, color_point_light);
                vis.AddLight(ChVector3d(0.0, 0.6, 0.0), r_point_light, color_point_light);
                // vis.AddLight(ChVector3d<>(0.0, -0.5, 0.0), r_point_light, color_point_light);
                // vis.EnableCollisionShapeDrawing(true); // Leads to segmentation fault with ChCollisionSystemType::CHRONO!

                vis.SetSymbolScale(2.5e-2);
                vis.EnableContactDrawing(irrlicht::ContactsDrawMode::CONTACT_FORCES);
                // vis.SetSymbolScale(1);
                vis.EnableContactDrawing(irrlicht::ContactsDrawMode::CONTACT_DISTANCES);
                //
                // irrlicht::tools::drawAllContactLabels(&(*vis));
                // irrlicht::tools::drawAllBoundingBoxes(&(*vis));
                // irrlicht::tools::drawPlot3D();

                // long long int image_counter{0};

                const std::string debug_path{std::string(EXOSIM_RESOURCES_DIR) + "/debug/"};

                // Simulation loop
                while (vis.Run() && run_simulation_) {
                    vis.BeginScene(true, true, background_color_);
                    vis.Render();

                    // CaptureMandibleKinematics();

                    stop_simulation_fct(sys_->GetChTime(), end_time, simulation_start_time, run_simulation_);

                    // Get system matrix for debugging
                    // sys_->GetSystemDescriptor()->BuildSystemMatrix(...);
                    // sys_->GetSystemDescriptor()->WriteMatrix(...);

                    // for (auto& m_prop: muscle_init_properties_) {
                    //     GetLog() << m_prop.name << ": " << muscle_map_.at(m_prop.name)->GetLength() << "\n";
                    // }
                    // GetLog() << "------------------\n";

                    if (enable_system_debugging_) {
                        sys_->WriteSystemMatrices(true, true, true, true, debug_path + "system/", true);
                    }

                    vis.EndScene();

                    // vis.WriteImageToFile(
                    //     std::string(EXOSIM_RESOURCES_DIR) + "/sim_images/img_" + std::to_string(
                    //         image_counter) + ".png");
                    // image_counter++;

                    // fmt::print(INFO_MSG, "mandible envelope: {}\n", mandible_->GetCollisionModel()->GetEnvelope());
                    // fmt::print(INFO_MSG, "mandible margin: {}\n", mandible_->GetCollisionModel()->GetSafeMargin());

                    sys_->DoStepDynamics(time_step_);
                    // rt.Spin(time_step_);
                }

                break;
#else
                std::cout << fmt::format(utils::WARNING_MSG, "[WARNING] [ChJaw] Irrlicht not available! "
                                                      "Falling back to VSG...\n");
#endif
            }
            case ChVisualizationMode::VSG: {
#ifdef CHRONO_VSG
                vsg3d::ChVisualJawSystemVSG vis(48);
                vis.SetCameraVertical(CameraVerticalDir::Y);
                vis.AttachSystem(sys_.get());
                vis.SetWindowSize(static_cast<int>(window_width_), static_cast<int>(window_height_));
                vis.SetWindowTitle(window_title_);
                vis.SetUseSkyBox(false);
                vis.SetLightIntensity(1.0f);
                vis.SetLightDirection(CH_PI_4, 0.0);
                vis.AddCamera(ChVector3d(0.5, -0.05, 0.5));
                vis.SetWireFrameMode(false);
                vis.SetClearColor(background_color_);
                // vis.AddGuiColorbar("Test Colorbar", 0.0, 10.0);

                const auto rebuild_button = std::make_shared<vsg3d::ChResetJawModelVSG>(this, &vis);
                const auto start_stop_gui = std::make_shared<vsg3d::ChStartStopJawSimVSG>(&vis, run_sim_at_startup_);
                const auto rewind_sim = std::make_shared<vsg3d::ChRewindJawSimVSG>(this, &vis);
                const auto toggle_comp_vis = std::make_shared<vsg3d::ChToggleJawVisibilityVSG>(&vis);
                const auto muscle_plot = std::make_shared<vsg3d::ChJawMuscleExcActVSG>(this, &vis);
                vis.AddGuiComponent(rebuild_button);
                vis.AddGuiComponent(start_stop_gui);
                vis.AddGuiComponent(rewind_sim);
                vis.AddGuiComponent(toggle_comp_vis);
                vis.AddGuiComponent(muscle_plot);

                // vis.AddEventHandler();
                vis.SetVerbose(true);
                vis.Initialize();

                ChSimulationState current_sim_state;

                // Simulation loop
                while (vis.Run() && run_simulation_) {
                    vis.Render();

                    if ((update_simulation_ = start_stop_gui->UpdateSimulation())) {
                        // CaptureMandibleKinematics();
                        stop_simulation_fct(sys_->GetChTime(), end_time, simulation_start_time, run_simulation_);
                        sys_->DoStepDynamics(time_step_);

                        // Add current state to the replay buffer
                        CaptureSimulationState(current_sim_state);
                        replay_buffer_.push(current_sim_state);
                    }

                    // rt.Spin(time_step_);
                }

                break;
#else
                std::cout << fmt::format(utils::WARNING_MSG, "[WARNING] [ChJaw] VSG not available! "
                                                      "Falling back to no visualization...\n");
#endif
            }
            case ChVisualizationMode::NONE: {
                while (run_simulation_) {
                    // CaptureMandibleKinematics();
                    stop_simulation_fct(sys_->GetChTime(), end_time, simulation_start_time, run_simulation_);

                    sys_->DoStepDynamics(time_step_);

                    // rt.Spin(time_step_);
                }
                break;
            }
            default: {
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No valid visualization mode!\n");
            }
        }
    }

    void ChJaw::DefineContactMaterials() {
        switch (contact_method_) {
            case ChContactMethod::NSC: {
                bone_contact_material_ = chrono_types::make_shared<ChContactMaterialNSC>();
                tmj_disc_contact_material_ = chrono_types::make_shared<ChContactMaterialNSC>();

                for (auto& [mat, props]: {
                         std::tuple{bone_contact_material_, bone_properties_nsc_},
                         std::tuple{tmj_disc_contact_material_, tmj_disc_properties_nsc_}
                     }) {
                    const auto mat_NSC = std::static_pointer_cast<ChContactMaterialNSC>(mat);
                    mat_NSC->SetFriction(props.friction);
                    mat_NSC->SetRestitution(props.restitution);
                    mat_NSC->SetDampingF(props.damping);
                    mat_NSC->SetCohesion(props.cohesion);
                    mat_NSC->SetCompliance(props.compliance);
                    mat_NSC->SetComplianceT(props.compliance_t);
                    mat_NSC->SetComplianceRolling(props.compliance_rolling);
                    mat_NSC->SetComplianceSpinning(props.compliance_spinning);
                }
                break;
            }
            case ChContactMethod::SMC: {
                bone_contact_material_ = chrono_types::make_shared<ChContactMaterialSMC>();
                tmj_disc_contact_material_ = chrono_types::make_shared<ChContactMaterialSMC>();

                for (auto& [mat, props]: {
                         std::tuple{bone_contact_material_, bone_properties_smc_},
                         std::tuple{tmj_disc_contact_material_, tmj_disc_properties_smc_}
                     }) {
                    const auto mat_SMC = std::static_pointer_cast<ChContactMaterialSMC>(mat);
                    mat_SMC->SetFriction(props.friction);
                    mat_SMC->SetRestitution(props.restitution);
                    mat_SMC->SetYoungModulus(props.young_modulus);
                    mat_SMC->SetPoissonRatio(props.poisson_ratio);
                    mat_SMC->SetAdhesion(props.adhesion);
                    mat_SMC->SetAdhesionSPerko(props.adhesion_s_perko);
                    mat_SMC->SetAdhesionMultDMT(props.adhesion_mult_dmt);
                    mat_SMC->SetKn(props.k_n);
                    mat_SMC->SetGn(props.g_n);
                    mat_SMC->SetKt(props.k_t);
                    mat_SMC->SetGt(props.g_t);
                }
                break;
            }
            default:
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No valid contact model!\n");
        }
    }

    void ChJaw::DefineFEAMeshes() {
        tmj_disc_left_fea_ = chrono_types::make_shared<fea::ChMesh>();
        tmj_disc_right_fea_ = chrono_types::make_shared<fea::ChMesh>();
        const std::list<std::shared_ptr<fea::ChMesh> > mesh_list{tmj_disc_left_fea_, tmj_disc_right_fea_};

        // FEA material properties
        tmj_disc_fea_material_ = chrono_types::make_shared<fea::ChContinuumElastic>();
        const auto fea_material_elastic = std::static_pointer_cast<fea::ChContinuumElastic>(tmj_disc_fea_material_);
        fea_material_elastic->SetDensity(tmj_disc_fea_density_);
        fea_material_elastic->SetYoungModulus(tmj_disc_fea_young_modulus_);
        fea_material_elastic->SetPoissonRatio(tmj_disc_fea_poisson_ratio_);
        fea_material_elastic->SetRayleighDampingAlpha(tmj_disc_fea_rayleigh_damping_alpha_);
        fea_material_elastic->SetRayleighDampingBeta(tmj_disc_fea_rayleigh_damping_beta_);

        try {
            // fea::ChMeshFileLoader::FromTetGenFile(tmj_disc_left_fea_, TMJ_DISC_LEFT_NODE_FILE_FEA_.c_str(),
            //                                       TMJ_DISC_LEFT_ELE_FILE_FEA_.c_str(), tmj_disc_fea_material_, VNULL,
            //                                       QUNIT);
            // fea::ChMeshFileLoader::FromTetGenFile(tmj_disc_right_fea_, TMJ_DISC_LEFT_NODE_FILE_FEA_.c_str(),
            //                                       TMJ_DISC_LEFT_ELE_FILE_FEA_.c_str(), tmj_disc_fea_material_, VNULL,
            //                                       QUNIT);

            /// Attention: If 'pos_transform' or 'rot_transform' are non-zero, the defined TMJ capsule ligament
            /// attachement positions might be invalidated, since they're given in absolute coordinates!
            fea::ChGmshMeshFileLoader::FromGmshFile(tmj_disc_left_fea_, tmj_disc_fea_element_type_,
                                                    tmj_disc_fea_implementation_,
                                                    tmj_disc_left_msh_file_fea_, tmj_disc_fea_material_,
                                                    tmj_disc_fea_mr_c_1_, tmj_disc_fea_mr_c_2_,
                                                    ChVector3d(0.0, 0.0, 0.0),
                                                    QUNIT * 1);
            fea::ChGmshMeshFileLoader::FromGmshFile(tmj_disc_right_fea_, tmj_disc_fea_element_type_,
                                                    tmj_disc_fea_implementation_,
                                                    tmj_disc_right_msh_file_fea_, tmj_disc_fea_material_,
                                                    tmj_disc_fea_mr_c_1_, tmj_disc_fea_mr_c_2_,
                                                    ChVector3d(0.0, 0.0, 0.0),
                                                    QUNIT * 1);
        } catch (std::invalid_argument& myerr) {
            std::cerr << fmt::format(utils::ERROR_MSG, myerr.what());
            return;
        } catch (std::runtime_error& myerr) {
            std::cerr << fmt::format(utils::ERROR_MSG, myerr.what());
            return;
        }

        // Names
        tmj_disc_left_fea_->SetName(TMJ_DISC_LEFT_NAME_);
        tmj_disc_right_fea_->SetName(TMJ_DISC_RIGHT_NAME_);

        ChCollisionModel::SetDefaultSuggestedEnvelope(collision_envelope_);
        ChCollisionModel::SetDefaultSuggestedMargin(collision_margin_);

        // Collision
        for (const auto& mesh: mesh_list) {
            const auto contact_surface = chrono_types::make_shared<fea::ChContactSurfaceMesh>(
                tmj_disc_contact_material_);
            contact_surface->AddFacesFromBoundary(*mesh, tmj_disc_mesh_swept_sphere_thickness_);
            mesh->AddContactSurface(contact_surface);
        }

        // Visualization
        if (vis_mode_ != ChVisualizationMode::NONE) {
            for (const auto& mesh: mesh_list) {
                const auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>();
                vis_mesh->SetFEMdataType(fea_vis_data_type_);
                vis_mesh->SetColorscaleMinMax(color_scale_min_max_.first, color_scale_min_max_.second);
                vis_mesh->SetSmoothFaces(fea_smooth_faces_);
                vis_mesh->SetWireframe(fea_wireframe_);
                // vis_mesh->SetDefaultMeshColor(ChColor(0.2, 0.2, 0.2));
                mesh->AddVisualShapeFEA(vis_mesh);
            }
        }
    }

    std::vector<std::shared_ptr<fea::ChNodeFEAxyz> > ChJaw::FindNodesWithinRadius(fea::ChMesh& mesh,
        const std::shared_ptr<fea::ChNodeFEAxyz>& center_node,
        double radius) {
        std::vector<std::shared_ptr<fea::ChNodeFEAxyz> > nodes_within_radius{};

        for (auto& node: mesh.GetNodes()) {
            if (auto feaNode = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(node);
                feaNode && (feaNode->GetPos() - center_node->GetPos()).Length() <= radius) {
                nodes_within_radius.push_back(feaNode);
            }
        }

        return nodes_within_radius;
    }

    std::shared_ptr<fea::ChNodeFEAxyz> ChJaw::FindClosestNode(fea::ChMesh& mesh, const ChVector3d& position) {
        double min_distance = std::numeric_limits<double>::max();
        std::shared_ptr<fea::ChNodeFEAxyz> closest_node;

        for (auto& node: mesh.GetNodes()) {
            if (const auto fea_node = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(node)) {
                if (const double distance = (fea_node->GetX0() - position).Length(); distance < min_distance) {
                    min_distance = distance;
                    closest_node = fea_node;
                }
            }
        }

        return closest_node;
    }

    std::vector<ChJaw::AttachNode<fea::ChNodeFEAxyz> > ChJaw::FindMiddleNodes(fea::ChMesh& mesh, float radius) {
        using NodeType = fea::ChNodeFEAxyz;

        // Initialize min and max values for each dimension
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();

        // Iterate over all nodes to find the min and max values for each dimension
        for (auto& node: mesh.GetNodes()) {
            if (const auto fea_node = std::dynamic_pointer_cast<NodeType>(node)) {
                const auto& pos = fea_node->GetPos();
                min_x = std::min(min_x, pos.x());
                max_x = std::max(max_x, pos.x());
                min_y = std::min(min_y, pos.y());
                max_y = std::max(max_y, pos.y());
                min_z = std::min(min_z, pos.z());
                max_z = std::max(max_z, pos.z());
            }
        }

        // Calculate the middle points for each dimension
        const double mid_x = (min_x + max_x) / 2.0;
        const double mid_y = (min_y + max_y) / 2.0;
        const double mid_z = (min_z + max_z) / 2.0;

        double min_dist_x1 = std::numeric_limits<double>::max();
        double min_dist_x2 = std::numeric_limits<double>::max();
        double min_dist_y1 = std::numeric_limits<double>::max();
        double min_dist_y2 = std::numeric_limits<double>::max();
        double min_dist_z1 = std::numeric_limits<double>::max();
        double min_dist_z2 = std::numeric_limits<double>::max();

        AttachNode<NodeType> mid_node_x_1;
        AttachNode<NodeType> mid_node_x_2;
        AttachNode<NodeType> mid_node_y_1;
        AttachNode<NodeType> mid_node_y_2;
        AttachNode<NodeType> mid_node_z_1;
        AttachNode<NodeType> mid_node_z_2;

        // Helper lambda to update the minimum distance and node
        auto update_min_dist = [](double& min_dist, const double& dist, const std::shared_ptr<NodeType>& node,
                                  std::shared_ptr<NodeType>& closest_node) -> void {
            if (dist < min_dist) {
                min_dist = dist;
                closest_node = node;
            }
        };

        // Iterate over all nodes to find the nodes closest to the extrema and middle points for each dimension
        for (auto& node: mesh.GetNodes()) {
            if (auto fea_node = std::dynamic_pointer_cast<NodeType>(node)) {
                const auto& pos = fea_node->GetPos();

                // Check for X dimension extrema
                if (std::abs(pos.y() - mid_y) <= radius && std::abs(pos.z() - mid_z) <= radius) {
                    double dist_x1 = std::abs(pos.x() - min_x);
                    double dist_x2 = std::abs(pos.x() - max_x);

                    update_min_dist(min_dist_x1, dist_x1, fea_node, mid_node_x_1.center_node);
                    update_min_dist(min_dist_x2, dist_x2, fea_node, mid_node_x_2.center_node);
                }

                // Check for Y dimension extrema
                if (std::abs(pos.x() - mid_x) <= radius && std::abs(pos.z() - mid_z) <= radius) {
                    double dist_y1 = std::abs(pos.y() - min_y);
                    double dist_y2 = std::abs(pos.y() - max_y);

                    update_min_dist(min_dist_y1, dist_y1, fea_node, mid_node_y_1.center_node);
                    update_min_dist(min_dist_y2, dist_y2, fea_node, mid_node_y_2.center_node);
                }

                // Check for Z dimension extrema
                if (std::abs(pos.x() - mid_x) <= radius && std::abs(pos.y() - mid_y) <= radius) {
                    double dist_z1 = std::abs(pos.z() - min_z);
                    double dist_z2 = std::abs(pos.z() - max_z);

                    update_min_dist(min_dist_z1, dist_z1, fea_node, mid_node_z_1.center_node);
                    update_min_dist(min_dist_z2, dist_z2, fea_node, mid_node_z_2.center_node);
                }
            }
        }

        // Find nodes within the specified radius around the middle nodes
        mid_node_x_1.nodes_around = FindNodesWithinRadius(mesh, mid_node_x_1.center_node, radius);
        mid_node_x_2.nodes_around = FindNodesWithinRadius(mesh, mid_node_x_2.center_node, radius);
        mid_node_y_1.nodes_around = FindNodesWithinRadius(mesh, mid_node_y_1.center_node, radius);
        mid_node_y_2.nodes_around = FindNodesWithinRadius(mesh, mid_node_y_2.center_node, radius);
        mid_node_z_1.nodes_around = FindNodesWithinRadius(mesh, mid_node_z_1.center_node, radius);
        mid_node_z_2.nodes_around = FindNodesWithinRadius(mesh, mid_node_z_2.center_node, radius);

        return {mid_node_x_1, mid_node_x_2, mid_node_y_1, mid_node_y_2, mid_node_z_1, mid_node_z_2};
    }

    void ChJaw::InitFEAAuxiliaryBodies() {
        assert(tmj_disc_left_fea_);
        assert(tmj_disc_right_fea_);

        using NodeType = fea::ChNodeFEAxyz;

        const long unsigned int num_aux_bodies{tmj_disc_left_fea_aux_bodies_.size()};

        const auto& left_aux = tmj_disc_left_fea_aux_bodies_;
        const auto& right_aux = tmj_disc_right_fea_aux_bodies_;

        // Initialize auxiliary bodies for the TMJ discs
        for (int i = 0; i < num_aux_bodies; i++) {
            *left_aux[i] = ChBody();
            *right_aux[i] = ChBody();

            left_aux[i]->SetName(TMJ_DISC_LEFT_FEA_AUX_PREFIX_ + std::to_string(i));
            right_aux[i]->SetName(TMJ_DISC_RIGHT_FEA_AUX_PREFIX_ + std::to_string(i));

            left_aux[i]->SetMass(0);
            right_aux[i]->SetMass(0);
            left_aux[i]->SetInertiaXX({0, 0, 0});
            right_aux[i]->SetInertiaXX({0, 0, 0});
        }

        // TODO: Either use 'FindMiddleNodes' or predefined positions in 'tmj_capsule_ligament_init_properties_'
        // auto tmj_disc_left_attach_nodes = FindMiddleNodes(*tmj_disc_left_fea_, tmj_disc_fea_aux_body_radius_);
        // auto tmj_disc_right_attach_nodes = FindMiddleNodes(*tmj_disc_right_fea_, tmj_disc_fea_aux_body_radius_);
        //
        // tmj_disc_left_attach_nodes.erase(std::next(tmj_disc_left_attach_nodes.begin(), 2),
        //                                  std::next(tmj_disc_left_attach_nodes.begin(), 4));
        // tmj_disc_right_attach_nodes.erase(std::next(tmj_disc_right_attach_nodes.begin(), 2),
        //                                   std::next(tmj_disc_right_attach_nodes.begin(), 4));
        //
        // tmj_disc_left_attach_nodes[0].aux_body = left_aux[3];   // Medial
        // tmj_disc_left_attach_nodes[1].aux_body = left_aux[2];   // Lateral
        // tmj_disc_left_attach_nodes[2].aux_body = left_aux[1];   // Posterior
        // tmj_disc_left_attach_nodes[3].aux_body = left_aux[0];   // Anterior
        //
        // tmj_disc_right_attach_nodes[0].aux_body = right_aux[3];  // Medial
        // tmj_disc_right_attach_nodes[1].aux_body = right_aux[2];  // Lateral
        // tmj_disc_right_attach_nodes[2].aux_body = right_aux[1];  // Posterior
        // tmj_disc_right_attach_nodes[3].aux_body = right_aux[0];  // Anterior

        // Use predefined positions for the attachment points
        std::vector<AttachNode<NodeType> > tmj_disc_left_attach_nodes;
        std::vector<AttachNode<NodeType> > tmj_disc_right_attach_nodes;

        // Anterior, posterior, lateral, medial
        // Left
        for (const auto& aux_body: left_aux) {
            const auto lig_prop_it = std::find_if(tmj_capsule_ligament_init_properties_.begin(),
                                                  tmj_capsule_ligament_init_properties_.end(),
                                                  [&aux_body](const ChLigament::ChLigamentProperties& l_prop) -> bool {
                                                      return l_prop.body_1 == aux_body;
                                                  });

            utils::throw_invalid_argument(lig_prop_it->local_coordinates,
                                          "[ERROR] [ChJaw] Please use absolute coordinates for the attachment points "
                                          "of the TMJ capsule ligaments!\n");

            AttachNode<NodeType> attach_node;
            attach_node.center_node = FindClosestNode(*tmj_disc_left_fea_, lig_prop_it->loc_1);
            attach_node.nodes_around = FindNodesWithinRadius(*tmj_disc_left_fea_, attach_node.center_node,
                                                             tmj_disc_fea_aux_body_radius_);
            attach_node.aux_body = aux_body;

            tmj_disc_left_attach_nodes.emplace_back(attach_node);
        }
        // Right
        for (const auto& aux_body: right_aux) {
            const auto lig_prop_it = std::find_if(tmj_capsule_ligament_init_properties_.begin(),
                                                  tmj_capsule_ligament_init_properties_.end(),
                                                  [&aux_body](const ChLigament::ChLigamentProperties& l_prop) -> bool {
                                                      return l_prop.body_1 == aux_body;
                                                  });

            utils::throw_invalid_argument(lig_prop_it->local_coordinates,
                                          "[ERROR] [ChJaw] Please use absolute coordinates for the attachment points "
                                          "of the TMJ capsule ligaments!\n");

            AttachNode<NodeType> attach_node;
            attach_node.center_node = FindClosestNode(*tmj_disc_right_fea_, lig_prop_it->loc_1);
            attach_node.nodes_around = FindNodesWithinRadius(*tmj_disc_right_fea_, attach_node.center_node,
                                                             tmj_disc_fea_aux_body_radius_);
            attach_node.aux_body = aux_body;

            tmj_disc_right_attach_nodes.emplace_back(attach_node);
        }

        // Set the center to the middle nodes
        for (int i = 0; i < num_aux_bodies; i++) {
            const auto& left_aux_body = tmj_disc_left_attach_nodes[i].aux_body;
            const auto& right_aux_body = tmj_disc_right_attach_nodes[i].aux_body;
            const auto& left_aux_pos = tmj_disc_left_attach_nodes[i].center_node->GetPos();
            const auto& right_aux_pos = tmj_disc_right_attach_nodes[i].center_node->GetPos();

            left_aux_body->SetPos(left_aux_pos);
            right_aux_body->SetPos(right_aux_pos);
        }

        // Create constraints between the auxiliary bodies and the middle nodes
        tmj_disc_left_fea_aux_links_.clear();
        tmj_disc_right_fea_aux_links_.clear();

        for (int i = 0; i < num_aux_bodies; i++) {
            // Left TMJ disc
            const auto cstr_left = chrono_types::make_shared<fea::ChLinkNodeFrame>();
            cstr_left->Initialize(tmj_disc_left_attach_nodes[i].center_node, tmj_disc_left_attach_nodes[i].aux_body,
                                  nullptr);
            tmj_disc_left_fea_aux_links_.push_back(cstr_left);

            // Right TMJ disc
            const auto cstr_right = chrono_types::make_shared<fea::ChLinkNodeFrame>();
            cstr_right->Initialize(tmj_disc_right_attach_nodes[i].center_node, tmj_disc_right_attach_nodes[i].aux_body,
                                   nullptr);
            tmj_disc_right_fea_aux_links_.push_back(cstr_right);
        }

        // Lambda expression to create constraints for the nodes around the middle nodes
        auto CreateVicinityConstraints = [](const std::vector<std::shared_ptr<fea::ChNodeFEAxyz> >& nodes_vicinity,
                                            const std::shared_ptr<ChBody>& aux_body,
                                            std::vector<std::shared_ptr<fea::ChLinkNodeFrame> >& constr_vector) ->
            void {
            for (const auto& node: nodes_vicinity) {
                const auto cstr_left = chrono_types::make_shared<fea::ChLinkNodeFrame>();
                cstr_left->Initialize(node, aux_body, nullptr);
                cstr_left->SetAttachReferenceInAbsoluteCoords(ChCoordsysd(node->GetX0()));

                constr_vector.push_back(cstr_left);
            }
        };

        // Attach nodes around the middle nodes to the auxiliary bodies
        for (int i = 0; i < num_aux_bodies; i++) {
            CreateVicinityConstraints(tmj_disc_left_attach_nodes[i].nodes_around, left_aux[i],
                                      tmj_disc_left_fea_aux_links_);
            CreateVicinityConstraints(tmj_disc_right_attach_nodes[i].nodes_around, right_aux[i],
                                      tmj_disc_right_fea_aux_links_);
        }

        // Visual model
        if (vis_mode_ != ChVisualizationMode::NONE) {
            constexpr double dim{0.75e-3};
            const auto vis_box = chrono_types::make_shared<ChVisualShapeSphere>(dim);
            for (int i = 0; i < num_aux_bodies; i++) {
                left_aux[i]->AddVisualShape(vis_box);
                right_aux[i]->AddVisualShape(vis_box);
            }
        }
    }

    void ChJaw::DefineBodyCollisionMeshes(const std::shared_ptr<ChBodyAuxRef>& body,
                                          double density,
                                          std::shared_ptr<ChContactMaterial>& mesh_material,
                                          const std::string& obj_file_vis,
                                          const std::string& obj_file_coll,
                                          bool enable_collision,
                                          bool mesh_static,
                                          bool mesh_convex,
                                          double mesh_swept_sphere_thickness,
                                          const ChColor& color) {
        double mass_per_density{0.0};
        ChVector3d cog{};
        ChVector3d principal_inertia_per_density{};
        ChMatrix33<double> inertia_per_density{};
        ChMatrix33<double> principal_inertia_rot{};

        std::shared_ptr<ChTriangleMeshConnected> mesh_vis{nullptr};
        std::shared_ptr<ChTriangleMeshConnected> mesh_coll{nullptr};

        // Load meshes from an OBJ files
        mesh_vis = ChTriangleMeshConnected::CreateFromWavefrontFile(obj_file_vis, true, true);
        utils::throw_runtime_error(!mesh_vis, "[ERROR] [ChJaw] mesh_vis file '{}' not found!\n", obj_file_vis);

        if (obj_file_coll.find("NONE") == std::string::npos) {
            mesh_coll = ChTriangleMeshConnected::CreateFromWavefrontFile(obj_file_coll, true, true);

            if (!mesh_coll) {
                enable_collision = false;
                fmt::print(utils::WARNING_MSG, "[WARNING] [ChJaw] mesh_coll file '{}' not found!\n", obj_file_coll);
            } else {
                mesh_coll->RepairDuplicateVertexes(1e-9);
            }
        }

        // Compute inertia_per_density properties with respect to the center of mass (bodyCoords=true!).
        // Important to use mesh_vis here!
        mesh_vis->RepairDuplicateVertexes(1e-9);
        mesh_vis->ComputeMassProperties(true, mass_per_density, cog, inertia_per_density);
        ChInertiaUtils::PrincipalInertia(inertia_per_density,
                                         principal_inertia_per_density, principal_inertia_rot);

        // Create a visual model containing a visualization mesh_vis
        const auto mesh_vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        mesh_vis_shape->SetMesh(mesh_vis, false);
        mesh_vis_shape->SetMutable(false);
        mesh_vis_shape->SetColor(color);
        mesh_vis_shape->SetBackfaceCull(true);

        // The following line is necessary for OpenGL to display the visualization meshes properly!
        // In Chrono::OpenGL, visualization meshes are maintained in a map keyed by the mesh name.
        // So if no name is set, the first mesh added to the visualization system will also be used
        // for all other bodies.
        mesh_vis_shape->SetName(body->GetName() + "_shape");
        // mesh_vis_shape->SetTexture(GetChronoDataFile("textures/concrete.jpg"));

        switch (render_mode_) {
            case ChRenderMode::SOLID: {
                mesh_vis_shape->SetWireframe(false);
                break;
            }
            case ChRenderMode::WIREFRAME: {
                mesh_vis_shape->SetWireframe(true);
                break;
            }
            default: {
                break;
            }
        }

        // Add visualization shape
        if (vis_mode_ != ChVisualizationMode::NONE) {
            if (const auto vis_tmp = body->GetVisualModel()) vis_tmp->Clear();
            body->AddVisualShape(mesh_vis_shape, ChFrame(VNULL, QUNIT));
        }

        // ------------------------------------------------
        // Incorporate mesh information into the rigid body
        // ------------------------------------------------

        // Set the COG coordinates to barycenter, without displacing the REF reference
        // Make the COG frame a principal frame
        body->SetFrameCOMToRef(ChFrame(cog, principal_inertia_rot));

        // Set inertia_per_density
        body->SetMass(mass_per_density * density);
        body->SetInertia(inertia_per_density * density);

        // InitModel collision model
        if (enable_collision && mesh_coll) {
            body->EnableCollision(true);

            // Set collision envelope and margin (just in the constructor won't work)
            ChCollisionModel::SetDefaultSuggestedEnvelope(collision_envelope_);
            ChCollisionModel::SetDefaultSuggestedMargin(collision_margin_);

            mesh_coll->RepairDuplicateVertexes(1e-9);

            auto tri_mesh_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                mesh_material, mesh_coll, mesh_static,
                mesh_convex, mesh_swept_sphere_thickness);
            body->AddCollisionShape(tri_mesh_shape);
        }
    }

    void ChJaw::DefineRigidTMJConstraints() {
        ChQuaterniond align_left, align_right;
        align_left.SetFromCardanAnglesXYZ(tmj_constr_cardan_xyz_left_ * CH_DEG_TO_RAD);
        align_right.SetFromCardanAnglesXYZ(tmj_constr_cardan_xyz_right_ * CH_DEG_TO_RAD);

        auto surface_nurbs_left = chrono_types::make_shared<ChSurfaceNurbs>();
        auto surface_nurbs_right = chrono_types::make_shared<ChSurfaceNurbs>();
        const auto shape_nurbs_left = chrono_types::make_shared<ChVisualShapeSurface>();
        const auto shape_nurbs_right = chrono_types::make_shared<ChVisualShapeSurface>();

        ChMatrixDynamic<ChVector3d> ctr_pts_surface_nurbs_left = Eigen::Map<ChMatrixDynamic<ChVector3d> >(
            tmj_constr_ctrl_pts_surface_nurbs_left_.data(),
            tmj_constr_ctrl_pts_per_dimension_.x(),
            tmj_constr_ctrl_pts_per_dimension_.y());
        ChMatrixDynamic<ChVector3d> ctr_pts_surface_nurbs_right = Eigen::Map<ChMatrixDynamic<ChVector3d> >(
            tmj_constr_ctrl_pts_surface_nurbs_right_.data(),
            tmj_constr_ctrl_pts_per_dimension_.x(),
            tmj_constr_ctrl_pts_per_dimension_.y());

        surface_nurbs_left->Setup(tmj_constr_surface_nurbs_order_.x(),
                                  tmj_constr_surface_nurbs_order_.y(),
                                  ctr_pts_surface_nurbs_left);
        surface_nurbs_right->Setup(tmj_constr_surface_nurbs_order_.x(),
                                   tmj_constr_surface_nurbs_order_.y(),
                                   ctr_pts_surface_nurbs_right);

        tmj_constr_left_ = chrono_types::make_shared<ChLinkLockPointSurface>(surface_nurbs_left, tmj_constr_verbose_);
        tmj_constr_right_ = chrono_types::make_shared<ChLinkLockPointSurface>(surface_nurbs_right, tmj_constr_verbose_);

        tmj_constr_left_->SetName("tmj_constr_left");
        tmj_constr_right_->SetName("tmj_constr_right");
        tmj_constr_left_->SetSamples(tmj_constr_point_surface_samples_);
        tmj_constr_right_->SetSamples(tmj_constr_point_surface_samples_);
        tmj_constr_left_->SetMaxIterations(tmj_constr_point_surface_max_iters_);
        tmj_constr_right_->SetMaxIterations(tmj_constr_point_surface_max_iters_);
        tmj_constr_left_->SetTolerance(tmj_constr_point_surface_tolerance_);
        tmj_constr_right_->SetTolerance(tmj_constr_point_surface_tolerance_);

        tmj_constr_left_->Initialize(mandible_, skull_, tmj_constr_use_rel_pos_,
                                     ChFrame(tmj_constr_pos_left_, align_left),
                                     ChFrame(tmj_constr_pos_left_, align_left));
        tmj_constr_right_->Initialize(mandible_, skull_, tmj_constr_use_rel_pos_,
                                      ChFrame(tmj_constr_pos_right_, align_right),
                                      ChFrame(tmj_constr_pos_right_, align_right));

        tmj_constr_left_->SetLimitActive(tmj_constr_u_limit_, tmj_constr_v_limit_);
        tmj_constr_right_->SetLimitActive(tmj_constr_u_limit_, tmj_constr_v_limit_);

        if (vis_mode_ != ChVisualizationMode::NONE) {
            shape_nurbs_left->SetSurfaceGeometry(surface_nurbs_left);
            shape_nurbs_right->SetSurfaceGeometry(surface_nurbs_right);
            shape_nurbs_left->SetWireframe(tmj_constr_vis_wireframe_);
            shape_nurbs_right->SetWireframe(tmj_constr_vis_wireframe_);
            shape_nurbs_left->SetColor(tmj_constr_vis_color_);
            shape_nurbs_right->SetColor(tmj_constr_vis_color_);

            skull_->AddVisualShape(shape_nurbs_left);
            skull_->AddVisualShape(shape_nurbs_right);
        }
    }

    void ChJaw::DefineLigaments(ChLigament::ChLigamentType type) {
        auto InitLigamentHelper = [](ChLigament::ChLigamentType lig_type,
                                     std::vector<ChLigament::ChLigamentProperties>& lig_props,
                                     std::map<std::string, std::shared_ptr<ChLigament> >& lig_map) -> void {
            switch (lig_type) {
                case ChLigament::ChLigamentType::DEFAULT: {
                    for (auto& l_prop: lig_props) {
                        auto lig = chrono_types::make_shared<ChLigamentDefault>(l_prop.params);
                        lig->SetName(l_prop.name);
                        lig_map.insert({l_prop.name, lig});
                    }
                    break;
                }
                case ChLigament::ChLigamentType::BLANKERVOORT: {
                    for (auto& l_prop: lig_props) {
                        auto lig = chrono_types::make_shared<ChLigamentBlankevoort>(l_prop.params);
                        lig->SetName(l_prop.name);
                        lig_map.insert({l_prop.name, lig});
                    }
                    break;
                }
                default: {
                    utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No supported ligament type!\n");
                }
            }
        };

        // Initialize ligaments
        ligament_map_.clear();
        InitLigamentHelper(type, ligament_init_properties_, ligament_map_);

        if (model_type_ == ChModelType::FEM) {
            tmj_capsule_ligament_map_.clear();
            InitLigamentHelper(type, tmj_capsule_ligament_init_properties_, tmj_capsule_ligament_map_);
        }

        // Set attachment points and visualization properties for the physical ligaments
        for (auto& l_prop: ligament_init_properties_) {
            const auto& l = ligament_map_.at(l_prop.name);

            ChFrame frame_loc_1{l_prop.loc_1};
            ChFrame frame_loc_2{l_prop.loc_2};

            if (l_prop.rel_to_ref) {
                const auto body_1 = std::dynamic_pointer_cast<ChBodyAuxRef>(l_prop.body_1);
                const auto body_2 = std::dynamic_pointer_cast<ChBodyAuxRef>(l_prop.body_2);

                utils::throw_runtime_error(!body_1 || !body_2,
                                           "[ERROR] [ChJaw] Failed to cast to ChBodyAuxRef in 'DefineLigaments'!\n");

                frame_loc_1 = frame_loc_1 >> body_1->GetFrameRefToCOM();
                frame_loc_2 = frame_loc_2 >> body_2->GetFrameRefToCOM();
            }

            l->Initialize(l_prop.body_1, l_prop.body_2, l_prop.local_coordinates,
                          frame_loc_1.GetPos(), frame_loc_2.GetPos());

            l->IsStiff(stiff_ligaments_);
            l->SetVerbose(ligaments_verbose_);

            if (vis_mode_ != ChVisualizationMode::NONE) {
                l->AddVisualShape(l_prop.shape);
                l->GetVisualShape(0)->SetColor(l_prop.color);
            }
        }

        if (model_type_ == ChModelType::FEM) {
            // Set attachment points and visualization properties for the TMJ capsule ligaments
            for (auto& l_prop: tmj_capsule_ligament_init_properties_) {
                const auto& l = tmj_capsule_ligament_map_.at(l_prop.name);

                l->Initialize(l_prop.body_1, l_prop.body_2, false,
                              l_prop.body_1->GetPos(), l_prop.loc_2);

                l->IsStiff(stiff_tmj_capsule_ligaments_);
                l->SetVerbose(tmj_capsule_ligaments_verbose_);

                if (vis_mode_ != ChVisualizationMode::NONE) {
                    l->AddVisualShape(l_prop.shape);
                    l->GetVisualShape(0)->SetColor(l_prop.color);
                }
            }
        }

        ligament_type_ = type;
        // ligament_init_properties_.clear();
        // tmj_capsule_ligament_init_properties_.clear();
    }

    void ChJaw::DefineMuscles(ChMuscle::ChMuscleType type) {
        auto InitMuscleHelper = [&](auto muscle_type,
                                    std::vector<ChMuscle::ChMuscleProperties>& muscle_props,
                                    std::map<std::string, std::shared_ptr<ChMuscle> >& muscle_map) -> void {
            using MuscleType = decltype(muscle_type);

            for (auto& m_prop: muscle_props) {
                auto muscle = chrono_types::make_shared<MuscleType>(chrono_types::make_shared<ChFunctionConst>(0),
                                                                    m_prop.params);

                if (muscle_activation_dynamics_) muscle->EnableActivationDynamics();

                muscle->SetName(m_prop.name);
                muscle_map.insert({m_prop.name, muscle});
            }
        };

        muscle_map_.clear();

        switch (type) {
            case ChMuscle::ChMuscleType::TSDA: {
                InitMuscleHelper(ChMuscleTSDA(), muscle_init_properties_, muscle_map_);
                break;
            }
            case ChMuscle::ChMuscleType::LINEAR: {
                InitMuscleHelper(ChMuscleLinear(), muscle_init_properties_, muscle_map_);
                break;
            }
            case ChMuscle::ChMuscleType::PECK: {
                InitMuscleHelper(ChMusclePeck(), muscle_init_properties_, muscle_map_);
                break;
            }
            case ChMuscle::ChMuscleType::MILLARD: {
                InitMuscleHelper(ChMuscleMillard(), muscle_init_properties_, muscle_map_);
                break;
            }
            default: {
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] No supported muscle type!\n");
            }
        }

        // // TODO: clean up -------------------
        // muscle_init_properties_.emplace_back("superior_lateral_pterygoid_left",
        //                                      std::vector<std::variant<double, bool> >{
        //                                          0.1, 17.0, 0.02106, 0.3, 0.15, 0.17, true, true
        //                                      },
        //                                      tmj_disc_left_fea_aux_bodies_[0],
        //                                      maxilla_, ChVector3<>(0.047009, -0.072507, 0.023149),
        //                                      ChVector3<>(0.044195, -0.064302, 0.039071),
        //                                      false, true,
        //                                      ChColor(0.6, 0.17, 0.17));
        // muscle_init_properties_.emplace_back("superior_lateral_pterygoid_right",
        //                                      std::vector<std::variant<double, bool> >{
        //                                          0.1, 17.0, 0.020977, 0.3, 0.15, 0.17, true, true
        //                                      },
        //                                      tmj_disc_left_fea_aux_bodies_[0],
        //                                      maxilla_, ChVector3<>(-0.047009, -0.072507, 0.023149),
        //                                      ChVector3<>(-0.044585, -0.064435, 0.039106),
        //                                      false, true,
        //                                      ChColor(0.6, 0.17, 0.17));
        // // ----------------------------------

        for (auto& m_prop: muscle_init_properties_) {
            const auto& m = muscle_map_.at(m_prop.name);

            ChFrame frame_loc_1{m_prop.loc_1};
            ChFrame frame_loc_2{m_prop.loc_2};

            if (m_prop.rel_to_ref && m_prop.name != "superior_lateral_pterygoid_left" && m_prop.name !=
                "superior_lateral_pterygoid_right") {
                const auto body_1 = std::dynamic_pointer_cast<ChBodyAuxRef>(m_prop.body_1);
                const auto body_2 = std::dynamic_pointer_cast<ChBodyAuxRef>(m_prop.body_2);

                utils::throw_runtime_error(!body_1 || !body_2,
                                           "[ERROR] [ChJaw] Failed to cast to ChBodyAuxRef in 'DefineMuscles'!\n");

                frame_loc_1 = frame_loc_1 >> body_1->GetFrameRefToCOM();
                frame_loc_2 = frame_loc_2 >> body_2->GetFrameRefToCOM();
            } else {
                const auto body_2 = std::dynamic_pointer_cast<ChBodyAuxRef>(m_prop.body_2);

                utils::throw_runtime_error(!body_2,
                                           "[ERROR] [ChJaw] Failed to cast to ChBodyAuxRef in 'DefineMuscles'!\n");

                frame_loc_1 = frame_loc_1 >> m_prop.body_1->GetFrameCOMToAbs().GetInverse();
                frame_loc_2 = frame_loc_2 >> body_2->GetFrameRefToCOM();
            }

            // // TODO: clean up -------------------
            // if (m_prop.name == "superior_lateral_pterygoid_left") {
            //     m->Initialize(tmj_disc_left_fea_aux_bodies_[0], m_prop.body_2, m_prop.local_coordinates,
            //                   frame_loc_1.GetPos(), frame_loc_2.GetPos());
            // } else if (m_prop.name == "superior_lateral_pterygoid_right") {
            //     m->Initialize(tmj_disc_right_fea_aux_bodies_[0], m_prop.body_2, m_prop.local_coordinates,
            //                   frame_loc_1.GetPos(), frame_loc_2.GetPos());
            // } else {
            //     m->Initialize(m_prop.body_1, m_prop.body_2, m_prop.local_coordinates,
            //               frame_loc_1.GetPos(), frame_loc_2.GetPos());
            // }
            // // ----------------------------------

            m->Initialize(m_prop.body_1, m_prop.body_2, m_prop.local_coordinates,
                          frame_loc_1.GetPos(), frame_loc_2.GetPos());

            m->IsStiff(stiff_muscles_);
            m->SetVerbose(muscles_verbose_);

            if (vis_mode_ != ChVisualizationMode::NONE) {
                // m->AddVisualShape(m_prop.shape);
                // auto shape = chrono_types::make_shared<ChVisualShapeMuscle>(7, 8, m);
                auto shape = chrono_types::make_shared<ChVisualShapeSpring>(0.0025, 150, 20);
                shape->SetColor(m_prop.color);
                // skull_->AddVisualShape(shape);
                m->AddVisualShape(shape);
                // m->GetVisualShape(0)->SetColor(m_prop.color);
            }
        }

        muscle_type_ = type;
        // muscle_init_properties_.clear();
    }

    void ChJaw::DefineObjectObjectContactBehavior() const {
        ChContactForceTorqueJawSMC::ContactModelsSpecifications general_contact_models{
            ChContactForceTorqueJawSMC::Hertz,
            ChContactForceTorqueJawSMC::Constant,
            ChContactForceTorqueJawSMC::OneStep
        };

        ChContactForceTorqueJawSMC::ContactModelsSpecifications efc_contact_models{
            ChContactForceTorqueJawSMC::EFC,
            ChContactForceTorqueJawSMC::Constant,
            ChContactForceTorqueJawSMC::NoneTangential
        };

        contact_behavior_map_->clear();
        contact_behavior_map_->insert({{&(*skull_), &(*mandible_)}, general_contact_models});
        contact_behavior_map_->insert({{&(*maxilla_), &(*mandible_)}, general_contact_models});
    }

    void ChJaw::CaptureMandibleKinematics() {
        write_lock lock(mtx_);

        mandible_kinematics_.time = sys_->GetChTime();

        mandible_kinematics_.position = {
            incisal_point_aux_body_->GetPos().x(),
            incisal_point_aux_body_->GetPos().y(),
            incisal_point_aux_body_->GetPos().z()
        };
        mandible_kinematics_.rotation = {
            incisal_point_aux_body_->GetRot().e0(),
            incisal_point_aux_body_->GetRot().e1(),
            incisal_point_aux_body_->GetRot().e2(),
            incisal_point_aux_body_->GetRot().e3()
        };
        mandible_kinematics_.linear_velocity = {
            incisal_point_aux_body_->GetPosDt().x(),
            incisal_point_aux_body_->GetPosDt().y(),
            incisal_point_aux_body_->GetPosDt().z()
        };
        mandible_kinematics_.angular_velocity = {
            incisal_point_aux_body_->GetRotDt().e0(),
            incisal_point_aux_body_->GetRotDt().e1(),
            incisal_point_aux_body_->GetRotDt().e2(),
            incisal_point_aux_body_->GetRotDt().e3()
        };
        mandible_kinematics_.linear_acceleration = {
            incisal_point_aux_body_->GetPosDt2().x(),
            incisal_point_aux_body_->GetPosDt2().y(),
            incisal_point_aux_body_->GetPosDt2().z()
        };
        mandible_kinematics_.angular_acceleration = {
            incisal_point_aux_body_->GetRotDt2().e0(),
            incisal_point_aux_body_->GetRotDt2().e1(),
            incisal_point_aux_body_->GetRotDt2().e2(),
            incisal_point_aux_body_->GetRotDt2().e3()
        };
    }

    void ChJaw::InfoString(std::string& info) {
        std::stringstream ss;
        auto title_str = fmt::format(utils::INFO_MSG,
                                     "---------- Jaw Model Properties ----------\n");
        auto sim_str = fmt::format(fmt::emphasis::bold,
                                   "--------------- Simulation ---------------\n");
        auto sys_str = fmt::format(fmt::emphasis::bold,
                                   "----------------- System -----------------\n");
        auto vis_str = fmt::format(fmt::emphasis::bold,
                                   "------------- Visualization --------------\n");
        auto sep_str = fmt::format(fmt::emphasis::bold,
                                   "------------------------------------------\n");
        auto end_str = fmt::format(utils::INFO_MSG,
                                   "------------------------------------------\n\n");

        auto solver_str = chrono::utils::EnumToString(solver_type_, chrono::utils::solver_enum_conversion_map);
        auto time_stepper_str = chrono::utils::EnumToString(time_stepper_type_,
                                                            chrono::utils::time_stepper_enum_conversion_map);

        ChJawEnumMapper::ChVisualizationMode_mapper vis_mapper;
        ChJawEnumMapper::ChRenderMode_mapper render_mapper;
        std::string vis_mode_str = vis_mapper(vis_mode_).GetValueAsString();
        std::string render_mode_str = render_mapper(render_mode_).GetValueAsString();

        ss << "Copyright (c) 2025 Paul-Otto Müller\n\n";
        ss << title_str;
        ss << sim_str;
        ss << "Threads:             " << std::to_string(threads_) << '\n';
        ss << "Contact method:      " << (contact_method_ == ChContactMethod::SMC ? "SMC" : "NSC") << '\n';
        ss << "Collision type:      " << (collision_type_ == ChCollisionSystem::Type::MULTICORE
                                              ? "MULTICORE"
                                              : "BULLET") << '\n';
        ss << "Solver type:         " << solver_str + '\n';
        ss << "Time stepper:        " << time_stepper_str + '\n';
        ss << "Time step:           " << time_step_ << '\n';
        ss << sep_str;
        ss << vis_str;
        ss << "Visualization Mode:  " << vis_mode_str << '\n';
        ss << "Render mode:         " << render_mode_str << '\n';
        ss << "Window size:         " << window_width_ << " x " << window_height_ << '\n';
        ss << sep_str;
        ss << sys_str;
        ss << "Bone density [kg/m³]: " << bone_density_ << "\n";
        ss << "Skull: \n" << " - Mass [kg]: " << skull_->GetMass() << "\n";
        ss << " - Inertia [kgm²]: \n\t" << skull_->GetInertia() << '\n';
        ss << "Maxilla: \n" << " - Mass [kg]: " << maxilla_->GetMass() << "\n";
        ss << " - Inertia [kgm²]: \n\t" << maxilla_->GetInertia() << '\n';
        ss << "Mandible: \n" << " - Mass [kg]: " << mandible_->GetMass() << "\n";
        ss << " - Inertia [kgm²]: \n\t" << mandible_->GetInertia() << '\n';
        ss << "Hyoid: \n" << " - Mass [kg]: " << hyoid_->GetMass() << "\n";
        ss << " - Inertia [kgm²]: \n\t" << hyoid_->GetInertia() << '\n';
        ss << sep_str;
        ss << end_str;

        info = ss.str();
    }

    void ChJaw::WriteState(ChArchiveOut& archive_out) {
        // Time, position, and orientation
        double time = sys_->GetChTime();
        const auto state_pose = chrono_types::make_shared<ChState>(sys_->GetNumCoordsPosLevel(), sys_.get());
        const auto state_vel = chrono_types::make_shared<ChStateDelta>(sys_->GetNumCoordsVelLevel(), sys_.get());
        sys_->StateGather(*state_pose, *state_vel, time);

        // Acceleration
        const auto state_acc = chrono_types::make_shared<ChStateDelta>(sys_->GetNumCoordsAccLevel(), sys_.get());
        sys_->StateGatherAcceleration(*state_acc);

        // Lagrangian multiplier
        const auto state_L = chrono_types::make_shared<ChVectorDynamic<> >(sys_->GetNumConstraints());
        sys_->StateGatherReactions(*state_L);

        // Write the state
        archive_out << CHNVP(time);
        archive_out << CHNVP(state_pose);
        archive_out << CHNVP(state_vel);
        archive_out << CHNVP(state_acc);
        archive_out << CHNVP(state_L);
    }

    void ChJaw::ReadState(ChArchiveIn& archive_in) {
        double time;
        const auto state_pose = chrono_types::make_shared<ChState>(sys_->GetNumCoordsPosLevel(), sys_.get());
        const auto state_acc = chrono_types::make_shared<ChStateDelta>(sys_->GetNumCoordsAccLevel(), sys_.get());
        const auto state_vel = chrono_types::make_shared<ChStateDelta>(sys_->GetNumCoordsVelLevel(), sys_.get());
        const auto state_L = chrono_types::make_shared<ChVectorDynamic<> >(sys_->GetNumConstraints());

        // Read the state
        archive_in >> CHNVP(time);
        archive_in >> CHNVP(state_pose);
        archive_in >> CHNVP(state_vel);
        archive_in >> CHNVP(state_acc);
        archive_in >> CHNVP(state_L);

        // Load the state into the system
        sys_->StateScatterAcceleration(*state_acc);
        sys_->StateScatterReactions(*state_L);
        sys_->StateScatter(*state_pose, *state_vel, time, true);
    }

    void ChJaw::ArchiveOut(ChArchiveOut& archive_out) {
        using namespace chrono::utils;

        // Version number
        archive_out.VersionWrite<ChJaw>();

        // Serialize enums
        ChJawEnumMapper::ChModelType_mapper model_mapper;
        ChJawEnumMapper::ChVisualizationMode_mapper vis_mapper;
        ChJawEnumMapper::ChRenderMode_mapper render_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::ContactForceTorqueModel_mapper
                contact_force_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::AdhesionForceTorqueModel_mapper
                adhesion_force_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::TangentialDisplacementModel_mapper
                tang_disp_mapper;
        fea::ChGmshMeshFileLoader::ChGmshMeshFileLoaderEnumMapper::ChFEAImplementationType_mapper fea_impl_mapper;
        ChLigament::ChLigamentEnumMapper::ChLigamentType_mapper lig_mapper;
        ChMuscle::ChMuscleEnumMapper::ChMuscleType_mapper mus_mapper;

        // Serialize all member data
        archive_out << CHNVP(model_mapper(model_type_), "model_type_");
        archive_out << CHNVP(name_);
        // archive_out << CHNVP(enable_rigid_body_model_);
        archive_out << CHNVP(enable_solver_debugging_);
        archive_out << CHNVP(enable_system_debugging_);
        archive_out << CHNVP(threads_);
        archive_out << CHNVP(gravity_);
        archive_out << CHNVP(time_step_);
        archive_out << CHNVP(simulation_time_limit_);

        archive_out << CHNVP(time_stepper_verbose_);
        archive_out << CHNVP(hht_step_control_);
        archive_out << CHNVP(euler_i_max_iters_);
        archive_out << CHNVP(trapezoidal_lin_max_iters_);
        archive_out << CHNVP(newmark_max_iters_);
        archive_out << CHNVP(hht_max_iters_);
        const auto time_stepper_type = EnumToString(time_stepper_type_, time_stepper_enum_conversion_map);
        archive_out << CHNVP(time_stepper_type);

        archive_out << CHNVP(solver_verbose_);
        archive_out << CHNVP(solver_tolerance_);
        archive_out << CHNVP(max_solver_iterations_);
        const auto solver_type = EnumToString(solver_type_, solver_enum_conversion_map);
        archive_out << CHNVP(solver_type);
        archive_out << CHNVP(admm_warm_start_);
        const auto admm_step_policy = EnumToString(admm_step_policy_, admm_step_enum_conversion_map);
        const auto admm_acceleration = EnumToString(admm_acceleration_, admm_acceleration_enum_conversion_map);
        archive_out << CHNVP(admm_step_policy);
        archive_out << CHNVP(admm_acceleration);
        archive_out << CHNVP(mkl_lock_sparsity_pattern_);
        archive_out << CHNVP(mkl_use_sparsity_pattern_learner_);

        archive_out << CHNVP(window_width_);
        archive_out << CHNVP(window_height_);
        archive_out << CHNVP(window_title_);
        archive_out << CHNVP(vis_mapper(vis_mode_), "vis_mode_");
        archive_out << CHNVP(render_mapper(render_mode_), "render_mode_");
        archive_out << CHNVP(bone_color_);
        archive_out << CHNVP(background_color_);

        archive_out << CHNVP(collision_margin_);
        archive_out << CHNVP(collision_envelope_);
        archive_out << CHNVP(bone_mesh_swept_sphere_thickness_);
        archive_out << CHNVP(tmj_disc_mesh_swept_sphere_thickness_);
        archive_out << CHNVP(contact_behavior_map_);
        const auto contact_method = EnumToString(contact_method_, contact_method_enum_conversion_map);
        const auto collision_type = EnumToString(collision_type_, collision_system_enum_conversion_map);
        const auto narrowphase_algo = EnumToString(narrowphase_algorithm_multicore_,
                                                   narrowphase_algorithm_enum_conversion_map);
        archive_out << CHNVP(contact_method);
        archive_out << CHNVP(collision_type);
        archive_out << CHNVP(narrowphase_algo);
        archive_out << CHNVP(broadphase_grid_resolution_multicore_);
        archive_out << CHNVP(contact_breaking_threshold_bullet_);
        archive_out << CHNVP(default_effective_curvature_radius_smc_);
        archive_out << CHNVP(contact_force_mapper(contact_force_model_smc_), "contact_force_model_smc_");
        archive_out << CHNVP(adhesion_force_mapper(adhesion_force_model_smc_), "adhesion_force_model_smc_");
        archive_out << CHNVP(tang_disp_mapper(tangential_displ_model_smc_), "tangential_displ_model_smc_");
        archive_out << CHNVP(min_bounce_speed_nsc_);
        archive_out << CHNVP(contact_recovery_speed_nsc_);

        archive_out << CHNVP(bone_density_);
        archive_out << CHNVP(bone_contact_material_);
        archive_out << CHNVP(tmj_disc_contact_material_);
        archive_out << CHNVP(use_material_properties_smc_);
        archive_out << CHNVP(bone_properties_smc_);
        archive_out << CHNVP(tmj_disc_properties_smc_);
        archive_out << CHNVP(bone_properties_nsc_);
        archive_out << CHNVP(tmj_disc_properties_nsc_);
        archive_out << CHNVP(tmj_disc_fea_use_mooney_rivlin_);
        archive_out << CHNVP(tmj_disc_fea_mr_c_1_);
        archive_out << CHNVP(tmj_disc_fea_mr_c_2_);
        archive_out << CHNVP(tmj_disc_fea_density_);
        archive_out << CHNVP(tmj_disc_fea_young_modulus_);
        archive_out << CHNVP(tmj_disc_fea_poisson_ratio_);
        archive_out << CHNVP(tmj_disc_fea_rayleigh_damping_alpha_);
        archive_out << CHNVP(tmj_disc_fea_rayleigh_damping_beta_);
        archive_out << CHNVP(tmj_disc_fea_material_);

        archive_out << CHNVP(sys_);
        archive_out << CHNVP(tmj_disc_fea_aux_body_radius_);
        archive_out << CHNVP(TMJ_DISC_LEFT_FEA_AUX_PREFIX_);
        archive_out << CHNVP(TMJ_DISC_RIGHT_FEA_AUX_PREFIX_);
        archive_out << CHNVP(tmj_disc_left_fea_aux_bodies_);
        archive_out << CHNVP(tmj_disc_right_fea_aux_bodies_);
        archive_out << CHNVP(tmj_disc_left_fea_aux_links_);
        archive_out << CHNVP(tmj_disc_right_fea_aux_links_);
        archive_out << CHNVP(tmj_disc_fea_element_type_);
        archive_out << CHNVP(fea_impl_mapper(tmj_disc_fea_implementation_), "tmj_disc_fea_implementation_");
        archive_out << CHNVP(TMJ_DISC_LEFT_NAME_);
        archive_out << CHNVP(TMJ_DISC_RIGHT_NAME_);
        archive_out << CHNVP(tmj_disc_left_fea_);
        archive_out << CHNVP(tmj_disc_right_fea_);
        archive_out << CHNVP(SKULL_NAME_);
        archive_out << CHNVP(MAXILLA_NAME_);
        archive_out << CHNVP(MANDIBLE_NAME_);
        archive_out << CHNVP(HYOID_NAME_);
        archive_out << CHNVP(skull_);
        archive_out << CHNVP(maxilla_);
        archive_out << CHNVP(mandible_);
        archive_out << CHNVP(hyoid_);
        archive_out << CHNVP(tmj_constr_left_);
        archive_out << CHNVP(tmj_constr_right_);
        archive_out << CHNVP(stiff_ligaments_);

        // auto& l1 = test_map_.at("l1");
        // fmt::print(utils::INFO_MSG, "test_map_: \n");

        // archive_out << CHNVP(test_map_);

        archive_out << CHNVP(lig_mapper(ligament_type_), "ligament_type_");
        // archive_out << CHNVP(ligament_map_);
        // archive_out << CHNVP(tmj_capsule_ligament_map_);
        archive_out << CHNVP(ligament_init_properties_);
        archive_out << CHNVP(tmj_capsule_ligament_init_properties_);
        archive_out << CHNVP(stiff_muscles_);
        archive_out << CHNVP(muscle_activation_dynamics_);
        archive_out << CHNVP(mus_mapper(muscle_type_), "muscle_type_");
        // archive_out << CHNVP(muscle_map_);
        archive_out << CHNVP(muscle_init_properties_);

        archive_out << CHNVP(mesh_dir_);
        archive_out << CHNVP(skull_obj_file_vis_);
        archive_out << CHNVP(maxilla_obj_file_vis_);
        archive_out << CHNVP(mandible_obj_file_vis_);
        archive_out << CHNVP(hyoid_obj_file_vis_);
        archive_out << CHNVP(skull_obj_file_coll_);
        archive_out << CHNVP(maxilla_obj_file_coll_);
        archive_out << CHNVP(mandible_obj_file_coll_);
        archive_out << CHNVP(skull_obj_file_rigid_coll_);
        archive_out << CHNVP(maxilla_obj_file_rigid_coll_);
        archive_out << CHNVP(mandible_obj_file_rigid_coll_);
        archive_out << CHNVP(tmj_disc_obj_file_coll_);
        archive_out << CHNVP(fea_wireframe_);
        archive_out << CHNVP(fea_smooth_faces_);
        archive_out << CHNVP(color_scale_min_max_);
        auto fea_vis_data_type = EnumToString(fea_vis_data_type_, visual_shape_fea_enum_conversion_map);
        archive_out << CHNVP(fea_vis_data_type);
        // archive_out << CHNVP(TMJ_DISC_LEFT_NODE_FILE_FEA_);
        // archive_out << CHNVP(TMJ_DISC_RIGHT_NODE_FILE_FEA_);
        // archive_out << CHNVP(TMJ_DISC_LEFT_ELE_FILE_FEA_);
        // archive_out << CHNVP(TMJ_DISC_RIGHT_ELE_FILE_FEA_);
        archive_out << CHNVP(tmj_disc_left_msh_file_fea_);
        archive_out << CHNVP(tmj_disc_right_msh_file_fea_);
    }

    void ChJaw::ArchiveIn(ChArchiveIn& archive_in) {
        using namespace chrono::utils;

        // Version number
        int version = archive_in.VersionRead<ChJaw>();

        // Deserialize enums
        ChJawEnumMapper::ChModelType_mapper model_type_mapper;
        ChJawEnumMapper::ChVisualizationMode_mapper vis_mapper;
        ChJawEnumMapper::ChRenderMode_mapper render_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::ContactForceTorqueModel_mapper
                contact_force_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::AdhesionForceTorqueModel_mapper
                adhesion_force_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::TangentialDisplacementModel_mapper
                tang_disp_mapper;
        fea::ChGmshMeshFileLoader::ChGmshMeshFileLoaderEnumMapper::ChFEAImplementationType_mapper fea_impl_mapper;
        ChLigament::ChLigamentEnumMapper::ChLigamentType_mapper lig_mapper;
        ChMuscle::ChMuscleEnumMapper::ChMuscleType_mapper mus_mapper;

        // Deserialize all member data
        archive_in >> CHNVP(model_type_mapper(model_type_), "model_type_");
        archive_in >> CHNVP(name_);
        // archive_in >> CHNVP(enable_rigid_body_model_);
        archive_in >> CHNVP(enable_solver_debugging_);
        archive_in >> CHNVP(enable_system_debugging_);
        archive_in >> CHNVP(threads_);
        archive_in >> CHNVP(gravity_);
        archive_in >> CHNVP(time_step_);
        archive_in >> CHNVP(simulation_time_limit_);

        archive_in >> CHNVP(time_stepper_verbose_);
        archive_in >> CHNVP(hht_step_control_);
        archive_in >> CHNVP(euler_i_max_iters_);
        archive_in >> CHNVP(trapezoidal_lin_max_iters_);
        archive_in >> CHNVP(newmark_max_iters_);
        archive_in >> CHNVP(hht_max_iters_);
        std::string time_stepper_type;
        archive_in >> CHNVP(time_stepper_type);
        time_stepper_type_ = StringToEnum(time_stepper_type, time_stepper_enum_conversion_map);

        archive_in >> CHNVP(solver_verbose_);
        archive_in >> CHNVP(solver_tolerance_);
        archive_in >> CHNVP(max_solver_iterations_);
        std::string solver_type;
        archive_in >> CHNVP(solver_type);
        solver_type_ = StringToEnum(solver_type, solver_enum_conversion_map);
        archive_in >> CHNVP(admm_warm_start_);
        std::string admm_step_policy;
        std::string admm_acceleration;
        archive_in >> CHNVP(admm_step_policy);
        archive_in >> CHNVP(admm_acceleration);
        admm_step_policy_ = StringToEnum(admm_step_policy, admm_step_enum_conversion_map);
        admm_acceleration_ = StringToEnum(admm_acceleration, admm_acceleration_enum_conversion_map);
        archive_in >> CHNVP(admm_step_policy_);
        archive_in >> CHNVP(admm_acceleration_);
        archive_in >> CHNVP(mkl_lock_sparsity_pattern_);
        archive_in >> CHNVP(mkl_use_sparsity_pattern_learner_);

        archive_in >> CHNVP(window_width_);
        archive_in >> CHNVP(window_height_);
        archive_in >> CHNVP(window_title_);
        archive_in >> CHNVP(vis_mapper(vis_mode_), "vis_mode_");
        archive_in >> CHNVP(render_mapper(render_mode_), "render_mode_");
        archive_in >> CHNVP(bone_color_);
        archive_in >> CHNVP(background_color_);

        archive_in >> CHNVP(collision_margin_);
        archive_in >> CHNVP(collision_envelope_);
        archive_in >> CHNVP(bone_mesh_swept_sphere_thickness_);
        archive_in >> CHNVP(tmj_disc_mesh_swept_sphere_thickness_);
        // archive_in >> CHNVP(contact_behavior_map_); TODO: doesn't work yet
        std::string contact_method;
        std::string collision_type;
        std::string narrowphase_algo;
        archive_in >> CHNVP(contact_method);
        archive_in >> CHNVP(collision_type);
        archive_in >> CHNVP(narrowphase_algo);
        contact_method_ = StringToEnum(contact_method, contact_method_enum_conversion_map);
        collision_type_ = StringToEnum(collision_type, collision_system_enum_conversion_map);
        narrowphase_algorithm_multicore_ = StringToEnum(narrowphase_algo, narrowphase_algorithm_enum_conversion_map);
        archive_in >> CHNVP(broadphase_grid_resolution_multicore_);
        archive_in >> CHNVP(contact_breaking_threshold_bullet_);
        archive_in >> CHNVP(default_effective_curvature_radius_smc_);
        archive_in >> CHNVP(contact_force_mapper(contact_force_model_smc_), "contact_force_model_smc_");
        archive_in >> CHNVP(adhesion_force_mapper(adhesion_force_model_smc_), "adhesion_force_model_smc_");
        archive_in >> CHNVP(tang_disp_mapper(tangential_displ_model_smc_), "tangential_displ_model_smc_");
        archive_in >> CHNVP(min_bounce_speed_nsc_);
        archive_in >> CHNVP(contact_recovery_speed_nsc_);

        archive_in >> CHNVP(bone_density_);
        archive_in >> CHNVP(bone_contact_material_);
        archive_in >> CHNVP(tmj_disc_contact_material_);
        archive_in >> CHNVP(use_material_properties_smc_);
        archive_in >> CHNVP(bone_properties_smc_);
        archive_in >> CHNVP(tmj_disc_properties_smc_);
        archive_in >> CHNVP(bone_properties_nsc_);
        archive_in >> CHNVP(tmj_disc_properties_nsc_);
        archive_in >> CHNVP(tmj_disc_fea_use_mooney_rivlin_);
        archive_in >> CHNVP(tmj_disc_fea_mr_c_1_);
        archive_in >> CHNVP(tmj_disc_fea_mr_c_2_);
        archive_in >> CHNVP(tmj_disc_fea_density_);
        archive_in >> CHNVP(tmj_disc_fea_young_modulus_);
        archive_in >> CHNVP(tmj_disc_fea_poisson_ratio_);
        archive_in >> CHNVP(tmj_disc_fea_rayleigh_damping_alpha_);
        archive_in >> CHNVP(tmj_disc_fea_rayleigh_damping_beta_);
        archive_in >> CHNVP(tmj_disc_fea_material_);

        archive_in >> CHNVP(sys_);
        archive_in >> CHNVP(tmj_disc_fea_aux_body_radius_);
        archive_in >> CHNVP(TMJ_DISC_LEFT_FEA_AUX_PREFIX_);
        archive_in >> CHNVP(TMJ_DISC_RIGHT_FEA_AUX_PREFIX_);
        archive_in >> CHNVP(tmj_disc_left_fea_aux_bodies_);
        archive_in >> CHNVP(tmj_disc_right_fea_aux_bodies_);
        archive_in >> CHNVP(tmj_disc_left_fea_aux_links_);
        archive_in >> CHNVP(tmj_disc_right_fea_aux_links_);
        archive_in >> CHNVP(tmj_disc_fea_element_type_);
        archive_in >> CHNVP(fea_impl_mapper(tmj_disc_fea_implementation_), "tmj_disc_fea_implementation_");
        archive_in >> CHNVP(TMJ_DISC_LEFT_NAME_);
        archive_in >> CHNVP(TMJ_DISC_RIGHT_NAME_);
        archive_in >> CHNVP(tmj_disc_left_fea_);
        archive_in >> CHNVP(tmj_disc_right_fea_);
        archive_in >> CHNVP(SKULL_NAME_);
        archive_in >> CHNVP(MAXILLA_NAME_);
        archive_in >> CHNVP(MANDIBLE_NAME_);
        archive_in >> CHNVP(HYOID_NAME_);
        archive_in >> CHNVP(skull_);
        archive_in >> CHNVP(maxilla_);
        archive_in >> CHNVP(mandible_);
        archive_in >> CHNVP(hyoid_);
        archive_in >> CHNVP(tmj_constr_left_);
        archive_in >> CHNVP(tmj_constr_right_);
        archive_in >> CHNVP(stiff_ligaments_);
        archive_in >> CHNVP(lig_mapper(ligament_type_), "ligament_type_");
        archive_in >> CHNVP(ligament_map_);
        archive_in >> CHNVP(tmj_capsule_ligament_map_);
        // archive_in >> CHNVP(ligament_init_properties_); TODO: doesn't work yet
        // archive_in >> CHNVP(tmj_capsule_ligament_init_properties_);
        archive_in >> CHNVP(stiff_muscles_);
        archive_in >> CHNVP(muscle_activation_dynamics_);
        archive_in >> CHNVP(mus_mapper(muscle_type_), "muscle_type_");
        archive_in >> CHNVP(muscle_map_);
        // archive_in >> CHNVP(muscle_init_properties_); TODO: doesn't work yet

        archive_in >> CHNVP(mesh_dir_);
        archive_in >> CHNVP(skull_obj_file_vis_);
        archive_in >> CHNVP(maxilla_obj_file_vis_);
        archive_in >> CHNVP(mandible_obj_file_vis_);
        archive_in >> CHNVP(hyoid_obj_file_vis_);
        archive_in >> CHNVP(skull_obj_file_coll_);
        archive_in >> CHNVP(maxilla_obj_file_coll_);
        archive_in >> CHNVP(mandible_obj_file_coll_);
        archive_in >> CHNVP(skull_obj_file_rigid_coll_);
        archive_in >> CHNVP(maxilla_obj_file_rigid_coll_);
        archive_in >> CHNVP(mandible_obj_file_rigid_coll_);
        archive_in >> CHNVP(tmj_disc_obj_file_coll_);
        archive_in >> CHNVP(fea_wireframe_);
        archive_in >> CHNVP(fea_smooth_faces_);
        archive_in >> CHNVP(color_scale_min_max_);
        std::string fea_vis_data_type;
        archive_in >> CHNVP(fea_vis_data_type);
        fea_vis_data_type_ = StringToEnum(fea_vis_data_type, visual_shape_fea_enum_conversion_map);
        // archive_in >> CHNVP(TMJ_DISC_LEFT_NODE_FILE_FEA_);
        // archive_in >> CHNVP(TMJ_DISC_RIGHT_NODE_FILE_FEA_);
        // archive_in >> CHNVP(TMJ_DISC_LEFT_ELE_FILE_FEA_);
        // archive_in >> CHNVP(TMJ_DISC_RIGHT_ELE_FILE_FEA_);
        archive_in >> CHNVP(tmj_disc_left_msh_file_fea_);
        archive_in >> CHNVP(tmj_disc_right_msh_file_fea_);
    }
} // namespace chrono::biomechanics
