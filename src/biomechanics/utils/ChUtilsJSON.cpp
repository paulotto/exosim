/**
 * @file ChUtilsJSON.cpp
 * @brief TODO
 * @ref Chrono sensor module ("chrono_sensor/utils/ChUtilsJSON.cpp")
 * @author Paul-Otto Müller
 * @date 10.08.2023
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

#include <vector>
#include <fstream>
#include <filesystem>
//
#include "ExoSimConfig.h"
#include "exosim/biomechanics/ChMuscle.h"
#include "exosim/biomechanics/ChLigament.h"
#include "exosim//biomechanics/utils/utils.h"
#include "exosim/biomechanics/utils/ChUtilsJSON.h"
#include "exosim/biomechanics/utils/ChGmshMeshFileLoader.h"
#include "exosim/biomechanics/utils/ChEnumStringConverter.h"
//
#include <chrono_thirdparty/rapidjson/prettywriter.h>
#include <chrono_thirdparty/rapidjson/filereadstream.h>
#include <chrono_thirdparty/rapidjson/istreamwrapper.h>
#include <chrono_thirdparty/rapidjson/ostreamwrapper.h>


namespace chrono::biomechanics::utils {
    using namespace rapidjson;

    const std::string EXOSIM_RESOURCES_DIR_STR{std::string(EXOSIM_RESOURCES_DIR) + "/"};
    const std::string MEMBER_MISSING_ERROR_JSON{"[ERROR] [ChUtilsJSON] JSON file does not have a member '{}': '{}'!"};
    const std::string MEMBER_WRONG_TYPE_JSON{
        "[ERROR] [ChUtilsJSON] JSON member '{}' has the wrong type (expected type: '{}'): '{}'!"
    };

    auto member_error_json = [](const rapidjson::Value& object, const std::string& member,
                                const std::string& expected_type, const std::string& filename) -> void {
        utils::throw_runtime_error(!object.HasMember(member.c_str()), MEMBER_MISSING_ERROR_JSON, member, filename);

        const bool cond = (expected_type == "string" && !object[member.c_str()].IsString()) ||
                          (expected_type == "int" && !object[member.c_str()].IsInt()) ||
                          (expected_type == "bool" && !object[member.c_str()].IsBool()) ||
                          (expected_type == "float" && !object[member.c_str()].IsFloat()) ||
                          (expected_type == "double" && !object[member.c_str()].IsDouble()) ||
                          (expected_type == "array" && !object[member.c_str()].IsArray()) ||
                          (expected_type == "object" && !object[member.c_str()].IsObject());
        utils::throw_runtime_error(cond, MEMBER_WRONG_TYPE_JSON, member, expected_type, filename);
    };

    // -----------------------------------------------------------------------------

    void ReadFileJSON(const std::string& filename, const std::string& type, rapidjson::Document& d) {
        std::ifstream ifs(filename);
        utils::throw_runtime_error(!ifs.good(), "[ERROR] [ChUtilsJSON] Could not open JSON file: '{}' for reading!\n",
                                   filename);

        IStreamWrapper isw(ifs);
        d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
        utils::throw_runtime_error(d.IsNull(),
                                   "[ERROR] [ChUtilsJSON] Invalid JSON file: '{}'!\n", filename);

        member_error_json(d, "type", "string", filename);

        utils::throw_invalid_argument(d["type"].GetString() != type,
                                      "[ERROR] [ChUtilsJSON] 'type' != {}: '{}'!\n", type, filename);
    }

    void WriteFileJSON(const std::string& filename, const rapidjson::Document& d) {
        if (std::filesystem::exists(filename)) {
            try {
                std::filesystem::copy_file(filename, filename + ".bak",
                                           std::filesystem::copy_options::skip_existing);
            } catch (const std::exception& e) {
                fmt::print(utils::ERROR_MSG, e.what());
                fmt::print(utils::ERROR_MSG, "Could not create backup file: '{}.bak'!\n", filename);
                return;
            }
        }

        std::ofstream ofs(filename);
        utils::throw_runtime_error(!ofs.good(), "[ERROR] [ChUtilsJSON] Could not open JSON file: '{}' for writing!\n",
                                   filename);

        OStreamWrapper osw(ofs);
        PrettyWriter<OStreamWrapper> writer(osw);
        d.Accept(writer);
    }

    // -----------------------------------------------------------------------------

    std::shared_ptr<ChJaw> ReadJawJSON(const std::string& filename) {
        auto jaw = chrono_types::make_shared<ChJaw>();
        ReadJawJSON(*jaw, filename);
        return jaw;
    }

    void ReadJawJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;

        try {
            ReadFileJSON(filename, "Jaw", d);
        } catch (const std::exception& e) {
            fmt::print(utils::ERROR_MSG, e.what());
            fmt::print(utils::WARNING_MSG,
                       "[WARNING] [ChUtilsJSON] Proceeding with default jaw model...\n");
            return;
        }

        member_error_json(d, "name", "string", filename);
        jaw.Name() = d["name"].GetString();

        member_error_json(d, "general_parameters", "object", filename);
        member_error_json(d, "time_stepper", "object", filename);
        member_error_json(d, "solver", "object", filename);
        member_error_json(d, "collision", "object", filename);
        member_error_json(d, "material_properties", "object", filename);
        member_error_json(d, "system_components", "object", filename);
        member_error_json(d, "visualization", "object", filename);

        const auto& general_parameters = d["general_parameters"].GetObject();
        const auto& time_stepper = d["time_stepper"].GetObject();
        const auto& solver = d["solver"].GetObject();
        const auto& collision = d["collision"].GetObject();
        const auto& material_properties = d["material_properties"].GetObject();
        const auto& system_components = d["system_components"].GetObject();
        const auto& visualization = d["visualization"].GetObject();

        member_error_json(system_components, "incisal_frame_abs", "object", filename);

        // General parameters
        member_error_json(general_parameters, "model_type", "string", filename);
        // member_error_json(general_parameters, "enable_rigid_body_model", "bool", filename);
        member_error_json(general_parameters, "enable_solver_debugging", "bool", filename);
        member_error_json(general_parameters, "enable_system_debugging", "bool", filename);
        member_error_json(general_parameters, "threads", "int", filename);
        member_error_json(general_parameters, "gravity", "double", filename);
        member_error_json(general_parameters, "time_step", "double", filename);
        member_error_json(general_parameters, "simulation_time_limit", "double", filename);

        ChJaw::ChJawEnumMapper::ChModelType_mapper model_type_mapper;
        ChJaw::ChModelType model_type;
        model_type_mapper(model_type).SetValueAsString(general_parameters["model_type"].GetString());

        jaw.ModelType() = model_type;
        // jaw.EnableRigidBodyModel() = general_parameters["enable_rigid_body_model"].GetBool();
        jaw.EnableSolverDebugging() = general_parameters["enable_solver_debugging"].GetBool();
        jaw.EnableSystemDebugging() = general_parameters["enable_system_debugging"].GetBool();
        jaw.Threads() = general_parameters["threads"].GetInt();
        jaw.Gravity() = general_parameters["gravity"].GetDouble();
        jaw.TimeStep() = general_parameters["time_step"].GetDouble();
        jaw.SimulationTimeLimit() = general_parameters["simulation_time_limit"].GetDouble();

        // Time stepper
        member_error_json(time_stepper, "time_stepper_verbose", "bool", filename);
        member_error_json(time_stepper, "hht_step_control", "bool", filename);
        member_error_json(time_stepper, "euler_i_max_iters", "int", filename);
        member_error_json(time_stepper, "trapezoidal_lin_max_iters", "int", filename);
        member_error_json(time_stepper, "newmark_max_iters", "int", filename);
        member_error_json(time_stepper, "hht_max_iters", "int", filename);
        member_error_json(time_stepper, "hht_alpha", "double", filename);
        member_error_json(time_stepper, "time_stepper_type", "string", filename);

        jaw.TimeStepperVerbose() = time_stepper["time_stepper_verbose"].GetBool();
        jaw.HHTStepControl() = time_stepper["hht_step_control"].GetBool();
        jaw.EulerIMaxIters() = time_stepper["euler_i_max_iters"].GetInt();
        jaw.TrapezoidalLinMaxIters() = time_stepper["trapezoidal_lin_max_iters"].GetInt();
        jaw.NewmarkMaxIters() = time_stepper["newmark_max_iters"].GetInt();
        jaw.HHTMaxIters() = time_stepper["hht_max_iters"].GetInt();
        jaw.HHTAlpha() = time_stepper["hht_alpha"].GetDouble();
        jaw.TimeStepperType() = chrono::utils::StringToEnum(time_stepper["time_stepper_type"].GetString(),
                                                            chrono::utils::time_stepper_enum_conversion_map);

        // Solver
        member_error_json(solver, "solver_verbose", "bool", filename);
        member_error_json(solver, "solver_tolerance", "double", filename);
        member_error_json(solver, "max_solver_iterations", "int", filename);
        member_error_json(solver, "solver_type", "string", filename);
        member_error_json(solver, "admm_warm_start", "bool", filename);
        member_error_json(solver, "admm_rho", "double", filename);
        member_error_json(solver, "admm_tolerance_dual", "double", filename);
        member_error_json(solver, "admm_tolerance_primal", "double", filename);
        member_error_json(solver, "admm_step_policy", "string", filename);
        member_error_json(solver, "admm_acceleration", "string", filename);
        member_error_json(solver, "mkl_lock_sparsity_pattern", "bool", filename);
        member_error_json(solver, "mkl_use_sparsity_pattern_learner", "bool", filename);

        jaw.SolverVerbose() = solver["solver_verbose"].GetBool();
        jaw.SolverTolerance() = solver["solver_tolerance"].GetDouble();
        jaw.MaxSolverIterations() = solver["max_solver_iterations"].GetInt();
        jaw.SolverType() = chrono::utils::StringToEnum(solver["solver_type"].GetString(),
                                                       chrono::utils::solver_enum_conversion_map);
        jaw.SolverADMMWarmStart() = solver["admm_warm_start"].GetBool();
        jaw.SolverADMMRho() = solver["admm_rho"].GetDouble();
        jaw.SolverADMMToleranceDual() = solver["admm_tolerance_dual"].GetDouble();
        jaw.SolverADMMTolerancePrimal() = solver["admm_tolerance_primal"].GetDouble();
        jaw.SolverADMMStepPolicy() = chrono::utils::StringToEnum(solver["admm_step_policy"].GetString(),
                                                                 chrono::utils::admm_step_enum_conversion_map);
        jaw.SolverADMMAcceleration() = chrono::utils::StringToEnum(solver["admm_acceleration"].GetString(),
                                                                   chrono::utils::admm_acceleration_enum_conversion_map);
        jaw.MKLLockSparsityPattern() = solver["mkl_lock_sparsity_pattern"].GetBool();
        jaw.MKLUseSparsityPatternLearner() = solver["mkl_use_sparsity_pattern_learner"].GetBool();

        // Collision
        member_error_json(collision, "collision_margin", "double", filename);
        member_error_json(collision, "collision_envelope", "double", filename);
        member_error_json(collision, "bone_mesh_swept_sphere_thickness", "double", filename);
        member_error_json(collision, "contact_method", "string", filename);
        member_error_json(collision, "collision_type", "string", filename);
        member_error_json(collision, "narrowphase_algorithm_multicore", "string", filename);
        member_error_json(collision, "broadphase_grid_resolution_multicore", "array", filename);
        member_error_json(collision, "contact_breaking_threshold_bullet", "double", filename);
        member_error_json(collision, "default_effective_curvature_radius_smc", "double", filename);
        member_error_json(collision, "contact_force_model_smc", "string", filename);
        member_error_json(collision, "adhesion_force_model_smc", "string", filename);
        member_error_json(collision, "tangential_displ_model_smc", "string", filename);
        member_error_json(collision, "min_bounce_speed_nsc", "double", filename);
        member_error_json(collision, "contact_recovery_speed_nsc", "double", filename);
        member_error_json(collision, "skull_obj_file_coll", "string", filename);
        member_error_json(collision, "maxilla_obj_file_coll", "string", filename);
        member_error_json(collision, "mandible_obj_file_coll", "string", filename);
        member_error_json(collision, "skull_obj_file_rigid_coll", "string", filename);
        member_error_json(collision, "maxilla_obj_file_rigid_coll", "string", filename);
        member_error_json(collision, "mandible_obj_file_rigid_coll", "string", filename);

        jaw.CollisionMargin() = collision["collision_margin"].GetDouble();
        jaw.CollisionEnvelope() = collision["collision_envelope"].GetDouble();
        jaw.BoneMeshSweptSphereThickness() = collision["bone_mesh_swept_sphere_thickness"].GetDouble();
        jaw.ContactMethod() = chrono::utils::StringToEnum(collision["contact_method"].GetString(),
                                                          chrono::utils::contact_method_enum_conversion_map);
        jaw.CollisionSystemType() = chrono::utils::StringToEnum(collision["collision_type"].GetString(),
                                                                chrono::utils::collision_system_enum_conversion_map);
        jaw.NarrowphaseAlgorithmMulticore() = chrono::utils::StringToEnum(
            collision["narrowphase_algorithm_multicore"].GetString(),
            chrono::utils::narrowphase_algorithm_enum_conversion_map);
        jaw.BroadphaseGridResolutionMulticore() = ReadVector3dJSON(
            collision["broadphase_grid_resolution_multicore"].GetArray());
        jaw.ContactBreakingThresholdBullet() = collision["contact_breaking_threshold_bullet"].GetDouble();
        jaw.DefaultEffectiveCurvatureRadiusSMC() = collision["default_effective_curvature_radius_smc"].GetDouble();

        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::ContactForceTorqueModel_mapper contact_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::AdhesionForceTorqueModel_mapper
                adhesion_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::TangentialDisplacementModel_mapper
                tangential_mapper;
        ChContactForceTorqueJawSMC::ContactForceTorqueModel contact_model;
        ChContactForceTorqueJawSMC::AdhesionForceTorqueModel adhesion_model;
        ChContactForceTorqueJawSMC::TangentialDisplacementModel tangential_model;
        contact_mapper(contact_model).SetValueAsString(collision["contact_force_model_smc"].GetString());
        adhesion_mapper(adhesion_model).SetValueAsString(collision["adhesion_force_model_smc"].GetString());
        tangential_mapper(tangential_model).SetValueAsString(collision["tangential_displ_model_smc"].GetString());

        jaw.ContactForceModelSMC() = contact_model;
        jaw.AdhesionForceModelSMC() = adhesion_model;
        jaw.TangentDisplacementModelSMC() = tangential_model;

        jaw.MinBounceSpeedNSC() = collision["min_bounce_speed_nsc"].GetDouble();
        jaw.ContactRecoverySpeedNSC() = collision["contact_recovery_speed_nsc"].GetDouble();
        jaw.SkullObjFileColl() = EXOSIM_RESOURCES_DIR_STR + collision["skull_obj_file_coll"].GetString();
        jaw.MaxillaObjFileColl() = EXOSIM_RESOURCES_DIR_STR + collision["maxilla_obj_file_coll"].GetString();
        jaw.MandibleObjFileColl() = EXOSIM_RESOURCES_DIR_STR + collision["mandible_obj_file_coll"].GetString();
        jaw.SkullObjFileRigidColl() = EXOSIM_RESOURCES_DIR_STR + collision["skull_obj_file_rigid_coll"].GetString();
        jaw.MaxillaObjFileRigidColl() = EXOSIM_RESOURCES_DIR_STR + collision["maxilla_obj_file_rigid_coll"].GetString();
        jaw.MandibleObjFileRigidColl() = EXOSIM_RESOURCES_DIR_STR + collision["mandible_obj_file_rigid_coll"].
                                         GetString();

        // Material properties
        member_error_json(material_properties, "bone_density", "float", filename);
        member_error_json(material_properties, "use_material_properties_smc", "bool", filename);
        member_error_json(material_properties, "bone_properties_smc", "array", filename);
        member_error_json(material_properties, "bone_properties_nsc", "array", filename);

        jaw.BoneDensity() = material_properties["bone_density"].GetFloat();
        jaw.UseMaterialPropertiesSMC() = material_properties["use_material_properties_smc"].GetBool();
        jaw.BonePropertiesSMC() = ReadContactMaterialPropertiesSMCJSON(material_properties["bone_properties_smc"]);
        jaw.BonePropertiesNSC() = ReadContactMaterialPropertiesNSCJSON(material_properties["bone_properties_nsc"]);

        // System components
        const auto& incisal_frame_abs = system_components["incisal_frame_abs"].GetObject();

        member_error_json(system_components, "muscles", "string", filename);
        member_error_json(system_components, "ligaments", "string", filename);
        member_error_json(system_components, "fea", "string", filename);
        member_error_json(incisal_frame_abs, "Location", "array", filename);
        member_error_json(incisal_frame_abs, "Orientation", "array", filename);
        member_error_json(system_components, "tmj_constr_verbose", "bool", filename);
        member_error_json(system_components, "tmj_constr_u_limit", "bool", filename);
        member_error_json(system_components, "tmj_constr_v_limit", "bool", filename);
        member_error_json(system_components, "tmj_constr_vis_wireframe", "bool", filename);
        member_error_json(system_components, "tmj_constr_use_rel_pos", "bool", filename);
        member_error_json(system_components, "tmj_constr_point_surface_samples", "int", filename);
        member_error_json(system_components, "tmj_constr_point_surface_max_iters", "int", filename);
        member_error_json(system_components, "tmj_constr_point_surface_tolerance", "double", filename);
        member_error_json(system_components, "tmj_constr_vis_color", "array", filename);
        member_error_json(system_components, "tmj_constr_pos_left", "array", filename);
        member_error_json(system_components, "tmj_constr_pos_right", "array", filename);
        member_error_json(system_components, "tmj_constr_cardan_xyz_left", "array", filename);
        member_error_json(system_components, "tmj_constr_cardan_xyz_right", "array", filename);
        member_error_json(system_components, "tmj_constr_surface_nurbs_order", "array", filename);
        member_error_json(system_components, "tmj_constr_ctrl_pts_per_dimension", "array", filename);
        member_error_json(system_components, "tmj_constr_ctrl_pts_surface_nurbs_left", "array", filename);
        member_error_json(system_components, "tmj_constr_ctrl_pts_surface_nurbs_right", "array", filename);

        ReadMusclePropertiesJSON(jaw, EXOSIM_RESOURCES_DIR_STR + system_components["muscles"].GetString());
        ReadLigamentPropertiesJSON(jaw, EXOSIM_RESOURCES_DIR_STR + system_components["ligaments"].GetString());
        ReadFEAPropertiesJSON(jaw, EXOSIM_RESOURCES_DIR_STR + system_components["fea"].GetString());

        jaw.IncisalFrameAbs() = ReadFrameJSON(incisal_frame_abs);
        jaw.TMJConstrVerbose() = system_components["tmj_constr_verbose"].GetBool();
        jaw.TMJConstrULimit() = system_components["tmj_constr_u_limit"].GetBool();
        jaw.TMJConstrVLimit() = system_components["tmj_constr_v_limit"].GetBool();
        jaw.TMJConstrVisWireframe() = system_components["tmj_constr_vis_wireframe"].GetBool();
        jaw.TMJConstrUseRelPos() = system_components["tmj_constr_use_rel_pos"].GetBool();
        jaw.TMJConstrPointSurfaceSamples() = system_components["tmj_constr_point_surface_samples"].GetInt();
        jaw.TMJConstrPointSurfaceMaxIters() = system_components["tmj_constr_point_surface_max_iters"].GetInt();
        jaw.TMJConstrPointSurfaceTolerance() = system_components["tmj_constr_point_surface_tolerance"].GetDouble();

        jaw.TMJConstrVisColor() = ReadColorJSON(system_components["tmj_constr_vis_color"]);

        jaw.TMJConstrPosLeft() = ReadVector3dJSON(system_components["tmj_constr_pos_left"].GetArray());
        jaw.TMJConstrPosRight() = ReadVector3dJSON(system_components["tmj_constr_pos_right"].GetArray());
        jaw.TMJConstrCardanXYZLeft() = ReadVector3dJSON(system_components["tmj_constr_cardan_xyz_left"].GetArray());
        jaw.TMJConstrCardanXYZRight() = ReadVector3dJSON(system_components["tmj_constr_cardan_xyz_right"].GetArray());

        jaw.TMJConstrSurfaceNURBSOrder() = ReadVector2iJSON(
            system_components["tmj_constr_surface_nurbs_order"].GetArray());
        jaw.TMJConstrCtrlPtsPerDimension() = ReadVector2iJSON(
            system_components["tmj_constr_ctrl_pts_per_dimension"].GetArray());

        std::vector<ChVector3d> tmj_constr_ctrl_pts_surface_nurbs_left, tmj_constr_ctrl_pts_surface_nurbs_right;
        for (int i = 0; i < system_components["tmj_constr_ctrl_pts_surface_nurbs_left"].Size(); ++i) {
            tmj_constr_ctrl_pts_surface_nurbs_left.push_back(
                ReadVector3dJSON(system_components["tmj_constr_ctrl_pts_surface_nurbs_left"][i].GetArray()));
        }
        for (int i = 0; i < system_components["tmj_constr_ctrl_pts_surface_nurbs_right"].Size(); ++i) {
            tmj_constr_ctrl_pts_surface_nurbs_right.push_back(
                ReadVector3dJSON(system_components["tmj_constr_ctrl_pts_surface_nurbs_right"][i].GetArray()));
        }
        jaw.TMJConstrCtrlPtsSurfaceNURBSLeft() = tmj_constr_ctrl_pts_surface_nurbs_left;
        jaw.TMJConstrCtrlPtsSurfaceNURBSRight() = tmj_constr_ctrl_pts_surface_nurbs_right;

        // Visualization
        member_error_json(visualization, "run_sim_at_startup", "bool", filename);
        member_error_json(visualization, "window_width", "int", filename);
        member_error_json(visualization, "window_height", "int", filename);
        member_error_json(visualization, "window_title", "string", filename);
        member_error_json(visualization, "vis_mode", "string", filename);
        member_error_json(visualization, "render_mode", "string", filename);
        member_error_json(visualization, "background_color", "array", filename);
        member_error_json(visualization, "bone_color", "array", filename);
        member_error_json(visualization, "skull_obj_file_vis", "string", filename);
        member_error_json(visualization, "maxilla_obj_file_vis", "string", filename);
        member_error_json(visualization, "mandible_obj_file_vis", "string", filename);
        member_error_json(visualization, "hyoid_obj_file_vis", "string", filename);

        jaw.RunSimAtStartup() = visualization["run_sim_at_startup"].GetBool();
        jaw.WindowWidth() = visualization["window_width"].GetInt();
        jaw.WindowHeight() = visualization["window_height"].GetInt();
        jaw.WindowTitle() = visualization["window_title"].GetString();

        ChJaw::ChJawEnumMapper::ChVisualizationMode_mapper vis_mode_mapper;
        ChJaw::ChJawEnumMapper::ChRenderMode_mapper render_mode_mapper;
        ChJaw::ChVisualizationMode vis_mode;
        ChJaw::ChRenderMode render_mode;
        vis_mode_mapper(vis_mode).SetValueAsString(visualization["vis_mode"].GetString());
        render_mode_mapper(render_mode).SetValueAsString(visualization["render_mode"].GetString());

        jaw.VisualizationMode() = vis_mode;
        jaw.RenderMode() = render_mode;

        jaw.BackgroundColor() = ReadColorJSON(visualization["background_color"]);
        jaw.BoneColor() = ReadColorJSON(visualization["bone_color"]);
        jaw.SkullObjFileVis() = EXOSIM_RESOURCES_DIR_STR + visualization["skull_obj_file_vis"].GetString();
        jaw.MaxillaObjFileVis() = EXOSIM_RESOURCES_DIR_STR + visualization["maxilla_obj_file_vis"].GetString();
        jaw.MandibleObjFileVis() = EXOSIM_RESOURCES_DIR_STR + visualization["mandible_obj_file_vis"].GetString();
        jaw.HyoidObjFileVis() = EXOSIM_RESOURCES_DIR_STR + visualization["hyoid_obj_file_vis"].GetString();
    }

    void WriteJawJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;
        d.SetObject();
        auto& allocator = d.GetAllocator();

        d.AddMember("name", Value(filename.c_str(), allocator), allocator);
        d.AddMember("type", "Jaw", allocator);

        // General parameters
        Value general_parameters(kObjectType);

        ChJaw::ChJawEnumMapper::ChModelType_mapper model_type_mapper;

        general_parameters.AddMember(
            "model_type", Value(model_type_mapper(jaw.ModelType()).GetValueAsString().c_str(), allocator),
            allocator);

        // general_parameters.AddMember("enable_rigid_body_model", jaw.EnableRigidBodyModel(), allocator);
        general_parameters.AddMember("enable_solver_debugging", jaw.EnableSolverDebugging(), allocator);
        general_parameters.AddMember("enable_system_debugging", jaw.EnableSystemDebugging(), allocator);
        general_parameters.AddMember("threads", jaw.Threads(), allocator);
        general_parameters.AddMember("gravity", jaw.Gravity(), allocator);
        general_parameters.AddMember("time_step", jaw.TimeStep(), allocator);
        general_parameters.AddMember("simulation_time_limit", jaw.SimulationTimeLimit(), allocator);
        d.AddMember("general_parameters", general_parameters, allocator);

        // Time stepper
        Value time_stepper(kObjectType);
        time_stepper.AddMember("time_stepper_verbose", jaw.TimeStepperVerbose(), allocator);
        time_stepper.AddMember("hht_step_control", jaw.HHTStepControl(), allocator);
        time_stepper.AddMember("euler_i_max_iters", jaw.EulerIMaxIters(), allocator);
        time_stepper.AddMember("trapezoidal_lin_max_iters", jaw.TrapezoidalLinMaxIters(), allocator);
        time_stepper.AddMember("newmark_max_iters", jaw.NewmarkMaxIters(), allocator);
        time_stepper.AddMember("hht_max_iters", jaw.HHTMaxIters(), allocator);
        time_stepper.AddMember("hht_alpha", jaw.HHTAlpha(), allocator);
        time_stepper.AddMember("time_stepper_type",
                               Value(chrono::utils::EnumToString(jaw.TimeStepperType(),
                                                                 chrono::utils::time_stepper_enum_conversion_map).
                                     c_str(), allocator), allocator);
        d.AddMember("time_stepper", time_stepper, allocator);

        // Solver
        Value solver(kObjectType);
        solver.AddMember("solver_verbose", jaw.SolverVerbose(), allocator);
        solver.AddMember("solver_tolerance", jaw.SolverTolerance(), allocator);
        solver.AddMember("max_solver_iterations", jaw.MaxSolverIterations(), allocator);
        solver.AddMember("solver_type",
                         Value(chrono::utils::EnumToString(jaw.SolverType(), chrono::utils::solver_enum_conversion_map).
                               c_str(), allocator), allocator);
        solver.AddMember("admm_warm_start", jaw.SolverADMMWarmStart(), allocator);
        solver.AddMember("admm_rho", jaw.SolverADMMRho(), allocator);
        solver.AddMember("admm_tolerance_dual", jaw.SolverADMMToleranceDual(), allocator);
        solver.AddMember("admm_tolerance_primal", jaw.SolverADMMTolerancePrimal(), allocator);
        solver.AddMember("admm_step_policy",
                         Value(chrono::utils::EnumToString(jaw.SolverADMMStepPolicy(),
                                                           chrono::utils::admm_step_enum_conversion_map).c_str(),
                               allocator), allocator);
        solver.AddMember("admm_acceleration", Value(chrono::utils::EnumToString(jaw.SolverADMMAcceleration(),
                                                        chrono::utils::admm_acceleration_enum_conversion_map).c_str(),
                                                    allocator), allocator);
        solver.AddMember("mkl_lock_sparsity_pattern", jaw.MKLLockSparsityPattern(), allocator);
        solver.AddMember("mkl_use_sparsity_pattern_learner", jaw.MKLUseSparsityPatternLearner(), allocator);
        d.AddMember("solver", solver, allocator);

        // Collision
        Value collision(kObjectType);
        collision.AddMember("collision_margin", jaw.CollisionMargin(), allocator);
        collision.AddMember("collision_envelope", jaw.CollisionEnvelope(), allocator);
        collision.AddMember("bone_mesh_swept_sphere_thickness", jaw.BoneMeshSweptSphereThickness(), allocator);
        collision.AddMember("contact_method", Value(chrono::utils::EnumToString(jaw.ContactMethod(),
                                                        chrono::utils::contact_method_enum_conversion_map).c_str(),
                                                    allocator), allocator);
        collision.AddMember("collision_type", Value(chrono::utils::EnumToString(jaw.CollisionSystemType(),
                                                        chrono::utils::collision_system_enum_conversion_map).c_str(),
                                                    allocator), allocator);
        collision.AddMember("narrowphase_algorithm_multicore",
                            Value(chrono::utils::EnumToString(jaw.NarrowphaseAlgorithmMulticore(),
                                                              chrono::utils::narrowphase_algorithm_enum_conversion_map).
                                  c_str(), allocator), allocator);

        Value broadphase_grid_resolution_multicore(kArrayType);
        WriteVector3dJSON(broadphase_grid_resolution_multicore, jaw.BroadphaseGridResolutionMulticore(), allocator);
        collision.AddMember("broadphase_grid_resolution_multicore", broadphase_grid_resolution_multicore, allocator);
        collision.AddMember("contact_breaking_threshold_bullet", jaw.ContactBreakingThresholdBullet(), allocator);
        collision.AddMember("default_effective_curvature_radius_smc", jaw.DefaultEffectiveCurvatureRadiusSMC(),
                            allocator);

        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::ContactForceTorqueModel_mapper contact_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::AdhesionForceTorqueModel_mapper
                adhesion_mapper;
        ChContactForceTorqueJawSMC::ChContactForceTorqueJawSMCEnumMapper::TangentialDisplacementModel_mapper
                tangential_mapper;

        collision.AddMember("contact_force_model_smc",
                            Value(contact_mapper(jaw.ContactForceModelSMC()).GetValueAsString().c_str(), allocator),
                            allocator);
        collision.AddMember("adhesion_force_model_smc",
                            Value(adhesion_mapper(jaw.AdhesionForceModelSMC()).GetValueAsString().c_str(), allocator),
                            allocator);
        collision.AddMember("tangential_displ_model_smc",
                            Value(tangential_mapper(jaw.TangentDisplacementModelSMC()).GetValueAsString().c_str(),
                                  allocator), allocator);

        collision.AddMember("min_bounce_speed_nsc", jaw.MinBounceSpeedNSC(), allocator);
        collision.AddMember("contact_recovery_speed_nsc", jaw.ContactRecoverySpeedNSC(), allocator);
        collision.AddMember("skull_obj_file_coll", Value(jaw.SkullObjFileColl().c_str(), allocator), allocator);
        collision.AddMember("maxilla_obj_file_coll", Value(jaw.MaxillaObjFileColl().c_str(), allocator), allocator);
        collision.AddMember("mandible_obj_file_coll", Value(jaw.MandibleObjFileColl().c_str(), allocator), allocator);
        collision.AddMember("skull_obj_file_rigid_coll", Value(jaw.SkullObjFileRigidColl().c_str(), allocator),
                            allocator);
        collision.AddMember("maxilla_obj_file_rigid_coll", Value(jaw.MaxillaObjFileRigidColl().c_str(), allocator),
                            allocator);
        collision.AddMember("mandible_obj_file_rigid_coll", Value(jaw.MandibleObjFileRigidColl().c_str(), allocator),
                            allocator);
        d.AddMember("collision", collision, allocator);

        // Material properties
        Value material_properties(kObjectType);
        material_properties.AddMember("bone_density", jaw.BoneDensity(), allocator);
        material_properties.AddMember("use_material_properties_smc", jaw.UseMaterialPropertiesSMC(), allocator);

        Value properties_smc(kArrayType);
        Value properties_nsc(kArrayType);
        WriteContactMaterialPropertiesSMCJSON(properties_smc, jaw.BonePropertiesSMC(), allocator);
        WriteContactMaterialPropertiesNSCJSON(properties_nsc, jaw.BonePropertiesNSC(), allocator);
        material_properties.AddMember("bone_properties_smc", properties_smc, allocator);
        material_properties.AddMember("bone_properties_nsc", properties_nsc, allocator);

        d.AddMember("material_properties", material_properties, allocator);

        // System components
        auto muscles_filename = ExtractPathAndFilenameWithoutExtension(filename) + "_muscles.json";
        auto ligaments_filename = ExtractPathAndFilenameWithoutExtension(filename) + "_ligaments.json";
        auto fea_filename = ExtractPathAndFilenameWithoutExtension(filename) + "_fea.json";

        Value system_components(kObjectType);
        Value incisal_frame_abs(kObjectType);
        WriteFrameJSON(incisal_frame_abs, jaw.IncisalFrameAbs(), allocator);
        system_components.AddMember("incisal_frame_abs", incisal_frame_abs, allocator);

        system_components.AddMember("tmj_constr_verbose", jaw.TMJConstrVerbose(), allocator);
        system_components.AddMember("tmj_constr_u_limit", jaw.TMJConstrULimit(), allocator);
        system_components.AddMember("tmj_constr_v_limit", jaw.TMJConstrVLimit(), allocator);
        system_components.AddMember("tmj_constr_vis_wireframe", jaw.TMJConstrVisWireframe(), allocator);
        system_components.AddMember("tmj_constr_use_rel_pos", jaw.TMJConstrUseRelPos(), allocator);
        system_components.AddMember("tmj_constr_point_surface_samples", jaw.TMJConstrPointSurfaceSamples(), allocator);
        system_components.AddMember("tmj_constr_point_surface_max_iters", jaw.TMJConstrPointSurfaceMaxIters(),
                                    allocator);
        system_components.AddMember("tmj_constr_point_surface_tolerance", jaw.TMJConstrPointSurfaceTolerance(),
                                    allocator);

        Value tmj_constr_vis_color(kArrayType);
        WriteColorJSON(tmj_constr_vis_color, jaw.TMJConstrVisColor(), allocator);
        system_components.AddMember("tmj_constr_vis_color", tmj_constr_vis_color, allocator);

        Value tmj_constr_pos_left(kArrayType), tmj_constr_pos_right(kArrayType),
                tmj_constr_cardan_xyz_left(kArrayType), tmj_constr_cardan_xyz_right(kArrayType);
        WriteVector3dJSON(tmj_constr_pos_left, jaw.TMJConstrPosLeft(), allocator);
        WriteVector3dJSON(tmj_constr_pos_right, jaw.TMJConstrPosRight(), allocator);
        WriteVector3dJSON(tmj_constr_cardan_xyz_left, jaw.TMJConstrCardanXYZLeft(), allocator);
        WriteVector3dJSON(tmj_constr_cardan_xyz_right, jaw.TMJConstrCardanXYZRight(), allocator);
        system_components.AddMember("tmj_constr_pos_left", tmj_constr_pos_left, allocator);
        system_components.AddMember("tmj_constr_pos_right", tmj_constr_pos_right, allocator);
        system_components.AddMember("tmj_constr_cardan_xyz_left", tmj_constr_cardan_xyz_left, allocator);
        system_components.AddMember("tmj_constr_cardan_xyz_right", tmj_constr_cardan_xyz_right, allocator);

        Value tmj_constr_surface_nurbs_order(kArrayType), tmj_constr_ctrl_pts_per_dimension(kArrayType);
        WriteVector2iJSON(tmj_constr_surface_nurbs_order, jaw.TMJConstrSurfaceNURBSOrder(), allocator);
        WriteVector2iJSON(tmj_constr_ctrl_pts_per_dimension, jaw.TMJConstrCtrlPtsPerDimension(), allocator);
        system_components.AddMember("tmj_constr_surface_nurbs_order", tmj_constr_surface_nurbs_order, allocator);
        system_components.AddMember("tmj_constr_ctrl_pts_per_dimension", tmj_constr_ctrl_pts_per_dimension, allocator);

        Value tmj_constr_ctrl_pts_surface_nurbs_left(kArrayType), tmj_constr_ctrl_pts_surface_nurbs_right(kArrayType);
        for (const auto& ctrl_pt: jaw.TMJConstrCtrlPtsSurfaceNURBSLeft()) {
            Value ctrl_pt_json_value(kArrayType);
            WriteVector3dJSON(ctrl_pt_json_value, ctrl_pt, allocator);
            tmj_constr_ctrl_pts_surface_nurbs_left.PushBack(ctrl_pt_json_value, allocator);
        }
        for (const auto& ctrl_pt: jaw.TMJConstrCtrlPtsSurfaceNURBSRight()) {
            Value ctrl_pt_json_value(kArrayType);
            WriteVector3dJSON(ctrl_pt_json_value, ctrl_pt, allocator);
            tmj_constr_ctrl_pts_surface_nurbs_right.PushBack(ctrl_pt_json_value, allocator);
        }
        system_components.AddMember("tmj_constr_ctrl_pts_surface_nurbs_left", tmj_constr_ctrl_pts_surface_nurbs_left,
                                    allocator);
        system_components.AddMember("tmj_constr_ctrl_pts_surface_nurbs_right", tmj_constr_ctrl_pts_surface_nurbs_right,
                                    allocator);

        system_components.AddMember("muscles", Value(muscles_filename.c_str(), allocator), allocator);
        system_components.AddMember("ligaments", Value(ligaments_filename.c_str(), allocator), allocator);
        system_components.AddMember("fea", Value(fea_filename.c_str(), allocator), allocator);
        d.AddMember("system_components", system_components, allocator);

        WriteMusclePropertiesJSON(jaw, muscles_filename);
        WriteLigamentPropertiesJSON(jaw, ligaments_filename);
        WriteFEAPropertiesJSON(jaw, fea_filename);

        // Visualization
        Value visualization(kObjectType);
        visualization.AddMember("run_sim_at_startup", jaw.RunSimAtStartup(), allocator);
        visualization.AddMember("window_width", jaw.WindowWidth(), allocator);
        visualization.AddMember("window_height", jaw.WindowHeight(), allocator);
        visualization.AddMember("window_title", Value(jaw.WindowTitle().c_str(), allocator), allocator);

        ChJaw::ChJawEnumMapper::ChVisualizationMode_mapper vis_mode_mapper;
        ChJaw::ChJawEnumMapper::ChRenderMode_mapper render_mode_mapper;

        visualization.AddMember(
            "vis_mode", Value(vis_mode_mapper(jaw.VisualizationMode()).GetValueAsString().c_str(), allocator),
            allocator);
        visualization.AddMember("render_mode",
                                Value(render_mode_mapper(jaw.RenderMode()).GetValueAsString().c_str(), allocator),
                                allocator);

        Value background_color(kArrayType);
        Value bone_color(kArrayType);
        WriteColorJSON(background_color, jaw.BackgroundColor(), allocator);
        WriteColorJSON(bone_color, jaw.BoneColor(), allocator);
        visualization.AddMember("background_color", background_color, allocator);
        visualization.AddMember("bone_color", bone_color, allocator);

        visualization.AddMember("skull_obj_file_vis", Value(jaw.SkullObjFileVis().c_str(), allocator), allocator);
        visualization.AddMember("maxilla_obj_file_vis", Value(jaw.MaxillaObjFileVis().c_str(), allocator), allocator);
        visualization.AddMember("mandible_obj_file_vis", Value(jaw.MandibleObjFileVis().c_str(), allocator), allocator);
        visualization.AddMember("hyoid_obj_file_vis", Value(jaw.HyoidObjFileVis().c_str(), allocator), allocator);
        d.AddMember("visualization", visualization, allocator);

        WriteFileJSON(filename, d);
    }

    // -----------------------------------------------------------------------------

    void ReadMusclePropertiesJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;

        try {
            ReadFileJSON(filename, "Muscle", d);
        } catch (const std::exception& e) {
            fmt::print(utils::ERROR_MSG, e.what());
            fmt::print(utils::WARNING_MSG, "[WARNING] [ChUtilsJSON] Proceeding with default muscle properties...\n");
            return;
        }

        member_error_json(d, "muscle_type", "string", filename);
        member_error_json(d, "muscles_verbose", "bool", filename);
        member_error_json(d, "stiff_muscles", "bool", filename);
        member_error_json(d, "muscle_properties", "array", filename);

        const auto muscle_type_str = d["muscle_type"].GetString();
        const auto muscles_verbose = d["muscles_verbose"].GetBool();
        const auto stiff_muscles = d["stiff_muscles"].GetBool();
        const auto muscle_activation_dynamics = d["muscle_activation_dynamics"].GetBool();

        ChMuscle::ChMuscleType muscle_type_enum;
        ChMuscle::ChMuscleEnumMapper::ChMuscleType_mapper muscle_mapper;
        muscle_mapper(muscle_type_enum).SetValueAsString(muscle_type_str);

        jaw.MuscleType() = muscle_type_enum;
        jaw.MusclesVerbose() = muscles_verbose;
        jaw.StiffMuscles() = stiff_muscles;
        jaw.MuscleActivationDynamics() = muscle_activation_dynamics;

        const auto& muscle_properties_json = d["muscle_properties"].GetArray();
        std::vector<ChMuscle::ChMuscleProperties> muscle_properties;

        for (const auto& object: muscle_properties_json) {
            member_error_json(object, "name", "string", filename);
            member_error_json(object, "parameter", "array", filename);
            member_error_json(object, "body_1", "string", filename);
            member_error_json(object, "body_2", "string", filename);
            member_error_json(object, "attachment_1", "array", filename);
            member_error_json(object, "attachment_2", "array", filename);
            member_error_json(object, "local_coordinates", "bool", filename);
            member_error_json(object, "rel_to_ref", "bool", filename);
            member_error_json(object, "color", "array", filename);
            // member_error_json(object, "visual_shape", "array", filename);

            try {
                auto& body_1 = jaw.GetBoneBody(object["body_1"].GetString());
                auto& body_2 = jaw.GetBoneBody(object["body_2"].GetString());

                muscle_properties.emplace_back(
                    object["name"].GetString(),
                    ReadMuscleParametersJSON(object["parameter"]),
                    body_1,
                    body_2,
                    ReadVector3dJSON(object["attachment_1"]),
                    ReadVector3dJSON(object["attachment_2"]),
                    object["local_coordinates"].GetBool(),
                    object["rel_to_ref"].GetBool(),
                    ReadColorJSON(object["color"]) /*,
                    ReadVisualShapeSpringJSON(object["visual_shape"])*/
                );
            } catch (std::invalid_argument& e) {
                fmt::print(utils::WARNING_MSG,
                           "[WARNING] [ChUtilsJSON] Skipping muscle... -> "
                           + std::string(e.what()));

                std::cout << object["name"].GetString() << std::endl;

                // TODO: Remove this temporary fix -------------------------------------
                std::shared_ptr<ChBody> bdy;
                if (std::string(object["name"].GetString()) == "superior_lateral_pterygoid_left") {
                    bdy = jaw.TMJDiscFEAAuxBodiesLeft()[0];

                    muscle_properties.emplace_back(object["name"].GetString(),
                                                     ReadMuscleParametersJSON(object["parameter"]),
                                                     bdy, jaw.Maxilla(),
                                                     ChVector3<>(0.047009, -0.072507, 0.023149),
                                                     ChVector3<>(0.044195, -0.064302, 0.039071),
                                                     true, true,
                                                     ChColor(0.6, 0.17, 0.17));
                    std::cout << "superior_lateral_pterygoid_left" << std::endl;
                }
                if (std::string(object["name"].GetString()) == "superior_lateral_pterygoid_right") {
                    bdy = jaw.TMJDiscFEAAuxBodiesRight()[0];

                    muscle_properties.emplace_back(object["name"].GetString(),
                                                     ReadMuscleParametersJSON(object["parameter"]),
                                                     bdy, jaw.Maxilla(),
                                                     ChVector3<>(-0.047009, -0.072507, 0.023149),
                                                     ChVector3<>(-0.044585, -0.064435, 0.039106),
                                                     true, true,
                                                     ChColor(0.6, 0.17, 0.17));
                    std::cout << "superior_lateral_pterygoid_right" << std::endl;
                }
                // ---------------------------------------------------------------------
            }
        }

        jaw.MuscleProperties() = muscle_properties;
    }

    void ReadLigamentPropertiesJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;

        try {
            ReadFileJSON(filename, "Ligament", d);
        } catch (const std::exception& e) {
            fmt::print(utils::ERROR_MSG, e.what());
            fmt::print(utils::WARNING_MSG, "[WARNING] [ChUtilsJSON] Proceeding with default ligament properties...\n");
            return;
        }

        member_error_json(d, "ligament_type", "string", filename);
        member_error_json(d, "ligaments_verbose", "bool", filename);
        member_error_json(d, "tmj_capsule_ligaments_verbose", "bool", filename);
        member_error_json(d, "stiff_ligaments", "bool", filename);
        member_error_json(d, "stiff_tmj_capsule_ligaments", "bool", filename);
        member_error_json(d, "ligament_properties", "array", filename);
        member_error_json(d, "capsule_ligament_properties", "array", filename);

        const auto ligament_type_str = d["ligament_type"].GetString();
        const auto ligaments_verbose = d["ligaments_verbose"].GetBool();
        const auto tmj_capsule_ligaments_verbose = d["tmj_capsule_ligaments_verbose"].GetBool();
        const auto stiff_ligaments = d["stiff_ligaments"].GetBool();
        const auto stiff_tmj_capsule_ligaments = d["stiff_tmj_capsule_ligaments"].GetBool();

        ChLigament::ChLigamentType ligament_type_enum;
        ChLigament::ChLigamentEnumMapper::ChLigamentType_mapper ligament_mapper;
        ligament_mapper(ligament_type_enum).SetValueAsString(ligament_type_str);

        jaw.LigamentType() = ligament_type_enum;
        jaw.LigamentsVerbose() = ligaments_verbose;
        jaw.TMJCapsuleLigamentsVerbose() = tmj_capsule_ligaments_verbose;
        jaw.StiffLigaments() = stiff_ligaments;
        jaw.StiffTMJCapsuleLigaments() = stiff_tmj_capsule_ligaments;

        const auto& ligament_properties_json = d["ligament_properties"].GetArray();
        const auto& capsule_ligament_properties_json = d["capsule_ligament_properties"].GetArray();
        std::vector<ChLigament::ChLigamentProperties> ligament_properties;
        std::vector<ChLigament::ChLigamentProperties> capsule_ligament_properties;

        for (const auto& object: ligament_properties_json) {
            member_error_json(object, "name", "string", filename);
            member_error_json(object, "parameter", "array", filename);
            member_error_json(object, "body_1", "string", filename);
            member_error_json(object, "body_2", "string", filename);
            member_error_json(object, "attachment_1", "array", filename);
            member_error_json(object, "attachment_2", "array", filename);
            member_error_json(object, "local_coordinates", "bool", filename);
            member_error_json(object, "rel_to_ref", "bool", filename);
            member_error_json(object, "color", "array", filename);
            member_error_json(object, "visual_shape", "array", filename);

            try {
                auto& body_1 = jaw.GetBoneBody(object["body_1"].GetString());
                auto& body_2 = jaw.GetBoneBody(object["body_2"].GetString());

                ligament_properties.emplace_back(
                    object["name"].GetString(),
                    ReadLigamentParametersJSON(object["parameter"]),
                    body_1,
                    body_2,
                    ReadVector3dJSON(object["attachment_1"]),
                    ReadVector3dJSON(object["attachment_2"]),
                    object["local_coordinates"].GetBool(),
                    object["rel_to_ref"].GetBool(),
                    ReadColorJSON(object["color"]),
                    ReadVisualShapeSpringJSON(object["visual_shape"])
                );
            } catch (std::exception& e) {
                fmt::print(utils::WARNING_MSG,
                           "[WARNING] [ChUtilsJSON] Skipping reading ligament properties... -> "
                           + std::string(e.what()));
                return;
            }
        }

        auto contains_substring = [](const std::string& str, const std::string& substring) -> bool {
            return str.find(substring) != std::string::npos;
        };

        for (const auto& object: capsule_ligament_properties_json) {
            member_error_json(object, "name", "string", filename);
            member_error_json(object, "parameter", "array", filename);
            member_error_json(object, "body_1", "string", filename);
            member_error_json(object, "body_2", "string", filename);
            member_error_json(object, "attachment_1", "array", filename);
            member_error_json(object, "attachment_2", "array", filename);
            member_error_json(object, "local_coordinates", "bool", filename);
            member_error_json(object, "rel_to_ref", "bool", filename);
            member_error_json(object, "color", "array", filename);
            member_error_json(object, "visual_shape", "array", filename);

            try {
                const std::string& body_1_str = object["body_1"].GetString();
                auto& body_1 = jaw.GetTMJDiscAuxBody(contains_substring(body_1_str, "left"), body_1_str.back() - '0');
                auto& body_2 = jaw.GetBoneBody(object["body_2"].GetString());

                capsule_ligament_properties.emplace_back(
                    object["name"].GetString(),
                    ReadLigamentParametersJSON(object["parameter"]),
                    body_1,
                    body_2,
                    ReadVector3dJSON(object["attachment_1"]),
                    ReadVector3dJSON(object["attachment_2"]),
                    object["local_coordinates"].GetBool(),
                    object["rel_to_ref"].GetBool(),
                    ReadColorJSON(object["color"]),
                    ReadVisualShapeSpringJSON(object["visual_shape"])
                );
            } catch (std::exception& e) {
                fmt::print(utils::WARNING_MSG,
                           "[WARNING] [ChUtilsJSON] Skipping reading capsule ligament properties... -> "
                           + std::string(e.what()));
                return;
            }
        }

        jaw.LigamentProperties() = ligament_properties;
        jaw.CapsuleLigamentProperties() = capsule_ligament_properties;
    }

    void ReadFEAPropertiesJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;

        try {
            ReadFileJSON(filename, "FEA", d);
        } catch (const std::exception& e) {
            fmt::print(utils::ERROR_MSG, e.what());
            fmt::print(utils::WARNING_MSG, "[WARNING] [ChUtilsJSON] Proceeding with default FEA properties...\n");
            return;
        }

        member_error_json(d, "material_properties", "object", filename);
        member_error_json(d, "collision_properties", "object", filename);
        member_error_json(d, "auxiliary_bodies", "object", filename);
        member_error_json(d, "meshes", "object", filename);
        member_error_json(d, "visualization", "object", filename);

        const auto& material_properties_json = d["material_properties"].GetObject();
        const auto& collision_properties_json = d["collision_properties"].GetObject();
        const auto& auxiliary_bodies_json = d["auxiliary_bodies"].GetObject();
        const auto& meshes_json = d["meshes"].GetObject();
        const auto& visualization_json = d["visualization"].GetObject();

        // Material properties
        member_error_json(material_properties_json, "tmj_disc_fea_use_mooney_rivlin", "bool", filename);
        member_error_json(material_properties_json, "tmj_disc_fea_mr_c_1", "double", filename);
        member_error_json(material_properties_json, "tmj_disc_fea_mr_c_2", "double", filename);
        member_error_json(material_properties_json, "tmj_disc_fea_density", "double", filename);
        member_error_json(material_properties_json, "tmj_disc_fea_young_modulus", "double", filename);
        member_error_json(material_properties_json, "tmj_disc_fea_poisson_ratio", "double", filename);
        member_error_json(material_properties_json, "tmj_disc_fea_rayleigh_damping_alpha", "double", filename);
        member_error_json(material_properties_json, "tmj_disc_fea_rayleigh_damping_beta", "double", filename);

        jaw.TMJDiscFEAUseMooneyRivlin() = material_properties_json["tmj_disc_fea_use_mooney_rivlin"].GetBool();
        jaw.TMJDiscFEAMRC1() = material_properties_json["tmj_disc_fea_mr_c_1"].GetDouble();
        jaw.TMJDiscFEAMRC2() = material_properties_json["tmj_disc_fea_mr_c_2"].GetDouble();
        jaw.TMJDiscFEADensity() = material_properties_json["tmj_disc_fea_density"].GetDouble();
        jaw.TMJDiscFEAYoungModulus() = material_properties_json["tmj_disc_fea_young_modulus"].GetDouble();
        jaw.TMJDiscFEAPoissonRatio() = material_properties_json["tmj_disc_fea_poisson_ratio"].GetDouble();
        jaw.TMJDiscFEARayleighDampingAlpha() = material_properties_json["tmj_disc_fea_rayleigh_damping_alpha"].
                GetDouble();
        jaw.TMJDiscFEARayleighDampingBeta() = material_properties_json["tmj_disc_fea_rayleigh_damping_beta"].
                GetDouble();

        // Collision properties
        member_error_json(collision_properties_json, "tmj_disc_properties_smc", "array", filename);
        member_error_json(collision_properties_json, "tmj_disc_properties_nsc", "array", filename);
        member_error_json(collision_properties_json, "tmj_disc_mesh_swept_sphere_thickness", "double", filename);
        member_error_json(collision_properties_json, "tmj_disc_obj_file_coll", "string", filename);

        jaw.TMJDiscPropertiesSMC() = ReadContactMaterialPropertiesSMCJSON(
            collision_properties_json["tmj_disc_properties_smc"].GetArray());
        jaw.TMJDiscPropertiesNSC() = ReadContactMaterialPropertiesNSCJSON(
            collision_properties_json["tmj_disc_properties_nsc"].GetArray());
        jaw.TMJDiscMeshSweptSphereThickness() = collision_properties_json["tmj_disc_mesh_swept_sphere_thickness"].
                GetDouble();
        jaw.TMJDiscObjFileColl() = collision_properties_json["tmj_disc_obj_file_coll"].GetString();

        // Auxiliary bodies
        member_error_json(auxiliary_bodies_json, "tmj_disc_fea_aux_body_radius", "float", filename);

        jaw.TMJDiscFEAAuxBodyRadius() = auxiliary_bodies_json["tmj_disc_fea_aux_body_radius"].GetFloat();

        // Meshes
        member_error_json(meshes_json, "tmj_disc_fea_element_type", "int", filename);
        member_error_json(meshes_json, "tmj_disc_fea_implementation", "string", filename);
        member_error_json(meshes_json, "tmj_disc_left_msh_file_fea", "string", filename);
        member_error_json(meshes_json, "tmj_disc_right_msh_file_fea", "string", filename);

        fea::ChGmshMeshFileLoader::ChFEAImplementationType fea_impl_type_enum;
        fea::ChGmshMeshFileLoader::ChGmshMeshFileLoaderEnumMapper::ChFEAImplementationType_mapper fea_impl_mapper;
        fea_impl_mapper(fea_impl_type_enum).SetValueAsString(meshes_json["tmj_disc_fea_implementation"].GetString());

        jaw.TMJDiscFEAElementType() = meshes_json["tmj_disc_fea_element_type"].GetInt();
        jaw.TMJDiscFEAImplementation() = fea_impl_type_enum;
        jaw.TMJDiscLeftMshFile() = EXOSIM_RESOURCES_DIR_STR + meshes_json["tmj_disc_left_msh_file_fea"].GetString();
        jaw.TMJDiscRightMshFile() = EXOSIM_RESOURCES_DIR_STR + meshes_json["tmj_disc_right_msh_file_fea"].GetString();

        // Visualization
        member_error_json(visualization_json, "fea_wireframe", "bool", filename);
        member_error_json(visualization_json, "fea_smooth_faces", "bool", filename);
        member_error_json(visualization_json, "color_scale_min_max", "array", filename);
        member_error_json(visualization_json, "fea_vis_data_type", "string", filename);

        jaw.FEAWireFrame() = visualization_json["fea_wireframe"].GetBool();
        jaw.FEASmoothFaces() = visualization_json["fea_smooth_faces"].GetBool();

        auto color_scale_min_max = ReadVector2dJSON(visualization_json["color_scale_min_max"].GetArray());
        jaw.ColorScaleMinMax() = {color_scale_min_max[0], color_scale_min_max[1]};

        jaw.FEAVisDataType() = chrono::utils::StringToEnum(visualization_json["fea_vis_data_type"].GetString(),
                                                           chrono::utils::visual_shape_fea_enum_conversion_map);
    }

    void WriteMusclePropertiesJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;
        d.SetObject();
        auto& allocator = d.GetAllocator();

        d.AddMember("name", Value(filename.c_str(), allocator), allocator);
        d.AddMember("type", "Muscle", allocator);

        // Add muscle properties
        ChMuscle::ChMuscleEnumMapper::ChMuscleType_mapper muscle_mapper;
        d.AddMember("muscle_type", Value(muscle_mapper(jaw.MuscleType()).GetValueAsString().c_str(), allocator),
                    allocator);
        d.AddMember("muscles_verbose", jaw.MusclesVerbose(), allocator);
        d.AddMember("stiff_muscles", jaw.StiffMuscles(), allocator);
        d.AddMember("muscle_activation_dynamics", jaw.MuscleActivationDynamics(), allocator);

        Value muscle_properties_array(kArrayType);
        for (const auto& muscle: jaw.MuscleProperties()) {
            Value muscle_object(kObjectType);
            muscle_object.AddMember("name", Value(muscle.name.c_str(), allocator), allocator);
            Value params(kArrayType);
            WriteMuscleParametersJSON(params, muscle.params, allocator);
            muscle_object.AddMember("parameter", params, allocator);
            muscle_object.AddMember("body_1", Value(muscle.body_1->GetName().c_str(), allocator), allocator);
            muscle_object.AddMember("body_2", Value(muscle.body_2->GetName().c_str(), allocator), allocator);
            Value loc_1(kArrayType);
            WriteVector3dJSON(loc_1, muscle.loc_1, allocator);
            muscle_object.AddMember("attachment_1", loc_1, allocator);
            Value loc_2(kArrayType);
            WriteVector3dJSON(loc_2, muscle.loc_2, allocator);
            muscle_object.AddMember("attachment_2", loc_2, allocator);
            muscle_object.AddMember("local_coordinates", muscle.local_coordinates, allocator);
            muscle_object.AddMember("rel_to_ref", muscle.rel_to_ref, allocator);
            Value color(kArrayType);
            WriteColorJSON(color, muscle.color, allocator);
            muscle_object.AddMember("color", color, allocator);
            /*Value visual_shape(kArrayType);
            auto shape = std::dynamic_pointer_cast<ChVisualShapeSpring>(muscle.shape);
            WriteVisualShapeSpringJSON(visual_shape, shape, allocator);
            muscle_object.AddMember("visual_shape", visual_shape, allocator);*/
            muscle_properties_array.PushBack(muscle_object, allocator);
        }
        d.AddMember("muscle_properties", muscle_properties_array, allocator);

        WriteFileJSON(filename, d);
    }

    void WriteLigamentPropertiesJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;
        d.SetObject();
        auto& allocator = d.GetAllocator();

        d.AddMember("name", Value(filename.c_str(), allocator), allocator);
        d.AddMember("type", "Ligament", allocator);

        // Add ligament properties
        ChLigament::ChLigamentEnumMapper::ChLigamentType_mapper ligament_mapper;
        d.AddMember("ligament_type", Value(ligament_mapper(jaw.LigamentType()).GetValueAsString().c_str(), allocator),
                    allocator);
        d.AddMember("ligaments_verbose", jaw.LigamentsVerbose(), allocator);
        d.AddMember("tmj_capsule_ligaments_verbose", jaw.TMJCapsuleLigamentsVerbose(), allocator);
        d.AddMember("stiff_ligaments", jaw.StiffLigaments(), allocator);
        d.AddMember("stiff_tmj_capsule_ligaments", jaw.StiffTMJCapsuleLigaments(), allocator);

        Value ligament_properties_array(kArrayType);
        for (const auto& ligament: jaw.LigamentProperties()) {
            Value ligament_object(kObjectType);
            ligament_object.AddMember("name", Value(ligament.name.c_str(), allocator), allocator);
            Value params(kArrayType);
            WriteLigamentParametersJSON(params, ligament.params, allocator);
            ligament_object.AddMember("parameter", params, allocator);
            ligament_object.AddMember("body_1", Value(ligament.body_1->GetName().c_str(), allocator), allocator);
            ligament_object.AddMember("body_2", Value(ligament.body_2->GetName().c_str(), allocator), allocator);
            Value loc_1(kArrayType);
            WriteVector3dJSON(loc_1, ligament.loc_1, allocator);
            ligament_object.AddMember("attachment_1", loc_1, allocator);
            Value loc_2(kArrayType);
            WriteVector3dJSON(loc_2, ligament.loc_2, allocator);
            ligament_object.AddMember("attachment_2", loc_2, allocator);
            ligament_object.AddMember("local_coordinates", ligament.local_coordinates, allocator);
            ligament_object.AddMember("rel_to_ref", ligament.rel_to_ref, allocator);
            Value color(kArrayType);
            WriteColorJSON(color, ligament.color, allocator);
            ligament_object.AddMember("color", color, allocator);
            Value visual_shape(kArrayType);
            auto shape = std::dynamic_pointer_cast<ChVisualShapeSpring>(ligament.shape);
            WriteVisualShapeSpringJSON(visual_shape, shape, allocator);
            ligament_object.AddMember("visual_shape", visual_shape, allocator);
            ligament_properties_array.PushBack(ligament_object, allocator);
        }
        d.AddMember("ligament_properties", ligament_properties_array, allocator);

        Value capsule_ligament_properties_array(kArrayType);
        for (const auto& capsule_ligament: jaw.CapsuleLigamentProperties()) {
            Value ligament_object(kObjectType);
            ligament_object.AddMember("name", Value(capsule_ligament.name.c_str(), allocator), allocator);
            Value params(kArrayType);
            WriteLigamentParametersJSON(params, capsule_ligament.params, allocator);
            ligament_object.AddMember("parameter", params, allocator);
            ligament_object.AddMember("body_1", Value(capsule_ligament.body_1->GetName().c_str(), allocator),
                                      allocator);
            ligament_object.AddMember("body_2", Value(capsule_ligament.body_2->GetName().c_str(), allocator),
                                      allocator);
            Value loc_1(kArrayType);
            WriteVector3dJSON(loc_1, capsule_ligament.loc_1, allocator);
            ligament_object.AddMember("attachment_1", loc_1, allocator);
            Value loc_2(kArrayType);
            WriteVector3dJSON(loc_2, capsule_ligament.loc_2, allocator);
            ligament_object.AddMember("attachment_2", loc_2, allocator);
            ligament_object.AddMember("local_coordinates", capsule_ligament.local_coordinates, allocator);
            ligament_object.AddMember("rel_to_ref", capsule_ligament.rel_to_ref, allocator);
            Value color(kArrayType);
            WriteColorJSON(color, capsule_ligament.color, allocator);
            ligament_object.AddMember("color", color, allocator);
            Value visual_shape(kArrayType);
            auto shape = std::dynamic_pointer_cast<ChVisualShapeSpring>(capsule_ligament.shape);
            WriteVisualShapeSpringJSON(visual_shape, shape, allocator);
            ligament_object.AddMember("visual_shape", visual_shape, allocator);
            capsule_ligament_properties_array.PushBack(ligament_object, allocator);
        }
        d.AddMember("capsule_ligament_properties", capsule_ligament_properties_array, allocator);

        WriteFileJSON(filename, d);
    }

    void WriteFEAPropertiesJSON(ChJaw& jaw, const std::string& filename) {
        rapidjson::Document d;
        d.SetObject();
        auto& allocator = d.GetAllocator();

        d.AddMember("name", Value(filename.c_str(), allocator), allocator);
        d.AddMember("type", "FEA", allocator);

        // Material properties
        Value material_properties(kObjectType);
        material_properties.AddMember("tmj_disc_fea_use_mooney_rivlin", jaw.TMJDiscFEAUseMooneyRivlin(), allocator);
        material_properties.AddMember("tmj_disc_fea_mr_c_1", jaw.TMJDiscFEAMRC1(), allocator);
        material_properties.AddMember("tmj_disc_fea_mr_c_2", jaw.TMJDiscFEAMRC2(), allocator);
        material_properties.AddMember("tmj_disc_fea_density", jaw.TMJDiscFEADensity(), allocator);
        material_properties.AddMember("tmj_disc_fea_young_modulus", jaw.TMJDiscFEAYoungModulus(), allocator);
        material_properties.AddMember("tmj_disc_fea_poisson_ratio", jaw.TMJDiscFEAPoissonRatio(), allocator);
        material_properties.AddMember("tmj_disc_fea_rayleigh_damping_alpha", jaw.TMJDiscFEARayleighDampingAlpha(),
                                      allocator);
        material_properties.AddMember("tmj_disc_fea_rayleigh_damping_beta", jaw.TMJDiscFEARayleighDampingBeta(),
                                      allocator);
        d.AddMember("material_properties", material_properties, allocator);

        // Collision properties
        Value collision_properties(kObjectType);

        Value properties_smc(kArrayType);
        Value properties_nsc(kArrayType);
        WriteContactMaterialPropertiesSMCJSON(properties_smc, jaw.TMJDiscPropertiesSMC(), allocator);
        WriteContactMaterialPropertiesNSCJSON(properties_nsc, jaw.TMJDiscPropertiesNSC(), allocator);
        collision_properties.AddMember("tmj_disc_properties_smc", properties_smc, allocator);
        collision_properties.AddMember("tmj_disc_properties_nsc", properties_nsc, allocator);
        collision_properties.AddMember("tmj_disc_mesh_swept_sphere_thickness", jaw.TMJDiscMeshSweptSphereThickness(),
                                       allocator);
        collision_properties.AddMember("tmj_disc_obj_file_coll", Value(jaw.TMJDiscObjFileColl().c_str(), allocator),
                                       allocator);
        d.AddMember("collision_properties", collision_properties, allocator);

        // Auxiliary bodies
        Value auxiliary_bodies(kObjectType);
        auxiliary_bodies.AddMember("tmj_disc_fea_aux_body_radius", jaw.TMJDiscFEAAuxBodyRadius(), allocator);
        d.AddMember("auxiliary_bodies", auxiliary_bodies, allocator);

        // Meshes
        Value meshes(kObjectType);
        meshes.AddMember("tmj_disc_fea_element_type", jaw.TMJDiscFEAElementType(), allocator);

        fea::ChGmshMeshFileLoader::ChGmshMeshFileLoaderEnumMapper::ChFEAImplementationType_mapper fea_impl_mapper;
        meshes.AddMember("tmj_disc_fea_implementation",
                         Value(fea_impl_mapper(jaw.TMJDiscFEAImplementation()).GetValueAsString().c_str(), allocator),
                         allocator);

        meshes.AddMember("tmj_disc_left_msh_file_fea", Value(jaw.TMJDiscLeftMshFile().c_str(), allocator), allocator);
        meshes.AddMember("tmj_disc_right_msh_file_fea", Value(jaw.TMJDiscRightMshFile().c_str(), allocator), allocator);
        d.AddMember("meshes", meshes, allocator);

        // Visualization
        Value visualization(kObjectType);
        visualization.AddMember("fea_wireframe", jaw.FEAWireFrame(), allocator);
        visualization.AddMember("fea_smooth_faces", jaw.FEASmoothFaces(), allocator);

        visualization.AddMember("color_scale_min_max", Value(kArrayType).SetArray()
                                .PushBack(jaw.ColorScaleMinMax().first, allocator)
                                .PushBack(jaw.ColorScaleMinMax().second, allocator), allocator);

        visualization.AddMember("fea_vis_data_type",
                                Value(chrono::utils::EnumToString(jaw.FEAVisDataType(),
                                                                  chrono::utils::visual_shape_fea_enum_conversion_map).
                                      c_str(), allocator),
                                allocator);
        d.AddMember("visualization", visualization, allocator);

        WriteFileJSON(filename, d);
    }

    // -----------------------------------------------------------------------------

    ChVector3d ReadVector3dJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 3, "[ERROR] [ChUtilsJSON] JSON array size != 3!");
        return {a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble()};
    }

    ChVector2d ReadVector2dJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 2, "[ERROR] [ChUtilsJSON] JSON array size != 2!");
        return {a[0u].GetDouble(), a[1u].GetDouble()};
    }

    ChVector2i ReadVector2iJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 2, "[ERROR] [ChUtilsJSON] JSON array size != 2!");
        return {a[0u].GetInt(), a[1u].GetInt()};
    }

    ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 4, "[ERROR] [ChUtilsJSON] JSON array size != 4!");
        return {a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble()};
    }

    ChFrame<> ReadFrameJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.HasMember("Location"),
                                   "[ERROR] [ChUtilsJSON] JSON array does not have a 'Location' member!");
        utils::throw_runtime_error(!a.HasMember("Orientation"),
                                   "[ERROR] [ChUtilsJSON] JSON array does not have a 'Orientation' member!");
        return {ReadVector3dJSON(a["Location"]), ReadQuaternionJSON(a["Orientation"])};
    }

    ChColor ReadColorJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 3, "[ERROR] [ChUtilsJSON] JSON array size != 3!");
        return {a[0u].GetFloat(), a[1u].GetFloat(), a[2u].GetFloat()};
    }

    std::shared_ptr<ChVisualShapeSpring> ReadVisualShapeSpringJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 3, "[ERROR] [ChUtilsJSON] JSON array size != 3!");
        return chrono_types::make_shared<ChVisualShapeSpring>(a[0u].GetDouble(), a[1u].GetInt(), a[2u].GetDouble());
    }

    ChMuscle::ChMuscleParameterVector ReadMuscleParametersJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        ChMuscle::ChMuscleParameterVector parameters;

        for (const auto& p: a.GetArray()) {
            if (p.IsBool()) {
                parameters.emplace_back(p.GetBool());
            } else {
                parameters.emplace_back(p.GetDouble());
            }
        }
        return parameters;
    }

    ChLigament::ChLigamentParameterVector ReadLigamentParametersJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        ChLigament::ChLigamentParameterVector parameters;

        for (const auto& p: a.GetArray()) {
            if (p.IsBool()) {
                parameters.emplace_back(p.GetBool());
            } else {
                parameters.emplace_back(p.GetDouble());
            }
        }
        return parameters;
    }

    ChJaw::ChContactMaterialPropertiesSMC ReadContactMaterialPropertiesSMCJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 11, "[ERROR] [ChUtilsJSON] JSON array size != 11!");
        return {
            a[0u].GetFloat(), a[1u].GetFloat(), a[2u].GetFloat(), a[3u].GetFloat(), a[4u].GetFloat(),
            a[5u].GetFloat(), a[6u].GetFloat(), a[7u].GetFloat(), a[8u].GetFloat(), a[9u].GetFloat(), a[10u].GetFloat()
        };
    }

    ChJaw::ChContactMaterialPropertiesNSC ReadContactMaterialPropertiesNSCJSON(const rapidjson::Value& a) {
        utils::throw_runtime_error(!a.IsArray(), "[ERROR] [ChUtilsJSON] JSON value is not an array!");
        utils::throw_runtime_error(a.Size() != 8, "[ERROR] [ChUtilsJSON] JSON array size != 8!");
        return {
            a[0u].GetFloat(), a[1u].GetFloat(), a[2u].GetFloat(), a[3u].GetFloat(),
            a[4u].GetFloat(), a[5u].GetFloat(), a[6u].GetFloat(), a[7u].GetFloat()
        };
    }

    void WriteVector3dJSON(rapidjson::Value& a, const ChVector3d& vec, rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        a.PushBack(vec.x(), allocator);
        a.PushBack(vec.y(), allocator);
        a.PushBack(vec.z(), allocator);
    }

    void WriteVector2dJSON(rapidjson::Value& a, const ChVector2d& vec, rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        a.PushBack(vec.x(), allocator);
        a.PushBack(vec.y(), allocator);
    }

    void WriteVector2iJSON(rapidjson::Value& a, const ChVector2i& vec, rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        a.PushBack(vec.x(), allocator);
        a.PushBack(vec.y(), allocator);
    }

    void WriteQuaternionJSON(rapidjson::Value& a,
                             const ChQuaternion<>& quat,
                             rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        a.PushBack(quat.e0(), allocator);
        a.PushBack(quat.e1(), allocator);
        a.PushBack(quat.e2(), allocator);
        a.PushBack(quat.e3(), allocator);
    }

    void WriteFrameJSON(rapidjson::Value& a, const ChFrame<>& frame, rapidjson::Document::AllocatorType& allocator) {
        a.SetObject();
        rapidjson::Value loc(rapidjson::kArrayType);
        rapidjson::Value orient(rapidjson::kArrayType);
        WriteVector3dJSON(loc, frame.GetPos(), allocator);
        WriteQuaternionJSON(orient, frame.GetRot(), allocator);
        a.AddMember("Location", loc, allocator);
        a.AddMember("Orientation", orient, allocator);
    }

    void WriteColorJSON(rapidjson::Value& a, const ChColor& color, rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        a.PushBack(color.R, allocator);
        a.PushBack(color.G, allocator);
        a.PushBack(color.B, allocator);
    }

    void WriteVisualShapeSpringJSON(rapidjson::Value& a,
                                    const std::shared_ptr<ChVisualShapeSpring>& spring,
                                    rapidjson::Document::AllocatorType& allocator) {
        utils::throw_invalid_argument(spring == nullptr,
                                      "[ERROR] [ChUtilsJSON] Visual shape spring is nullptr!");
        a.SetArray();
        a.PushBack(spring->GetRadius(), allocator);
        a.PushBack(spring->GetResolution(), allocator);
        a.PushBack(spring->GetTurns(), allocator);
    }

    void WriteMuscleParametersJSON(rapidjson::Value& a,
                                   const ChMuscle::ChMuscleParameterVector& params,
                                   rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        for (const auto& p: params) {
            if (std::holds_alternative<bool>(p)) {
                a.PushBack(std::get<bool>(p), allocator);
            } else {
                a.PushBack(std::get<double>(p), allocator);
            }
        }
    }

    void WriteLigamentParametersJSON(rapidjson::Value& a,
                                     const ChLigament::ChLigamentParameterVector& params,
                                     rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        for (const auto& p: params) {
            if (std::holds_alternative<bool>(p)) {
                a.PushBack(std::get<bool>(p), allocator);
            } else {
                a.PushBack(std::get<double>(p), allocator);
            }
        }
    }

    void WriteContactMaterialPropertiesSMCJSON(rapidjson::Value& a,
                                               const ChJaw::ChContactMaterialPropertiesSMC& props,
                                               rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        a.PushBack(props.friction, allocator);
        a.PushBack(props.restitution, allocator);
        a.PushBack(props.young_modulus, allocator);
        a.PushBack(props.poisson_ratio, allocator);
        a.PushBack(props.adhesion, allocator);
        a.PushBack(props.adhesion_s_perko, allocator);
        a.PushBack(props.adhesion_mult_dmt, allocator);
        a.PushBack(props.k_n, allocator);
        a.PushBack(props.g_n, allocator);
        a.PushBack(props.k_t, allocator);
        a.PushBack(props.g_t, allocator);
    }

    void WriteContactMaterialPropertiesNSCJSON(rapidjson::Value& a,
                                               const ChJaw::ChContactMaterialPropertiesNSC& props,
                                               rapidjson::Document::AllocatorType& allocator) {
        a.SetArray();
        a.PushBack(props.friction, allocator);
        a.PushBack(props.restitution, allocator);
        a.PushBack(props.damping, allocator);
        a.PushBack(props.cohesion, allocator);
        a.PushBack(props.compliance, allocator);
        a.PushBack(props.compliance_t, allocator);
        a.PushBack(props.compliance_rolling, allocator);
        a.PushBack(props.compliance_spinning, allocator);
    }

    // -----------------------------------------------------------------------------

    void GetStringMemberWithDefault(std::string& str,
                                    const rapidjson::Value& value,
                                    const char* member,
                                    const char* def) {
        if (value.HasMember(member)) {
            str = value[member].GetString();
        } else {
            str = def;
        }
    }

    std::string ExtractPathAndFilenameWithoutExtension(const std::string& filepath) {
        const std::filesystem::path path_obj(filepath);
        return path_obj.parent_path().string() + "/" + path_obj.stem().string();
    }
} // chrono::biomechanics
