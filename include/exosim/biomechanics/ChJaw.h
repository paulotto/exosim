/**
 * @file ChJaw.h
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

#ifndef EXOSIM_CH_JAW_H
#define EXOSIM_CH_JAW_H

#include <mutex>
#include <thread>
#include <shared_mutex>
//
#include "ExoSimConfig.h"
#include "exosim/biomechanics/ChMuscle.h"
#include "exosim/biomechanics/ChLigament.h"
#include "exosim/biomechanics/utils/utils.h"
#include "exosim/biomechanics/utils/ChGmshMeshFileLoader.h"
#include "exosim/biomechanics/physics/ChContactForceTorqueJawSMC.h"
#include "exosim/biomechanics/physics/ChLinkLockPointSurface.h"
//
#include <chrono/core/ChApiCE.h>
#include <chrono/core/ChClassFactory.h>
//
#include <chrono/serialization/ChArchive.h>
#include <chrono/serialization/ChOutputASCII.h>
#include <chrono/serialization/ChArchiveJSON.h>
#include <chrono/serialization/ChArchiveBinary.h>
//
#include <chrono/timestepper/ChTimestepper.h>
//
#include <chrono/solver/ChSolver.h>
#include <chrono/solver/ChSolverADMM.h>
//
#ifdef CHRONO_PARDISO_MKL
#include <chrono_pardisomkl/ChSolverPardisoMKL.h>
#endif
//
#include <chrono/physics/ChSystem.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChInertiaUtils.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChMarker.h>
#include <chrono/physics/ChContactMaterialNSC.h>
#include <chrono/physics/ChContactMaterialSMC.h>
//
#include <chrono/collision/ChCollisionModel.h>
#include <chrono/collision/ChCollisionShapeTriangleMesh.h>
#include <chrono/collision/bullet/ChCollisionSystemBullet.h>
#include <chrono/collision/multicore/ChCollisionSystemMulticore.h>

#ifdef CHRONO_COLLISION
#include <chrono/collision/multicore/ChCollisionSystemMulticore.h> // Doesn't compile with CMAKE_CXX_STANDARD 20.
#endif
//
#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChLinkNodeFrame.h>
#include <chrono/fea/ChMeshFileLoader.h>
#include <chrono/fea/ChContactSurface.h>
#include <chrono/fea/ChLoadContactSurfaceMesh.h>
#include <chrono/fea/ChContinuumMaterial.h>
#include <chrono/fea/ChElementGeneric.h>
#include <chrono/fea/ChElementHexaANCF_3813.h>
//
#include <chrono/geometry/ChSurface.h>
#include <chrono/geometry/ChSurfaceNurbs.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
//
#include <boost/smart_ptr/make_shared_object.hpp>

#ifdef CHRONO_OPENGL
#include <chrono_opengl/ChVisualSystemOpenGL.h>
#endif
//#include "fmt/chrono.h"
#ifdef CHRONO_IRRLICHT
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#endif
#ifdef CHRONO_VSG
#include <chrono_vsg/ChVisualSystemVSG.h>
#include "exosim/biomechanics/vis/ChVisualJawSystemVSG.h"
#endif


namespace chrono::biomechanics {
    /**
     * @class ChJaw
     * @brief TODO
     */
    class ChApi ChJaw {
        public:
            using mutex_type = std::shared_mutex;
            using read_lock = std::shared_lock<mutex_type>;
            using write_lock = std::unique_lock<mutex_type>;

            /**
             * @enum ChModelType
             * @brief Used to switch between different jaw models.
             */
            enum ChModelType {
                RIGID, ///< Point-on-surface constrained TMJ model.
                RIGID_SURFACE, ///< TMJ constrained by mandibular condyle and fossa mesh surfaces.
                FEM ///< TMJ disc included and modeled with finite elements.
            };

            /**
             * @enum ChVisualizationMode
             * @brief Used to switch between Irrlicht and OpenGL visualization tools.
             */
            enum ChVisualizationMode {
                NONE,
                IRRLICHT,
                OPENGL,
                VSG
            };

            /**
             * @enum ChRenderMode
             * @brief Render mode of the visualization mesh. Note that \p POINT is available for OpenGL only.
             */
            enum ChRenderMode {
                SOLID,
                WIREFRAME,
                POINTS ///< Only for OpenGL
            };

            /**
             * @class ChJawEnumMapper
             * @brief Can be used to convert between enums and strings.
             */
            class ChJawEnumMapper final {
                public:
                    CH_ENUM_MAPPER_BEGIN(ChModelType)
                                CH_ENUM_VAL(RIGID)
                                CH_ENUM_VAL(RIGID_SURFACE)
                                CH_ENUM_VAL(FEM)
                    CH_ENUM_MAPPER_END(ChModelType)

                    CH_ENUM_MAPPER_BEGIN(ChVisualizationMode)
                                CH_ENUM_VAL(NONE)
                                CH_ENUM_VAL(IRRLICHT)
                                CH_ENUM_VAL(OPENGL)
                                CH_ENUM_VAL(VSG)
                    CH_ENUM_MAPPER_END(ChVisualizationMode)

                    CH_ENUM_MAPPER_BEGIN(ChRenderMode)
                                CH_ENUM_VAL(SOLID)
                                CH_ENUM_VAL(WIREFRAME)
                                CH_ENUM_VAL(POINTS)
                    CH_ENUM_MAPPER_END(ChRenderMode)
            };

            /**
             * @struct ChContactMaterialPropertiesSMC
             * @brief A struct for creating a larger amount of SMC contact materials more conveniently.
             */
            struct ChContactMaterialPropertiesSMC {
                float friction{0.0}; ///< [-]
                float restitution{0.0}; ///< [-]
                float young_modulus{0.0}; ///< [Pa]
                float poisson_ratio{0.0}; ///< [-]
                float adhesion{0.0}; ///< [N]
                float adhesion_s_perko{1.0}; ///< [-]
                float adhesion_mult_dmt{1.0}; ///< [-]
                float k_n{0.0}; ///< Normal stiffness coefficient [N/m]
                float g_n{0.0}; ///< Tangential stiffness coefficient [N/m]
                float k_t{0.0}; ///< Normal damping coefficient [N/(m/s)]
                float g_t{0.0}; ///< Tangential damping coefficient [N/(m/s)]
            };

            /**
             * @struct ChContactMaterialPropertiesNSC
             * @brief A struct for creating a larger amount of NSC contact materials more conveniently.
             */
            struct ChContactMaterialPropertiesNSC {
                float friction{0.0}; ///< [-]
                float restitution{0.0}; ///< [-]
                float damping{0.0}; ///< [s]
                float cohesion{0.0}; ///< [N]
                float compliance{0.0}; ///< [m/N] (normal)
                float compliance_t{0.0}; ///< [m/N] (tangential)
                float compliance_rolling{0.0}; ///< If there is no rolling friction, this has no effect [rad/Nm]
                float compliance_spinning{0.0}; ///< If there is no spinning friction, this has no effect [rad/Nm]
            };

            /**
             * @struct ChBodyKinematics
             * @brief A struct for used for storing the kinematics of a ChBody. Container from the standard library are
             * used to create an interface with pybind11 more easily.
             */
            struct ChBodyKinematics {
                double time; ///< [s]
                std::array<double, 3> position; ///< [m]
                std::array<double, 4> rotation; ///< [rad]
                std::array<double, 3> linear_velocity; ///< [m/s]
                std::array<double, 4> angular_velocity; ///< [rad/s]
                std::array<double, 3> linear_acceleration; ///< [m/s^2]
                std::array<double, 4> angular_acceleration; ///< [rad/s^2]
            };

            /**
             * @struct ChSimulationState
             * @brief A struct for storing the state of the simulation, which can be used to reset the simulation.
             */
            struct ChSimulationState {
                double time; ///< [s]
                ChState state; ///< State of the system.
                ChStateDelta state_dt; ///< State derivative of the system.
            };

            /**
             * Base constructor.
             * @param threads Max. number of threads used.
             */
            explicit ChJaw(unsigned int threads = 1);

            /**
             * Destructor.
             */
            virtual ~ChJaw() = default;

            // /**
            //  * Copy constructor.
            //  * @param other The ChJaw object to copy.
            //  */
            // ChJaw(const ChJaw& other);
            //
            // /**
            //  * Move constructor.
            //  * @param other The ChJaw object to move.
            //  */
            // ChJaw(ChJaw&& other) noexcept;
            //
            // /**
            //  * Copy assignment operator.
            //  * @param other The ChJaw object to copy.
            //  * @return A reference to this ChJaw object.
            //  */
            // ChJaw& operator=(const ChJaw& other);
            //
            // /**
            //  * Move assignment operator.
            //  * @param other The ChJaw object to move.
            //  * @return A reference to this ChJaw object.
            //  */
            // ChJaw& operator=(ChJaw&& other) noexcept;

            // /**
            //  * Copy constructor.
            //  * @param other The ChJaw object to copy.
            //  */
            // ChJaw(const ChJaw& other);
            //
            // /**
            //  * Copy assignment operator.
            //  * @param other The ChJaw object to copy.
            //  * @return A reference to this ChJaw object.
            //  */
            // ChJaw& operator=(const ChJaw& other);
            //
            // /**
            //  * Move constructor.
            //  * @param other The ChJaw object to move.
            //  */
            // ChJaw(ChJaw&& other) noexcept;
            //
            // /**
            //  * Move assignment operator.
            //  * @param other The ChJaw object to move.
            //  * @return A reference to this ChJaw object.
            //  */
            // ChJaw& operator=(ChJaw&& other) noexcept;

            /**
             * Builds the model so that the simulation can be started. Can also be used to reset the model.
             * \b NOTE: Needs to be called before \p Simulate() can be executed!
             */
            virtual void Build();

            /**
             * Initializes the system by setting the system type, collision system type, time stepper and
             * solver type, and general parameters.
             * @throws std::invalid_argument If the contact method type is not supported.
             */
            virtual void InitSystem();

            /**
             * Initializes the system components by defining the contact materials, rigid bodies,
             * ligaments and muscles.
             */
            virtual void InitModel();

            /**
             * Simulates the system in a separate thread. Different visualizers can be chosen with the corresponding
             * enum.\n
             * \b NOTE: Call \p Build() before this!
             * @throws std::invalid_argument If the visualization mode is not supported.
             */
            virtual void StartSimulation();

            /**
             * Stops the simulation.
             * @param wait If true, the function waits until the simulation thread has finished. If false, the function
             * will terminate the simulation thread immediately.
             */
            virtual void StopSimulation(bool wait);

            /**
             * Resets the simulation to the initial state by default. The reset state can be changed by using
             * \p ResetTime(), \p ResetState(), and \p ResetStateDt():
             * \code
             * ChJaw::System()->StateGather(ChJaw::ResetState(), ChJaw::ResetStateDt(), ChJaw::ResetTime());
             * \endcode
             */
            virtual void ResetSimulation();

            /**
             * Resets the simulation to a specific state. The state can be defined by:
             * \code
             * ChJaw::ChSimulationState reset_state{
             *  ChJaw::System()->GetChTime(),
             *  ChState(ChJaw::System()->GetNumCoordsPosLevel(), ChJaw::System().get()),
             *  ChStateDelta(ChJaw::System()->GetNumCoordsVelLevel(), ChJaw::System().get())
             * };
             * ChJaw::System()->StateGather(reset_state.state, reset_state.state_dt, reset_state.time);
             * \endcode
             * @param reset_state The state to reset the simulation to.
             */
            virtual void ResetSimulation(const ChSimulationState& reset_state);

            /**
             * Rewinds to the state of the replay buffer at a specific time percentage.
             * 0% rewinds to the current state, 100% rewinds to the start of the replay buffer.
             * @param time_percentage The time percentage at which to rewind the simulation.
             * @throws std::invalid_argument If the time percentage is not in the range [0.0, 100.0].
             */
            virtual void RewindSimulation(double time_percentage);

            /**
             * Captures the simulation state (time, state, state derivative).
             * @param state The parameter to store the simulation state.
             */
            virtual void CaptureSimulationState(ChSimulationState& state);

            /**
             * Loads model parameters from a JSON file. \n
             * \b NOTE: \p Build() must be called on the jaw object \b before
             * (only once after creation of a ChJaw object) and \b after loading the JSON file.
             * @param filename The name of the JSON file (including the path).
             */
            virtual void LoadFromJSON(const std::string& filename);

            /**
             * Writes the model parameters to a JSON file.
             * @param filename The name of the JSON file (including the path).
             * @throws std::runtime_error Function not implemented yet.
             */
            virtual void WriteToJSON(const std::string& filename);

            // ---------------------------------------------------
            // Setter (simple setters are at the end of this file)
            // ---------------------------------------------------

            /**
             * Sets the collision system type.
             * @param type The type of the collision system.
             * @throws std::invalid_argument If the collision system type is not supported.
             */
            virtual void SetCollisionSystemType(ChCollisionSystem::Type type);

            /**
             * Sets the integrator type.
             * @param type The type of the integrator.
             * @throws std::invalid_argument If the time stepper type is not supported.
             */
            virtual void SetTimeStepperType(ChTimestepper::Type type);

            /**
             * Sets the solver type.
             * @param type The type of the solver.
             * @throws std::invalid_argument If the solver type is not supported.
             */
            virtual void SetSolverType(ChSolver::Type type);

            /**
             * @brief Sets the mandible's reference frame in the absolute coordinate system, either directly or indirectly
             *        through a relative transformation.
             *
             * This method allows setting the mandible's state in two ways:
             * 1. **Direct Setting**: If the provided `ref_frame_local` is the default frame (`ChFrameMoving<>()`),
             *    the method assumes that the provided `state_abs` directly defines the absolute pose of the mandible's
             *    reference frame. In this case, the mandible's reference frame is directly set to `state_abs`.
             *
             * 2. **Relative Transformation**: If the provided `ref_frame_local` is not the default frame, it is assumed
             *    that `state_abs` defines the absolute pose of a different point on the mandible, rather than the
             *    mandible's reference frame. In this case, the relative transformation from this other point to the
             *    mandible's reference frame (provided by `ref_frame_local`) is used to compute the absolute pose of the
             *    mandible's reference frame. This computed pose is then applied to the mandible's reference frame.
             *
             * Regardless of the method used, the following steps are performed:
             * - The relative transformation from the provided local frame (`ref_frame_local`) to the absolute frame is
             *   applied to compute the new pose of the mandible's reference frame in the absolute frame.
             * - The mandible's reference frame is updated with the computed pose, including its first and second
             *   derivatives (velocity and acceleration).
             * - A full system assembly analysis is performed to ensure that the new state is feasible. If the assembly
             *   fails to converge, an exception is thrown.
             *
             * @param state_abs The absolute pose (position, orientation, velocity, and acceleration) of either:
             *                  - The mandible's reference frame (if `ref_frame_local` is the default frame).
             *                  - Another point on the mandible (if `ref_frame_local` is not the default frame).
             * @param ref_frame_local The local transformation from the provided point (`state_abs`) to the mandible's
             *                       reference frame. If this parameter is the default frame (`ChFrameMoving<>()`),
             *                       the method assumes that `state_abs` directly defines the absolute pose of the
             *                       mandible's reference frame.
             *
             * @throws std::runtime_error If the new mandible state is not feasible (e.g., due to a failure in the
             *                            assembly analysis).
             *
             * ### Example Usage:
             * Directly setting the mandible's reference frame:
             * @code
             * ChJaw jaw;
             * ChFrameMoving<> mandible_pose_abs;
             * jaw.SetMandibleState(mandible_pose_abs, ChFrameMoving<>());
             * @endcode
             *
             * Setting the mandible's reference frame via a relative transformation:
             * @code
             * ChJaw jaw;
             * ChFrameMoving<> point_on_mandible_abs; // Absolute pose of another point on the mandible
             * ChFrameMoving<> relative_to_ref_frame; // Relative transformation to the mandible's reference frame
             * jaw.SetMandibleState(point_on_mandible_abs, relative_to_ref_frame);
             * @endcode
             */
            virtual void SetMandibleState(const ChFrameMoving<>& state_abs, const ChFrameMoving<>& ref_frame_local);

            /**
             * Sets the muscle excitation by muscle name.
             * @param name The name of the muscle.
             * @param excitation The muscle excitation function.
             */
            virtual void SetMuscleExcitation(const std::string& name, std::shared_ptr<ChFunction> excitation) {
                try {
                    muscle_map_.at(name)->SetExcitation(std::move(excitation));
                } catch (std::out_of_range& e) {
                    fmt::print(utils::WARNING_MSG, "[WARNING] [ChJaw] Muscle '{}' not found in jaw model!\n", name);
                }
            }

            // ---------------------------------------------------
            // Getter (simple getters are at the end of this file)
            // ---------------------------------------------------

            /**
             * Gets a bone by its name.
             * @param name The name of the bone.
             * @return A shared pointer to the bone body.
             * @throws std::invalid_argument If the bone is not found.
             */
            std::shared_ptr<ChBodyAuxRef>& GetBoneBody(const std::string& name) {
                if (name == SKULL_NAME_) return skull_;
                if (name == MAXILLA_NAME_) return maxilla_;
                if (name == MANDIBLE_NAME_) return mandible_;
                if (name == HYOID_NAME_) return hyoid_;
                utils::throw_invalid_argument(true, "[ERROR] [ChJaw] Bone with name '{}' not found!\n", name);
                return skull_;
            }

            /**
             * Gets a TMJ capsule ligament auxiliary body.
             * @param left If the left TMJ is to be used.
             * @param index The index of the TMJ capsule ligament (0-3).
             * @return A shared pointer to the TMJ capsule ligament auxialiary body.
             */
            std::shared_ptr<ChBody>& GetTMJDiscAuxBody(bool left, unsigned int index) {
                if (left) {
                    return tmj_disc_left_fea_aux_bodies_.at(index);
                } else {
                    return tmj_disc_right_fea_aux_bodies_.at(index);
                }
            }

            /**
             * Gets the muscle excitation by muscle name. Thread safety needs to be ensured by the user while reading
             * or writing to the muscle excitation functions.
             * @param name The name of the muscle.
             * @return A shared pointer to the muscle excitation function.
             * @throws std::invalid_argument If the muscle name is not found.
             */
            virtual std::shared_ptr<ChFunction> GetMuscleExcitation(const std::string& name) const {
                try {
                    return muscle_map_.at(name)->GetExcitation();
                } catch (std::out_of_range& e) {
                    utils::throw_invalid_argument(true,
                                                  "[ERROR] [ChJaw] Muscle '{}' not found in jaw model!\n", name);
                }
                return muscle_map_.at("posterior_temporalis_left")->GetExcitation(); // Won't be reached, therefore OK
            }

            /**
             * Get the incisal point's kinematics. Thread safety needs to be ensured by the user while reading or
             * writing.
             * @return \p ChBodyKinematics struct containing the incisal point's kinematics.
             */
            virtual const ChBodyKinematics& GetMandibleKinematics() {
                CaptureMandibleKinematics();

                read_lock lock(mtx_);
                return mandible_kinematics_;
            }

            /**
             * Get general information about the simulation system.
             * @param info The string to which the information is appended.
             */
            virtual void InfoString(std::string& info);

            // -------------
            // Serialization
            // -------------

            /**
             * Writes the system state comprising position and orientation, velocity, acceleration,
             * time, and Lagrange multipliers to a file. \n
             * Reference: \link https://groups.google.com/g/projectchrono/c/ifCUsoUjFsI \endlink
             * @param archive_out ChArchiveOut.
             */
            virtual void WriteState(ChArchiveOut& archive_out);

            /**
             * Reads the system state comprising position and orientation, velocity, acceleration,
             * time, and Lagrange multipliers from a file. \n
             * Reference: \link https://groups.google.com/g/projectchrono/c/ifCUsoUjFsI \endlink
             * @param archive_in ChArchiveIn.
             */
            virtual void ReadState(ChArchiveIn& archive_in);

            /**
             * Method to allow serialization of transient data to archives.
             * @param archive_out ChArchiveOut.
             */
            virtual void ArchiveOut(ChArchiveOut& archive_out);

            /**
             * Method to allow serialization of transient data from archives.
             * @param archive_in ChArchiveIn.
             */
            virtual void ArchiveIn(ChArchiveIn& archive_in);

        protected:
            /**
             * @struct AttachNode
             * A struct for storing a node of an FEA mesh and the nodes in a radius around it.
             * @tparam Node The node type.
             */
            template<class Node>
            struct AttachNode {
                std::shared_ptr<Node> center_node{chrono_types::make_shared<Node>()};
                std::vector<std::shared_ptr<Node> > nodes_around{};
                std::shared_ptr<ChBody> aux_body{nullptr};
            };

            /**
             * Simulates the system. Different visualizers can be chosen with the corresponding enum.\n
             * \b NOTE: Call \p Build() before this!
             * @param end_time The time at which the simulation ends. If negative, the simulation runs indefinitely.
             * @throws std::invalid_argument If the visualization mode is not supported.
             */
            virtual void Simulate(double end_time);

            /**
             * Simulates the system. Different visualizers can be chosen with the corresponding enum.
             * Simulation duration can be set with \p SetSimulationTimeLimit(). \n
             * \b NOTE: Call \p Build() before this!
             */
            void Simulate() {
                Simulate(simulation_time_limit_);
            }

            /**
             * Defines the contact material properties for the system components.
             * @throws std::invalid_argument If the contact model type is not supported.
             */
            virtual void DefineContactMaterials();

            /**
             * Defines the meshes used for the finite-elements analysis.
             */
            virtual void DefineFEAMeshes();

            /**
             * Find the nodes of a \p ChMesh that are within a certain radius of a given node.
             * @param mesh The \p ChMesh containing the nodes.
             * @param center_node The node around which the search is performed.
             * @param radius The radius within which the nodes are searched.
             * @return A vector of shared pointers to the nodes within the radius.
             */
            virtual std::vector<std::shared_ptr<fea::ChNodeFEAxyz> > FindNodesWithinRadius(fea::ChMesh& mesh,
                const std::shared_ptr<fea::ChNodeFEAxyz>& center_node,
                double radius);

            /**
             * Find the closest node in a \p ChMesh to a given position.
             * @param mesh The \p ChMesh containing the nodes.
             * @param position The position around which the search is performed (absolute coordinates).
             * @return A shared pointer to the closest node.
             */
            virtual std::shared_ptr<fea::ChNodeFEAxyz> FindClosestNode(fea::ChMesh& mesh, const ChVector3d& position);

            /**
             * @brief Finds the two nodes for each dimension (X, Y, Z) that are close to the extrema in that dimension
             * and have coordinates of the other two dimensions close to the middle values of these other directions.
             *
             * This function identifies nodes in an FEA mesh that are near the min and max values in one dimension
             * (X, Y, Z) while ensuring their coordinates in the other two dimensions are close to the middle values.
             * It also finds nodes within a specified radius around these identified middle nodes.
             *
             * @param mesh The finite element analysis (FEA) mesh to search within.
             * @param radius The radius within which to find nodes around the identified middle nodes.
             * @return A vector of AttachNode structs containing the middle nodes and the surrounding nodes.
             * Order: (min, max) for each dimension (X, Y, Z).
             */
            virtual std::vector<AttachNode<fea::ChNodeFEAxyz> > FindMiddleNodes(fea::ChMesh& mesh, float radius);

            /**
             * Initializes the auxiliary bodies used for the finite element analysis since a \p ChLinkTSDA cannot be
             * attached to a \p ChBodyFrame.
             * @throws std::invalid_argument If no absolute coordinates for the attachment points of the TMJ capsule
             * ligaments were used.
             */
            virtual void InitFEAAuxiliaryBodies();

            /**
             * Defines the meshes used for the collision detection and visualization.
             * @param body The body to which the meshes are attached.
             * @param density The density of the body.
             * @param mesh_material The contact material of the body.
             * @param obj_file_vis The OBJ file for the visualization.
             * @param obj_file_coll The OBJ file for the collision detection.
             * @param enable_collision If collision is to be enabled for the given body.
             * @param mesh_static True if the mesh is static.
             * @param mesh_convex True if the mesh is convex.
             * @param mesh_swept_sphere_thickness The thickness of the swept sphere. Used to blow up the mesh
             * for more robust collision detection.
             * @param color The color of the visualization mesh.
             * @throws std::runtime_error When a mesh file was not found.
             */
            virtual void DefineBodyCollisionMeshes(const std::shared_ptr<ChBodyAuxRef>& body,
                                                   double density,
                                                   std::shared_ptr<ChContactMaterial>& mesh_material,
                                                   const std::string& obj_file_vis,
                                                   const std::string& obj_file_coll,
                                                   bool enable_collision,
                                                   bool mesh_static,
                                                   bool mesh_convex,
                                                   double mesh_swept_sphere_thickness,
                                                   const ChColor& color);

            /**
             * Defines the point on a surface constraints for the rigid body model's TMJs.
             * Not used for the FEA model.
             */
            virtual void DefineRigidTMJConstraints();

            /**
             * Defines the ligaments.
             * @param type The type of the ligament.
             * @throws std::invalid_argument If the ligament type is not supported.
             * @throws std::runtime_error If an attachment body couldn't be cast to \p ChBodyAuxRef.
             */
            virtual void DefineLigaments(ChLigament::ChLigamentType type);

            /**
             * Defines the muscles.
             * @param type The type of the muscle.
             * @throws std::invalid_argument If the muscle type is not supported.
             * @throws std::runtime_error If an attachment body couldn't be cast to \p ChBodyAuxRef.
             */
            virtual void DefineMuscles(ChMuscle::ChMuscleType type);

            /**
             * Defines the contact behavior between two objects. This makes it possible to define multiple contact
             * behaviors for a single body depending on the object with which it collides.
             */
            virtual void DefineObjectObjectContactBehavior() const;

            /**
             * Captures the incisal point's kinematics (position, rotation, velocity, acceleration)
             * and stores it in the variable \p mandible_kinematics_.
             */
            virtual void CaptureMandibleKinematics();

        private:
            // ------------------
            // General parameters
            // ------------------

            ChModelType model_type_{RIGID}; ///< The type of the jaw model

            std::string name_{"Biomechanical Jaw Model"}; ///< Name of the jaw model

            bool build_called_{false}; ///< Indicates if the model was built

            // bool enable_rigid_body_model_{false}; ///< Enable rigid body model

            bool enable_solver_debugging_{false}; ///< Write solver debugging information to file
            bool enable_system_debugging_{false}; ///< Write system debugging information to file

            unsigned int threads_{1}; ///< Max. number of threads used

            double gravity_{9.81}; ///< [m/s²]
            double time_step_{1.0e-3}; ///< [s]

            mutable mutex_type mtx_;
            std::thread simulation_thread_;
            bool run_simulation_{true}; ///< Run the simulation while true
            bool update_simulation_{true}; ///< Update the simulation while true TODO: add getter & constructor
            /// The time limit for the simulation [s]. If negative, the simulation runs indefinitely
            double simulation_time_limit_{-1.0};

            ChSimulationState reset_simulation_state_; ///< The simulation state to which the simulation is reset

            unsigned int replay_buffer_size_{60000}; ///< Size of the replay buffer TODO: add getter & constructor v
            utils::CircularBuffer<ChSimulationState> replay_buffer_{60000}; ///< Circular buffer for simulation states

            // ----------
            // Integrator
            // ----------

            bool time_stepper_verbose_{false};
            bool hht_step_control_{true};

            int euler_i_max_iters_{10};
            int trapezoidal_lin_max_iters_{15};
            int newmark_max_iters_{10};
            int hht_max_iters_{15}; ///< Max number of iterations using the Newton Raphson procedure

            double hht_alpha_{-0.2};

            ///- \p SMC (DAE): \p EULER_IMPLICIT (1st), \p HHT (2nd), \p NEWMARK (1st)
            ///- \p NSC (DVI/CCP/MLCP): \p EULER_IMPLICIT_LINEARIZED (1st), [\p TRAPEZOIDAL_LINEARIZED (2nd)]
            ChTimestepper::Type time_stepper_type_{ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED};

            // ------
            // Solver
            // ------

            bool solver_verbose_{false};

            double solver_tolerance_{1.0e-8};

            ///< While using iterative solvers it is highly recommended, especially in case of systems with
            ///< links/constraints, to increase the number of iterations until the links do not get violated anymore
            int max_solver_iterations_{300};

            /// DAE: \p PARDISO_MKL (direct solver), \p MUMPS (direct solver), \p MINRES
            ///    - \p MINRES: Krylov, only linear (no CCP), can be troublesome if large mass-ratios
            ///    - \p PARDISO_MKL: parallel direct solver, no CCP (+ \p HHT -> best for FEA)
            ///    - \p MUMPS: parallel direct solver, no CCP
            /// DVI/CCP/MLCP:
            ///    - Krylov and spectral methods (linear + CCP, better convergence than fixed-point):
            ///        - \p BARZILAIBORWEIN (+ \p EULER_IMPLICIT_LINEARIZED -> DAE or DVI)
            ///        - \p APGD (+ \p EULER_IMPLICIT_LINEARIZED -> DAE or DVI)
            ///        - \p ADMM (can handle both FEA and NSC!) (can be combined with \p Pardiso_MKL)
            ///    - Projected fixed-point methods (linear + CCP, robust, very slow convergence):
            ///        - \p PSOR (+ \p EULER_IMPLICIT_LINEARIZED -> DAE or DVI)
            ///        - \p PSSOR [deprecated]
            ///        - \p PJACOBI
            ChSolver::Type solver_type_{ChSolver::Type::ADMM};

            // ADMM
            /// Enable this option only if using the Euler implicit linearized integrator!
            bool admm_warm_start_{true};

            /// Initial ADMM step. Could change later if adaptive step is used
            double admm_rho_{1.0};
            /// Absolute tolerance for the dual residual (constraint speed error), if the iteration falls below this
            /// and the primal tolerance, it stops iterating
            double admm_tolerance_dual_{1.0e-6};
            /// Absolute tolerance for the primal residual (force impulses error), if the iteration falls below this
            /// and the dual tolerance, it stops iterating
            double admm_tolerance_primal_{1.0e-6};

            ///- ADMM: Alternating Direction Method of Multipliers
            ///- Step adjust policy: \p NONE, \p BALANCED_UNSCALED, \p BALANCED_FAST (default), \p BALANCED_RANGE
            ChSolverADMM::AdmmStepType admm_step_policy_{ChSolverADMM::AdmmStepType::BALANCED_FAST};
            /// Enhancements: \p BASIC (default), \p NESTEROV
            ChSolverADMM::AdmmAcceleration admm_acceleration_{ChSolverADMM::AdmmAcceleration::NESTEROV};

            // PARDISO_MKL
            bool mkl_lock_sparsity_pattern_{true};
            bool mkl_use_sparsity_pattern_learner_{true};

            // NSC specifications TODO: Only NSC?
            //        uint max_iteration_normal_{0};
            //        uint max_iteration_sliding_{0};
            //        uint max_iteration_spinning_{100};
            //        uint max_iteration_bilateral_{0};
            //
            //        real alpha_{0.0};

            //        SolverMode nsc_solver_mode_{SolverMode::SPINNING};

            //        SolverMode smc_solver_mode_{SolverMode::SPINNING};

            // -------------
            // Visualization
            // -------------

            bool run_sim_at_startup_{false}; ///< Run the simulation at startup of the visualizer

            uint window_width_{1920};
            uint window_height_{1080};

            std::string window_title_{"=== Human Jaw Model ==="};

            ChVisualizationMode vis_mode_{ChJaw::ChVisualizationMode::VSG};
            ChRenderMode render_mode_{ChJaw::ChRenderMode::SOLID};

            ChColor bone_color_{0.5f, 0.5f, 0.5f};
            ChColor background_color_{0.15f, 0.15f, 0.15f}; // 0.55f, 0.63f, 0.75f

            // ----------------
            // Collision system
            // ----------------

            // General

            double collision_margin_{1.0e-3}; ///< \p ChCollisionModelMulticore: will be set to 0 automatically [m]
            double collision_envelope_{3.0e-5}; ///< \p SMC: automatically set to 0 when initializing \p ChSystemSMC [m]

            /// Extrudes a surface mesh to 3D space (for more robust contact detection) [m]
            double bone_mesh_swept_sphere_thickness_{1.0e-4};
            double tmj_disc_mesh_swept_sphere_thickness_{1.0e-4};

            std::shared_ptr<ChContactForceTorqueJawSMC::ContactBehaviorMap> contact_behavior_map_{
                std::make_shared<ChContactForceTorqueJawSMC::ContactBehaviorMap>()
            };

            ///- \p SMC (DAE):
            ///     - Hard constraints won't work (e.g. \p ChLinkLockPointSurface limits)
            ///     - FEA and penalty-based contact lead to stiff differential equations --> very small time steps (\p HHT)
            ///- \p NSC (DVI/CCP/MLCP)
            ///     - Set 'collision_envelope_=3.0e-5' & 'collision_margin_=1.0e-3'
            ChContactMethod contact_method_{ChContactMethod::NSC};

            /// \p MULTICORE:
            ///     - Doesn't work with FEA meshes yet!
            ///     - Doesn't allow for high young's modulus!
            ChCollisionSystem::Type collision_type_{ChCollisionSystem::Type::BULLET};
            ChNarrowphase::Algorithm narrowphase_algorithm_multicore_{ChNarrowphase::Algorithm::HYBRID};

            // Multicore

            ChVector3i broadphase_grid_resolution_multicore_{10, 10, 10};

            // Bullet

            double contact_breaking_threshold_bullet_{1.0e-2}; ///< [m]

            // SMC

            /// Curvature radius of the contact surface [m]
            double default_effective_curvature_radius_smc_{1.0e-6};

            ChContactForceTorqueJawSMC::ContactForceTorqueModel contact_force_model_smc_{
                ChContactForceTorqueJawSMC::Hertz
            };
            ChContactForceTorqueJawSMC::AdhesionForceTorqueModel adhesion_force_model_smc_{
                ChContactForceTorqueJawSMC::Constant
            };
            ChContactForceTorqueJawSMC::TangentialDisplacementModel tangential_displ_model_smc_{
                ChContactForceTorqueJawSMC::OneStep
            }; ///< \p MultiStep not implemented yet

            // NSC

            /// Minimum rebounce speed for elastic collision
            double min_bounce_speed_nsc_{0.01};
            /// Used for unilateral constraints with \p EULER_IMPLICIT_LINEARIZED.
            /// When using compliance, exp. for large compliances, the max. penetration recovery speed
            /// also affects reaction forces, thus it must be deactivated (or used as a very large value: 100000)
            double contact_recovery_speed_nsc_{1000000}; /// 0.01

            // -------------------
            // Material properties
            // -------------------

            float bone_density_{3214.529403}; ///< [kg/m³]

            std::shared_ptr<ChContactMaterial> bone_contact_material_{nullptr};
            std::shared_ptr<ChContactMaterial> tmj_disc_contact_material_{nullptr};

            // SMC

            /// Use material properties to compute the paramaters of the contact model
            bool use_material_properties_smc_{true};

            ChContactMaterialPropertiesSMC bone_properties_smc_{
                0.1, 0.1, 20.0e9, 0.1, 0.05, 1.0, 1.0
            };

            ChContactMaterialPropertiesSMC tmj_disc_properties_smc_{
                0.1, 0.1, 9.0e5, 0.1, 0.1, 1.0, 1.0
            };

            // NSC

            ChContactMaterialPropertiesNSC bone_properties_nsc_{
                0.1, 0.1, 0.2, 0.0, 9.0e-9, 0.0, 0.0, 0.0
            };

            ChContactMaterialPropertiesNSC tmj_disc_properties_nsc_{
                0.1, 0.1, 0.2, 0.0, 9.0e-9, 0.0, 0.0, 0.0
            };

            // FEA

            bool tmj_disc_fea_use_mooney_rivlin_{true}; ///< Only for the \p ChElementHexaANCF_3813 element

            double tmj_disc_fea_mr_c_1_{9.0e5}; ///< 1st coefficient for Mooney-Rivlin material model [Pa]
            double tmj_disc_fea_mr_c_2_{9.0e2}; ///< 2nd coefficient for Mooney-Rivlin material model [Pa]

            double tmj_disc_fea_density_{1000.0}; ///< [kg/m³]
            double tmj_disc_fea_young_modulus_{9.0e5}; ///< [Pa]
            double tmj_disc_fea_poisson_ratio_{0.3}; ///< [-]
            double tmj_disc_fea_rayleigh_damping_alpha_{0.0}; ///< [-]
            double tmj_disc_fea_rayleigh_damping_beta_{0.1}; ///< [-]

            std::shared_ptr<fea::ChContinuumMaterial> tmj_disc_fea_material_{nullptr};

            // -----------------
            // System components
            // -----------------

            // System
            std::shared_ptr<ChSystem> sys_{nullptr};

            // FEA auxiliary bodies

            /// Used to create a sphere around the auxiliary bodies determining the nodes to attach to the bodies [m].
            float tmj_disc_fea_aux_body_radius_{1.5e-3};

            const std::string TMJ_DISC_LEFT_FEA_AUX_PREFIX_{"tmj_disc_left_fea_aux_bodies_"};
            const std::string TMJ_DISC_RIGHT_FEA_AUX_PREFIX_{"tmj_disc_right_fea_aux_bodies_"};

            std::array<std::shared_ptr<ChBody>, 4> tmj_disc_left_fea_aux_bodies_{
                chrono_types::make_shared<ChBody>(),
                chrono_types::make_shared<ChBody>(),
                chrono_types::make_shared<ChBody>(),
                chrono_types::make_shared<ChBody>()
            };
            std::array<std::shared_ptr<ChBody>, 4> tmj_disc_right_fea_aux_bodies_{
                chrono_types::make_shared<ChBody>(),
                chrono_types::make_shared<ChBody>(),
                chrono_types::make_shared<ChBody>(),
                chrono_types::make_shared<ChBody>()
            };

            std::vector<std::shared_ptr<fea::ChLinkNodeFrame> > tmj_disc_left_fea_aux_links_;
            std::vector<std::shared_ptr<fea::ChLinkNodeFrame> > tmj_disc_right_fea_aux_links_;

            // FEA meshes

            unsigned int tmj_disc_fea_element_type_{4}; ///< 4-node tetrahedron=4, 8-node hexahedron=5
            fea::ChGmshMeshFileLoader::ChFEAImplementationType tmj_disc_fea_implementation_{
                fea::ChGmshMeshFileLoader::COROTATIONAL
            };

            const std::string TMJ_DISC_LEFT_NAME_{"tmj_disc_left"};
            const std::string TMJ_DISC_RIGHT_NAME_{"tmj_disc_right"};

            std::shared_ptr<fea::ChMesh> tmj_disc_left_fea_{nullptr};
            std::shared_ptr<fea::ChMesh> tmj_disc_right_fea_{nullptr};

            // Bones

            const std::string SKULL_NAME_{"skull"};
            const std::string MAXILLA_NAME_{"maxilla"};
            const std::string MANDIBLE_NAME_{"mandible"};
            const std::string HYOID_NAME_{"hyoid"};

            std::shared_ptr<ChBodyAuxRef> skull_{chrono_types::make_shared<ChBodyAuxRef>()};
            std::shared_ptr<ChBodyAuxRef> maxilla_{chrono_types::make_shared<ChBodyAuxRef>()};
            std::shared_ptr<ChBodyAuxRef> mandible_{chrono_types::make_shared<ChBodyAuxRef>()};
            std::shared_ptr<ChBodyAuxRef> hyoid_{chrono_types::make_shared<ChBodyAuxRef>()};

            ChBodyKinematics mandible_kinematics_{};

            // ChVector3d incisal_pos_abs_{-0.000907, -0.100700, 0.090598}; ///< [m]
            ChFramed incisal_frame_abs_{{-0.000907, -0.100700, 0.090598}, QUNIT}; ///< [m, rad] // TODO: Add to JSON
            /// Needed to render a coordinate system in the visualization
            std::shared_ptr<ChBody> incisal_point_aux_body_{chrono_types::make_shared<ChBody>()};
            std::shared_ptr<ChLinkMateFix> incisal_point_link_{chrono_types::make_shared<ChLinkMateFix>()};

            // TMJ constraints (only for rigid-body model)

            bool tmj_constr_verbose_{false}; // TODO: add to constructor
            bool tmj_constr_u_limit_{true};
            bool tmj_constr_v_limit_{true};
            bool tmj_constr_vis_wireframe_{true};
            /// Use relative poses for the TMJ constraints 'tmj_constr_left_' and 'tmj_constr_right_'.
            bool tmj_constr_use_rel_pos_{false};
            /// Test points used in the 1st approximation of the nearest point search of 'ChLinkLockPointSurface'.
            int tmj_constr_point_surface_samples_{50};
            /// Maximum number of iterations for the refinement step of the nearest point search of 'ChLinkLockPointSurface'.
            int tmj_constr_point_surface_max_iters_{11};
            /// Tolerance for the nearest point search of 'ChLinkLockPointSurface'.
            double tmj_constr_point_surface_tolerance_{1.0e-5}; ///< [m]

            ChColor tmj_constr_vis_color_{1.0, 0.84, 0};

            ChVector3d tmj_constr_pos_left_{0.049409, -0.055916, 0.015102}; ///< [m]
            ChVector3d tmj_constr_pos_right_{-0.05222, -0.055916, 0.015102}; ///< [m]
            ChVector3d tmj_constr_cardan_xyz_left_{-41.4, 0, 0}; ///< [°]
            ChVector3d tmj_constr_cardan_xyz_right_{-41.4, 0, 0}; ///< [°]

            std::shared_ptr<ChLinkLockPointSurface> tmj_constr_left_{nullptr};
            std::shared_ptr<ChLinkLockPointSurface> tmj_constr_right_{nullptr};

            ChVector2i tmj_constr_surface_nurbs_order_{1, 2}; ///< (u, v)
            ChVector2i tmj_constr_ctrl_pts_per_dimension_{2, 5}; ///< (u, v)

            std::vector<ChVector3d> tmj_constr_ctrl_pts_surface_nurbs_left_{
                {0.047338, -0.057386, 0.020452},
                {0.047338, -0.05785, 0.019027},
                {0.047338, -0.05475, 0.015634},
                {0.047338, -0.051808, 0.01302},
                {0.047338, -0.051915, 0.012147},
                {0.060477, -0.057386, 0.020452},
                {0.060477, -0.05785, 0.019027},
                {0.060477, -0.05475, 0.015634},
                {0.060477, -0.051808, 0.01302},
                {0.060477, -0.051915, 0.012147}
            };

            std::vector<ChVector3d> tmj_constr_ctrl_pts_surface_nurbs_right_{
                {-0.062662, -0.057386, 0.020452},
                {-0.062662, -0.05785, 0.019027},
                {-0.062662, -0.05475, 0.015634},
                {-0.062662, -0.051808, 0.01302},
                {-0.062662, -0.051915, 0.012147},
                {-0.049523, -0.057386, 0.020452},
                {-0.049523, -0.05785, 0.019027},
                {-0.049523, -0.05475, 0.015634},
                {-0.049523, -0.051808, 0.01302},
                {-0.049523, -0.051915, 0.012147}
            };

            // Ligaments

            bool ligaments_verbose_{false};
            /// Declare the forces generated by the ligaments as stiff.
            /// If stiff, Jacobian information will be generated.
            bool stiff_ligaments_{true};

            ChLigament::ChLigamentType ligament_type_{ChLigament::ChLigamentType::BLANKERVOORT};

            // std::map<std::string, std::shared_ptr<ChLigament> > test_map_{
            // {"l1", chrono_types::make_shared<ChLigamentBlankevoort>()}
            // };
            std::map<std::string, ChLinkTSDA> test_map_{
                // TODO: ChJaw test map
                {"l1", ChLinkTSDA()}
            };

            // std::vector<ChLinkTSDA> test_vec_{ChLinkTSDA()};
            std::vector<std::pair<std::string, ChLinkTSDA> > test_vec_{{"l1", ChLinkTSDA()}};
            std::pair<std::string, ChLinkTSDA> test_pair_{"l1", ChLinkTSDA()};

            std::map<std::string, std::shared_ptr<ChLigament> > ligament_map_;
            /// Used as approximation for the TMJ capsule.
            std::map<std::string, std::shared_ptr<ChLigament> > tmj_capsule_ligament_map_;

            double k_e_{1.0e1};
            double slack_length_factor_{1.3};
            std::vector<ChLigament::ChLigamentProperties> ligament_init_properties_{
                {
                    "stylomandibular_left",
                    {slack_length_factor_ * 0.0321025, 0.1222139, k_e_, 0.75 * 0.1222139, 0.003},
                    mandible_, skull_,
                    ChVector3d(0.045159, -0.099225, 0.012996),
                    ChVector3d(0.039412, -0.072062, 0.000869),
                    true, true
                },
                {
                    "stylomandibular_right",
                    {slack_length_factor_ * 0.0309789, 0.091033, k_e_, 0.75 * 0.091033, 0.003},
                    mandible_, skull_,
                    ChVector3d(-0.046965, -0.098519, 0.013375),
                    ChVector3d(-0.040649, -0.072677, 0.001349),
                    true, true
                },
                {
                    "sphenomandibular_left",
                    {slack_length_factor_ * 0.0353808, 0.0988304, k_e_, 0.75 * 0.0988304, 0.003},
                    mandible_, skull_,
                    ChVector3d(0.043112, -0.086093, 0.02110),
                    ChVector3d(0.032362, -0.056498, 0.009353),
                    true, true
                },
                {
                    "sphenomandibular_right",
                    {slack_length_factor_ * 0.0355359, 0.0620286, k_e_, 0.75 * 0.0620286, 0.003},
                    mandible_, skull_,
                    ChVector3d(-0.044918, -0.086157, 0.02115),
                    ChVector3d(-0.032595, -0.057076, 0.009112),
                    true, true
                },
                {
                    "temporomandibular_left",
                    {slack_length_factor_ * 0.0224029, 0.0776531, k_e_, 0.75 * 0.0776531, 0.003},
                    mandible_, skull_,
                    ChVector3d(0.052755, -0.06908, 0.015994),
                    ChVector3d(0.059902, -0.050539, 0.021368),
                    true, true
                },
                {
                    "temporomandibular_right",
                    {slack_length_factor_ * 0.0227670, 0.0886738, k_e_, 0.75 * 0.0886738, 0.003},
                    mandible_, skull_,
                    ChVector3d(-0.054442, -0.069513, 0.015986),
                    ChVector3d(-0.061801, -0.050485, 0.020677),
                    true, true
                }
            };

            bool tmj_capsule_ligaments_verbose_{false};
            bool stiff_tmj_capsule_ligaments_{true};
            double tmj_capsule_k_e_{1.0e2}; ///< [N]
            double slack_length_factor_capsule_{1.5};
            ChColor tmj_capsule_ligament_color_{0.0f, 0.39f, 0.0f};
            /// Used as approximation for the TMJ capsule.
            std::vector<ChLigament::ChLigamentProperties> tmj_capsule_ligament_init_properties_{
                {
                    "tmj_capsule_anterior_left",
                    {0.01086, 0.0, tmj_capsule_k_e_, 0.1, 0.01},
                    tmj_disc_left_fea_aux_bodies_[0], mandible_,
                    ChVector3d(0.047176, -0.049148, 0.014277),
                    ChVector3d(0.04836, -0.055025, 0.010947),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                },
                {
                    "tmj_capsule_anterior_right",
                    {0.00738, 0.0, tmj_capsule_k_e_, 0.1, 0.01},
                    tmj_disc_right_fea_aux_bodies_[0], mandible_,
                    ChVector3d(-0.048584, -0.050541, 0.016541),
                    ChVector3d(-0.050273, -0.055031, 0.010929),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                },
                {
                    "tmj_capsule_posterior_left",
                    {slack_length_factor_capsule_ * 0.01211, 0.0, tmj_capsule_k_e_ / 10, 0.1, 0.01},
                    tmj_disc_left_fea_aux_bodies_[1], skull_,
                    ChVector3d(0.050679, -0.046954, 0.003682),
                    ChVector3d(0.050024, -0.045035, 0.005328),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                },
                {
                    "tmj_capsule_posterior_right",
                    {slack_length_factor_capsule_ * 0.00387, 0.0, tmj_capsule_k_e_ / 10, 0.1, 0.01},
                    tmj_disc_right_fea_aux_bodies_[1], skull_,
                    ChVector3d(-0.052442, -0.047219, 0.003765),
                    ChVector3d(-0.04944, -0.045716, 0.005688),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                },
                {
                    "tmj_capsule_lateral_left",
                    {0.00621, 0.0, tmj_capsule_k_e_, 0.1, 0.01},
                    tmj_disc_left_fea_aux_bodies_[2], mandible_,
                    ChVector3d(0.059551, -0.04921, 0.013155),
                    ChVector3d(0.059284, -0.053409, 0.01306),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                },
                {
                    "tmj_capsule_lateral_right",
                    {0.00397, 0.0, tmj_capsule_k_e_, 0.1, 0.01},
                    tmj_disc_right_fea_aux_bodies_[2], mandible_,
                    ChVector3d(-0.060113, -0.049276, 0.013384),
                    ChVector3d(-0.060465, -0.053028, 0.014633),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                },
                {
                    "tmj_capsule_medial_left",
                    {0.00681, 0.0, tmj_capsule_k_e_, 0.1, 0.01},
                    tmj_disc_left_fea_aux_bodies_[3], mandible_,
                    ChVector3d(0.037926, -0.048312, 0.005544),
                    ChVector3d(0.039402, -0.052888, 0.005442),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                },
                {
                    "tmj_capsule_medial_right",
                    {0.00402, 0.0, tmj_capsule_k_e_, 0.1, 0.01},
                    tmj_disc_right_fea_aux_bodies_[3], mandible_,
                    ChVector3d(-0.041112, -0.048367, 0.004315),
                    ChVector3d(-0.041171, -0.052103, 0.005808),
                    false, false,
                    tmj_capsule_ligament_color_,
                    chrono_types::make_shared<ChVisualShapeSpring>(0.0005, 100, 10)
                }
            };

            // Muscles

            bool muscles_verbose_{false};
            /// Declare the forces generated by the muscles as stiff.
            /// If stiff, Jacobian information will be generated.
            bool stiff_muscles_{true};

            /// Use muscle activation dynamics represented by an ODE.
            bool muscle_activation_dynamics_{false};

            ChMuscle::ChMuscleType muscle_type_{ChMuscle::ChMuscleType::PECK};

            std::map<std::string, std::shared_ptr<ChMuscle> > muscle_map_;

            /// {damping, f_max_active, passive_tension_frac, l_max_active_force,
            /// l_max_passive_force, l_zero_passive_force, force_scale, r_tendon}
            float f_frac_{1.0}; // 0.05 (linear)
            float f_frac_open_{1.0};
            float l_max_p_scale_{1.5};
            std::vector<ChMuscle::ChMuscleProperties> muscle_init_properties_{
                {
                    "posterior_temporalis_left",
                    {
                        23.0 / 2, 75.6, f_frac_, 0.0530996, 0.0530996 * l_max_p_scale_, 0.0530996, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(0.047215, -0.055056, 0.042397),
                    ChVector3d(0.062806, -0.02174, 0.002613)
                },
                {
                    "posterior_temporalis_right",
                    {
                        23.0 / 2, 75.6, f_frac_, 0.0543636, 0.0543636 * l_max_p_scale_, 0.0543636, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(-0.048671, -0.054747, 0.04191),
                    ChVector3d(-0.064912, -0.019746, 0.001934)
                },
                {
                    "medial_temporalis_left",
                    {
                        29.0 / 2, 95.6, f_frac_, 0.0683086, 0.0683086 * l_max_p_scale_, 0.0683086, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(0.047215, -0.055056, 0.042397),
                    ChVector3d(0.063054, 0.005202, 0.01254)
                },
                {
                    "medial_temporalis_right",
                    {
                        29.0 / 2, 95.6, f_frac_, 0.0661974, 0.0661974 * l_max_p_scale_, 0.0661974, 1.0, 0.
                    },
                    mandible_, skull_,
                    ChVector3d(-0.048671, -0.054747, 0.04191),
                    ChVector3d(-0.064853, 0.00306, 0.012336)
                },
                {
                    "anterior_temporalis_left",
                    {
                        35.0 / 2, 158.0, f_frac_, 0.0793189, 0.0793189 * l_max_p_scale_, 0.0793189, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(0.036254, -0.084559, 0.030948),
                    ChVector3d(0.046386, -0.008272, 0.049717)
                },
                {
                    "anterior_temporalis_right",
                    {
                        35.0 / 2, 158.0, f_frac_, 0.0789344, 0.0789344 * l_max_p_scale_, 0.0789344, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(-0.03795, -0.084572, 0.031389),
                    ChVector3d(-0.048262, -0.008615, 0.049245)
                },
                {
                    "medial_pterygoid_left",
                    {
                        60.0 / 2, 174.8, f_frac_, 0.0514868, 0.0514868 * l_max_p_scale_, 0.0514868, 1.0, 0.0
                    },
                    mandible_, maxilla_,
                    ChVector3d(0.043096, -0.105472, 0.018688),
                    ChVector3d(0.021578, -0.058945, 0.028063)
                },
                {
                    "medial_pterygoid_right",
                    {
                        60.0 / 2, 174.8, f_frac_, 0.0529574, 0.0529574 * l_max_p_scale_, 0.0529574, 1.0, 0.0
                    },
                    mandible_, maxilla_,
                    ChVector3d(-0.042609, -0.10678, 0.02273),
                    ChVector3d(-0.021578, -0.058945, 0.028063)
                },
                {
                    "superior_lateral_pterygoid_left",
                    {
                        6.0 / 2, 17.0, f_frac_, 0.0157325, 0.0157325 * l_max_p_scale_, 0.0157325, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(0.048572, -0.054953, 0.01278),
                    ChVector3d(0.045256, -0.047287, 0.025278)
                },
                {
                    "superior_lateral_pterygoid_right",
                    {
                        6.0 / 2, 17.0, f_frac_, 0.0164609, 0.0164609 * l_max_p_scale_, 0.0164609, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(-0.05043, -0.055028, 0.012816),
                    ChVector3d(-0.044508, -0.048169, 0.024792)
                },
                {
                    "inferior_lateral_pterygoid_left",
                    {
                        21.0 / 2, 66.9, f_frac_, 0.0307379, 0.0307379 * l_max_p_scale_, 0.0307379, 1.0, 0.0
                    },
                    mandible_, maxilla_,
                    ChVector3d(0.048572, -0.054953, 0.01278),
                    ChVector3d(0.02317, -0.056196, 0.030068)
                },
                {
                    "inferior_lateral_pterygoid_right",
                    {
                        21.0 / 2, 66.9, f_frac_, 0.03307868, 0.0330786 * l_max_p_scale_, 0.0330786, 1.0, 0.0
                    },
                    mandible_, maxilla_,
                    ChVector3d(-0.05043, -0.055028, 0.012816),
                    ChVector3d(-0.02317, -0.056196, 0.030068)
                },
                {
                    "superficial_masseter_left",
                    {
                        53.0 / 2, 190.4, f_frac_, 0.0570705, 0.0570705 * l_max_p_scale_, 0.0570705, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(0.044219, -0.106982, 0.027933),
                    ChVector3d(0.054185, -0.057122, 0.052238)
                },
                {
                    "superficial_masseter_right",
                    {
                        53.0 / 2, 190.4, f_frac_, 0.0572753, 0.0572753 * l_max_p_scale_, 0.0572753, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(-0.046031, -0.106983, 0.027932),
                    ChVector3d(-0.05792, -0.055888, 0.049303)
                },
                {
                    "deep_masseter_left",
                    {
                        38.0 / 2, 81.6, f_frac_, 0.0368628, 0.0368628 * l_max_p_scale_, 0.0368628, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(0.045611, -0.08051, 0.02789),
                    ChVector3d(0.059972, -0.047911, 0.036003)
                },
                {
                    "deep_masseter_right",
                    {
                        38.0 / 2, 81.6, f_frac_, 0.0384437, 0.0384437 * l_max_p_scale_, 0.0384437, 1.0, 0.0
                    },
                    mandible_, skull_,
                    ChVector3d(-0.047891, -0.081603, 0.027029),
                    ChVector3d(-0.063298, -0.047693, 0.036609)
                },
                {
                    "geniohyoid_left",
                    {
                        4.0 / 2, 20.0, f_frac_open_, 0.0269713, 0.0269713 * l_max_p_scale_, 0.0269713, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(0.002726, -0.138493, 0.067702),
                    ChVector3d(0.002302, -0.138597, 0.037668)
                },
                {
                    "geniohyoid_right",
                    {
                        4.0 / 2, 20.0, f_frac_open_, 0.0270043, 0.0270043 * l_max_p_scale_, 0.0270043, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(-0.002726, -0.138493, 0.067702),
                    ChVector3d(-0.002302, -0.138597, 0.037668)
                },
                {
                    "anterior_digastric_left",
                    {
                        19.0 / 2, 40.0, f_frac_open_, 0.0327036, 0.0327036 * l_max_p_scale_, 0.0327036, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(0.011652, -0.140753, 0.06462),
                    ChVector3d(0.014849, -0.135836, 0.029409)
                },
                {
                    "anterior_digastric_right",
                    {
                        19.0 / 2, 40.0, f_frac_open_, 0.0323775, 0.0323775 * l_max_p_scale_, 0.0323775, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(-0.011652, -0.140753, 0.06462),
                    ChVector3d(-0.01604, -0.135984, 0.029808)
                },
                {
                    "posterior_mylohyoid_left",
                    {
                        4.0 / 2, 20.0, f_frac_open_, 0.036568, 0.036568 * l_max_p_scale_, 0.036568, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(0.030925, -0.110696, 0.041199),
                    ChVector3d(0.00703, -0.138632, 0.03655)
                },
                {
                    "posterior_mylohyoid_right",
                    {
                        4.0 / 2, 20.0, f_frac_open_, 0.038586, 0.038586 * l_max_p_scale_, 0.038586, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(-0.032761, -0.110693, 0.041188),
                    ChVector3d(-0.00703, -0.138632, 0.03655)
                },
                {
                    "anterior_mylohyoid_left",
                    {
                        4.0 / 2, 20.0, f_frac_open_, 0.0247217, 0.0247217 * l_max_p_scale_, 0.0247217, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(0.019889, -0.128969, 0.055189),
                    ChVector3d(0.00177, -0.140065, 0.038735)
                },
                {
                    "anterior_mylohyoid_right",
                    {
                        4.0 / 2, 20.0, f_frac_open_, 0.0272054, 0.0272054 * l_max_p_scale_, 0.0272054, 1.0, 0.0
                    },
                    mandible_, hyoid_,
                    ChVector3d(-0.021666, -0.12872, 0.055453),
                    ChVector3d(-0.00177, -0.140065, 0.038735)
                }
            };

            // ---------------------------------------------
            // Collision & Visualization mesh specifications
            // ---------------------------------------------

            std::string mesh_dir_{std::string(EXOSIM_RESOURCES_DIR) + "/meshes/jaw_1/"}; // TODO: Add to JSON

            std::string skull_obj_file_vis_{mesh_dir_ + "visualization/" + SKULL_NAME_ + "_vis.obj"};
            std::string maxilla_obj_file_vis_{mesh_dir_ + "visualization/" + MAXILLA_NAME_ + "_vis.obj"};
            std::string mandible_obj_file_vis_{mesh_dir_ + "visualization/" + MANDIBLE_NAME_ + "_vis.obj"};
            std::string hyoid_obj_file_vis_{mesh_dir_ + "visualization/" + HYOID_NAME_ + "_vis.obj"};

            std::string skull_obj_file_coll_{mesh_dir_ + "collision/" + SKULL_NAME_ + "_smooth_coll.obj"};
            std::string maxilla_obj_file_coll_{mesh_dir_ + "collision/" + MAXILLA_NAME_ + "_coll.obj"};
            std::string mandible_obj_file_coll_{mesh_dir_ + "collision/" + MANDIBLE_NAME_ + "_smooth_coll.obj"};
            std::string skull_obj_file_rigid_coll_{mesh_dir_ + "collision/" + SKULL_NAME_ + "_smooth_coll.obj"};
            std::string maxilla_obj_file_rigid_coll_{mesh_dir_ + "collision/" + MAXILLA_NAME_ + "_coll.obj"};
            std::string mandible_obj_file_rigid_coll_{mesh_dir_ + "collision/" + MANDIBLE_NAME_ + "_smooth_coll.obj"};

            std::string tmj_disc_obj_file_coll_{mesh_dir_ + "collision/" + MANDIBLE_NAME_ + "_coll.obj"}; ///< Not used

            // FEA

            bool fea_wireframe_{false};
            bool fea_smooth_faces_{true};
            std::pair<double, double> color_scale_min_max_{0.0, 5.5};
            ChVisualShapeFEA::DataType fea_vis_data_type_{ChVisualShapeFEA::DataType::ELEM_STRAIN_VONMISES};

            // TetGen files for FEA

            // const std::string TMJ_DISC_LEFT_NODE_FILE_FEA_{mesh_dir_ + "fea/" + TMJ_DISC_LEFT_NAME_ + "_fea.node"};
            // const std::string TMJ_DISC_RIGHT_NODE_FILE_FEA_{mesh_dir_ + "fea/" + TMJ_DISC_LEFT_NAME_ + "_fea.node"};
            // const std::string TMJ_DISC_LEFT_ELE_FILE_FEA_{mesh_dir_ + "fea/" + TMJ_DISC_LEFT_NAME_ + "_fea.ele"};
            // const std::string TMJ_DISC_RIGHT_ELE_FILE_FEA_{mesh_dir_ + "fea/" + TMJ_DISC_RIGHT_NAME_ + "_fea.ele"};

            // Gmsh files for FEA

            std::string tmj_disc_left_msh_file_fea_{
                mesh_dir_ + "fea/" + TMJ_DISC_LEFT_NAME_ + "_imprint_fea.msh"
            };
            std::string tmj_disc_right_msh_file_fea_{
                mesh_dir_ + "fea/" + TMJ_DISC_RIGHT_NAME_ + "_imprint_fea.msh"
            };

        public:
            // ----------
            // Properties
            // ----------

            /// General parameters

            ChModelType& ModelType() { return model_type_; }
            const ChModelType& ModelType() const { return model_type_; }
            std::string& Name() { return name_; }
            const std::string& Name() const { return name_; }
            // bool& EnableRigidBodyModel() { return enable_rigid_body_model_; }
            // const bool& EnableRigidBodyModel() const { return enable_rigid_body_model_; }
            bool& EnableSolverDebugging() { return enable_solver_debugging_; }
            const bool& EnableSolverDebugging() const { return enable_solver_debugging_; }
            bool& EnableSystemDebugging() { return enable_system_debugging_; }
            const bool& EnableSystemDebugging() const { return enable_system_debugging_; }
            unsigned int& Threads() { return threads_; }
            const unsigned int& Threads() const { return threads_; }
            double& Gravity() { return gravity_; }
            const double& Gravity() const { return gravity_; }
            double& TimeStep() { return time_step_; }
            const double& TimeStep() const { return time_step_; }
            bool& RunSimulation() { return run_simulation_; }
            const bool& RunSimulation() const { return run_simulation_; }
            bool& UpdateSimulation() { return update_simulation_; }
            const bool& UpdateSimulation() const { return update_simulation_; }
            double& SimulationTimeLimit() { return simulation_time_limit_; }
            const double& SimulationTimeLimit() const { return simulation_time_limit_; }
            ChSimulationState& ResetSimulationState() { return reset_simulation_state_; }
            const ChSimulationState& ResetSimulationState() const { return reset_simulation_state_; }
            unsigned int& ReplayBufferSize() { return replay_buffer_size_; }
            const unsigned int& ReplayBufferSize() const { return replay_buffer_size_; }
            utils::CircularBuffer<ChSimulationState>& ReplayBuffer() { return replay_buffer_; }
            const utils::CircularBuffer<ChSimulationState>& ReplayBuffer() const { return replay_buffer_; }

            /// Time stepper

            ChTimestepper::Type& TimeStepperType() { return time_stepper_type_; }
            const ChTimestepper::Type& TimeStepperType() const { return time_stepper_type_; }
            bool& TimeStepperVerbose() { return time_stepper_verbose_; }
            const bool& TimeStepperVerbose() const { return time_stepper_verbose_; }
            bool& HHTStepControl() { return hht_step_control_; }
            const bool& HHTStepControl() const { return hht_step_control_; }
            int& EulerIMaxIters() { return euler_i_max_iters_; }
            const int& EulerIMaxIters() const { return euler_i_max_iters_; }
            int& TrapezoidalLinMaxIters() { return trapezoidal_lin_max_iters_; }
            const int& TrapezoidalLinMaxIters() const { return trapezoidal_lin_max_iters_; }
            int& NewmarkMaxIters() { return newmark_max_iters_; }
            const int& NewmarkMaxIters() const { return newmark_max_iters_; }
            int& HHTMaxIters() { return hht_max_iters_; }
            const int& HHTMaxIters() const { return hht_max_iters_; }
            double& HHTAlpha() { return hht_alpha_; }
            const double& HHTAlpha() const { return hht_alpha_; }

            /// Solver

            ChSolver::Type& SolverType() { return solver_type_; }
            const ChSolver::Type& SolverType() const { return solver_type_; }
            bool& SolverVerbose() { return solver_verbose_; }
            const bool& SolverVerbose() const { return solver_verbose_; }
            double& SolverTolerance() { return solver_tolerance_; }
            const double& SolverTolerance() const { return solver_tolerance_; }
            int& MaxSolverIterations() { return max_solver_iterations_; }
            const int& MaxSolverIterations() const { return max_solver_iterations_; }
            bool& SolverADMMWarmStart() { return admm_warm_start_; }
            const bool& SolverADMMWarmStart() const { return admm_warm_start_; }
            double& SolverADMMRho() { return admm_rho_; }
            const double& SolverADMMRho() const { return admm_rho_; }
            double& SolverADMMToleranceDual() { return admm_tolerance_dual_; }
            const double& SolverADMMToleranceDual() const { return admm_tolerance_dual_; }
            double& SolverADMMTolerancePrimal() { return admm_tolerance_primal_; }
            const double& SolverADMMTolerancePrimal() const { return admm_tolerance_primal_; }

            ChSolverADMM::AdmmStepType& SolverADMMStepPolicy() {
                return admm_step_policy_;
            }

            const ChSolverADMM::AdmmStepType& SolverADMMStepPolicy() const {
                return admm_step_policy_;
            }

            ChSolverADMM::AdmmAcceleration& SolverADMMAcceleration() {
                return admm_acceleration_;
            }

            const ChSolverADMM::AdmmAcceleration& SolverADMMAcceleration() const {
                return admm_acceleration_;
            }

            bool& MKLLockSparsityPattern() { return mkl_lock_sparsity_pattern_; }
            const bool& MKLLockSparsityPattern() const { return mkl_lock_sparsity_pattern_; }
            bool& MKLUseSparsityPatternLearner() { return mkl_use_sparsity_pattern_learner_; }
            const bool& MKLUseSparsityPatternLearner() const { return mkl_use_sparsity_pattern_learner_; }

            /// Visualization

            bool& RunSimAtStartup() { return run_sim_at_startup_; }
            const bool& RunSimAtStartup() const { return run_sim_at_startup_; }
            uint& WindowWidth() { return window_width_; }
            const uint& WindowWidth() const { return window_width_; }
            uint& WindowHeight() { return window_height_; }
            const uint& WindowHeight() const { return window_height_; }
            std::string& WindowTitle() { return window_title_; }
            const std::string& WindowTitle() const { return window_title_; }
            ChVisualizationMode& VisualizationMode() { return vis_mode_; }
            const ChVisualizationMode& VisualizationMode() const { return vis_mode_; }
            ChRenderMode& RenderMode() { return render_mode_; }
            const ChRenderMode& RenderMode() const { return render_mode_; }
            ChColor& BoneColor() { return bone_color_; }
            const ChColor& BoneColor() const { return bone_color_; }
            ChColor& BackgroundColor() { return background_color_; }
            const ChColor& BackgroundColor() const { return background_color_; }

            /// Collision

            ChCollisionSystem::Type& CollisionSystemType() { return collision_type_; }
            const ChCollisionSystem::Type& CollisionSystemType() const { return collision_type_; }
            double& CollisionMargin() { return collision_margin_; }
            const double& CollisionMargin() const { return collision_margin_; }
            double& CollisionEnvelope() { return collision_envelope_; }
            const double& CollisionEnvelope() const { return collision_envelope_; }

            double& BoneMeshSweptSphereThickness() {
                return bone_mesh_swept_sphere_thickness_;
            }

            const double& BoneMeshSweptSphereThickness() const {
                return bone_mesh_swept_sphere_thickness_;
            }

            double& TMJDiscMeshSweptSphereThickness() {
                return tmj_disc_mesh_swept_sphere_thickness_;
            }

            const double& TMJDiscMeshSweptSphereThickness() const {
                return tmj_disc_mesh_swept_sphere_thickness_;
            }

            ChContactMethod& ContactMethod() { return contact_method_; }
            const ChContactMethod& ContactMethod() const { return contact_method_; }

            ChNarrowphase::Algorithm& NarrowphaseAlgorithmMulticore() {
                return narrowphase_algorithm_multicore_;
            }

            const ChNarrowphase::Algorithm& NarrowphaseAlgorithmMulticore() const {
                return narrowphase_algorithm_multicore_;
            }

            ChVector3i& BroadphaseGridResolutionMulticore() {
                return broadphase_grid_resolution_multicore_;
            }

            const ChVector3i& BroadphaseGridResolutionMulticore() const {
                return broadphase_grid_resolution_multicore_;
            }

            double& ContactBreakingThresholdBullet() {
                return contact_breaking_threshold_bullet_;
            }

            const double& ContactBreakingThresholdBullet() const {
                return contact_breaking_threshold_bullet_;
            }

            double& DefaultEffectiveCurvatureRadiusSMC() {
                return default_effective_curvature_radius_smc_;
            }

            const double& DefaultEffectiveCurvatureRadiusSMC() const {
                return default_effective_curvature_radius_smc_;
            }

            ChContactForceTorqueJawSMC::ContactForceTorqueModel& ContactForceModelSMC() {
                return contact_force_model_smc_;
            }

            const ChContactForceTorqueJawSMC::ContactForceTorqueModel& ContactForceModelSMC() const {
                return contact_force_model_smc_;
            }

            ChContactForceTorqueJawSMC::AdhesionForceTorqueModel& AdhesionForceModelSMC() {
                return adhesion_force_model_smc_;
            }

            const ChContactForceTorqueJawSMC::AdhesionForceTorqueModel& AdhesionForceModelSMC() const {
                return adhesion_force_model_smc_;
            }

            ChContactForceTorqueJawSMC::TangentialDisplacementModel& TangentDisplacementModelSMC() {
                return tangential_displ_model_smc_;
            }

            const ChContactForceTorqueJawSMC::TangentialDisplacementModel& TangentDisplacementModelSMC() const {
                return tangential_displ_model_smc_;
            }

            double& MinBounceSpeedNSC() { return min_bounce_speed_nsc_; }
            const double& MinBounceSpeedNSC() const { return min_bounce_speed_nsc_; }
            double& ContactRecoverySpeedNSC() { return contact_recovery_speed_nsc_; }
            const double& ContactRecoverySpeedNSC() const { return contact_recovery_speed_nsc_; }

            /// Material properties

            float& BoneDensity() { return bone_density_; }
            const float& BoneDensity() const { return bone_density_; }
            bool& UseMaterialPropertiesSMC() { return use_material_properties_smc_; }
            const bool& UseMaterialPropertiesSMC() const { return use_material_properties_smc_; }

            ChContactMaterialPropertiesSMC& BonePropertiesSMC() {
                return bone_properties_smc_;
            }

            const ChContactMaterialPropertiesSMC& BonePropertiesSMC() const {
                return bone_properties_smc_;
            }

            ChContactMaterialPropertiesSMC& TMJDiscPropertiesSMC() {
                return tmj_disc_properties_smc_;
            }

            const ChContactMaterialPropertiesSMC& TMJDiscPropertiesSMC() const {
                return tmj_disc_properties_smc_;
            }

            ChContactMaterialPropertiesNSC& BonePropertiesNSC() {
                return bone_properties_nsc_;
            }

            const ChContactMaterialPropertiesNSC& BonePropertiesNSC() const {
                return bone_properties_nsc_;
            }

            ChContactMaterialPropertiesNSC& TMJDiscPropertiesNSC() {
                return tmj_disc_properties_nsc_;
            }

            const ChContactMaterialPropertiesNSC& TMJDiscPropertiesNSC() const {
                return tmj_disc_properties_nsc_;
            }

            // FEA

            bool& TMJDiscFEAUseMooneyRivlin() { return tmj_disc_fea_use_mooney_rivlin_; }
            const bool& TMJDiscFEAUseMooneyRivlin() const { return tmj_disc_fea_use_mooney_rivlin_; }
            double& TMJDiscFEAMRC1() { return tmj_disc_fea_mr_c_1_; }
            const double& TMJDiscFEAMRC1() const { return tmj_disc_fea_mr_c_1_; }
            double& TMJDiscFEAMRC2() { return tmj_disc_fea_mr_c_2_; }
            const double& TMJDiscFEAMRC2() const { return tmj_disc_fea_mr_c_2_; }
            double& TMJDiscFEADensity() { return tmj_disc_fea_density_; }
            const double& TMJDiscFEADensity() const { return tmj_disc_fea_density_; }
            double& TMJDiscFEAYoungModulus() { return tmj_disc_fea_young_modulus_; }
            const double& TMJDiscFEAYoungModulus() const { return tmj_disc_fea_young_modulus_; }
            double& TMJDiscFEAPoissonRatio() { return tmj_disc_fea_poisson_ratio_; }
            const double& TMJDiscFEAPoissonRatio() const { return tmj_disc_fea_poisson_ratio_; }
            double& TMJDiscFEARayleighDampingAlpha() { return tmj_disc_fea_rayleigh_damping_alpha_; }
            const double& TMJDiscFEARayleighDampingAlpha() const { return tmj_disc_fea_rayleigh_damping_alpha_; }
            double& TMJDiscFEARayleighDampingBeta() { return tmj_disc_fea_rayleigh_damping_beta_; }
            const double& TMJDiscFEARayleighDampingBeta() const { return tmj_disc_fea_rayleigh_damping_beta_; }

            /// System Components

            // System

            std::shared_ptr<ChSystem> System() const { return sys_; }

            // FEA auxiliary bodies

            float& TMJDiscFEAAuxBodyRadius() { return tmj_disc_fea_aux_body_radius_; }
            const float& TMJDiscFEAAuxBodyRadius() const { return tmj_disc_fea_aux_body_radius_; }

            // auto& TMJDiscFEAAuxBodiesLeft() { return tmj_disc_left_fea_aux_bodies_; }
            const auto& TMJDiscFEAAuxBodiesLeft() const { return tmj_disc_left_fea_aux_bodies_; }
            // auto& TMJDiscFEAAuxBodiesRight() { return tmj_disc_right_fea_aux_bodies_; }
            const auto& TMJDiscFEAAuxBodiesRight() const { return tmj_disc_right_fea_aux_bodies_; }

            // FEA meshes

            unsigned int& TMJDiscFEAElementType() { return tmj_disc_fea_element_type_; }
            const unsigned int& TMJDiscFEAElementType() const { return tmj_disc_fea_element_type_; }

            fea::ChGmshMeshFileLoader::ChFEAImplementationType& TMJDiscFEAImplementation() {
                return tmj_disc_fea_implementation_;
            }

            const fea::ChGmshMeshFileLoader::ChFEAImplementationType& TMJDiscFEAImplementation() const {
                return tmj_disc_fea_implementation_;
            }

            std::shared_ptr<fea::ChMesh> TMJDiscLeftFEA() const { return tmj_disc_left_fea_; }
            std::shared_ptr<fea::ChMesh> TMJDiscRightFEA() const { return tmj_disc_right_fea_; }

            // Bones

            std::shared_ptr<ChBodyAuxRef> Skull() const { return skull_; }
            std::shared_ptr<ChBodyAuxRef> Maxilla() const { return maxilla_; }
            std::shared_ptr<ChBodyAuxRef> Mandible() const { return mandible_; }
            std::shared_ptr<ChBodyAuxRef> Hyoid() const { return hyoid_; }

            ChFramed& IncisalFrameAbs() { return incisal_frame_abs_; }
            const ChFramed& IncisalFrameAbs() const { return incisal_frame_abs_; }

            // TMJ constraints (only for rigid-body model)

            bool& TMJConstrVerbose() { return tmj_constr_verbose_; }
            const bool& TMJConstrVerbose() const { return tmj_constr_verbose_; }
            bool& TMJConstrULimit() { return tmj_constr_u_limit_; }
            const bool& TMJConstrULimit() const { return tmj_constr_u_limit_; }
            bool& TMJConstrVLimit() { return tmj_constr_v_limit_; }
            const bool& TMJConstrVLimit() const { return tmj_constr_v_limit_; }
            bool& TMJConstrVisWireframe() { return tmj_constr_vis_wireframe_; }
            const bool& TMJConstrVisWireframe() const { return tmj_constr_vis_wireframe_; }
            bool& TMJConstrUseRelPos() { return tmj_constr_use_rel_pos_; }
            const bool& TMJConstrUseRelPos() const { return tmj_constr_use_rel_pos_; }
            int& TMJConstrPointSurfaceSamples() { return tmj_constr_point_surface_samples_; }
            const int& TMJConstrPointSurfaceSamples() const { return tmj_constr_point_surface_samples_; }
            int& TMJConstrPointSurfaceMaxIters() { return tmj_constr_point_surface_max_iters_; }
            const int& TMJConstrPointSurfaceMaxIters() const { return tmj_constr_point_surface_max_iters_; }
            double& TMJConstrPointSurfaceTolerance() { return tmj_constr_point_surface_tolerance_; }
            const double& TMJConstrPointSurfaceTolerance() const { return tmj_constr_point_surface_tolerance_; }

            ChColor& TMJConstrVisColor() { return tmj_constr_vis_color_; }
            const ChColor& TMJConstrVisColor() const { return tmj_constr_vis_color_; }

            ChVector3d& TMJConstrPosLeft() { return tmj_constr_pos_left_; }
            const ChVector3d& TMJConstrPosLeft() const { return tmj_constr_pos_left_; }
            ChVector3d& TMJConstrPosRight() { return tmj_constr_pos_right_; }
            const ChVector3d& TMJConstrPosRight() const { return tmj_constr_pos_right_; }
            ChVector3d& TMJConstrCardanXYZLeft() { return tmj_constr_cardan_xyz_left_; }
            const ChVector3d& TMJConstrCardanXYZLeft() const { return tmj_constr_cardan_xyz_left_; }
            ChVector3d& TMJConstrCardanXYZRight() { return tmj_constr_cardan_xyz_right_; }
            const ChVector3d& TMJConstrCardanXYZRight() const { return tmj_constr_cardan_xyz_right_; }

            std::shared_ptr<ChLinkLockPointSurface> TMJConstrLeft() const { return tmj_constr_left_; }
            std::shared_ptr<ChLinkLockPointSurface> TMJConstrRight() const { return tmj_constr_right_; }

            ChVector2i& TMJConstrSurfaceNURBSOrder() { return tmj_constr_surface_nurbs_order_; }
            const ChVector2i& TMJConstrSurfaceNURBSOrder() const { return tmj_constr_surface_nurbs_order_; }
            ChVector2i& TMJConstrCtrlPtsPerDimension() { return tmj_constr_ctrl_pts_per_dimension_; }
            const ChVector2i& TMJConstrCtrlPtsPerDimension() const { return tmj_constr_ctrl_pts_per_dimension_; }

            std::vector<ChVector3d>& TMJConstrCtrlPtsSurfaceNURBSLeft() {
                return tmj_constr_ctrl_pts_surface_nurbs_left_;
            }

            const std::vector<ChVector3d>& TMJConstrCtrlPtsSurfaceNURBSLeft() const {
                return tmj_constr_ctrl_pts_surface_nurbs_left_;
            }

            std::vector<ChVector3d>& TMJConstrCtrlPtsSurfaceNURBSRight() {
                return tmj_constr_ctrl_pts_surface_nurbs_right_;
            }

            const std::vector<ChVector3d>& TMJConstrCtrlPtsSurfaceNURBSRight() const {
                return tmj_constr_ctrl_pts_surface_nurbs_right_;
            }

            // Ligaments

            bool& LigamentsVerbose() { return ligaments_verbose_; }
            const bool& LigamentsVerbose() const { return ligaments_verbose_; }
            bool& TMJCapsuleLigamentsVerbose() { return tmj_capsule_ligaments_verbose_; }
            const bool& TMJCapsuleLigamentsVerbose() const { return tmj_capsule_ligaments_verbose_; }
            bool& StiffLigaments() { return stiff_ligaments_; }
            const bool& StiffLigaments() const { return stiff_ligaments_; }
            bool& StiffTMJCapsuleLigaments() { return stiff_tmj_capsule_ligaments_; }
            const bool& StiffTMJCapsuleLigaments() const { return stiff_tmj_capsule_ligaments_; }
            ChLigament::ChLigamentType& LigamentType() { return ligament_type_; }
            const ChLigament::ChLigamentType& LigamentType() const { return ligament_type_; }

            std::vector<ChLigament::ChLigamentProperties>& LigamentProperties() {
                return ligament_init_properties_;
            }

            const std::vector<ChLigament::ChLigamentProperties>& LigamentProperties() const {
                return ligament_init_properties_;
            }

            std::vector<ChLigament::ChLigamentProperties>& CapsuleLigamentProperties() {
                return tmj_capsule_ligament_init_properties_;
            }

            const std::vector<ChLigament::ChLigamentProperties>& CapsuleLigamentProperties() const {
                return tmj_capsule_ligament_init_properties_;
            }

            virtual std::map<std::string, std::shared_ptr<ChLigament> >& LigamentMap() { return ligament_map_; }

            virtual const std::map<std::string, std::shared_ptr<ChLigament> >& LigamentMap() const {
                return ligament_map_;
            }

            virtual std::map<std::string, std::shared_ptr<ChLigament> >& CapsuleLigamentMap() {
                return tmj_capsule_ligament_map_;
            }

            virtual const std::map<std::string, std::shared_ptr<ChLigament> >& CapsuleLigamentMap() const {
                return tmj_capsule_ligament_map_;
            }

            // Muscles

            bool& MusclesVerbose() { return muscles_verbose_; }
            const bool& MusclesVerbose() const { return muscles_verbose_; }
            bool& StiffMuscles() { return stiff_muscles_; }
            const bool& StiffMuscles() const { return stiff_muscles_; }
            bool& MuscleActivationDynamics() { return muscle_activation_dynamics_; }
            const bool& MuscleActivationDynamics() const { return muscle_activation_dynamics_; }
            ChMuscle::ChMuscleType& MuscleType() { return muscle_type_; }
            const ChMuscle::ChMuscleType& MuscleType() const { return muscle_type_; }

            std::vector<ChMuscle::ChMuscleProperties>& MuscleProperties() { return muscle_init_properties_; }

            const std::vector<ChMuscle::ChMuscleProperties>& MuscleProperties() const {
                return muscle_init_properties_;
            }

            virtual std::map<std::string, std::shared_ptr<ChMuscle> >& MuscleMap() { return muscle_map_; }
            virtual const std::map<std::string, std::shared_ptr<ChMuscle> >& MuscleMap() const { return muscle_map_; }

            // Mesh files

            std::string& MeshDirectory() { return mesh_dir_; }
            const std::string& MeshDirectory() const { return mesh_dir_; }
            std::string& SkullObjFileVis() { return skull_obj_file_vis_; }
            const std::string& SkullObjFileVis() const { return skull_obj_file_vis_; }
            std::string& MaxillaObjFileVis() { return maxilla_obj_file_vis_; }
            const std::string& MaxillaObjFileVis() const { return maxilla_obj_file_vis_; }
            std::string& MandibleObjFileVis() { return mandible_obj_file_vis_; }
            const std::string& MandibleObjFileVis() const { return mandible_obj_file_vis_; }
            std::string& HyoidObjFileVis() { return hyoid_obj_file_vis_; }
            const std::string& HyoidObjFileVis() const { return hyoid_obj_file_vis_; }
            std::string& SkullObjFileColl() { return skull_obj_file_coll_; }
            const std::string& SkullObjFileColl() const { return skull_obj_file_coll_; }
            std::string& MaxillaObjFileColl() { return maxilla_obj_file_coll_; }
            const std::string& MaxillaObjFileColl() const { return maxilla_obj_file_coll_; }
            std::string& MandibleObjFileColl() { return mandible_obj_file_coll_; }
            const std::string& MandibleObjFileColl() const { return mandible_obj_file_coll_; }
            std::string& SkullObjFileRigidColl() { return skull_obj_file_rigid_coll_; }
            const std::string& SkullObjFileRigidColl() const { return skull_obj_file_rigid_coll_; }
            std::string& MaxillaObjFileRigidColl() { return maxilla_obj_file_rigid_coll_; }
            const std::string& MaxillaObjFileRigidColl() const { return maxilla_obj_file_rigid_coll_; }
            std::string& MandibleObjFileRigidColl() { return mandible_obj_file_rigid_coll_; }
            const std::string& MandibleObjFileRigidColl() const { return mandible_obj_file_rigid_coll_; }
            std::string& TMJDiscObjFileColl() { return tmj_disc_obj_file_coll_; }
            const std::string& TMJDiscObjFileColl() const { return tmj_disc_obj_file_coll_; }
            bool& FEAWireFrame() { return fea_wireframe_; }
            const bool& FEAWireFrame() const { return fea_wireframe_; }
            bool& FEASmoothFaces() { return fea_smooth_faces_; }
            const bool& FEASmoothFaces() const { return fea_smooth_faces_; }
            std::pair<double, double>& ColorScaleMinMax() { return color_scale_min_max_; }
            const std::pair<double, double>& ColorScaleMinMax() const { return color_scale_min_max_; }
            ChVisualShapeFEA::DataType& FEAVisDataType() { return fea_vis_data_type_; }
            const ChVisualShapeFEA::DataType& FEAVisDataType() const { return fea_vis_data_type_; }
            std::string& TMJDiscLeftMshFile() { return tmj_disc_left_msh_file_fea_; }
            const std::string& TMJDiscLeftMshFile() const { return tmj_disc_left_msh_file_fea_; }
            std::string& TMJDiscRightMshFile() { return tmj_disc_right_msh_file_fea_; }
            const std::string& TMJDiscRightMshFile() const { return tmj_disc_right_msh_file_fea_; }
    };
} // namespace chrono::biomechanics

namespace chrono {
    CH_CLASS_VERSION(biomechanics::ChJaw, 0)
}


#endif // EXOSIM_CH_JAW_H
