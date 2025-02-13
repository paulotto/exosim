/**
* @file ChGmshMeshFileLoader.cpp
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 26.04.2024
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

#include "exosim/biomechanics/utils/ChGmshMeshFileLoader.h"
//
#include "MshIO/include/mshio/mshio.h"
//
#include <chrono/fea/ChNodeFEAxyz.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
#include <chrono/fea/ChNodeFEAxyzP.h>
#include <chrono/fea/ChContinuumPoisson3D.h>

#include <chrono/fea/ChElementTetraCorot_4.h>
#include <chrono/fea/ChElementHexaANCF_3813.h>
#include <chrono/fea/ChElementHexaCorot_8.h>


namespace chrono::fea {
    // Register into the object factory, to enable run-time dynamic creation and persistence
    CH_FACTORY_REGISTER(ChGmshMeshFileLoader)

    void ChGmshMeshFileLoader::FromGmshFile(const std::shared_ptr<ChMesh>& mesh,
                                            unsigned int element_type,
                                            ChFEAImplementationType fea_implementation,
                                            const std::string& filename,
                                            const std::shared_ptr<ChContinuumMaterial>& material,
                                            double mooney_rivlin_c1,
                                            double mooney_rivlin_c2,
                                            const ChVector3d& pos_transform,
                                            const ChMatrix33<>& rot_transform) {
        mshio::MshSpec msh_spec;
        const unsigned int node_off = mesh->GetNumNodes();

        biomechanics::utils::throw_invalid_argument(element_type != 4 && element_type != 5,
                                                    "[ChGmshMeshFileLoader] Unsupported element type: {}",
                                                    element_type);
        if (element_type == 4 && fea_implementation == ANCF) {
            fmt::print(biomechanics::utils::WARNING_MSG,
                       "[WARNING] [ChGmshMeshFileLoader] FEA implementation scheme 'ANCF' not "
                       "supported in combination with element_type 'tetrahedron'!\n");
        }

        const auto mat_elastic = std::dynamic_pointer_cast<ChContinuumElastic>(material);
        const auto mat_poisson = std::dynamic_pointer_cast<ChContinuumPoisson3D>(material);

        biomechanics::utils::throw_invalid_argument(
            (!mat_elastic && !mat_poisson) || (mat_poisson && element_type == 5),
            "[ERROR] [ChGmshMeshFileLoader] Unsupported material type: {}", typeid(material).name());

        try {
            msh_spec = mshio::load_msh(filename);
        } catch (std::exception& e) {
            biomechanics::utils::throw_runtime_error(true, "[ChGmshMeshFileLoader] Failed to load MSH file: {}",
                                                     e.what());
        };

        // Load nodes
        for (auto& [entity_dim, entity_tag, parametric, num_nodes_in_block, tags, data]
             : msh_spec.nodes.entity_blocks) {
            biomechanics::utils::throw_runtime_error(parametric == 1,
                                                     "[ERROR] [ChGmshMeshFileLoader] Parametric nodes not supported!");

            for (auto node_it = data.begin(); node_it != data.end(); std::advance(node_it, 3)) {
                ChVector3d node_position{node_it[0], node_it[1], node_it[2]};
                node_position = rot_transform * node_position + pos_transform;

                if (mat_elastic) {
                    auto node = chrono_types::make_shared<ChNodeFEAxyz>(node_position);
                    mesh->AddNode(node);
                }
                if (mat_poisson) {
                    auto node = chrono_types::make_shared<ChNodeFEAxyzP>(node_position);
                    mesh->AddNode(node);
                }
            }
        }

        int elem_tag{0};
        bool element_type_found{false};
        const bool use_mooney_rivlin = mooney_rivlin_c1 != 0.0 || mooney_rivlin_c2 != 0.0;

        // Load elements
        for (auto& [entity_dim, entity_tag, block_elem_type, num_elements_in_block, data]
             : msh_spec.elements.entity_blocks) {
            if (entity_dim != 3) continue;

            if (block_elem_type == 4 && element_type == 4) {
                element_type_found = true;
                for (auto elem_it = data.begin(); elem_it != data.end(); std::advance(elem_it, 5)) {
                    if (mat_elastic) {
                        auto elem = chrono_types::make_shared<ChElementTetraCorot_4>();
                        elem->SetNodes(
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[1] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[2] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[3] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[4] - 1)));
                        elem->SetMaterial(std::static_pointer_cast<ChContinuumElastic>(material));
                        mesh->AddElement(elem);
                    } else if (mat_poisson) {
                        auto elem = chrono_types::make_shared<ChElementTetraCorot_4_P>();
                        elem->SetNodes(
                            std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(node_off + elem_it[1] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(node_off + elem_it[2] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(node_off + elem_it[3] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzP>(mesh->GetNode(node_off + elem_it[4] - 1)));
                        elem->SetMaterial(std::static_pointer_cast<ChContinuumPoisson3D>(material));
                        mesh->AddElement(elem);
                    }
                }
            } else if (block_elem_type == 5 && element_type == 5) {
                element_type_found = true;
                for (auto elem_it = data.begin(); elem_it != data.end(); std::advance(elem_it, 9)) {
                    std::shared_ptr<ChElementBase> elem{nullptr};

                    if (fea_implementation == ANCF) {
                        elem = chrono_types::make_shared<ChElementHexaANCF_3813>();
                        const auto hexa_ancf = std::dynamic_pointer_cast<ChElementHexaANCF_3813>(elem);
                        hexa_ancf->SetNodes(
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[1] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[2] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[3] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[4] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[5] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[6] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[7] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[8] - 1)));
                        hexa_ancf->SetMaterial(std::static_pointer_cast<ChContinuumElastic>(material));
                        hexa_ancf->SetMRCoefficients(mooney_rivlin_c1, mooney_rivlin_c2);
                        hexa_ancf->SetMooneyRivlin(use_mooney_rivlin);

                        // Calculate the dimensions for this element
                        std::array<ChVector3d, 8> nodes{};
                        for (int i = 0; i < 8; ++i) {
                            nodes[i] = std::dynamic_pointer_cast<ChNodeFEAxyz>(
                                mesh->GetNode(node_off + elem_it[i + 1] - 1))->GetX0();
                        }

                        ChVector3d center_ADHE = (nodes[0] + nodes[3] + nodes[7] + nodes[4]) / 4.0;
                        ChVector3d center_BCGF = (nodes[1] + nodes[2] + nodes[6] + nodes[5]) / 4.0;
                        ChVector3d center_ABCD = (nodes[0] + nodes[1] + nodes[2] + nodes[3]) / 4.0;
                        ChVector3d center_EFGH = (nodes[4] + nodes[5] + nodes[6] + nodes[7]) / 4.0;
                        ChVector3d center_ABFE = (nodes[0] + nodes[1] + nodes[5] + nodes[4]) / 4.0;
                        ChVector3d center_DCGH = (nodes[3] + nodes[2] + nodes[6] + nodes[7]) / 4.0;

                        double length_x = (center_ADHE - center_BCGF).Length();
                        double length_y = (center_ABCD - center_EFGH).Length();
                        double length_z = (center_ABFE - center_DCGH).Length();

                        hexa_ancf->SetInertFlexVec({length_x, length_y, length_z});

                        hexa_ancf->SetElemNum(elem_tag++);
                        hexa_ancf->SetStockAlpha(0, 0, 0, 0, 0, 0, 0, 0, 0);
                    } else if (fea_implementation == COROTATIONAL) {
                        elem = chrono_types::make_shared<ChElementHexaCorot_8>();
                        const auto hexa_corot = std::dynamic_pointer_cast<ChElementHexaCorot_8>(elem);
                        hexa_corot->SetNodes(
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[1] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[2] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[3] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[4] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[5] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[6] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[7] - 1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(node_off + elem_it[8] - 1)));
                        hexa_corot->SetMaterial(std::static_pointer_cast<ChContinuumElastic>(material));
                    } else {
                        biomechanics::utils::throw_invalid_argument(true, "[ERROR] [ChGmshMeshFileLoader] "
                                                                    "Unsupported FEA implementation scheme!");
                    }
                    mesh->AddElement(elem);
                }
            }
        }

        biomechanics::utils::throw_runtime_error(!element_type_found,
                                                 "[ERROR] [ChGmshMeshFileLoader] No element of type {} found in {}!",
                                                 element_type, filename);
    }
} // namespace chrono::fea
