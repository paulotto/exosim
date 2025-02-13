/**
* @file ChGmshMeshFileLoader.h
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

#ifndef EXOSIM_CH_GMSH_MESH_FILE_LOADER_H
#define EXOSIM_CH_GMSH_MESH_FILE_LOADER_H

#include "exosim/biomechanics/utils/utils.h"
//
#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChContinuumMaterial.h>


namespace chrono::fea {
    /**
    * @class ChGmshMeshFileLoader
    * @brief Loads a mesh provided in the MSH file format.
    */
    class ChApi ChGmshMeshFileLoader {
        public:
            /**
             * @enum ChFEAImplementationType
             * @brief Supported FEA implementation schemes.
             */
            enum ChFEAImplementationType {
                COROTATIONAL,
                ANCF
            };

            /**
             * @class ChGmshMeshFileLoaderEnumMapper
             * @brief Can be used to convert between enums and strings.
             */
            class ChGmshMeshFileLoaderEnumMapper final {
                public:
                    CH_ENUM_MAPPER_BEGIN(ChFEAImplementationType)
                                CH_ENUM_VAL(COROTATIONAL)
                                CH_ENUM_VAL(ANCF)
                    CH_ENUM_MAPPER_END(ChFEAImplementationType)
            };

            /**
             * Load polyhedrons from a MSH file. If the material is a Mooney-Rivlin material,
             * the coefficients C1 and C2 must be provided (only supported for ChElementHexaANCF_3813).
             * @param mesh Destination mesh.
             * @param element_type Type of elements to create. Supported: 4-node tetrahedron=4, 8-node hexahedron=5.
             * @param fea_implementation FEA implementation scheme.
             * @param filename Path to the MSH file.
             * @param material Material to assign to the created polyhedrons.
             * @param mooney_rivlin_c1 Mooney-Rivlin coefficient C1.
             * @param mooney_rivlin_c2 Mooney-Rivlin coefficient C2.
             * @param pos_transform Optional displacement/scaling of the imported mesh.
             * @param rot_transform Optional rotation/scaling of the imported mesh.
             */
            static void FromGmshFile(const std::shared_ptr<ChMesh>& mesh,
                                     unsigned int element_type,
                                     ChFEAImplementationType fea_implementation,
                                     const std::string& filename,
                                     const std::shared_ptr<ChContinuumMaterial>& material,
                                     double mooney_rivlin_c1 = 0.0,
                                     double mooney_rivlin_c2 = 0.0,
                                     const ChVector3d& pos_transform = VNULL,
                                     const ChMatrix33<>& rot_transform = QUNIT);
    };
} // namespace chrono::fea

namespace chrono {
    CH_CLASS_VERSION(fea::ChGmshMeshFileLoader, 0)
}

#endif // EXOSIM_CH_GMSH_MESH_FILE_LOADER_H
