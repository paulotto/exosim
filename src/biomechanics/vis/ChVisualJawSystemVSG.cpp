/**
 * @file ChVisualJawSystemVSG.cpp
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 16.08.2024
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

#include "exosim/biomechanics/vis/ChVisualJawSystemVSG.h"


namespace chrono::vsg3d {
    void ChVisualJawSystemVSG::BindAll(ChSystem* sys) {
        for (const auto& body : sys->GetAssembly().GetBodies()) {
            this->BindItem(body);
        }

        // Bind visual models associated with links in the system
        for (const auto& link : sys->GetLinks()) {
            if (auto link1 = std::dynamic_pointer_cast<ChLink>(link)) {
                this->BindItem(link1);
            }
        }

        // Bind visual models associated with FEA meshes
        for (const auto& mesh : sys->GetAssembly().GetMeshes()) {
            this->BindItem(mesh);
        }

        // Bind visual models associated with other physics items in the system
        for (const auto& item : sys->GetOtherPhysicsItems()) {
            this->BindItem(item);
        }
    }

    void ChVisualJawSystemVSG::UnbindItem(std::shared_ptr<ChPhysicsItem> item) {
        ChVisualSystemVSG::UnbindItem(item);

        if (const auto body = std::dynamic_pointer_cast<ChBody>(item)) {
            for (auto it = m_bodyScene->children.begin(); it != m_bodyScene->children.end(); ++it) {
                if (const auto modelGroup = dynamic_cast<vsg::Group*>(it->get())) {
                    std::shared_ptr<ChBody> stored_body;
                    modelGroup->getValue("Body", stored_body);
                    if (stored_body == body) {
                        // Remove the group from m_bodyScene
                        m_bodyScene->children.erase(it);
                        break;
                    }
                }
            }
            return;
        }

        // if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        //     BindBodyFrame(body);
        //     BindBody(body);
        //     return;
        // }
        //
        // if (auto link = std::dynamic_pointer_cast<ChLink>(item)) {
        //     BindLinkFrame(link);
        //     BindPointPoint(link);
        //     return;
        // }
        //
        // if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        //     mesh->UpdateVisualModel();
        //     BindDeformableMesh(mesh);
        //     return;
        // }
        //
        // if (item->GetVisualModel()) {
        //     BindDeformableMesh(item);
        //     BindPointPoint(item);
        //     if (const auto& pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item))
        //         BindParticleCloud(pcloud);
        // }
    }
} // namespace chrono::vsg3d
