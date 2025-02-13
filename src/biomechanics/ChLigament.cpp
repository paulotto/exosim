/**
 * @file ChLigament.cpp
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 24.08.2023
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

#include "exosim/biomechanics/ChLigament.h"


namespace chrono::biomechanics {
    CH_FACTORY_REGISTER(ChLigamentDefault)

    CH_FACTORY_REGISTER(ChLigamentBlankevoort)

    ChLigament::ChLigamentProperties::ChLigamentProperties(
        std::string name, ChLigamentParameterVector params,
        const std::shared_ptr<ChBody>& body_1,
        const std::shared_ptr<ChBody>& body_2,
        const ChVector3d& loc_1,
        const ChVector3d& loc_2,
        bool local_coordinates,
        bool rel_to_ref,
        const ChColor& color,
        const std::shared_ptr<ChVisualShapePointPoint>& shape): name(std::move(name)),
                                                                params(std::move(params)),
                                                                body_1(body_1),
                                                                body_2(body_2),
                                                                loc_1(loc_1),
                                                                loc_2(loc_2),
                                                                local_coordinates(local_coordinates),
                                                                rel_to_ref(rel_to_ref),
                                                                color(color),
                                                                shape(shape) {
    }

    void ChLigament::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChLigament>();

        // Serialize parent class
        ChLinkTSDA::ArchiveOut(archive_out);

        // Serialize enums
        ChLigamentEnumMapper::ChLigamentType_mapper type_mapper;
        archive_out << CHNVP(type_mapper(type_), "type_");

        // Serialize all member data
        archive_out << CHNVP(k_c_);
        archive_out << CHNVP(k_e_);
        archive_out << CHNVP(l_rest_percentage_);
        archive_out << CHNVP(ligament_force_);
    }

    void ChLigament::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChLigament>();

        // Deserialize parent class
        ChLinkTSDA::ArchiveIn(archive_in);

        // Deserialize enums
        ChLigamentEnumMapper::ChLigamentType_mapper type_mapper;
        archive_in >> CHNVP(type_mapper(type_), "type_");

        // Deserialize all member data
        archive_in >> CHNVP(k_c_);
        archive_in >> CHNVP(k_e_);
        archive_in >> CHNVP(l_rest_percentage_);
        archive_in >> CHNVP(ligament_force_);
    }

    double ChLigamentDefault::ComputeLigamentForce(double time,
                                                   double rest_length,
                                                   double length,
                                                   double vel,
                                                   const ChLigament& ligament) {
        double f{0.0};
        const double delta_l{length - rest_length};
        const double w{l_rest_percentage_ * rest_length};
        const double s{(delta_l + w / 2.0) / w};
        const double f_c{k_c_ * delta_l};
        const double f_e{k_e_ * delta_l};

        // GetLog() << "rest length: " << rest_length << '\n'; // TODO: test

        if (s >= 1.0) {
            f = f_e + this->GetDampingCoefficient() * vel;
        } else if (s <= 0.0) {
            f = f_c + this->GetDampingCoefficient() * vel;
        } else {
            // l < l_0 + w/2
            double a_3 = 2.0 * f_c - 2.0 * f_e + w * (k_c_ + k_e_);
            double a_2 = -3.0 * f_c + 3.0 * f_e - 2.0 * w * k_c_ - w * k_e_;
            double a_1 = w * k_c_;
            double a_0 = f_c;

            f = ((a_3 * s + a_2) * s + a_1) * s + a_0 + this->GetDampingCoefficient() * vel;
        }

        return -f;
    }

    double ChLigamentBlankevoort::ComputeLigamentForce(double time,
                                                       double rest_length,
                                                       double length,
                                                       double vel,
                                                       const ChLigament& ligament) {
        double f{0.0}, f_s{0.0}, f_d{0.0}, e{0.0}, e_dot{0.0};

        // Slack length / Zero load length
        const double& l_slack = this->GetRestLength();

        // Strain
        e = (length - l_slack) / l_slack;

        // Strain rate
        e_dot = vel / l_slack;

        // Linear part
        if (e > 2 * e_transition_) f_s = k_e_ * (e - e_transition_);
        // Quadratic part
        if (e >= 0.0 && e <= 2 * e_transition_) f_s = 0.25 * k_e_ * e * e / e_transition_;

        if (e < 0.0) f_s = 0.0;

        // Damping force
        f_d = this->GetDampingCoefficient() * e_dot;

        // Total force
        f = std::max<double>(0.0, f_s + f_d);

        if (verbose_ && time > time_prev_) {
            fmt::print(utils::DEBUG_MSG, "[DEBUG] [{:^9.6}] [ChLigamentBlankevoort] {:<55s} "
                       "| force: {:>10.6f} | length: {:>10.6f} | vel: {:>10.6f} | strain: {:>10.6f} | strain rate: {:>10.6f}\n",
                       time, ligament.GetName(), -f, length, vel, e, e_dot);
        }
        time_prev_ = time;

        return -f;
    }

    void ChLigamentBlankevoort::ArchiveOut(ChArchiveOut& archive_out) {
        read_lock lock(mtx_);

        // Version number
        archive_out.VersionWrite<ChLigamentBlankevoort>();

        // Serialize parent class
        ChLigament::ArchiveOut(archive_out);

        // Serialize all member data
        archive_out << CHNVP(l_ref_);
        archive_out << CHNVP(e_ref_);
        archive_out << CHNVP(e_transition_);
    }

    void ChLigamentBlankevoort::ArchiveIn(ChArchiveIn& archive_in) {
        write_lock lock(mtx_);

        // Version number
        int version = archive_in.VersionRead<ChLigamentBlankevoort>();

        // Deserialize parent class
        ChLigament::ArchiveIn(archive_in);

        // Deserialize all member data
        archive_in >> CHNVP(l_ref_);
        archive_in >> CHNVP(e_ref_);
        archive_in >> CHNVP(e_transition_);
    }
} // namespace chrono::biomechanics
