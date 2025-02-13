/**
 * @file ChUtilsJSON.h
 * @brief TODO
 * @ref Chrono sensor module ("chrono_sensor/utils/ChUtilsJSON.h")
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

#ifndef EXOSIM_BIOMECHANICS_CH_UTILS_JSON_H
#define EXOSIM_BIOMECHANICS_CH_UTILS_JSON_H

#include "exosim/biomechanics/ChJaw.h"
//
#include <chrono/core/ChQuaternion.h>
#include <chrono/core/ChVector3.h>
//
#include <chrono_thirdparty/rapidjson/document.h>


namespace chrono::biomechanics::utils {
    /**
     * Load and return a RapidJSON document from the specified file.
     * A Null document is returned if the file cannot be opened.
     * @param filename The path to the file to be parsed.
     * @param type The type of the file to be read (JSON file needs to have a member 'Type').
     * @param d The rapid JSON document corresponding to the file name.
     */
    void ReadFileJSON(const std::string& filename, const std::string& type, rapidjson::Document& d);

    /**
     * Write the specified RapidJSON document to the specified file.
     * @param filename The path to the file to be written.
     * @param d The rapid JSON document to be written to the file.
     */
    void WriteFileJSON(const std::string& filename, const rapidjson::Document& d);

    // -----------------------------------------------------------------------------

    /**
     * Load and return a jaw model from the specified JSON file.\n
     * \b NOTE: Build() must be called on the jaw object \b before (only once after creation of a ChJaw object)
     * and \b after loading the JSON file.
     * @param filename The name/path to the JSON file defining the jaw parameters.
     * @return A shared pointer to a ChJaw constructed from the JSON file.
     */
    std::shared_ptr<ChJaw> ReadJawJSON(const std::string& filename);

    /**
     * Load the jaw model from the specified JSON file.\n
     * \b NOTE: Build() must be called on the jaw object \b before (only once after creation of a ChJaw object)
     * and \b after loading the JSON file.
     * @param jaw The jaw object to be updated with the JSON file.
     * @param filename The name/path to the JSON file defining the jaw parameters.
     */
    void ReadJawJSON(ChJaw& jaw, const std::string& filename);

    /**
     * Write the jaw model to the specified JSON file.
     * @param jaw The jaw object to be written to the JSON file.
     * @param filename The name/path to the JSON file defining the jaw parameters.
     */
    void WriteJawJSON(ChJaw& jaw, const std::string& filename);

    // -----------------------------------------------------------------------------

    /**
     * Load the muscle properties from the specified JSON file.
     * @param jaw The jaw object to get the attachement bodies from.
     * @param filename The name/path to the JSON file defining the muscle parameters.
     */
    void ReadMusclePropertiesJSON(ChJaw& jaw, const std::string& filename);

    /**
     * Load the ligament properties from the specified JSON file.
     * @param jaw The jaw object to get the attachement bodies from.
     * @param filename The name/path to the JSON file defining the ligament parameters.
     */
    void ReadLigamentPropertiesJSON(ChJaw& jaw, const std::string& filename);

    /**
     * Load the FEM properties from the specified JSON file.
     * @param jaw The jaw object to be updated with the FEM properties.
     * @param filename The name/path to the JSON file defining the FEM properties.
     */
    void ReadFEAPropertiesJSON(ChJaw& jaw, const std::string& filename);

    /**
     * Write the muscle properties to the specified JSON file.
     * @param jaw The jaw object to be written to the JSON file.
     * @param filename The name/path to the JSON file defining the muscle parameters.
     */
    void WriteMusclePropertiesJSON(ChJaw& jaw, const std::string& filename);

    /**
     * Write the ligament properties to the specified JSON file.
     * @param jaw The jaw object to be written to the JSON file.
     * @param filename The name/path to the JSON file defining the ligament parameters.
     */
    void WriteLigamentPropertiesJSON(ChJaw& jaw, const std::string& filename);

    /**
     * Write the FEM properties to the specified JSON file.
     * @param jaw The jaw object to be written to the JSON file.
     * @param filename
     */
    void WriteFEAPropertiesJSON(ChJaw& jaw, const std::string& filename);

    // -----------------------------------------------------------------------------

    /**
     * Load and return a ChVector3d from the specified JSON array.
     * @param a The value to be read (array).
     * @return A ChVector containing the values in ChVector format.
     */
    ChVector3d ReadVector3dJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChVector2d from the specified JSON array.
     * @param a The value to be read (array).
     * @return A ChVector2d containing the values in ChVector2d format.
     */
    ChVector2d ReadVector2dJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChVector2i from the specified JSON array.
     * @param a The value to be read (array).
     * @return A ChVector2i containing the values in ChVector2i format.
     */
    ChVector2i ReadVector2iJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChQuaternion from the specified JSON array.
     * @param a The value to be read (array).
     * @return A ChQuaternion generated from the JSON value.
     */
    ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChFrame from the specified JSON array.
     * @param a The value to be read  (array).
     * @return A ChFrame generated from the JSON value.
     */
    ChFrame<> ReadFrameJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChColor from the specified JSON array.
     * @param a The value to be read (array).
     * @return A ChColor generated from the JSON value.
     */
    ChColor ReadColorJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChVisualMaterial from the specified JSON value.
     * @param a The value to be read (array).
     * @return A ChVisualShapeSpring generated from the JSON value.
     */
    std::shared_ptr<ChVisualShapeSpring> ReadVisualShapeSpringJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChMuscleParameterVector from the specified JSON value.
     * @param a The value to be read (array).
     * @return A ChMuscleParameterVector generated from the JSON value.
     */
    ChMuscle::ChMuscleParameterVector ReadMuscleParametersJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChLigamentParameterVector from the specified JSON value.
     * @param a The value to be read (array).
     * @return A ChLigamentParameterVector generated from the JSON value.
     */
    ChLigament::ChLigamentParameterVector ReadLigamentParametersJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChContactMaterialPropertiesSMC object from the specified JSON value.
     * @param a The value to be read (array).
     * @return A ChContactMaterialPropertiesSMC object generated from the JSON value.
     */
    ChJaw::ChContactMaterialPropertiesSMC ReadContactMaterialPropertiesSMCJSON(const rapidjson::Value& a);

    /**
     * Load and return a ChContactMaterialPropertiesNSC object from the specified JSON value.
     * @param a The value to be read (array).
     * @return A ChContactMaterialPropertiesNSC object generated from the JSON value.
     */
    ChJaw::ChContactMaterialPropertiesNSC ReadContactMaterialPropertiesNSCJSON(const rapidjson::Value& a);

    /**
     * Write a ChVector3d to the specified JSON value.
     * @param a The value to be written to.
     * @param vec The ChVector3d to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteVector3dJSON(rapidjson::Value& a, const ChVector3d& vec, rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChVector2d to the specified JSON value.
     * @param a The value to be written to.
     * @param vec The ChVector2d to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteVector2dJSON(rapidjson::Value& a, const ChVector2d& vec, rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChVector2i to the specified JSON value.
     * @param a The value to be written to.
     * @param vec The ChVector2i to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteVector2iJSON(rapidjson::Value& a, const ChVector2i& vec, rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChQuaternion to the specified JSON value.
     * @param a The value to be written to.
     * @param quat The ChQuaternion to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteQuaternionJSON(rapidjson::Value& a,
                             const ChQuaternion<>& quat,
                             rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChFrame to the specified JSON value.
     * @param a The value to be written to.
     * @param frame The ChFrame to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteFrameJSON(rapidjson::Value& a, const ChFrame<>& frame, rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChColor to the specified JSON value.
     * @param a The value to be written to.
     * @param color The ChColor to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteColorJSON(rapidjson::Value& a, const ChColor& color, rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChVisualShapeSpring to the specified JSON value.
     * @param a The value to be written to.
     * @param spring The ChVisualShapeSpring to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteVisualShapeSpringJSON(rapidjson::Value& a,
                                    const std::shared_ptr<ChVisualShapeSpring>& spring,
                                    rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChMuscleParameterVector to the specified JSON value.
     * @param a The value to be written to.
     * @param params The ChMuscleParameterVector to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteMuscleParametersJSON(rapidjson::Value& a,
                                   const ChMuscle::ChMuscleParameterVector& params,
                                   rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChLigamentParameterVector to the specified JSON value.
     * @param a The value to be written to.
     * @param params The ChLigamentParameterVector to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteLigamentParametersJSON(rapidjson::Value& a,
                                     const ChLigament::ChLigamentParameterVector& params,
                                     rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChContactMaterialPropertiesSMC to the specified JSON value.
     * @param a The value to be written to.
     * @param props The ChContactMaterialPropertiesSMC to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteContactMaterialPropertiesSMCJSON(rapidjson::Value& a,
                                               const ChJaw::ChContactMaterialPropertiesSMC& props,
                                               rapidjson::Document::AllocatorType& allocator);

    /**
     * Write a ChContactMaterialPropertiesNSC to the specified JSON value.
     * @param a The value to be written to.
     * @param props The ChContactMaterialPropertiesNSC to be written.
     * @param allocator The allocator to be used for the JSON value.
     */
    void WriteContactMaterialPropertiesNSCJSON(rapidjson::Value& a,
                                               const ChJaw::ChContactMaterialPropertiesNSC& props,
                                               rapidjson::Document::AllocatorType& allocator);

    // -----------------------------------------------------------------------------

    /**
     * Load and return a std::string from the specified JSON value.
     * Will check if member exists and returns if it does, def if not.
     * @param str A string parsed from the JSON value or default if none exists.
     * @param value The JSON value to be parsed.
     * @param member A member from the file to be read.
     * @param def A default value to use if not defined in JSON file.
     */
    inline void GetStringMemberWithDefault(std::string& str,
                                           const rapidjson::Value& value,
                                           const char* member,
                                           const char* def = "");

    /**
     * Extracts the path and filename without extension from the specified filepath.
     * @param filepath The path to the file.
     * @return The path and filename without extension.
     */
    inline std::string ExtractPathAndFilenameWithoutExtension(const std::string& filepath);
} // namespace chrono::biomechanics

#endif //EXOSIM_BIOMECHANICS_CH_UTILS_JSON_H
