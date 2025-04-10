# ExoSim
<a href="#"><img src="https://img.shields.io/badge/C++-17+-blue?logo=c%2B%2B&style=for-the-badge" /></a>
<a href="https://paulotto.github.io"><img src="https://img.shields.io/badge/Website-ExoSim-color?style=for-the-badge&color=rgb(187%2C38%2C73)" /></a>

This project is intended as an extension of the [*Project Chrono*](https://projectchrono.org/) multi-physics simulation engine 
for simulating biomechanical systems and exoskeletons. For now, the main focus is on the simulation of the 
human jaw, which is supposed to provide the basis for developing a jaw exoskeleton for treating temporomandibular
disorders (TMDs) in the future.

By abstracting the model parameters and components to *JSON* files, the simulation can be more easily configured, 
extended, and adapted to different individuals and applications. 

![ExoSim](resources/.media/jaw_fem_vsg.gif)

## Table Of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Roadmap](#roadmap)
- [License](#license)
- [Copyright Notice](#copyright-notice)
- [Third-Party Dependencies](#third-party-dependencies)
- [Authors and Acknowledgment](#authors-and-acknowledgment)

## Installation
Installation instructions can be found in the [scripts/install/README.md](scripts/install/README.md) file. 
Different installation methods are provided, including a script to install the dependencies and the project from source
automatically. The script is only intended for Ubuntu, but it should work on other Linux distributions and Windows or
macOS with a few modifications as well.

## Features
- **Biomechanical Jaw Simulation**: Three jaw model variants are available depending on the intended use case:
  - A basic rigid body model with the TMJs constrained by point-on-surface constraints -> useful for general motion
    analysis, kinematics, and rapid prototyping of jaw exoskeletons
  - A basic rigid body model with the TMJs constrained by the natural bone meshes -> useful for simulating the jaw
    with a more realistic contact model and getting rough estimates of the forces acting on the TMJs 
  - A more complex, hybrid model combining rigid body and finite element models (the articular disc is modeled with FEM)
    -> useful for simulating the jaw with more realistic articular discs and getting more accurate estimates of the 
    forces acting on and inside the TMJs (although with a significant time overhead)
- **Configurable Simulation**: The simulation can be configured using *JSON* files, which allows for easy adjustments of
  the model parameters and components. This makes it easier to adapt the simulation to different individuals and 
  applications with less programming effort.

## Usage
For now, there are only three jaw model variants available, which can be used to simulate the jaw with different levels 
of complexity.

The parameters of the jaw simulation can be adjusted conveniently in the configuration JSON files located in the 
[resources/json](/resources/json) directory. A simple example can be found in the 
[jaw_model.cpp](/src/apps/jaw_model.cpp) file, which can be started by running the executable *jaw_model* after 
building the project.

There are four JSON files that can be used to configure the simulation:
- [jaw.json](/resources/json/jaw.json): Contains the general parameters for the jaw simulation.
- [muscles.json](/resources/json/muscles.json): Contains the parameters for the muscles.
- [ligaments.json](/resources/json/ligaments.json): Contains the parameters for the ligaments.
- [fea.json](/resources/json/fea.json): Contains the parameters for the finite element articular disc.

A description and explanation of the simulation configuration can be found in the 
[resources/json/README.md](resources/json/README.md) file.

## Roadmap
- [x] Add three jaw model variants 
  - [x] Basic rigid body model with the TMJs constrained by point-on-surface constraints
  - [x] Basic rigid body model with the TMJs constrained by the natural bone meshes
  - [x] More complex, hybrid model combining rigid body and finite element models
- [ ] Add optimization framework to generate jaw muscle activations 
- [ ] Simulation of exoskeletons and tools for developing and evaluating exoskeletons

## License
This project uses two licenses to cover different parts of the repository:

1. **Source Code**:  
   This project is licensed under the terms of the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html) or (at your option) any later version.

   You are free to use, modify, and distribute this software under the terms of this license. See the [LICENSE](LICENSE.md) file for the full text of the license.

2. **Blender Models**:  
   The Blender models located in the [blender](resources/blender) directory are licensed under the Creative Commons Attribution 4.0 International License (CC BY 4.0). See the [LICENSE_MODELS](LICENSE_MODELS.md) file in the [blender](resources/blender) directory for details.

## Copyright Notices

### Project Chrono
This project uses the [Project Chrono](https://projectchrono.org/) library, which is copyrighted by the
*Project Chrono Development Team* and licensed under the  BSD-3 license. 
See [Project Chrono LICENSE](https://projectchrono.org/license-chrono.txt) for details.

### Spline
This project uses the following third-party library, which is licensed under the [GNU General Public License v2](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html):

- **spline**: A lightweight implementation of cubic splines to interpolate points f(xi) = yi
  - License: GNU General Public License v2
  - Source: https://github.com/ttk592/spline

The use of this library requires this project to also comply with the terms of the GNU General Public License v2 
or later.

## Authors and Acknowledgment
- Paul-Otto Müller, SIM Group, TU Darmstadt
- TODO: Add citation
