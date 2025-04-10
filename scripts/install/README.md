# Installation Instructions
To install the project, you need to install some dependencies and clone the repository. The development is done on
Linux, but should generally work, with some modifications, on Windows and macOS as well.

There are three options to install the project:
1. Install and build the dependencies and project manually (or automated by script).
2. Use the binaries and packages provided in the [releases](TODO) section [added in the future].
3. Use the provided Docker container [added in the future].

## Dependencies
| Dependency                                                     | Version       | Notes                                       |
|----------------------------------------------------------------|---------------|---------------------------------------------|
| [Git](https://git-scm.com/)                                    | 2.30+         | Version control tool                        |
| [CMake](https://cmake.org/)                                    | 3.18+         | Needed to build the project                 |
| [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)  | 3.4.0+        | A linear algebra library                    |
| [GCC](https://gcc.gnu.org/) / [Clang](https://clang.llvm.org/) | 11.4+ / 14.0+ | Recommended C++ compiler                    |
| [Ccache](https://ccache.dev/)                                  | 4.4+          | Compiler cache for faster builds (optional) |
| [Doxygen](https://www.doxygen.nl)                              | 1.9+          | Auto documentation generation (optional)    |
| [Project Chrono](https://projectchrono.org/) (BSD-3 license)   | 9.0.1+        | Multi-physics simulation engine             |

**Project Chrono** exact commit hash: 988297616c4b7ec77906abe28620baba5f62b1bb

## 1. Install and Build the Dependencies and Project Manually
There is a script ([install_exosim.sh](/scripts/install/install_exosim.sh)) provided to install and build the
dependencies and the project from source automatically. For now, the script is only intended for Ubuntu, but it
should work on other Linux distributions with a few modifications as well.

### Manual Installation Automated by Script
Adjust the variables in the script to your needs and run the script
[install_exosim.sh](/scripts/install/install_exosim.sh) (without sudo!).
```bash
chmod +x scripts/install/install_exosim.sh
scripts/install/install_exosim.sh
```

### Manual Installation
The [*Project Chrono*](https://api.projectchrono.org/development/install_guides.html) website provides a detailed guide
on how to install the project and the required module
[*VSG*](https://api.projectchrono.org/development/module_vsg_installation.html) and optional module
[*PardisoMKL*](https://api.projectchrono.org/development/module_mkl_installation.html).
The following steps are a summary of the guide. There is also a
[Project Chrono Forum](https://groups.google.com/g/projectchrono) where you can ask questions if you encounter
problems during the installation or in general.

First of all, we can install the needed packages that can be found through package managers.
```bash
sudo apt-get update && \
sudo apt-get install -y git cmake libeigen3-dev gcc g++ clang doxygen ccache ninja-build
```
Then, we can proceed with [*Project Chrono*](https://projectchrono.org/). *ExoSim* is built on top of *Project Chrono*
and uses some of its optional modules, namely the visualization module *VSG* and the sparse linear solver module
*PardisoMKL* (which is also optional for *ExoSim* but recommended). We will first install the dependencies for
[*Project Chrono*](https://projectchrono.org/) and then build [*Project Chrono*](https://projectchrono.org/) from
source itself.

In order to be able to enable the *PardisoMKL* module later we need to install the
[*Intel MKL library*](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html?operatingsystem=linux&distributions=aptpackagemanager).
On the website, different methods are described to download the library. We can use the following commands to install
the library using the APT package manager.
```bash
sudo apt-get install -y gpg-agent wget && \
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
 | gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null && \
echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
 | sudo tee /etc/apt/sources.list.d/oneAPI.list && \
sudo apt-get update && \
sudo apt-get install -y intel-oneapi-mkl
```

In the next step, we need to install the [Vulkan SDK](https://vulkan.lunarg.com/) for the visualization module *VSG*.
Again, we can follow the instructions on the website. Exemplarily, we can use the following commands to install the SDK
for Ubuntu 22.04.
```bash
wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo tee /etc/apt/trusted.gpg.d/lunarg.asc && \
sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-1.3.296-jammy.list https://packages.lunarg.com/vulkan/1.3.296/lunarg-vulkan-1.3.296-jammy.list && \
sudo apt-get update && \
sudo apt-get -y install vulkan-sdk
```

Additional packages needed for the *VSG* module can be installed using a script provided in the
[*Project Chrono* repository](https://github.com/projectchrono/chrono/tree/main/contrib/build-scripts/vsg).
To get the correct version, we will first clone the [*Project Chrono* repository](https://github.com/projectchrono/chrono)
and checkout the desired branch. Subsequently, we can adjust the script as needed and execute it.
Substitute CHRONO_DIR and VSG_DIR with the paths where you want to place and install *Project Chrono* and the *VSG*
dependencies.
```bash
git clone --single-branch --branch 9.0.1 https://github.com/projectchrono/chrono.git CHRONO_DIR && \
cd VSG_DIR && \
cp CHRONO_DIR/contrib/build-scripts/vsg/buildVSG.sh . && \
chmod +x buildVSG.sh && \
./buildVSG.sh VSG_DIR/install
```

Finally, we can build [*Project Chrono*](https://projectchrono.org/) itself. For that, we switch to the directory where
we cloned the repository and create a build directory. We can then configure the project with CMake and build and
install it. Remember to adjust the paths (CHRONO_DIR and VSG_DIR).
```bash
cd CHRONO_DIR && \
source ~/.bashrc && \
mkdir build && mkdir install && \
cmake -G "${BUILDSYSTEM}" -B "${CHRONO_DIR}"/build -S "${CHRONO_DIR}" \
      -DCMAKE_INSTALL_PREFIX:PATH="CHRONO_DIR/install" \
      -DBUILD_SHARED_LIBS:BOOL=ON \
      -DBUILD_DEMOS:BOOL=OFF \
      -DBUILD_BENCHMARKING:BOOL=OFF \
      -DBUILD_TESTING:BOOL=OFF \
      -DENABLE_MODULE_IRRLICHT:BOOL=OFF \
      -DENABLE_MODULE_VSG:BOOL=ON \
      -DENABLE_MODULE_OPENGL:BOOL=OFF \
      -DENABLE_MODULE_VEHICLE:BOOL=OFF \
      -DIOMP5_LIBRARY="${IOMP5_DIR}" \
      -DENABLE_MODULE_POSTPROCESS:BOOL=OFF \
      -DENABLE_MODULE_MULTICORE:BOOL=OFF \
      -DENABLE_MODULE_FSI:BOOL=OFF \
      -DENABLE_MODULE_GPU:BOOL=OFF \
      -DENABLE_MODULE_PARDISO_MKL:BOOL=ON \
      -DENABLE_MODULE_CASCADE:BOOL=OFF \
      -DENABLE_MODULE_COSIMULATION:BOOL=OFF \
      -DENABLE_MODULE_SENSOR:BOOL=OFF \
      -DENABLE_MODULE_MODAL:BOOL=OFF \
      -DENABLE_MODULE_MATLAB:BOOL=OFF \
      -DENABLE_MODULE_CSHARP:BOOL=OFF \
      -DENABLE_MODULE_PYTHON:BOOL=OFF \
      -DENABLE_MODULE_SYNCHRONO:BOOL=OFF \
      -DENABLE_OPENMP:BOOL=ON \
      -DENABLE_OPENCRG:BOOL=OFF \
      -DUSE_BULLET_DOUBLE:BOOL=ON \
      -DUSE_BULLET_OPENMP:BOOL=ON \
      -DUSE_EIGEN_OPENMP:BOOL=ON \
      -DUSE_CUDA_NVRTC:BOOL=OFF \
      -DUSE_FAST_DDS:BOOL=ON \
      -DUSE_CCACHE:BOOL=ON \
      -Dvsg_DIR:PATH="VSG_DIR/install/lib/cmake/vsg" \
      -DvsgImGui_DIR:PATH="VSG_DIR/install/lib/cmake/vsgImGui" \
      -DvsgXchange_DIR:PATH="VSG_DIR/install/lib/cmake/vsgXchange" \
      -DCMAKE_BUILD_TYPE="Release" && \
cmake --build ./build --config Release --target install -- -j"$(nproc)" && \
{
  echo ""
  echo "# >>>Project Chrono<<<"
  echo "export Chrono_DIR=${CHRONO_DIR}/install/lib/cmake/Chrono"
  echo "export LD_LIBRARY_PATH=${CHRONO_DIR}/install/lib:\$LD_LIBRARY_PATH"
} >> ~/.bashrc
```

### Build ExoSim
In the last step, we can clone the *ExoSim* repository and build it.
```bash
git clone --single-branch --branch chrono_main_build git@git.sim.informatik.tu-darmstadt.de:pm75hyvy/exosim.git && \
cd exosim && mkdir build && mkdir install && \
cmake -G "${BUILDSYSTEM}" -B "${EXOSIM_DIR}"/build -S "${EXOSIM_DIR}" \
      -DCMAKE_INSTALL_PREFIX:PATH="${EXOSIM_DIR}"/install \
      -DBUILD_SHARED_LIBS:BOOL=ON \
      -DBUILD_DOCS:BOOL=ON \
      -DBUILD_OPTIMIZATION:BOOL=OFF \
      -DChrono_DIR:PATH="${Chrono_DIR}" \
      -Dvsg_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsg" \
      -DvsgImGui_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsgImGui" \
      -DvsgXchange_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsgXchange" \
      -DCMAKE_BUILD_TYPE=Release && \
cmake --build ./build --config Release --target install -- -j"$(nproc)"
```

## 2. Use the Binaries and Packages Provided in the Releases Section [added in the future]
TODO

### Project Chrono - Release Packages
If you don't want to build Project Chrono from source, you can download the pre-built binaries from the releases page.
The release packages contain the following modules:
- ExoSim (main project)
- Dependencies:
    - [Project Chrono](https://projectchrono.org/)
    - VSG: Visualization System for Chrono
    - Pardiso MKL: Sparse linear solver

## 3. Use the Provided Docker Container [added in the future]
TODO