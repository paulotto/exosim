#!/bin/bash

# ---------------------------------------------------------------------------------------------------------------------
# A bash script to install the ExoSim library with its dependencies more easily.
# ---------------------------------------------------------------------------------------------------------------------

UBUNTU_VERSION="22.04" # 24.04
VULKAN_VERSION="1.3.296"
CHRONO_VERSION="988297616c4b7ec77906abe28620baba5f62b1bb"

EXOSIM_BUILD_TYPE="Release"
EXOSIM_DIR=$(dirname "$(dirname "$(dirname "$(realpath "$0")")")")
SCRIPT_EXECUTED_DIR=$(dirname "$(realpath "$0")")

CHRONO_BUILD_TYPE="Release"
CHRONO_DIR="${EXOSIM_DIR}/../exosim_dependencies/chrono"
VSG_DIR="${EXOSIM_DIR}/../exosim_dependencies/vsg"

BUILDSYSTEM="Ninja Multi-Config"

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Install the required dependencies.
sudo apt-get update
echo -e "${BLUE}Installing required dependencies...${NC}"
sudo apt-get install -y git cmake libeigen3-dev gcc g++ clang doxygen ccache ninja-build gpg-agent wget unzip
echo -e "${GREEN}Dependencies installed.${NC}"

# Install PardisoMKL
echo -e "${BLUE}Installing PardisoMKL...${NC}"
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
 | gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null

echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
 | sudo tee /etc/apt/sources.list.d/oneAPI.list

sudo apt-get update
sudo apt-get install -y intel-oneapi-mkl-devel || { echo -e "${RED}Failed to install PardisoMKL.${NC}"; exit 1; }
# sudo apt install intel-oneapi-mkl
echo -e "${GREEN}PardisoMKL installed.${NC}"

# Clone Project Chrono
echo -e "${BLUE}Cloning Project Chrono...${NC}"
mkdir -p "${CHRONO_DIR}"
git clone --branch "${CHRONO_VERSION}" https://github.com/projectchrono/chrono.git "${CHRONO_DIR}" \
  || { echo -e "${RED}Failed to clone Project Chrono.${NC}"; exit 1; }
echo -e "${GREEN}Project Chrono cloned.${NC}"

# Install Project Chrono dependencies
echo -e "${BLUE}Installing Project Chrono dependencies...${NC}"

## Install Vulkan SDK
echo -e "${BLUE}Installing Vulkan SDK...${NC}"
if [ "${UBUNTU_VERSION}" = "22.04" ]; then
  wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo tee /etc/apt/trusted.gpg.d/lunarg.asc
  sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-"${VULKAN_VERSION}"-jammy.list \
   https://packages.lunarg.com/vulkan/"${VULKAN_VERSION}"/lunarg-vulkan-"${VULKAN_VERSION}"-jammy.list
elif [ "${UBUNTU_VERSION}" = "24.04" ]; then
  wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo apt-key add -
  sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan-"${VULKAN_VERSION}"-noble.list \
   https://packages.lunarg.com/vulkan/"${VULKAN_VERSION}"/lunarg-vulkan-"${VULKAN_VERSION}"-noble.list
else
  echo "${RED}Only Ubuntu 22.04 and 24.04 are supported with this script. Please install the Vulkan SDK manually.${NC}"
  exit 1
fi

sudo apt-get update
sudo apt-get install -y vulkan-sdk || { echo -e "${RED}Failed to install Vulkan SDK.${NC}"; exit 1; }
echo -e "${GREEN}Vulkan SDK installed.${NC}"

## Build & install VSG
echo -e "${BLUE}Building and installing VSG...${NC}"
mkdir -p "${VSG_DIR}"
cd "${VSG_DIR}" || { echo -e "${RED}Failed to change directory to ${VSG_DIR}.${NC}"; exit 1; }
cp "${CHRONO_DIR}"/contrib/build-scripts/vsg/buildVSG.sh .
chmod +x "${VSG_DIR}"/buildVSG.sh
"${VSG_DIR}"/buildVSG.sh "${VSG_DIR}"/install || { echo -e "${RED}Failed to build VSG.${NC}"; exit 1; }
cd "${SCRIPT_EXECUTED_DIR}" || { echo -e "${RED}Failed to change directory to ${SCRIPT_EXECUTED_DIR}.${NC}"; exit 1; }

### Add VSG environment variables to .bashrc
{
  echo ""
  echo "# >>>VSG<<<"
  echo "export LD_LIBRARY_PATH=${VSG_DIR}/install/lib:\$LD_LIBRARY_PATH"
  echo "export CMAKE_PREFIX_PATH=${VSG_DIR}/install/lib/cmake:\$CMAKE_PREFIX_PATH"
  echo "export VSG_FILE_PATH=${VSG_DIR}/install/share/vsgExamples"
} >> ~/.bashrc

export LD_LIBRARY_PATH=${VSG_DIR}/install/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=${VSG_DIR}/install/lib/cmake:$CMAKE_PREFIX_PATH

echo -e "${GREEN}VSG built and installed.${NC}"
echo -e "${GREEN}Project Chrono dependencies installed.${NC}"

# Build & install Project Chrono
echo -e "${BLUE}Building and installing Project Chrono...${NC}"
mkdir -p "${CHRONO_DIR}"/build && mkdir -p "${CHRONO_DIR}"/install

cmake -G "${BUILDSYSTEM}" -B "${CHRONO_DIR}"/build -S "${CHRONO_DIR}" \
      -DCMAKE_INSTALL_PREFIX:PATH="${CHRONO_DIR}"/install \
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
      -Dvsg_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsg" \
      -DvsgImGui_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsgImGui" \
      -DvsgXchange_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsgXchange" \
      -DCMAKE_BUILD_TYPE="${CHRONO_BUILD_TYPE}" # \
#      -DMKL_INCLUDE_DIR:PATH="/opt/intel/oneapi/mkl/latest/include" \
#      -DMKL_RT_LIBRARY:FILEPATH="/opt/intel/oneapi/mkl/latest/lib/libmkl_rt.so" \
#      -DCCACHE_EXE:PATH="${CCACHE_EXE}" \
#      -DEIGEN3_INCLUDE_DIR:PATH="${EIGEN3_INSTALL_DIR}" \
#      -DIRRLICHT_INSTALL_DIR:PATH="${IRRLICHT_INSTALL_DIR}" \
#      -DIRRLICHT_LIBRARY:FILEPATH="${IRRLICHT_LIBRARY}" \
#      -DBLAZE_INSTALL_DIR:PATH="${BLAZE_INSTALL_DIR}" \
#      -DTHRUST_INCLUDE_DIR:PATH="${THRUST_INSTALL_DIR}" \
#      -DOptiX_INSTALL_DIR:PATH="${OPTIX_INSTALL_DIR}" \
#      -Dfastrtps_INSTALL_DIR:PATH="${FASTRTPS_INSTALL_DIR}" \
#      -DGLEW_DIR="${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glew"\
#      -Dglfw3_DIR="${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glfw3" \
#      -DGLM_INCLUDE_DIR:PATH="${GL_INSTALL_DIR}/include" \
#      -DOpenCRG_INCLUDE_DIR:PATH="${CRG_INCLUDE_DIR}" \
#      -DOpenCRG_LIBRARY:FILEPATH="${CRG_LIBRARY}" \
#      -DOpenCASCADE_DIR:PATH="${CASCADE_INSTALL_DIR}/adm" \
#      -DSPECTRA_INCLUDE_DIR:PATH="${SPECTRA_INSTALL_DIR}/include" \
#      -DMATLAB_SDK_ROOT:PATH="${MATLAB_INSTALL_DIR}/extern" \
#      -Durdfdom_DIR:PATH="${URDF_INSTALL_DIR}/lib/urdfdom/cmake" \
#      -Durdfdom_headers_DIR:PATH="${URDF_INSTALL_DIR}/lib/urdfdom_heders/cmake" \
#      -Dconsole_bridge_DIR:PATH="${URDF_INSTALL_DIR}/lib/console_bridge/cmake" \
#      -Dtinyxml2_DIR:PATH="${URDF_INSTALL_DIR}/CMake" \
#      -DSWIG_EXECUTABLE:FILEPATH="${SWIG_EXE}" \

cmake --build "${CHRONO_DIR}"/build --config "${CHRONO_BUILD_TYPE}" --target install -- -j"$(nproc)" \
 || { echo -e "${RED}Failed to build Project Chrono.${NC}"; exit 1; }

# Add Chrono environment variables to .bashrc
{
  echo ""
  echo "# >>>Project Chrono<<<"
  echo "export Chrono_DIR=${CHRONO_DIR}/install/lib/cmake/Chrono"
  echo "export LD_LIBRARY_PATH=${CHRONO_DIR}/install/lib:\$LD_LIBRARY_PATH"
} >> ~/.bashrc

export Chrono_DIR=${CHRONO_DIR}/install/lib/cmake/Chrono
export LD_LIBRARY_PATH=${CHRONO_DIR}/install/lib:$LD_LIBRARY_PATH

echo -e "${GREEN}Project Chrono built and installed.${NC}"

# Build & install ExoSim
echo -e "${BLUE}Building and installing ExoSim...${NC}"
mkdir -p "${EXOSIM_DIR}"/build && mkdir -p "${EXOSIM_DIR}"/install

cmake -G "${BUILDSYSTEM}" -B "${EXOSIM_DIR}"/build -S "${EXOSIM_DIR}" \
      -DCMAKE_INSTALL_PREFIX:PATH="${EXOSIM_DIR}"/install \
      -DBUILD_SHARED_LIBS:BOOL=ON \
      -DBUILD_DOCS:BOOL=ON \
      -DBUILD_TESTING:BOOL=ON \
      -DBUILD_OPTIMIZATION:BOOL=OFF \
      -DChrono_DIR:PATH="${Chrono_DIR}" \
      -Dvsg_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsg" \
      -DvsgImGui_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsgImGui" \
      -DvsgXchange_DIR:PATH="${VSG_DIR}/install/lib/cmake/vsgXchange" \
      -DCMAKE_BUILD_TYPE="${EXOSIM_BUILD_TYPE}"

cmake --build "${EXOSIM_DIR}"/build --config "${EXOSIM_BUILD_TYPE}" --target jaw_model -- -j"$(nproc)" \
 || { echo -e "${RED}Failed to build ExoSim.${NC}"; exit 1; }
#cmake --build "${EXOSIM_DIR}"/build --config "${EXOSIM_BUILD_TYPE}" --target install -- -j"$(nproc)" \
# || { echo -e "${RED}Failed to build ExoSim.${NC}"; exit 1; }

echo -e "${GREEN}ExoSim built and installed.${NC}"
