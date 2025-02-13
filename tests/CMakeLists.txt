include(FetchContent)
set(FETCHCONTENT_QUIET TRUE)

#-----------------------------------------------------------------------------
# === GooglTest ===
#-----------------------------------------------------------------------------
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Make content available
FetchContent_MakeAvailable(googletest)

# Include GoogleTest headers
#include_directories("${googletest_SOURCE_DIR}/googletest/include/gtest")

# Add test executable
#add_executable(test_hdf5 optimization/test_hdf5.cpp)

# Link test executable with GoogleTest and HDF5 libraries
#target_link_libraries(test_hdf5 PRIVATE GTest::gtest_main exosim::optimization)

# Enable C++20 standard
#target_compile_features(test_hdf5 PRIVATE cxx_std_17)

# Register the test with CTest
#add_test(NAME test_hdf5 COMMAND test_hdf5)
