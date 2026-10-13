# Python bindings: one extension module, pclapp._core, built with pybind11.
# Normally driven by `pip install .` (scikit-build-core, see pyproject.toml),
# which provides pybind11 and runs the install() rule below into the wheel.

set(PYBIND11_FINDPYTHON ON)  # use CMake's FindPython, not pybind11's legacy search
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(pclapp_python MODULE src/bindings.cpp)
set_target_properties(pclapp_python PROPERTIES OUTPUT_NAME _core)
target_link_libraries(pclapp_python PRIVATE pclapp_core pclapp_warnings)

install(TARGETS pclapp_python DESTINATION pclapp)

# Windows with vcpkg: PCL and its dependencies are DLLs, and Python looks for
# the DLLs of an extension module only in the module's own folder. vcpkg's
# toolchain provides this function to copy them there. It does nothing on
# Linux, and it does not exist when vcpkg is not used.
if(COMMAND x_vcpkg_install_local_dependencies)
  x_vcpkg_install_local_dependencies(TARGETS pclapp_python DESTINATION pclapp)
endif()
