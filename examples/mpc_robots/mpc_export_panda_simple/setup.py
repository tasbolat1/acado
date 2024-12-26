from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy as np

# Define the paths to header files
include_dirs = [
    np.get_include(),              # Include NumPy headers
    "./qpoases/INCLUDE",             # qpOASES headers (.hpp)
    "./qpoases/SRC",
    ".",                           # Current directory (e.g., acado_common.h)
]

# Define the source files
sources = [
    "acado_qpoases_interface.cpp", # ACADO-qpoases interface (.cpp)
    "acado_solver.c",
    "acado_integrator.c",
    "acado_auxiliary_functions.c",    # Cython wrapper
    "solver_holder_wrapper.pyx",
    "solver_holder.c",             # Main C file
    "quaternion.c",
    "qpoases/SRC/Bounds.cpp",
	"qpoases/SRC/Constraints.cpp",
	"qpoases/SRC/CyclingManager.cpp",
	"qpoases/SRC/Indexlist.cpp",
	"qpoases/SRC/MessageHandling.cpp",
	"qpoases/SRC/QProblem.cpp",
	"qpoases/SRC/QProblemB.cpp",
	"qpoases/SRC/SubjectTo.cpp",
	"qpoases/SRC/Utils.cpp",
	"qpoases/SRC/EXTRAS/SolutionAnalysis.cpp"
]

# Create the extension module
ext_modules = [
    Extension(
        "solver_holder_wrapper",   # Name of the resulting Python module
        sources=sources,           # Source files to compile
        include_dirs=include_dirs, # Directories containing .hpp and .h files
        # libraries=["blas", "lapack"],  # Link against BLAS and LAPACK (if required)
        language="c++",            # Use C++ for ACADO-qpoases
        extra_compile_args=["-std=c++11", "-O3"],  # C++11 and optimization
        extra_link_args=["-I./qpoases"]         # Optional linker flags
    )
]

# Setup script
setup(
    name="solver_holder_wrapper",
    ext_modules=cythonize(ext_modules, verbose=True),
)
