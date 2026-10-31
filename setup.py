import os
import sys
from setuptools import setup, find_packages, Extension

try:
    from Cython.Build import cythonize
    USE_CYTHON = True
except ImportError:
    USE_CYTHON = False

source_ext = ".pyx" if USE_CYTHON else ".cpp"

extra_compile_args = []
extra_link_args = []

if sys.platform != "win32":
    extra_compile_args.extend(["-O3", "-std=c++11"])

pkg_dir = os.path.abspath("dijkstra_fast_cpp")

extensions = [
    Extension(
        name="dijkstra_fast_cpp._dijkstra_cython",
        sources=[
            os.path.join("dijkstra_fast_cpp", f"_dijkstra_cython{source_ext}"),
            os.path.join("dijkstra_fast_cpp", "dijkstra.cpp"),
        ],
        include_dirs=[pkg_dir],
        language="c++",
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
    ),
]

if USE_CYTHON:
    ext_modules = cythonize(
        extensions,
        include_path=[pkg_dir, "."],
        compiler_directives={"language_level": "3"},
    )
else:
    ext_modules = extensions

setup(
    packages=find_packages(),
    ext_modules=ext_modules,
    package_data={
        "dijkstra_fast_cpp": ["*.h", "*.pxd", "*.pyx", "*.cpp", "data/*.json"],
    },
)
