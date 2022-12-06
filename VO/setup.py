from distutils.core import setup, Extension
from xml.etree.ElementInclude import include
from numpy import source
from pybind11.setup_helpers import Pybind11Extension, build_ext

functions_module = Pybind11Extension(
    "VO_util",
    sources=["src/VO.cpp"],
    extra_complie_args = ["-03", "-fPIC"],
    include_dirs=[
        "/home/aaa/anaconda3/envs/wpy/include/pybind11",
        "/home/aaa/anaconda3/envs/wpy/include/xtensor",
        "/home/aaa/anaconda3/envs/wpy/include",
        "include"
    ],
    language="c++"
)

setup( 
	name = "VO_util",
    version      = "0.0.1",
    description  = "a library for VO region calculation",
    author       = "Pengyu Wang",
    packages     = [],
    setup_requires = ["pybind11"],
    install_requires = ["pybind11"],
    include_package_data=True,
    zip_safe=False,
    ext_modules  = [functions_module],
    cmdclass={"build_ext":build_ext}
	)
