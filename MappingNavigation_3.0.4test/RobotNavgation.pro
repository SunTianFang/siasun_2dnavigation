TEMPLATE = subdirs

SUBDIRS += \
           src/Modules/ThirdParty \
           src/Modules/Tools \
           src/Modules/Geometry \
           src/Modules/Carto \
           src/Modules/Csm \
           src/Modules/Scan \
           src/Modules/NavBase \
           src/Modules/Feature \
           src/Modules/Ndt/ndt_oru \
           src/Modules/Ndt/ndt_omp \
           src/Modules/Methods \
           src/Modules/Diagnosis \
           src/Modules/SoftPls \
#           src/Modules/World \
#           src/Modules/Traj \
           src/App/NavApp

CONFIG += ordered
CONFIG += c++14
