TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ../asv-swarm/control.cc \
    ../plugins/models/asvdynamics/ASVDynamicsPlugin.cc \
    ../plugins/sensors/gps/GpsPlugin.cc

DISTFILES += \
    ../README.txt \
    ../README.asciidoc \
    ../models/create_swarm_asv_models.sh \
    ../worlds/surface.world.template \
    ../worlds/create_swarm_world.py \
    ../models/asv_model/model.config \
    ../models/asv_model/model.sdf

HEADERS += \
    ../plugins/models/asvdynamics/ASVDynamicsPlugin.hh \
    ../plugins/sensors/gps/GpsPlugin.hh
