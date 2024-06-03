INCLUDEPATH += $$PWD/../src/Modules/Feature/FeatureObject/include \
                $$PWD/../src/Modules/Feature/FeatureMatch/include
DEPENDPATH += $$PWD/../src/Modules/Feature
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lFeature

