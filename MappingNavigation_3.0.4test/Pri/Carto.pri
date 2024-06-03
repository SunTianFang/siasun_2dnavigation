

INCLUDEPATH += $$PWD/../src/Modules/Carto/common/include/ \
            $$PWD/../src/Modules/Carto/mapping/include/ \
            $$PWD/../src/Modules/Carto/sensor/include/ \
            $$PWD/../src/Modules/Carto/transform/include/ \
            $$PWD/../src/Modules/Carto/myceres/include/ \
            $$PWD/../src/Modules/Carto

DEPENDPATH += $$PWD/../src/Modules/Carto/common/include/ \
            $$PWD/../src/Modules/Carto/mapping/include/ \
            $$PWD/../src/Modules/Carto/sensor/include/ \
            $$PWD/../src/Modules/Carto/transform/include/ \
            $$PWD/../src/Modules/Carto/myceres/include/ \
             $$PWD/../src/Modules/Carto

include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lCarto

