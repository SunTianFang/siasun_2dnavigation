INCLUDEPATH +=$$PWD/../src/Modules/NavBase/BlackBox/include \
            $$PWD/../src/Modules/NavBase/CanDev/src/CanOpen/include/ \
            $$PWD/../src/Modules/NavBase/CanDev/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/pf_r2000/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/pf_r2000/include/pepperl_fuchs_r2000/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/Hokuyo_uam05lp/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/Sick_nanoScan3/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/WJ/include/ \
            $$PWD/../src/Modules/NavBase/LaserSensor/include/ \
            $$PWD/../src/Modules/NavBase/Network/include \
            $$PWD/../src/Modules/NavBase/Protocol/include \
            $$PWD/../src/Modules/NavBase/Mapping/include \
            $$PWD/../src/Modules/NavBase/lcm/include \
            $$PWD/../src/Modules/NavBase/Protocol/include \
            $$PWD/../src/Modules/NavBase/LaserDrivers/fr_R2/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/fr_R2/include/pepperl_fuchs_r2000_fr/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/Leimou_f30/include/

DEPENDPATH += $$PWD/../src/Modules/NavBase/BlackBox/include \
            $$PWD/../src/Modules/NavBase/CanDev/src/CanOpen/include/ \
            $$PWD/../src/Modules/NavBase/CanDev/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/pf_r2000/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/pf_r2000/include/pepperl_fuchs_r2000/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/Hokuyo_uam05lp/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/Sick_nanoScan3/include/ \
            $$PWD/../src/Modules/NavBase/LaserSensor/include/ \
            $$PWD/../src/Modules/NavBase/Network/include \
            $$PWD/../src/Modules/NavBase/Protocol/include \
            $$PWD/../src/Modules/NavBase/Mapping/include \
            $$PWD/../src/Modules/NavBase/lcm/include \
            $$PWD/../src/Modules/NavBase/Protocol/include \
            $$PWD/../src/Modules/NavBase/LaserDrivers/fr_R2/include/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/fr_R2/include/pepperl_fuchs_r2000_fr/ \
            $$PWD/../src/Modules/NavBase/LaserDrivers/Leimou_f30/include/
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lNavBase

