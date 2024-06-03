INCLUDEPATH += $$PWD/../src/Modules/ThirdParty/yaml-cpp/include \
            $$PWD/../src/Modules/ThirdParty/jsoncpp/include/ \
            $$PWD/../src/Modules/ThirdParty/jsoncpp/ \
            $$PWD/../src/Modules/ThirdParty/jsoncpp/src/lib_json/ \
            $$PWD/../src/Modules/ThirdParty/lcm/include
            #$$PWD/src/Modules/ThirdParty/eigen3/
            
DEPENDPATH += $$PWD/../src/Modules/ThirdParty/yaml-cpp/include \
            $$PWD/../src/Modules/ThirdParty/jsoncpp/include/ \
            $$PWD/../src/Modules/ThirdParty/jsoncpp/ \
            $$PWD/../src/Modules/ThirdParty/jsoncpp/src/lib_json/ \
            $$PWD/../src/Modules/ThirdParty/lcm/include
            #$$PWD/src/Modules/ThirdParty/eigen3/
            
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lThirdParty

