INCLUDEPATH += $$PWD/../src/Modules/Diagnosis \
                $$PWD/../src/Modules/Diagnosis/src/WebtoolServer \
                $$PWD/../src/Modules/Diagnosis/include

DEPENDPATH += $$PWD/../src/Modules/Diagnosis \
              $$PWD/../src/Modules/Diagnosis/src/WebtoolServer
include($$PWD/common.pri)
LIBS += -L$$LIFELONG_SLAM_DIR -lDiagnosis

