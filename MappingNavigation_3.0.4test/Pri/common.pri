CONFIG(debug, debug|release) {
    contains(QT_ARCH, i386) {
        LIFELONG_SLAM_DIR = $$PWD ../bin_qt$$[QT_VERSION]_debug
    }
    else
    {
        LIFELONG_SLAM_DIR = $$PWD ../bin_qt$$[QT_VERSION]_debug_64
    }
}
else
{
    contains(QT_ARCH, i386) {
        LIFELONG_SLAM_DIR = $$PWD/../bin_qt$$[QT_VERSION]_release
    }
    else
    {
        LIFELONG_SLAM_DIR = $$PWD/../bin_qt$$[QT_VERSION]_release_64
    }
}
