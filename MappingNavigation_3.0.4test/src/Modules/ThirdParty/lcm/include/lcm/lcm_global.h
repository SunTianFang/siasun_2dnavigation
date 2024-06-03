#ifndef LCM_GLOBAL_H
#define LCM_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(LCM_LIBRARY)
#  define LCMSHARED_EXPORT Q_DECL_EXPORT
#else
#  define LCMSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // LCM_GLOBAL_H
