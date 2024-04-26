#ifndef CARTO_GLOBAL_H
#define CARTO_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(CARTO_LIBRARY)
#  define CARTOSHARED_EXPORT Q_DECL_EXPORT
#else
#  define CARTOSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // CARTO_GLOBAL_H
