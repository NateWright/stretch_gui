#include <QtCore/QtGlobal>

#if defined(MYSHAREDLIB_LIBRARY)
#define MYSHAREDLIB_EXPORT Q_DECL_EXPORT
#else
#define MYSHAREDLIB_EXPORT Q_DECL_IMPORT
#endif