/* config.h — generated for embedded Ipopt build on macOS.
 * MUMPS backend provided by libdmumps.dylib from homebrew ipopt cellar.
 * BLAS/LAPACK backend provided by macOS Accelerate framework. */
#define PACKAGE_VERSION "3.14.20"
#define PACKAGE "Ipopt"
#define PACKAGE_NAME "Ipopt"
#define PACKAGE_TARNAME "Ipopt"
#define PACKAGE_URL "https://github.com/coin-or/Ipopt"
#define PACKAGE_BUGREPORT "http://projects.coin-or.org/Ipopt"
#define IPOPT_VERSION "3.14.20"
#define IPOPT_VERSION_MAJOR 3
#define IPOPT_VERSION_MINOR 14
#define IPOPT_VERSION_RELEASE 20

/* Enable MUMPS linear solver (linked against homebrew libdmumps.dylib) */
#define IPOPT_HAS_MUMPS 1

/* Enable LAPACK (via Accelerate framework on macOS) */
#define IPOPT_HAS_LAPACK 1

/* BLAS/LAPACK functions — use Accelerate's Fortran name-mangling convention */
#define F77_FUNC(name,NAME) name ## _
#define F77_FUNC_(name,NAME) name ## _
#define IPOPT_LAPACK_FUNC(name,NAME) F77_FUNC(name,NAME)
#define IPOPT_LAPACK_FUNC_(name,NAME) F77_FUNC_(name,NAME)

/* Random number generator — standard rand() is always available */
#define IPOPT_HAS_RAND 1
#define IPOPT_HAS_DRAND48 1

/* Available standard C++ features */
#define HAVE_CMATH 1
#define HAVE_CFLOAT 1
#define HAVE_STD_ISNAN 1
#define HAVE_STD_ISINF 1

/* Threading: no OpenMP */
/* #undef HAVE_OPENMP */

/* No HSL (MA27/MA57/etc.) built in; can be loaded dynamically at runtime */
/* #undef IPOPT_HAS_HSL */
/* #undef IPOPT_HAS_PARDISO */
/* #undef IPOPT_HAS_PARDISO_MKL */
/* #undef IPOPT_HAS_SPRAL */
/* #undef IPOPT_HAS_WSMP */
