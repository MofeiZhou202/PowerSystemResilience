/* config.h - generated for embedded Ipopt build.
 * This is intentionally self-contained: Ipopt's config_default.h delegates to
 * an MSVC-only fallback header and fails on Clang/GCC when HAVE_CONFIG_H is set. */

#ifndef IPOPTLIB_EXPORT
# if defined(_WIN32) && defined(DLL_EXPORT)
#  define IPOPTLIB_EXPORT __declspec(dllexport)
# elif defined(__GNUC__) && __GNUC__ >= 4
#  define IPOPTLIB_EXPORT __attribute__((__visibility__("default")))
# else
#  define IPOPTLIB_EXPORT
# endif
#endif

#ifndef SIPOPTLIB_EXPORT
# if defined(_WIN32) && defined(DLL_EXPORT)
#  define SIPOPTLIB_EXPORT __declspec(dllexport)
# elif defined(__GNUC__) && __GNUC__ >= 4
#  define SIPOPTLIB_EXPORT __attribute__((__visibility__("default")))
# else
#  define SIPOPTLIB_EXPORT
# endif
#endif

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

#ifdef _MSC_VER
#define F77_FUNC(name,NAME) NAME
#define F77_FUNC_(name,NAME) NAME
#define IPOPT_C_FINITE _finite
#define IPOPT_HAS_FOPEN_S 1
#define IPOPT_HAS_GETENV_S 1
#define HAVE_WINDOWS_H 1
#else
#define F77_FUNC(name,NAME) name ## _
#define F77_FUNC_(name,NAME) name ## _
#define IPOPT_C_FINITE std::isfinite
#define HAVE_DLFCN_H 1
#define IPOPT_HAS_VA_COPY 1
#endif

/* Enable LAPACK (Accelerate on macOS, MKL on Windows, BLAS/LAPACK on Linux) */
#define IPOPT_HAS_LAPACK 1

/* Enable MUMPS linear solver */
#define IPOPT_HAS_MUMPS 1
/* #undef IPOPT_HAS_PARDISO_MKL */

#define IPOPT_BLAS_FUNC(name,NAME) F77_FUNC(name,NAME)
#define IPOPT_LAPACK_FUNC(name,NAME) F77_FUNC(name,NAME)
#define IPOPT_LAPACK_FUNC_(name,NAME) F77_FUNC_(name,NAME)
#define IPOPT_PARDISO_FUNC(name,NAME) F77_FUNC(name,NAME)

/* Random number generator - standard rand() is always available */
#define IPOPT_HAS_RAND 1
#ifndef _WIN32
#define IPOPT_HAS_DRAND48 1
#endif

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
/* #undef IPOPT_HAS_SPRAL */
/* #undef IPOPT_HAS_WSMP */
