dnl Process this file with autoconf to produce a configure script.

AC_INIT([goctf],[1.2.0])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_FILES([Makefile])
AM_INIT_AUTOMAKE([subdir-objects])

INPUT_CPPFLAGS="$CXXFLAGS"
INPUT_CXXFLAGS="$CXXFLAGS"

AC_PROG_CC([icc gcc])
AC_PROG_CXX([icpc g++ gcc])

AM_PROG_CC_C_O

AC_PROG_INSTALL
LT_INIT([dlopen])
AC_LANG(C++)

# don't let the AC_PROG_CXX set default flags, set them back to input..
CXXFLAGS="$INPUT_CXXFLAGS"
CPPFLAGS="$INPUT_CPPFLAGS"

# set up initial flags
if test "x$CXX" = "xicpc"; then
  CXXFLAGS="$CXXFLAGS -O3 -no-prec-div -no-prec-sqrt -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE"
  CPPFLAGS="$CPPFLAGS -O3 -no-prec-div -no-prec-sqrt -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE"
else
  CXXFLAGS="$CXXFLAGS -DNDEBUG -funroll-loops -O3 -Wall -pipe -fexpensive-optimizations -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE"
  CPPFLAGS="$CPPFLAGS -DNDEBUG -funroll-loops -O3 -Wall -pipe -fexpensive-optimizations -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE"

  #sqlite needs dl on gcc
  LIBS="$LIBS -ldl"
fi

# Use all available processor instruction sets
AC_ARG_ENABLE([latest-instruction-set],
[AS_HELP_STRING([--enable-latest-instruction-set],
  [Use latest available CPU instruction set [default=yes]])],
[
  if test "$enableval" = yes; then
    if test "x$CXX" = "xicpc"; then
      CXXFLAGS="$CXXFLAGS -xHost"
    fi
  fi
])

# MKL
use_mkl=1
AC_ARG_ENABLE([mkl],
[AS_HELP_STRING([--disable-mkl],
  [Do not use the Intel Math Kernel Library (MKL)])],
[
  if test "$enableval" = no; then
    use_mkl=0
  fi
])


# Static linking
static_link="false"
AC_ARG_ENABLE([staticmode],
[AS_HELP_STRING([--enable-staticmode],
  [Link statically [default=no]])],
[
  if test "$enableval" = yes; then
    static_link="true"
  fi
])


AC_MSG_NOTICE(static_link = $static_link)
AM_CONDITIONAL([STATIC_LINKING_AM],[test x$static_link = xtrue])

# wx-config
WXCONFIG=wx-config
AC_ARG_WITH([wx-config],
[AS_HELP_STRING([--with-wx-config=FILE],
  [Use the given path to wx-config when determining wxWidgets configuration; defaults to "wx-config"])],
[
  if test "$withval" != "yes" -a "$withval" != ""; then
    WXCONFIG=$withval
  fi
])

# debugmode
AC_ARG_ENABLE([debugmode],
[AS_HELP_STRING([--enable-debugmode],
  [Compile in debug mode [default=no]])],
[
  if test "$enableval" = yes; then
    CPPFLAGS="-O2 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -DDEBUG"
    CXXFLAGS="-O2 -g -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -DDEBUG"
  fi
])

# fftw
AC_ARG_WITH([fftw-dir],
[AS_HELP_STRING([--with-fftw-dir=DIR],
  [Declare the root directory of fftw3, if its current location is not system accessible])],
[
  if test "$withval" != "yes" -a "$withval" != ""; then
    CPPFLAGS="$CPPFLAGS -I$withval/include -L$withval/lib"
    CXXFLAGS="$CXXFLAGS -I$withval/include -L$withval/lib"
    LDFLAGS="$LDFLAGS -L$withval/lib"
  fi
])

# tiff
AC_ARG_WITH([libtiff-dir],
[AS_HELP_STRING([--with-libtiff-dir=DIR],
  [Declare the root directory of libtiff, if its current location is not system accessible])],
[
  if test "$withval" != "yes" -a "$withval" != ""; then
      CPPFLAGS="$CPPFLAGS -I$withval/include -L$withval/lib"
      CXXFLAGS="$CXXFLAGS -I$withval/include -L$withval/lib"
      LDFLAGS="$LDFLAGS -L$withval/lib"
  fi
])

# zlib
AC_ARG_WITH([zlib-dir],
[AS_HELP_STRING([--with-zlib-dir=DIR],
  [Declare the root directory of zlib, if its current location is not system accessible])],
[
  if test "$withval" != "yes" -a "$withval" != ""; then
    CPPFLAGS="$CPPFLAGS -I$withval/include -L$withval/lib"
    CXXFLAGS="$CXXFLAGS -I$withval/include -L$withval/lib"
    LDFLAGS="$LDFLAGS -L$withval/lib"
  fi
])

# wxWidgets
wxversion=0
AC_DEFUN([WXTEST],
[
  AC_REQUIRE([AC_PROG_AWK])
  AC_MSG_CHECKING([wxWidgets version])
  if wxversion=`$WXCONFIG --version`; then
    AC_MSG_RESULT([$wxversion])
    AC_DEFINE_UNQUOTED([wx_is_available], 1, [Define to 1 if wx is available])
  else
    AC_MSG_RESULT([not found])
    AC_DEFINE_UNQUOTED([wx_is_available], 0, [Define to 0 if wx is unavailable])
  fi
])

# Call WXTEST func
WXTEST

# Verify minimus requires
vers=`echo $wxversion | $AWK 'BEGIN { FS = "."; } { printf "% d", ($1 * 1000 + $2) * 1000 + $3;}'`
if test -n "$vers" && test "$vers" -ge 3000000; then
  WX_CPPFLAGS="`$WXCONFIG --cppflags`"
  WX_CXXFLAGS="`$WXCONFIG --cxxflags | sed -e 's/-fno-exceptions//'`"
  WX_LIBS="`$WXCONFIG --libs richtext,std,aui`"
  
  WX_CPPFLAGS_BASE="`$WXCONFIG --cppflags base`"
  WX_CXXFLAGS_BASE="`$WXCONFIG --cxxflags base | sed -e 's/-fno-exceptions//'`"
  WX_LIBS_BASE="`$WXCONFIG --libs base,net`"
  wx_is_available=1
else
  AC_MSG_ERROR("wxWidgets version > 3.0.0 required.")
  wx_is_available=0
fi

# Do FFTW3 Checks

# Check whether Intel's MKL is available. If not, link against FFTW.
AC_CHECK_HEADER(fftw3.h, [], AC_MSG_ERROR("Can't find fftw3.h"), [])
AC_CHECK_LIB([mkl_intel_lp64],[fftwf_malloc],[HAVE_MKL="yes"],[HAVE_MKL="no"],[-lmkl_core -lmkl_sequential -lpthread])

if test "x$use_mkl" = "x0"; then
  HAVE_MKL="no"
fi

if test "x$HAVE_MKL" = "xyes"; then

  AC_MSG_NOTICE(Using Intel MKL for fast Fourier transforms)

  CXXFLAGS="$CXXFLAGS -DMKL"
  CPPFLAGS="$CPPFLAGS -DMKL"

  if test "x$CXX" = "xicpc"; then
    CPPFLAGS="$CPPFLAGS -mkl=sequential"
    LIBS="$LIBS -mkl=sequential"
  else
    LDFLAGS="$LDFLAGS -L\$(MKLROOT)/lib/intel64"
    if test "x$static_link" = "xtrue"; then
      AC_MSG_ERROR("Static builds with Intel MKL are not supported when using the GNU compiler. Please try with the Intel compiler instead.")
    else
      LIBS="$LIBS -lmkl_rt"
    fi
  fi
else
  AC_CHECK_LIB(fftw3f, fftwf_malloc, [LIBS="$LIBS -lfftw3f"],AC_MSG_ERROR("Can't find FFTW3's libraries. Please check your installation of FFTW3."))
  AC_MSG_NOTICE(Using FFTW for Fourier transforms)
fi

# Static build using the Intel compiler
#
# If we are doing a static build with the Intel compiler,
# let's have Intel libraries linked statically as well.
# It's important that -static-intel be specified after -mkl,
# otherwise mkl will be linked dynamically.
#
if test "x$static_link" = "xtrue"; then
 if test "x$CXX" = "xicpc"; then
  LDFLAGS="$LDFLAGS -static -static-intel"
 else
  LDFLAGS="$LDFLAGS -static"
 fi
fi

# TIFF (If we're not using wxWidget's builtin TIFF support)
#
AC_CHECK_LIB([wxtiff-3.0],[TIFFOpen],[WX_LIBS_BASE="-lwxtiff-3.0 $WX_LIBS_BASE"],[wxtiff=0],`$WXCONFIG --libs base`)
if test "x$wxtiff" = "x0"; then
  AC_CHECK_HEADER(tiffio.h, [], AC_MSG_ERROR(Could not find the TIFF library header tiffio.h), [])
  AC_SEARCH_LIBS([inflate],[z],[],[AC_MSG_ERROR(Could not find your installation of the zlib library)])
  AC_SEARCH_LIBS([TIFFOpen],[tiff],[],[AC_MSG_ERROR(Could not find your installation of the TIFF library)],[-lm -lz])
fi

# GNU Scientific Library
# Gleaned from:
# https://www.gnu.org/software/gsl/manual/html_node/Autoconf-Macros.html
# AX_PATH_GSL(GSL_VERSION,[action-if-found],[action-if-not-found])
AX_PATH_GSL(2.5, [], AC_MSG_ERROR(could not find the required version of GSL.))

AC_SUBST(WX_LIBS)
AC_SUBST(WX_CPPFLAGS)
AC_SUBST(WX_LIBS_BASE)
AC_SUBST(WX_CPPFLAGS_BASE)

AC_OUTPUT