noinst_HEADERS = 	src/core/asset_group.h \
			     src/core/assets.h \
			     src/core/socket_codes.h \
			     src/core/userinput.h \
			     src/core/randomnumbergenerator.h \
                 src/core/core_headers.h \
                 src/core/defines.h \
                 src/core/functions.h \
                 src/core/image.h \
                 src/core/mrc_file.h \
                 src/core/mrc_header.h \
                 src/core/dm_file.h \
                 src/core/matrix.h \
                 src/core/symmetry_matrix.h \
                 src/core/ctf.h \
                 src/core/curve.h \
                 src/core/angles_and_shifts.h \
                 src/core/parameter_constraints.h \
                 src/core/empirical_distribution.h \
                 src/core/resolution_statistics.h \
                 src/core/reconstructed_volume.h \
                 src/core/particle.h \
                 src/core/reconstruct_3d.h \
                 src/core/electron_dose.h \
                 src/core/run_profiles.h \
                 src/core/database.h \
                 src/core/project.h \
                 src/core/job_packager.h \
                 src/core/job_tracker.h \
                 src/core/numeric_text_file.h \
                 src/core/progressbar.h \
                 src/core/downhill_simplex.h \
                 src/core/brute_force_search.h \
                 src/core/conjugate_gradient.h \
                 src/core/euler_search.h \
                 src/core/frealign_parameter_file.h \
                 src/core/myapp.h \
                 src/core/f2c.h \
                 src/core/refinement_package.h \
                 src/core/refinement.h \
                 src/core/particle_finder.h \
                 src/core/abstract_image_file.h \
                 src/core/tiff/tiff.h \
                 src/core/tiff/tiffconf.h \
                 src/core/tiff/tiffio.h \
                 src/core/tiff/tiffvers.h \
                 src/core/tiff_file.h \
                 src/core/image_file.h \
                 src/core/classification.h \
                 src/core/classification_selection.h \
                 src/core/sqlite/sqlite3.h

bin_PROGRAMS = goctf

bin_SCRIPTS = scripts/particles_split.py

CLEANFILES = ${bin_SCRIPTS}

#dist_bin_SCRIPTS = 	install-sh \
#                        config.sub \
#                        config.guess \
#                        ltmain.sh \
#                        depcomp

image_trim.cpp:
	if [ -e $(top_srcdir)/src/core/image.cpp ]; then \
		$(AWK) 'BEGIN { in_include_block=0} {\
			   if (/core_headers.h/) { \
	                  	printf "#include \"src/core/core_headers.h\"" \
	             	} \
	           else { \
	           			if (/BEGIN_FOR_STAND_ALONE_goctf/) { \
	           				in_include_block=1 \
	           			} \
	           			if (in_include_block==1) { \
	           				if (/END_FOR_STAND_ALONE_goctf/) { \
	           					in_include_block=0 \
	           				} \
	           			} \
	           			if ( ! /BEGIN_FOR_STAND_ALONE_goctf/) { \
	           				if (in_include_block==1) { print $0 } \
	           			} \
	           		} \
	         }'  $(top_srcdir)/src/core/image.cpp > $(top_srcdir)/image_trim.cpp; \
	fi



goctf_SOURCES = src/programs/goctf/goctf.cpp \
					src/core/functions.cpp \
					   src/core/globals.cpp \
					   src/core/randomnumbergenerator.cpp \
                       src/core/userinput.cpp \
                       src/core/curve.cpp \
                       image_trim.cpp \
                       src/core/matrix.cpp \
                       src/core/symmetry_matrix.cpp \
                       src/core/abstract_image_file.cpp \
                       src/core/mrc_header.cpp \
                       src/core/mrc_file.cpp \
                       src/core/tiff_file.cpp \
                       src/core/dm_file.cpp \
                       src/core/image_file.cpp \
                       src/core/job_packager.cpp \
                       src/core/job_tracker.cpp \
                       src/core/run_profiles.cpp \
                       src/core/myapp.cpp \
                       src/core/empirical_distribution.cpp \
                       src/core/ctf.cpp \
                       src/core/numeric_text_file.cpp \
                       src/core/progressbar.cpp \
                       src/core/downhill_simplex.cpp \
                       src/core/brute_force_search.cpp \
                       src/core/va04.cpp \
                       src/core/conjugate_gradient.cpp
goctf_CXXFLAGS = $(WX_CPPFLAGS_BASE) $(GSL_CFLAGS)
goctf_CPPFLAGS = $(WX_CPPFLAGS_BASE) $(GSL_CFLAGS)
goctf_LDFLAGS = $(WX_LIBS_BASE) $(GSL_LIBS)

ACLOCAL_AMFLAGS = -I m4


