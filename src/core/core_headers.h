/*
 *
 * Copyright (c) 2018, Howard Hughes Medical Institute, All rights reserved.
 *
 * The Janelia Research Campus Software License 1.2
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Howard Hughes Medical Institute nor the names of
 *    its contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * REASONABLE ROYALTIES; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

typedef struct Peak {
  float x;
  float y;
  float z;
  float value;
  long  physical_address_within_image;
} Peak;

typedef struct Kernel2D {
  int   pixel_index[4];
  float pixel_weight[4];
} Kernel2D;

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <cstring>
#include <cstdarg>
#include <cfloat>
#include <complex>
const std::complex<float> I(0.0,1.0);
#include <fftw3.h>
#include <math.h>
#include "sqlite/sqlite3.h"
#include <wx/wx.h>
#include <wx/socket.h>
#include <wx/cmdline.h>
#include <wx/stdpaths.h>
#include <wx/filename.h>
#include <wx/wfstream.h>
#include <wx/tokenzr.h>
#include <wx/txtstrm.h>
#include <wx/textfile.h>
#include "defines.h"
#include "assets.h"
#include "asset_group.h"
#include "socket_codes.h"
#include "userinput.h"
#include "functions.h"
#include "randomnumbergenerator.h"
#include "tiff/tiffio.h"
#include "abstract_image_file.h"
#include "mrc_header.h"
#include "mrc_file.h"
#include "dm_file.h"
#include "tiff_file.h"
#include "image_file.h"
#include "matrix.h"
#include "symmetry_matrix.h"
#include "ctf.h"
#include "curve.h"
#include "angles_and_shifts.h"
#include "parameter_constraints.h"
#include "empirical_distribution.h"
#include "image.h"
#include "resolution_statistics.h"
#include "reconstructed_volume.h"
#include "particle.h"
#include "reconstruct_3d.h"
#include "electron_dose.h"
#include "run_profiles.h"
#include "refinement_package.h"
#include "refinement.h"
#include "classification.h"
#include "classification_selection.h"
#include "database.h"
#include "project.h"
#include "job_packager.h"
#include "job_tracker.h"
#include "numeric_text_file.h"
#include "progressbar.h"
#include "downhill_simplex.h"
#include "brute_force_search.h"
#include "conjugate_gradient.h"
#include "euler_search.h"
#include "frealign_parameter_file.h"
#include "particle_finder.h"
#include "myapp.h"

#ifdef MKL
#include <mkl.h>
#endif

extern RandomNumberGenerator global_random_number_generator;
