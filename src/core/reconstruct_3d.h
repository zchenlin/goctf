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

/*  \brief  Reconstruct3D class (derived from Frealign Pinsert) */

class Particle;

class Reconstruct3D {

public:

	float						pixel_size;
	float						original_pixel_size;
	float						average_occupancy;
	float						average_score;
	float						score_weights_conversion;
	SymmetryMatrix				symmetry_matrices;
	bool						edge_terms_were_added;

	Image						image_reconstruction;
	float						*ctf_reconstruction;

	int 						logical_x_dimension;		// !< Logical (X) dimensions of the image., Note that this does not necessarily correspond to memory allocation dimensions (ie physical dimensions).
	int 						logical_y_dimension;		// !< Logical (Y) dimensions of the image., Note that this does not necessarily correspond to memory allocation dimensions (ie physical dimensions).
	int 						logical_z_dimension;		// !< Logical (Z) dimensions of the image., Note that this does not necessarily correspond to memory allocation dimensions (ie physical dimensions).
	int 						original_x_dimension;
	int 						original_y_dimension;
	int 						original_z_dimension;

	int							images_processed;

	Reconstruct3D(float wanted_pixel_size = 0.0, float wanted_average_occupancy = 0.0, float wanted_average_score = 0.0, float wanted_score_weights_conversion = 0.0);
	Reconstruct3D(float wanted_pixel_size, float wanted_average_occupancy, float wanted_average_score, float wanted_score_weights_conversion, wxString wanted_symmetry);
	Reconstruct3D(int wanted_logical_x_dimension, int wanted_logical_y_dimension, int wanted_logical_z_dimension, float wanted_pixel_size, float wanted_average_occupancy, float wanted_average_score, float wanted_score_weights_conversion, wxString wanted_symmetry);	// constructor with size
	~Reconstruct3D();							// destructor

	void Init(int wanted_logical_x_dimension, int wanted_logical_y_dimension, int wanted_logical_z_dimension, float wanted_pixel_size, float wanted_average_occupancy, float wanted_average_score, float wanted_score_weights_conversion);
	void InsertSliceWithCTF(Particle &particle_to_insert);
	void InsertSliceNoCTF(Particle &particle_to_insert);
	void AddByLinearInterpolation(float &wanted_x_coordinate, float &wanted_y_coordinate, float &wanted_z_coordinate, std::complex<float> &wanted_value, std::complex<float> &ctf_value, float wanted_weight);
	void CompleteEdges();
	float Correct3DCTF(Image &buffer3d);
	void DumpArrays(wxString filename, bool insert_even);
	void ReadArrayHeader(wxString filename, int &logical_x_dimension, int &logical_y_dimension, int &logical_z_dimension,
			int &original_x_dimension, int &original_y_dimension, int &original_z_dimension, int &images_processed, float &pixel_size, float &original_pixel_size,
			float &average_occupancy, float &average_score, float &score_weights_conversion, wxString &symmetry_symbol, bool &insert_even);
	void ReadArrays(wxString filename);
	Reconstruct3D operator + (const Reconstruct3D &other);
	Reconstruct3D &operator = (const Reconstruct3D &other);
	Reconstruct3D &operator = (const Reconstruct3D *other);
	Reconstruct3D &operator += (const Reconstruct3D &other);
	Reconstruct3D &operator += (const Reconstruct3D *other);
};
