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

class EulerSearch
{
	// Brute-force search to find matching projections
public:
	int			number_of_search_dimensions;
	int			refine_top_N;
	int			number_of_search_positions;
	int			best_parameters_to_keep;
	float 		angular_step_size;
	float		max_search_x;
	float		max_search_y;
	float		phi_max;
	float		phi_start;
	float		theta_max;
	float		theta_start;
	float		psi_max;
	float		psi_step;
	float		psi_start;
	float		**list_of_search_parameters;
	float		**list_of_best_parameters;
//	Kernel2D	**kernel_index;
//	float		*best_values;
//	float		best_score;
	float		resolution_limit;
	bool		*parameter_map;
	bool		test_mirror;
	wxString	symmetry_symbol;

	// Constructors & destructors
	EulerSearch();
	~EulerSearch();

	// Methods
	void Init(float wanted_resolution_limit, bool *wanted_parameter_map, int wanted_parameters_to_keep);
	void InitGrid(wxString wanted_symmetry_symbol, float angular_step_size, float wanted_phi_start, float wanted_theta_start, float wanted_psi_max, float wanted_psi_step, float wanted_psi_start, float wanted_resolution_limit, bool *parameter_map, int wanted_parameters_to_keep);
	void InitRandom(wxString wanted_symmetry_symbol, float wanted_psi_step, int wanted_number_of_search_positions, float wanted_resolution_limit, bool *wanted_parameter_map, int wanted_parameters_to_keep);
	void Run(Particle &particle, Image &input_3d, float *starting_values, Image *projections);
	void CalculateGridSearchPositions();
	void CalculateRandomSearchPositions();
	void SetSymmetryLimits();
//	void RotateFourier2DFromIndex(Image &image_to_rotate, Image &rotated_image, Kernel2D &kernel_index);
//	void RotateFourier2DIndex(Image &image_to_rotate, Kernel2D &kernel_index, AnglesAndShifts &rotation_angle, float resolution_limit = 1.0, float padding_factor = 1.0);
	Kernel2D ReturnLinearInterpolatedFourierKernel2D(Image &image_to_rotate, float &x, float &y);
};
