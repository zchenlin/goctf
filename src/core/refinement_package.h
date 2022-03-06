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

class RefinementPackageParticleInfo {

public :
	RefinementPackageParticleInfo();
	~RefinementPackageParticleInfo();

	long parent_image_id;
	long position_in_stack;
	long original_particle_position_asset_id;
	float x_pos;
	float y_pos;
	float pixel_size;
	float defocus_1;
	float defocus_2;
	float defocus_angle;
	float phase_shift;
	float spherical_aberration;
	float amplitude_contrast;
	float microscope_voltage;

};


WX_DECLARE_OBJARRAY(RefinementPackageParticleInfo, ArrayOfRefinmentPackageParticleInfos);


class RefinementPackage {

  public:

	RefinementPackage();
	~RefinementPackage();

	long asset_id;
	wxString stack_filename;
	wxString name;
	int stack_box_size;

	int number_of_classes;

	wxString symmetry;
	double estimated_particle_size_in_angstroms;
	double estimated_particle_weight_in_kda;
	double lowest_resolution_of_intial_parameter_generated_3ds;


	int number_of_run_refinments;
	long last_refinment_id;

	wxArrayLong references_for_next_refinement;
	wxArrayLong refinement_ids;
	wxArrayLong classification_ids;

	ArrayOfRefinmentPackageParticleInfos contained_particles;

	RefinementPackageParticleInfo ReturnParticleInfoByPositionInStack(long wanted_position_in_stack);

	long ReturnLastRefinementID();

};

WX_DECLARE_OBJARRAY(RefinementPackage, ArrayOfRefinementPackages);


