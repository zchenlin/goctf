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

class ClassificationResult {

public:

	ClassificationResult();
	~ClassificationResult();
	long position_in_stack;
	float psi;
	float xshift;
	float yshift;
	int best_class;
	float sigma;
	float logp;
};

WX_DECLARE_OBJARRAY(ClassificationResult, ArrayofClassificationResults);

class Classification {

public :
	Classification();
	~Classification();

	long classification_id;
	long refinement_package_asset_id;
	wxString name;
	wxString class_average_file;
	bool classification_was_imported_or_generated;
	wxDateTime datetime_of_run;
	long starting_classification_id;
	long number_of_particles;
	int number_of_classes;
	float low_resolution_limit;
	float high_resolution_limit;
	float mask_radius;
	float angular_search_step;
	float search_range_x;
	float search_range_y;
	float smoothing_factor;
	bool exclude_blank_edges;
	bool auto_percent_used;
	float percent_used;


	void SizeAndFillWithEmpty(long number_of_particles);

	ArrayofClassificationResults classification_results;
	wxString WriteFrealignParameterFiles(wxString base_filename, RefinementPackage *parent_refinement_package);
};

WX_DECLARE_OBJARRAY(Classification, ArrayofClassifications);

class ShortClassificationInfo {

public :
	ShortClassificationInfo();

	long classification_id;
	long refinement_package_asset_id;
	wxString name;
	wxString class_average_file;
	long number_of_particles;
	int number_of_classes;

	ShortClassificationInfo & operator = (const Classification &other_classification);
	ShortClassificationInfo & operator = (const Classification *other_classification);
};

WX_DECLARE_OBJARRAY(ShortClassificationInfo, ArrayofShortClassificationInfos);



