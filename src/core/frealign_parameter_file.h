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

/*  \brief  FrealignParameterFile class */

class FrealignParameterFile {

public:

	FILE		*parameter_file;
	wxString	filename;
	int			access_type;
	int			records_per_line;
	int			number_of_lines;
	int			current_line;
	float		*parameter_cache;
	float		average_defocus;
	float		defocus_coeff_a;
	float		defocus_coeff_b;

	FrealignParameterFile();
	~FrealignParameterFile();

	FrealignParameterFile(wxString wanted_filename, int wanted_access_type, int wanted_records_per_line = 17);
	void Open(wxString wanted_filename, int wanted_access_type, int wanted_records_per_line = 17);
	void Close();
	void WriteCommentLine(wxString comment_string);
	void WriteLine(float *parameters, bool comment = false);
	int ReadFile(bool exclude_negative_film_numbers = false);
	void ReadLine(float *parameters);
	float ReadParameter(int wanted_line_number, int wanted_parameter);
	void UpdateParameter(int wanted_line_number, int wanted_parameter, float wanted_value);
	void Rewind();
	float ReturnMin(int wanted_index, bool exclude_negative_film_numbers = false);
	float ReturnMax(int wanted_index, bool exclude_negative_film_numbers = false);
	double ReturnAverage(int wanted_index, bool exclude_negative_film_numbers = false);
	void RemoveOutliers(int wanted_index, float wanted_standard_deviation, bool exclude_negative_film_numbers = false);
	float ReturnThreshold(float wanted_percentage, bool exclude_negative_film_numbers = false);
	void CalculateDefocusDependence(bool exclude_negative_film_numbers = false);
	void AdjustScores(bool exclude_negative_film_numbers = false);
	float ReturnScoreAdjustment(float defocus);
};
