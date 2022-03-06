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

class Curve {

private:

public:

	bool have_polynomial;
	bool have_savitzky_golay;
	int number_of_points;
	int allocated_space_for_points;

	float *data_x;
	float *data_y;

	float *polynomial_fit;
	float *savitzky_golay_fit;

	int savitzky_golay_window_size;
	int savitzky_golay_polynomial_order;
	float **savitzky_golay_coefficients;

	int polynomial_order;
	float *polynomial_coefficients;


	// Constructors, destructors
	Curve();
	~Curve();
	Curve( const Curve &other_curve);

	Curve & operator = (const Curve &other_curve);
	Curve & operator = (const Curve *other_curve);

	void ResampleCurve(Curve *input_curve, int wanted_number_of_points);
	float ReturnLinearInterpolationFromI(float wanted_i);
	float ReturnLinearInterpolationFromX(float wanted_x);
	void AddValueAtXUsingLinearInterpolation(float wanted_x, float value_to_add, bool assume_linear_x);
	void AddValueAtXUsingNearestNeighborInterpolation(float wanted_x, float value_to_add);
	int ReturnIndexOfNearestPreviousBin(float wanted_x);
	void PrintToStandardOut();
	void WriteToFile(wxString output_file);
	void CopyFrom(Curve *other_curve);
	void ClearData();
	void MultiplyByConstant(float constant_to_multiply_by);
	void MultiplyXByConstant(float constant_to_multiply_by);
	void AddPoint(float x_value, float y_value);
	void FitPolynomialToData(int wanted_polynomial_order = 6);
	void FitSavitzkyGolayToData(int wanted_window_size, int wanted_polynomial_order);
	float ReturnSavitzkyGolayInterpolationFromX( float wanted_x );
	int ReturnIndexOfNearestPointFromX( float wanted_x );
	void DeleteSavitzkyGolayCoefficients();
	void AllocateSavitzkyGolayCoefficients();
	void CheckMemory();
	void AddWith(Curve *other_curve);
	void SetupXAxis(const float lower_bound, const float upper_bound, const int wanted_number_of_points);
	float ReturnMaximumValue();
	float ReturnMode();
	void ComputeMaximumValueAndMode(float &maximum_value, float &mode);
	float ReturnFullWidthAtGivenValue(const float &wanted_value);
	void NormalizeMaximumValue();
	void ZeroYData();
	void ApplyCTF(CTF ctf_to_apply, float azimuth_in_radians =  0.0);
	void SquareRoot();
	void Reciprocal();
	void ZeroAfterIndex(int index);
};
