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

class ElectronDose {
private:

public:

	float acceleration_voltage;

	float critical_dose_a;
	float critical_dose_b;
	float critical_dose_c;

	float voltage_scaling_factor;

	float pixel_size;

	ElectronDose();
	ElectronDose(float wanted_acceleration_voltage, float wanted_pixel_size);


	void Init(float wanted_acceleration_voltage, float wanted_pixel_size);
	float ReturnCriticalDose(float spatial_frequency);
	float ReturnDoseFilter(float dose_at_end_of_frame, float critical_dose);
	void CalculateDoseFilterAs1DArray(Image *ref_image, float *filter_array, float dose_start, float dose_finish);
};

inline float ElectronDose::ReturnCriticalDose(float spatial_frequency)
{
	return (critical_dose_a * pow(spatial_frequency, critical_dose_b) + critical_dose_c) * voltage_scaling_factor;
}


inline float ElectronDose::ReturnDoseFilter(float dose_at_end_of_frame, float critical_dose)
{
	return exp((-0.5 * dose_at_end_of_frame) / critical_dose);
}

