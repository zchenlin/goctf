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

#include "core_headers.h"

// Initialise an empirical distribution
EmpiricalDistribution::EmpiricalDistribution()
{
	Reset();
}


EmpiricalDistribution::~EmpiricalDistribution()
{
	// Here, would deallocate sample_values array
}

void EmpiricalDistribution::Reset()
{
	sum_of_samples = 0.0;
	sum_of_squared_samples = 0.0;
	number_of_samples = 0;
	minimum = std::numeric_limits<float>::max();
	maximum = - std::numeric_limits<float>::max();
	is_constant = true;
	last_added_value = 0.0;
}

void EmpiricalDistribution::AddSampleValue(float sample_value)
{
	sum_of_samples += sample_value;
	sum_of_squared_samples += pow(sample_value,2);
	number_of_samples++;
	minimum = std::min(minimum,sample_value);
	maximum = std::max(maximum,sample_value);
	if (number_of_samples == 1)
	{
		is_constant = true;
	}
	else
	{
		is_constant = is_constant && last_added_value == sample_value;
	}
	last_added_value = sample_value;
}

bool EmpiricalDistribution::IsConstant()
{
	return is_constant;
}

float EmpiricalDistribution::GetSampleSumOfSquares()
{
	return sum_of_squared_samples;
}

float EmpiricalDistribution::GetNumberOfSamples()
{
	return number_of_samples;
}

float EmpiricalDistribution::GetSampleSum()
{
	return sum_of_samples;
}

float EmpiricalDistribution::GetSampleMean()
{
	if (number_of_samples > 0)
	{
		return sum_of_samples / float(number_of_samples);
	}
	else
	{
		return 0.0;
	}
}

float EmpiricalDistribution::GetSampleVariance()
{
	if (number_of_samples > 0)
	{
		return (sum_of_squared_samples / float(number_of_samples)) - pow(sum_of_samples/float(number_of_samples),2);
	}
	else
	{
		return 0.0;
	}
}

float EmpiricalDistribution::GetUnbiasedEstimateOfPopulationVariance()
{
	if (number_of_samples > 0)
	{
		return GetSampleVariance() * float(number_of_samples) / float(number_of_samples-1);
	}
	else
	{
		return 0.0;
	}
}
