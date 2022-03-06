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

RandomNumberGenerator::RandomNumberGenerator(bool internal) {
	use_internal = internal;
	SetSeed(4711);
}


RandomNumberGenerator::RandomNumberGenerator(int random_seed, bool internal) {
	use_internal = internal;
	SetSeed(random_seed);
}

// Set seed for random number generator
void RandomNumberGenerator::SetSeed(int random_seed) {

	if (random_seed < 0)
	{
		this->random_seed = time(NULL);
	}
	else this->random_seed = random_seed;

	if (use_internal) Internal_srand((unsigned int) this->random_seed);
	else srand((unsigned int) this->random_seed);
}

// Return a random number in the interval [-1,1]
// from a uniform distribution
float RandomNumberGenerator::GetUniformRandom() {
	float rnd1;
	float hmax;
	if (use_internal)
	{
		rnd1 = (float)Internal_rand();
		hmax = ((float)32767) / 2.0;
//		wxPrintf("rnd1 = %g\n", rnd1);
	}
	else
	{
		rnd1 = (float)rand();
		hmax = ((float) RAND_MAX) / 2.0;
	}
	float rnd2 = (rnd1 - hmax) / hmax;
	return rnd2;
}

// Return a random number according to the normal distribution using the
// the Box-Muller transformation (polar variant)
// standard normal distribution:
//              p(X) = 1/(2*Pi)^0.5 * exp(-x^2/2)
float RandomNumberGenerator::GetNormalRandom() {
	float x1;
	float x2;
	float R;
	float y1;
	do {
		x1 = GetUniformRandom();
		x2 = GetUniformRandom();
		R = x1 * x1 + x2 * x2;
	} while (R == 0.0 || R > 1.0);
	y1 = x1 * sqrtf(-2.0 * log(R) / R);
	// y2 = x2 * sqrtf(-2.0 * log(R) / R);
	return y1;
}

void RandomNumberGenerator::Internal_srand(unsigned int random_seed)
{
	next_seed = random_seed;
}

int RandomNumberGenerator::Internal_rand()
{
	next_seed = next_seed * 1103515245 + 12345;
//	wxPrintf("next = %i\n", next_seed);
    return((unsigned int)(next_seed / 65536) % 32768);
}
