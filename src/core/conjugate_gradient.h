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

class ConjugateGradient
{
	//
private:
	bool 	is_in_memory;
	int		n;						//	Number of parameters to refine
	int		num_function_calls;		//	Number of calls to the scoring function
	float	*best_values;			//	Final best parameters determined by conjugate gradient minimization
	float 	*e;						//	Recalculated accuracy for subroutine va04
	float 	escale;					//	Anticipated change in the parameters during conjugate gradient minimization
	float	best_score;				//	Final best score
	//
	float (*target_function)(void* parameters, float []);
	void *parameters;

public:
	// Constructors & destructors
	ConjugateGradient();
	~ConjugateGradient();

	// Methods
	float Init(float (*function_to_minimize)(void* parameters, float []), void *parameters, int num_dim, float starting_value[], float accuracy[] );
	float Run();
	inline float GetBestValue(int index) { return best_values[index]; };
	inline float GetBestScore() { return best_score; };
	inline float *GetPointerToBestValues() { return best_values; };

};

extern "C" {
int va04a_(int *n, float *e, float *escale, int *self_num_function_calls__,
		float (*self_target_function) (void* parameters, float[]),
		void *self_parameter_ptr__, float *self_best_score__, int *iprint,
		int *icon, int *maxit, float *x);
}
