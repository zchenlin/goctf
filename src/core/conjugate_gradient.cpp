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

ConjugateGradient::ConjugateGradient()
{
	is_in_memory = false;
	n = 0;
	num_function_calls = 0;
	best_values = NULL;
	e = NULL;
	escale = 0;
	best_score = std::numeric_limits<float>::max();
	target_function = NULL;
	parameters = NULL;
}

ConjugateGradient::~ConjugateGradient()
{
	if (is_in_memory)
	{
		delete [] best_values;
		delete [] e;
		is_in_memory = false;
	}
}

float ConjugateGradient::Init(float (*function_to_minimize)(void* parameters, float []), void *parameters_to_pass, int num_dim, float starting_value[], float accuracy[] )
{

	MyDebugAssertTrue(num_dim > 0,"Initializing conjugate gradient with zero dimensions");

// Copy pointers to the target function and the needed parameters
	target_function = function_to_minimize;
	parameters = parameters_to_pass;

	if (is_in_memory)
	{
		delete [] best_values;
		delete [] e;
		is_in_memory = false;
	}

	// Allocate memory
	n 					= 	num_dim;
	best_values 		=	new float[n];
	e					=	new float[n];
	is_in_memory		=	true;

	// Initialise values
	escale = 100.0;
	num_function_calls = 0;


	for (int dim_counter=0; dim_counter < n; dim_counter++)
	{
		best_values[dim_counter]	=	starting_value[dim_counter];
		e[dim_counter]				=	accuracy[dim_counter];
	}

	// Call the target function to find out our starting score
	best_score = target_function(parameters,starting_value);

//	MyDebugPrint("Starting score = %f\n",best_score);
	return best_score;

}

float ConjugateGradient::Run()
{
	int iprint = 0;
	int icon = 1;
	int maxit = 50;
	int va04_success = 0;

	va04_success = va04a_(&n,e,&escale,&num_function_calls,target_function,parameters,&best_score,&iprint,&icon,&maxit,best_values);

	return best_score;
}
