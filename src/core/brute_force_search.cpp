/*
 *
 * Copyright (c) 2018 The Regents of the University of Michigan
 * Min Su, Ph.D.
 *
 * This file is part of goCTF.
 *
 * goCTF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * goCTF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with goCTF.  If not, see <https://www.gnu.org/licenses/>.
 *
 * This complete copyright notice must be included in any revised version of the
 * source code. Additional authorship citations may be added, but existing
 * author citations must be preserved.
 *
 *
 * Portions Copyright (c) 2018, Howard Hughes Medical Institute, All rights reserved.
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

BruteForceSearch::BruteForceSearch()
{
	// Nothing to do until Init is called
	number_of_dimensions = 0;
	is_in_memory = false;
	target_function = NULL;
	parameters = NULL;
	starting_value = NULL;
	best_value = NULL;
	half_range = NULL;
	step_size = NULL;
	dimension_at_max = NULL;
	best_score = std::numeric_limits<float>::max();
	num_iterations = 0;
	minimise_at_every_step = false;
	print_progress_bar = false;
}

BruteForceSearch::~BruteForceSearch()
{
	//
	if (is_in_memory)
	{
		delete [] starting_value;
		delete [] best_value;
		delete [] half_range;
		delete [] step_size;
		delete [] dimension_at_max;
		is_in_memory = false;
	}
}

void BruteForceSearch::Init(float (*function_to_minimize)(void* parameters, float [] ), void *wanted_parameters, int num_dim, float wanted_starting_value[], float wanted_half_range[], float wanted_step_size[], bool should_minimise_at_every_step, bool should_print_progress_bar )
{
	MyDebugAssertFalse(is_in_memory,"Brute force search object is already setup");
	MyDebugAssertTrue(num_dim > 0,"Bad number of dimensions: %i",num_dim);
	
	//wxPrintf("ASTIG initiate --> %0.2f\n", wanted_starting_value[4]*180.0/PI);
	// Local variables
	float current_values[num_dim];
	bool search_done;

	// Allocate memory
	number_of_dimensions 	= 	num_dim;
	starting_value 			= 	new float[number_of_dimensions];
	best_value    			= 	new float[number_of_dimensions];
	half_range				= 	new float[number_of_dimensions];
	step_size				= 	new float[number_of_dimensions];
	dimension_at_max		=	new bool[number_of_dimensions];
	is_in_memory			=	true;

	// Copy starting values, half range and step size over; initialise
	for (int dim_counter=0; dim_counter < number_of_dimensions; dim_counter++)
	{
		starting_value[dim_counter]		=	wanted_starting_value[dim_counter];
		half_range[dim_counter]			=	wanted_half_range[dim_counter];
		step_size[dim_counter]			=	wanted_step_size[dim_counter];
		dimension_at_max[dim_counter]	=	false;
		current_values[dim_counter]     =   - std::numeric_limits<float>::max();
	}
	
	starting_value[4] = wanted_starting_value[4]; //Pass astigmatism to starting_value[4]
	//
	minimise_at_every_step = should_minimise_at_every_step;
	MyDebugAssertFalse(minimise_at_every_step,"Minimisation at every step not yet implemented. Sorry.");


	// Work out how many iterations the exhaustive search will take
	// we will start at lowest value for each dimension, then go in step sizes, until we get to upper limit.
	// we will also do extra iterations for the starting value and the upper limit
	num_iterations = 0;
	while (true)
	{
		IncrementCurrentValues(current_values,search_done);
		if (search_done) { break; }
		num_iterations++;
	}

	//
	target_function = function_to_minimize;
	parameters = wanted_parameters;
	print_progress_bar = should_print_progress_bar;
}

float BruteForceSearch::GetBestValue(int index)
{
	MyDebugAssertTrue(index < number_of_dimensions,"Index %i does not exist (number of dimensions = %i",index,number_of_dimensions);
	return best_value[index];
}

// In the BF loop, this should be called before the scoring function. Before the BF loop, set all values in current_values to -huge
void BruteForceSearch::IncrementCurrentValues(float *current_values, bool &search_is_now_completed)
{
	MyDebugAssertTrue(is_in_memory,"Brute force search object not allocated");

	int i;
	int j;


	// do the increment
	search_is_now_completed = true;
	for (i=0;i<number_of_dimensions;i++)
	{
		// if we haven't reached the max for this dimension, increment it, and reset all previous dimensions to their starting point
		if (! dimension_at_max[i])
		{
			// if we got here, it means our search is not over yet
			search_is_now_completed = false;
			// increment the ith dimension
			// it could be that the ith dimension has a very large negative value, perhaps because the search hasn't even started yet
			if (current_values[i] < starting_value[i] - half_range[i])
			{
				current_values[i] = starting_value[i] - half_range[i];
			}
			else
			{
				current_values[i] += step_size[i];
			}
			// Reset all previous dimensions to their starting values and reset their dimension_at_max flags to false
			if (i > 0)
			{
				for (j=0;j<i;j++)
				{
					current_values[j] = starting_value[j] - half_range[j];
					MyDebugAssertTrue(dimension_at_max[j],"failed sanity check");
					dimension_at_max[j] = false;
				}
			}
			// if the ith dimension has reached or gone over its max, set it at the max, and set its logical flag dimension_at_max to .true.
			if (current_values[i] >= starting_value[i] + half_range[i])
			{
				current_values[i] = starting_value[i] + half_range[i];
				dimension_at_max[i] = true;
				if (i == number_of_dimensions - 1 )
				{
					search_is_now_completed = true;
				}
			}
			break;
		}
	} // end of loop over dimensions

	for (i=0;i<number_of_dimensions;i++)
	{
		if (current_values[i] < starting_value[i] - half_range[i])
		{
			current_values[i] = starting_value[i] - half_range[i];
		}
	}

}

// Run the brute force minimiser's exhaustive search
void BruteForceSearch::Run()
{
	MyDebugAssertTrue(is_in_memory,"BruteForceSearch object not allocated");

	// Private variables
	//float current_values[number_of_dimensions];
	float current_values[4];
	int i;
	int num_iterations_completed;
	int current_iteration;
	bool search_completed;
	float current_score;
	ProgressBar *my_progress_bar;

	// The starting values and the corresponding score
	//wxPrintf("ASTIG !!! --> %0.2f\n",starting_value[4]*180.0/PI);
	best_score = target_function(parameters,starting_value);
	for (i=0;i<number_of_dimensions;i++) { best_value[i] = starting_value[i]; }

	// The starting point for the brute-force search
	for (i=0; i<number_of_dimensions;i++)
	{
		current_values[i] = - std::numeric_limits<float>::max();
		dimension_at_max[i] = false;
	}
	
	current_values[4] = starting_value[4];
	
	if ( print_progress_bar ) { my_progress_bar = new ProgressBar(num_iterations); }

	// How many iterations have we completed?
	num_iterations_completed = 0;

	// start the brute-force search iterations
	for (current_iteration=0; current_iteration < num_iterations; current_iteration++)
	{
		// what values for the parameters should we try now?
		IncrementCurrentValues(current_values,search_completed);
		MyDebugAssertFalse(search_completed,"Failed sanity check");
		
		// Try the current parameters by calling the scoring function
		// ADD HERE: minimization if minimise_at_every_step
		current_score = target_function(parameters,current_values);

		// Print debug
		/*wxPrintf("(BF dbg, it %04i) Values: ",current_iteration);
		for (i=0;i<number_of_dimensions;i++)
		{
			wxPrintf("%g ",current_values[i]);
		}
		wxPrintf(" Score: %g\n",current_score); */

		// if the score is the best we've seen so far, remember the values and the score
		if (current_score < best_score)
		{
			best_score = current_score;
			//wxPrintf("best score: %f \n", best_score);
			//wxPrintf("new best values: ");
			for (i=0;i<number_of_dimensions;i++)
			{
				best_value[i] = current_values[i];
				//wxPrintf("%g ",best_value[i]);
			}
			//wxPrintf("\n");
		}

		// Progress
		num_iterations_completed++;
		if (print_progress_bar) { my_progress_bar->Update(num_iterations_completed); }

	} // End of loop over exhaustive search operation

	if (print_progress_bar) { delete my_progress_bar; }

}

