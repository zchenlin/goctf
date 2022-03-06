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

// <class_header>
// <class>ProgressBar</class>
//
// <author>Timothy Grant</author>
//
// <date>20090109</date>
//
// <description>
//
// A Simple Class to show a basic command line progress bar.  It
// includes the total percent complete, and also a self estimated
// Time Remaining..
//
// To use, simply call the constructor with the "number_of_ticks"
// required.  For example when dealing with images this would
// typically be the total number of images - although it can of
// course be anything.
//
// Once the ProgressBar object is constructed it will be printed
// to the console in it's initialised form (it is important that
// you output no other text during the lifetime of the progress bar
// or everything will look a bit weird.  In order to update the
// progress bar you must call the Update method with the tick
// you are currently on (in the above example this would be the
// image you are currently working on).
//
// After finishing you should delete the progress bar, which erases it.
//
// NOTE : If you call the status bar with one tick it will do nothing.
// I implemented this for sanity as it is stupid to have a progress
// bar for 1 image or 1 anything really.  However rather than write
// code in every program that checks how many images there are before
// creating a progress bar, I thought it would be easier to just do
// the check here.
//
// A Quick Example, meanfiltering a bunch of images :-
//
// void main(void)
// {
//    TigrisImage my_image("my_file", 1);
//    long number_of_images = my_image.number_following + 1;
//
//    ProgressBar *my_progress_bar = new ProgressBar(number_of_images);
//
//    for (long counter = 1; counter <= number_of_images; counter++)
//    {
//  	  input_image.Read(input_filename, counter);
//  	  input_image.MeanFilter(filter_size);
//  	  input_image.Write(output_filename, counter);
//  	  my_progress_bar->Update(counter);
//    }
//
//    delete my_progress_bar;
// }
//
// </class_header>

class ProgressBar {

private:

	long total_number_of_ticks;
	long start_time;
	long last_update_time;

public :

	// Constructors

	ProgressBar();
	ProgressBar(long wanted_total_number_of_ticks);

	// Destructor

	~ProgressBar();

	// Methods

	void Update(long current_tick);


};
