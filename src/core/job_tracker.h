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

typedef struct TimeRemaining {
  int hours;
  int minutes;
  int seconds;
} TimeRemaining;

class JobTracker {

public :

	int total_number_of_jobs;
	int total_running_processes;
	int total_number_of_finished_jobs;
	int last_update_time_total_running_processes;

	long start_time;
	long last_update_time;

	float last_update_seconds_per_job_per_process;
	long time_of_last_remaining_time_call;

	int old_percent_complete;
	TimeRemaining old_time_remaining;
	TimeRemaining time_remaining;



	JobTracker();
	~JobTracker();

	void StartTracking(int wanted_total_number_of_jobs);
	void AddConnection();
	void MarkJobFinished();
	TimeRemaining ReturnRemainingTime();
	inline int ReturnPercentCompleted() {return int(myround((float(total_number_of_finished_jobs) / float(total_number_of_jobs)) * 100.0)); wxPrintf("Returning %i%\n", int(myround((float(total_number_of_finished_jobs) / float(total_number_of_jobs)) * 100.0)));};

	bool ShouldUpdate();

};
