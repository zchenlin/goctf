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

#define NONE        0
#define TEXT		1
#define INTEGER		2
#define FLOAT		3
#define BOOL		4

// we need a header to describe how to decode the stream. - which i'm about to make up off the top of my head..  SENDER AND RECEIVER MUST HAVE THE SAME ENDIANESS

	// 4 bytes = number_of_jobs (int)
	// 4 bytes = number_of_processes (int)
	// 4 bytes = length_of_command_string (int)
	// n bytes = command_string (chars)
	// then we loop over each job.. of which there are (1st 4 bytes)
	// 4 bytes = number_of_arguments;
	// 1 byte = type of argument;
	// (on of following depending on type of argument :-)
	// 4 bytes - int
	// 4 bytes - float
	// 4 bytes - length of text (int) followed by n bytes where n is length of string
	// 1 byte - bool (0 or 1)

class RunArgument {

	public :

	bool is_allocated;
	int type_of_argument;

	std::string *string_argument;
	int *integer_argument;
	float *float_argument;
	bool *bool_argument;

	RunArgument();
	~RunArgument();

	void Deallocate();

	void SetStringArgument(const char *wanted_text);
	void SetIntArgument(int wanted_argument);
	void SetFloatArgument(float wanted_argument);
	void SetBoolArgument(bool wanted_argument);

	inline std::string ReturnStringArgument() {MyDebugAssertTrue(type_of_argument == TEXT, "Returning wrong type!"); return string_argument[0];}
	inline int ReturnIntegerArgument() {MyDebugAssertTrue(type_of_argument == INTEGER, "Returning wrong type!"); return integer_argument[0];}
	inline float ReturnFloatArgument() {MyDebugAssertTrue(type_of_argument == FLOAT, "Returning wrong type!"); return float_argument[0];}
	inline bool ReturnBoolArgument() {MyDebugAssertTrue(type_of_argument == BOOL, "Returning wrong type!"); return bool_argument[0];}

	long ReturnEncodedByteTransferSize();

};


class RunJob {


public:

	int job_number;
	int number_of_arguments;

	bool has_been_run;
	RunArgument *arguments;

	RunJob();
	~RunJob();

	void Reset(int wanted_number_of_arguments);
	void Deallocate();
	void SetArguments(const char *format, va_list args);
	void ManualSetArguments(const char *format, ...);
	long ReturnEncodedByteTransferSize();
	void SendJob(wxSocketBase *socket);
	void RecieveJob(wxSocketBase *socket);
	void PrintAllArguments();


};

class JobPackage {

public :

	int number_of_jobs;
	int number_of_added_jobs;

	RunProfile my_profile;
	RunJob *jobs;

	JobPackage(RunProfile wanted_profile, wxString wanted_executable_name, int wanted_number_of_jobs);
	JobPackage();
	~JobPackage();

	void Reset(RunProfile wanted_profile, wxString wanted_executable_name, int wanted_number_of_jobs);
	void AddJob(const char *format, ...);
	void SendJobPackage(wxSocketBase *socket);
	void ReceiveJobPackage(wxSocketBase *socket);

	long ReturnEncodedByteTransferSize();
	int ReturnNumberOfJobsRemaining();

};

WX_DECLARE_OBJARRAY(JobPackage, ArrayofJobPackages);

class JobResult
{

public:

	int job_number;
	int result_size;
	float *result_data;

	JobResult();
	JobResult(int wanted_result_size, float *wanted_result_data);
	JobResult( const JobResult &obj); // copy contructor

	~JobResult();

	JobResult & operator = (const JobResult &other_result);
	JobResult & operator = (const JobResult *other_result);

	void SetResult(int wanted_result_size, float *wanted_result_data);
	void SendToSocket(wxSocketBase *wanted_socket);
	void ReceiveFromSocket(wxSocketBase *wanted_socket);

};

WX_DECLARE_OBJARRAY(JobResult, ArrayofJobResults);

void ReceiveResultQueueFromSocket(wxSocketBase *socket, ArrayofJobResults &my_array);
void SendResultQueueToSocket(wxSocketBase *socket, ArrayofJobResults &my_array);
