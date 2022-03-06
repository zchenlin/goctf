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

class RunCommand {

public:
	RunCommand();
	~RunCommand();

	wxString command_to_run;
	int number_of_copies;
	int delay_time_in_ms;

	void SetCommand(wxString wanted_command, int wanted_number_of_copies, int wanted_delay_time_in_ms);
};

class RunProfile {

public:

	int id;
	long number_of_run_commands;
	long number_allocated;

	public :

	RunProfile();
	RunProfile( const RunProfile &obj); // copy contructor
	~RunProfile();

	wxString name;
	wxString manager_command;

	wxString gui_address;
	wxString controller_address;

	RunCommand *run_commands;

	wxString executable_name;

	void AddCommand(RunCommand wanted_command);
	void AddCommand(wxString wanted_command, int wanted_number_of_copies, int wanted_delay_time_in_ms);
	void RemoveCommand(int number_to_remove);
	void RemoveAll();
	long ReturnTotalJobs();
	void SubstituteExecutableName(wxString executable_name);

	RunProfile & operator = (const RunProfile &t);
	RunProfile & operator = (const RunProfile *t);
};


class RunProfileManager {

public:

	int current_id_number;
	long number_of_run_profiles;
	long number_allocated;

	RunProfile *run_profiles;

	void AddProfile(RunProfile *profile_to_add);
	void AddBlankProfile();
	void RemoveProfile(int number_to_remove);
	void RemoveAllProfiles();

	RunProfile * ReturnLastProfilePointer();
	RunProfile * ReturnProfilePointer(int wanted_profile);

	wxString ReturnProfileName(long wanted_profile);
	long ReturnProfileID(long wanted_profile);
	long ReturnTotalJobs(long wanted_profile);


	RunProfileManager();
	~RunProfileManager();


};
