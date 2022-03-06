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

RunCommand::RunCommand()
{
	command_to_run = "";
	number_of_copies = 0;
	delay_time_in_ms = 0;

}

RunCommand::~RunCommand()
{

}

void RunCommand::SetCommand(wxString wanted_command, int wanted_number_of_copies, int wanted_delay_time_in_ms)
{
	command_to_run = wanted_command;
	number_of_copies = wanted_number_of_copies;
	delay_time_in_ms = wanted_delay_time_in_ms;
}

RunProfile::RunProfile()
{
	id = 1;
	number_of_run_commands = 0;

	manager_command = "$command";
	run_commands = new RunCommand[5];
	number_allocated = 5;

	executable_name = "";
	gui_address = "";
	controller_address = "";

}

RunProfile::RunProfile( const RunProfile &obj) // copy contructor
{
	id = obj.id;
	name = obj.name;
	number_of_run_commands = obj.number_of_run_commands;
	manager_command = obj.manager_command;
	number_allocated = obj.number_allocated;

	executable_name = obj.executable_name;

	gui_address = obj.gui_address;
	controller_address = obj.controller_address;

	run_commands = new RunCommand[number_allocated];

	for (long counter = 0; counter < number_of_run_commands; counter++)
	{
	   run_commands[counter] = obj.run_commands[counter];
	}
}

RunProfile::~RunProfile()
{
	delete [] run_commands;
}

void RunProfile::AddCommand(RunCommand wanted_command)
{
	// check we have enough memory

	RunCommand *buffer;

	if (number_of_run_commands >= number_allocated)
	{
		// reallocate..

		if (number_allocated < 100) number_allocated *= 2;
		else number_allocated += 100;

		buffer = new RunCommand[number_allocated];

		for (long counter = 0; counter < number_of_run_commands; counter++)
		{
			buffer[counter] = run_commands[counter];
		}

		delete [] run_commands;
		run_commands = buffer;
	}

	// Should be fine for memory, so just add one.

	run_commands[number_of_run_commands] = wanted_command;
	number_of_run_commands++;
}

void RunProfile::AddCommand(wxString wanted_command, int wanted_number_of_copies, int wanted_delay_time_in_ms)
{
	// check we have enough memory

	RunCommand *buffer;

	if (number_of_run_commands >= number_allocated)
	{
		// reallocate..

		if (number_allocated < 100) number_allocated *= 2;
		else number_allocated += 100;

		buffer = new RunCommand[number_allocated];

		for (long counter = 0; counter < number_of_run_commands; counter++)
		{
			buffer[counter] = run_commands[counter];
		}

		delete [] run_commands;
		run_commands = buffer;
	}

	// Should be fine for memory, so just add one.

	run_commands[number_of_run_commands].command_to_run = wanted_command;
	run_commands[number_of_run_commands].number_of_copies = wanted_number_of_copies;
	run_commands[number_of_run_commands].delay_time_in_ms = wanted_delay_time_in_ms;
	number_of_run_commands++;
}

void RunProfile::RemoveCommand(int number_to_remove)
{
	MyDebugAssertTrue(number_to_remove >= 0 && number_to_remove < number_of_run_commands, "Error: Trying to remove a command that doesnt't exist");

	for (long counter = number_to_remove; counter < number_of_run_commands - 1; counter++)
	{
		run_commands[counter] = run_commands[counter + 1];
	}

	number_of_run_commands--;
}

void RunProfile::RemoveAll()
{
	number_of_run_commands = 0;
}

long RunProfile::ReturnTotalJobs()
{
	long total_jobs = 0;

	for (long counter = 0; counter < number_of_run_commands; counter++)
	{
		total_jobs += run_commands[counter].number_of_copies;
	}

	return total_jobs;

}

void RunProfile::SubstituteExecutableName(wxString executable_name)
{
	for (long counter = 0; counter < number_of_run_commands; counter++)
	{
		run_commands[counter].command_to_run.Replace("$command", executable_name);
	}
}


RunProfile & RunProfile::operator = (const RunProfile &t)
{
   // Check for self assignment
   if(this != &t)
   {
	   if(this->number_allocated != t.number_allocated)
	   {
		   delete [] this->run_commands;

		   this->run_commands = new RunCommand[t.number_allocated];
		   this->number_allocated = t.number_allocated;
	   }

	   this->id = t.id;
	   this->name = t.name;
	   this->number_of_run_commands = t.number_of_run_commands;
	   this->manager_command = t.manager_command;
	   this->executable_name = t.executable_name;
	   this->gui_address = t.gui_address;
	   this->controller_address = t.controller_address;

	   for (long counter = 0; counter < t.number_of_run_commands; counter++)
	   {
		   this->run_commands[counter] = t.run_commands[counter];
	   }
   }

   return *this;
}

RunProfile & RunProfile::operator = (const RunProfile *t)
{
   // Check for self assignment
   if(this != t)
   {
	   if(this->number_allocated != t->number_allocated)
	   {
		   delete [] this->run_commands;

		   this->run_commands = new RunCommand[t->number_allocated];
		   this->number_allocated = t->number_allocated;
	   }

	   this->id = t->id;
	   this->name = t->name;
	   this->number_of_run_commands = t->number_of_run_commands;
	   this->manager_command = t->manager_command;
	   this->executable_name = t->executable_name;
	   this->gui_address = t->gui_address;
	   this->controller_address = t->controller_address;

	   for (long counter = 0; counter < t->number_of_run_commands; counter++)
	   {
		   this->run_commands[counter] = t->run_commands[counter];
	   }
   }

   return *this;
}


RunProfileManager::RunProfileManager()
{
	current_id_number = 0;
	number_of_run_profiles = 0;
	run_profiles = new RunProfile[5];
	number_allocated = 5;
}

RunProfileManager::~RunProfileManager()
{
	delete [] run_profiles;
}

void RunProfileManager::AddProfile(RunProfile *profile_to_add)
{
	// check we have enough memory

	RunProfile *buffer;

	if (number_of_run_profiles >= number_allocated)
	{
		// reallocate..

		if (number_allocated < 100) number_allocated *= 2;
		else number_allocated += 100;

		buffer = new RunProfile[number_allocated];

		for (long counter = 0; counter < number_of_run_profiles; counter++)
		{
			buffer[counter] = run_profiles[counter];
		}

		delete [] run_profiles;
		run_profiles = buffer;
	}

	// Should be fine for memory, so just add one.

	run_profiles[number_of_run_profiles] = profile_to_add;
	number_of_run_profiles++;

	if (profile_to_add->id > current_id_number) current_id_number = profile_to_add->id;
}

void RunProfileManager::AddBlankProfile()
{
	// check we have enough memory

	RunProfile *buffer;

	if (number_of_run_profiles >= number_allocated)
	{
		// reallocate..

		if (number_allocated < 100) number_allocated *= 2;
		else number_allocated += 100;

		buffer = new RunProfile[number_allocated];

		for (long counter = 0; counter < number_of_run_profiles; counter++)
		{
			buffer[counter] = run_profiles[counter];
		}

		delete [] run_profiles;
		run_profiles = buffer;
	}

	current_id_number++;
	run_profiles[number_of_run_profiles].id = current_id_number;
	run_profiles[number_of_run_profiles].name = "New Profile";
	run_profiles[number_of_run_profiles].number_of_run_commands = 0;
	run_profiles[number_of_run_profiles].manager_command = "$command";
	run_profiles[number_of_run_profiles].gui_address = "";
	run_profiles[number_of_run_profiles].controller_address = "";

	number_of_run_profiles++;

}

RunProfile * RunProfileManager::ReturnLastProfilePointer()
{
	return &run_profiles[number_of_run_profiles - 1];
}


RunProfile * RunProfileManager::ReturnProfilePointer(int wanted_profile)
{
	return &run_profiles[wanted_profile];
}

void RunProfileManager::RemoveProfile(int number_to_remove)
{
	MyDebugAssertTrue(number_to_remove >= 0 && number_to_remove < number_of_run_profiles, "Error: Trying to remove a profile that doesnt't exist");

	for (long counter = number_to_remove; counter < number_of_run_profiles - 1; counter++)
	{
		run_profiles[counter] = run_profiles[counter + 1];
	}

	number_of_run_profiles--;

}

void RunProfileManager::RemoveAllProfiles()
{
	number_of_run_profiles = 0;

	if (number_allocated > 100)
	{
		delete [] run_profiles;
		number_allocated = 100;
		run_profiles = new RunProfile[number_allocated];
	}

}

wxString RunProfileManager::ReturnProfileName(long wanted_profile)
{
	return run_profiles[wanted_profile].name;
}

long RunProfileManager::ReturnProfileID(long wanted_profile)
{
	return run_profiles[wanted_profile].id;
}

long RunProfileManager::ReturnTotalJobs(long wanted_profile)
{
	long total_jobs = 0;

	for (long counter = 0; counter < run_profiles[wanted_profile].number_of_run_commands; counter++)
	{
		total_jobs += run_profiles[wanted_profile].run_commands[counter].number_of_copies;
	}

	return total_jobs;
}


