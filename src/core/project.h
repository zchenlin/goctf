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

class Project {



public :


	Database database;

	bool is_open;
	wxString project_name;

	wxFileName project_directory;
	wxFileName movie_asset_directory;
	wxFileName image_asset_directory;
	wxFileName volume_asset_directory;
	wxFileName ctf_asset_directory;
	wxFileName particle_position_asset_directory;
	wxFileName particle_stack_directory;
	wxFileName class_average_directory;

	wxFileName parameter_file_directory;
	wxFileName scratch_directory;

	double total_cpu_hours;
	int total_jobs_run;

	Project();
	~Project();

	void Close();
	bool CreateNewProject(wxFileName database_file, wxString project_directory, wxString project_name);
	bool OpenProjectFromFile(wxFileName file_to_open);
	bool ReadMasterSettings();


};
