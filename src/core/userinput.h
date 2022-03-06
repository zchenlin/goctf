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

// User Input Class :-

class UserInput {

        private:

                void Init(const char *program_name, wxString program_version);

                FILE *defaults_file;
                FILE *new_defaults_file;

                char defaults_filename[1000];
                char new_defaults_filename[1000];
                char memory_string[10000];

                void GetDefault(const char *my_text, const char *default_default_value, char *default_value);
                void CarryOverDefaults();
                void DoGotValidAnswer(const char *question_text, const char *new_default);
                void DoGotInvalidAnswer();

                bool output_is_a_tty;
                bool input_is_a_tty;

	public:


		// Constructors
		UserInput();
		UserInput(const char *program_name, float program_version);
		UserInput(const char *program_name, wxString program_version);
		~UserInput();

		// Methods
				void AskQuestion(const char *question_text, const char *help_text, const char *default_value, char *received_input);

                float GetFloatFromUser(const char * my_text, const char * help_text, const char * wanted_default_value = 0, float min_value = -FLT_MAX, float max_value = FLT_MAX);
                int GetIntFromUser(const char * my_text, const char * help_text, const char * wanted_default_value = 0, int min_value = INT_MIN, int max_value = INT_MAX);
                std::string GetFilenameFromUser(const char * my_question_text, const char * help_text, const char * wanted_default_value = 0, bool must_exist = false);
                std::string GetStringFromUser(const char * my_question_text, const char * help_text, const char * wanted_default_value = 0);
                std::string GetSymmetryFromUser(const char * my_question_text, const char * help_text, const char * wanted_default_value = 0);
                bool GetYesNoFromUser(const char * my_test, const char * help_text, const char * wanted_default_value = 0);

                //void GetTextFromUser(char *text_buffer, const char * my_text, const char * help_text, const char * wanted_default_value = 0);
                //void GetTXTFilenameFromUser(char *Filename, const char * my_text, const char * help_text, long status, const char * wanted_default_value = 0);
                //void GetPLTFilenameFromUser(char *Filename, const char * my_text, const char * help_text, long status, const char * wanted_default_value = 0);
                // void GetCLSFilenameFromUser(char *Filename, const char * my_text, const char * help_text, long status, const char * wanted_default_value = 0);
                //void GetPlainFilenameFromUser(char *Filename, const char * my_text, const char * help_text, long status, const char * wanted_default_value);
                //void GetGeneralFilenameFromUser(char *Filename, const char * my_text, const char * help_text, long status, const char * extension, const char * wanted_default_value);
                //void GetSymmetryFromUser(char *symmetry_type, long *symmetry_number, const char * my_text, const char * help_text, const char * wanted_default_value = 0);
                //long GetOptionFromUser(char *option_list, long number_of_options, const char * my_text, const char * help_text, const char * wanted_default_value = 0);

};
