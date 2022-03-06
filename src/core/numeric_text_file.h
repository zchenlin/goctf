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

#define OPEN_TO_READ 0
#define OPEN_TO_WRITE 1
#define OPEN_TO_APPEND 2

class NumericTextFile {

        private:

                void Init();
                wxString text_filename;
                long access_type;
                wxFileInputStream *input_file_stream;
                wxTextInputStream *input_text_stream;
                wxFileOutputStream *output_file_stream;
                wxTextOutputStream *output_text_stream;

	public:


		// Constructors
		NumericTextFile();
		NumericTextFile(wxString Filename, long wanted_access_type, long wanted_records_per_line = 1);
		~NumericTextFile();

		// data

		int number_of_lines;
		int records_per_line;

		// Methods

        void Open(wxString Filename, long wanted_access_type, long wanted_records_per_line = 1);
        void Close();
        void Rewind();
        void Flush();
        wxString ReturnFilename();

		void ReadLine(float *data_array);
        void WriteLine(float *data_array);
        void WriteLine(double *data_array);
        void WriteCommentLine(const char * format, ...);

};
