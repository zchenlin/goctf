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

class AbstractImageFile {

public:

	wxFileName filename;

	AbstractImageFile();
	AbstractImageFile(std::string filename, bool overwrite = false);
	~AbstractImageFile();

	virtual int ReturnXSize() = 0;
	virtual int ReturnYSize() = 0;
	virtual int ReturnZSize() = 0;
	virtual int ReturnNumberOfSlices() = 0;

	virtual bool IsOpen() = 0;

	virtual bool OpenFile(std::string filename, bool overwrite = false) = 0; // Return true if everything about the file looks OK
	virtual void CloseFile() = 0;

	virtual void ReadSliceFromDisk(int slice_number, float *output_array) = 0;
	virtual void ReadSlicesFromDisk(int start_slice, int end_slice, float *output_array) = 0;

	virtual void WriteSliceToDisk(int slice_number, float *input_array) = 0;
	virtual void WriteSlicesToDisk(int start_slice, int end_slice, float *input_array) = 0;

	virtual void PrintInfo() = 0;


};
