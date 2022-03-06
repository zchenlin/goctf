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

/*
 * Provide a common interface to interact with all the different
 * files we support (MRC, DM, TIF, etc.)
 */

enum supported_file_types
{
	MRC_FILE,
	TIFF_FILE,
	DM_FILE,
	UNSUPPORTED_FILE_TYPE
};


class ImageFile : public AbstractImageFile {
private:
	// These are the actual file objects doing the work
	MRCFile mrc_file;
	TiffFile tiff_file;
	DMFile dm_file;


	int file_type;
	wxString file_type_string;
	void SetFileTypeFromExtension();

public:
	ImageFile();
	ImageFile(std::string wanted_filename, bool overwrite = false);
	~ImageFile();

	int ReturnXSize();
	int ReturnYSize();
	int ReturnZSize();
	int ReturnNumberOfSlices();

	bool IsOpen();

	bool OpenFile(std::string wanted_filename, bool overwrite);
	void CloseFile();

	void ReadSliceFromDisk(int slice_number, float *output_array);
	void ReadSlicesFromDisk(int start_slice, int end_slice, float *output_array);

	void WriteSliceToDisk(int slice_number, float *input_array);
	void WriteSlicesToDisk(int start_slice, int end_slice, float *input_array);

	void PrintInfo();
};


