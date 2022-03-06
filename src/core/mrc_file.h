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

//  \brief  Object for manipulating MRC Files..
//

class MRCFile : public AbstractImageFile {

	public:

	std::fstream my_file;
	MRCHeader my_header;
	wxString filename;

	bool rewrite_header_on_close;


	MRCFile();
	MRCFile(std::string filename, bool overwrite = false);
	~MRCFile();

	inline int ReturnXSize() {MyDebugAssertTrue(my_file.is_open(), "File not open!");	return my_header.ReturnDimensionX();};
	inline int ReturnYSize() {MyDebugAssertTrue(my_file.is_open(), "File not open!");	return my_header.ReturnDimensionY();};
	inline int ReturnZSize() {MyDebugAssertTrue(my_file.is_open(), "File not open!");	return my_header.ReturnDimensionZ();};
	inline int ReturnNumberOfSlices() {MyDebugAssertTrue(my_file.is_open(), "File not open!");	return my_header.ReturnDimensionZ();};

	inline bool IsOpen() {return my_file.is_open();}

	bool OpenFile(std::string filename, bool overwrite = false);
	void CloseFile();

	inline void ReadSliceFromDisk(int slice_number, float *output_array) {ReadSlicesFromDisk(slice_number, slice_number, output_array);}
	void ReadSlicesFromDisk(int start_slice, int end_slice, float *output_array);

	inline void WriteSliceToDisk(int slice_number, float *input_array) {WriteSlicesToDisk(slice_number, slice_number, input_array);}
	void WriteSlicesToDisk(int start_slice, int end_slice, float *input_array);

	inline void WriteHeader() {my_header.WriteHeader(&my_file);};

	void PrintInfo();

	void SetPixelSize(float wanted_pixel_size);
	inline void SetDensityStatistics( float wanted_min, float wanted_max, float wanted_mean, float wanted_rms ){my_header.SetDensityStatistics(wanted_min, wanted_max, wanted_mean, wanted_rms);}

};
