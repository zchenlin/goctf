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




ImageFile::ImageFile()
{
	filename = wxFileName("");
	file_type = UNSUPPORTED_FILE_TYPE;
	file_type_string = "Unsupported file type";
}

ImageFile::ImageFile(std::string wanted_filename, bool overwrite)
{
	OpenFile(wanted_filename,overwrite);
}

ImageFile::~ImageFile()
{
	CloseFile();
}

void ImageFile::SetFileTypeFromExtension()
{
	wxString ext = filename.GetExt();
	if (ext.IsSameAs("tif") || ext.IsSameAs("tiff"))
	{
		file_type = TIFF_FILE;
		file_type_string = "TIFF";
	}
	else if (ext.IsSameAs("mrc") || ext.IsSameAs("mrcs") || ext.IsSameAs("ccp4"))
	{
		file_type = MRC_FILE;
		file_type_string = "MRC";
	}
	else if (ext.IsSameAs("dm3") || ext.IsSameAs("dm4") || ext.IsSameAs("dm"))
	{
		file_type = DM_FILE;
		file_type_string = "DM";
	}
	else
	{
		file_type = UNSUPPORTED_FILE_TYPE;
		file_type_string = "Unsupported file type";
	}
}

bool ImageFile::OpenFile(std::string wanted_filename, bool overwrite)
{
	bool file_seems_ok = false;
	filename = wanted_filename;
	SetFileTypeFromExtension();
	switch(file_type)
	{
	case TIFF_FILE: file_seems_ok = tiff_file.OpenFile(wanted_filename, overwrite); break;
	case MRC_FILE: file_seems_ok = mrc_file.OpenFile(wanted_filename, overwrite); break;
	case DM_FILE: file_seems_ok = dm_file.OpenFile(wanted_filename, overwrite); break;
	default: MyPrintWithDetails("Unsupported file type\n"); MyDebugAssertTrue(false,"Unsupported file type: %s",filename.GetFullPath().ToStdString()); abort; break;
	}
	return file_seems_ok;
}

void ImageFile::CloseFile()
{
	switch(file_type)
	{
	case TIFF_FILE: tiff_file.CloseFile(); break;
	case MRC_FILE: mrc_file.CloseFile(); break;
	case DM_FILE: dm_file.CloseFile(); break;
	}
}

void ImageFile::ReadSliceFromDisk(int slice_number, float *output_array)
{
	ReadSlicesFromDisk(slice_number,slice_number,output_array);
}

void ImageFile::ReadSlicesFromDisk(int start_slice, int end_slice, float *output_array)
{
	switch(file_type)
	{
	case TIFF_FILE: tiff_file.ReadSlicesFromDisk(start_slice, end_slice, output_array); break;
	case MRC_FILE: mrc_file.ReadSlicesFromDisk(start_slice, end_slice, output_array); break;
	case DM_FILE: dm_file.ReadSlicesFromDisk(start_slice-1, end_slice-1, output_array); break;
	default: MyPrintWithDetails("Unsupported file type\n"); abort; break;
	}
}

void ImageFile::WriteSliceToDisk(int slice_number, float *input_array)
{
	WriteSlicesToDisk(slice_number, slice_number, input_array);
}

void ImageFile::WriteSlicesToDisk(int start_slice, int end_slice, float *input_array)
{
	switch(file_type)
	{
	case TIFF_FILE: tiff_file.WriteSlicesToDisk(start_slice, end_slice, input_array); break;
	case MRC_FILE: mrc_file.WriteSlicesToDisk(start_slice, end_slice, input_array); break;
	case DM_FILE: dm_file.WriteSlicesToDisk(start_slice, end_slice, input_array); break;
	default: MyPrintWithDetails("Unsupported file type\n"); abort; break;
	}
}


int ImageFile::ReturnXSize()
{
	switch(file_type)
	{
	case TIFF_FILE: return tiff_file.ReturnXSize(); break;
	case MRC_FILE: return mrc_file.ReturnXSize(); break;
	case DM_FILE: return dm_file.ReturnXSize(); break;
	default: MyPrintWithDetails("Unsupported file type\n"); abort; break;
	}
}

int ImageFile::ReturnYSize()
{
	switch(file_type)
	{
	case TIFF_FILE: return tiff_file.ReturnYSize(); break;
	case MRC_FILE: return mrc_file.ReturnYSize(); break;
	case DM_FILE: return dm_file.ReturnYSize(); break;
	default: MyPrintWithDetails("Unsupported file type\n"); abort; break;
	}
}

int ImageFile::ReturnZSize()
{
	switch(file_type)
	{
	case TIFF_FILE: return tiff_file.ReturnZSize(); break;
	case MRC_FILE: return mrc_file.ReturnZSize(); break;
	case DM_FILE: return dm_file.ReturnZSize(); break;
	default: MyPrintWithDetails("Unsupported file type\n"); abort; break;
	}
}

int ImageFile::ReturnNumberOfSlices()
{
	switch(file_type)
	{
	case TIFF_FILE: return tiff_file.ReturnNumberOfSlices(); break;
	case MRC_FILE: return mrc_file.ReturnNumberOfSlices(); break;
	case DM_FILE: return dm_file.ReturnNumberOfSlices(); break;
	default: MyPrintWithDetails("Unsupported file type\n"); abort; break;
	}
}

bool ImageFile::IsOpen()
{
	switch(file_type)
	{
	case TIFF_FILE: return tiff_file.IsOpen(); break;
	case MRC_FILE: return mrc_file.IsOpen(); break;
	case DM_FILE: return dm_file.IsOpen();
	default: MyPrintWithDetails("Unsupported file type\n"); abort; break;
	}
}

void ImageFile::PrintInfo()
{
	wxPrintf("File name: %s\n",filename.GetFullName());
	wxPrintf("File type: %s\n",file_type_string);
	wxPrintf("Dimensions: X = %i Y = %i Z = %i\n",ReturnXSize(),ReturnYSize(),ReturnZSize());
	wxPrintf("Number of slices: %i\n",ReturnNumberOfSlices());
}

