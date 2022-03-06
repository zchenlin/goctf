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

/*  \brief  RotationMatrix class */

class RotationMatrix {

public:

	float		m[3][3];                /* 3D rotation matrix*/

	RotationMatrix();
//	~RotationMatrix();

	RotationMatrix operator + (const RotationMatrix &other);
	RotationMatrix operator - (const RotationMatrix &other);
	RotationMatrix operator * (const RotationMatrix &other);
	RotationMatrix &operator = (const RotationMatrix &other);
	RotationMatrix &operator = (const RotationMatrix *other);
	RotationMatrix &operator += (const RotationMatrix &other);
	RotationMatrix &operator += (const RotationMatrix *other);
	RotationMatrix &operator -= (const RotationMatrix &other);
	RotationMatrix &operator -= (const RotationMatrix *other);
	RotationMatrix &operator *= (const RotationMatrix &other);
	RotationMatrix &operator *= (const RotationMatrix *other);
	RotationMatrix ReturnTransposed();
	void SetToIdentity();
	void SetToConstant(float constant);
	void SetToValues(float m00, float m10, float m20, float m01, float m11, float m21, float m02, float m12, float m22);
	inline void RotateCoords(float &input_x_coord, float &input_y_coord, float &input_z_coord, float &output_x_coord, float &output_y_coord, float &output_z_coord)
	{
		output_x_coord = this->m[0][0] * input_x_coord + this->m[0][1] * input_y_coord + this->m[0][2] * input_z_coord;
		output_y_coord = this->m[1][0] * input_x_coord + this->m[1][1] * input_y_coord + this->m[1][2] * input_z_coord;
		output_z_coord = this->m[2][0] * input_x_coord + this->m[2][1] * input_y_coord + this->m[2][2] * input_z_coord;
	};
	inline void RotateCoords2D(float &input_x_coord, float &input_y_coord, float &output_x_coord, float &output_y_coord)
	{
		output_x_coord = this->m[0][0] * input_x_coord + this->m[0][1] * input_y_coord;
		output_y_coord = this->m[1][0] * input_x_coord + this->m[1][1] * input_y_coord;
	};
};
