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

RotationMatrix::RotationMatrix()
{
	SetToConstant(0.0);
}

RotationMatrix RotationMatrix::operator + (const RotationMatrix &other)
{
	RotationMatrix temp_matrix;

	temp_matrix.m[0][0] = this->m[0][0] + other.m[0][0];
	temp_matrix.m[0][1] = this->m[0][1] + other.m[0][1];
	temp_matrix.m[0][2] = this->m[0][2] + other.m[0][2];
	temp_matrix.m[1][0] = this->m[1][0] + other.m[1][0];
	temp_matrix.m[1][1] = this->m[1][1] + other.m[1][1];
	temp_matrix.m[1][2] = this->m[1][2] + other.m[1][2];
	temp_matrix.m[2][0] = this->m[2][0] + other.m[2][0];
	temp_matrix.m[2][1] = this->m[2][1] + other.m[2][1];
	temp_matrix.m[2][2] = this->m[2][2] + other.m[2][2];

    return temp_matrix;
}

RotationMatrix RotationMatrix::operator - (const RotationMatrix &other)
{
	RotationMatrix temp_matrix;

	temp_matrix.m[0][0] = this->m[0][0] - other.m[0][0];
	temp_matrix.m[0][1] = this->m[0][1] - other.m[0][1];
	temp_matrix.m[0][2] = this->m[0][2] - other.m[0][2];
	temp_matrix.m[1][0] = this->m[1][0] - other.m[1][0];
	temp_matrix.m[1][1] = this->m[1][1] - other.m[1][1];
	temp_matrix.m[1][2] = this->m[1][2] - other.m[1][2];
	temp_matrix.m[2][0] = this->m[2][0] - other.m[2][0];
	temp_matrix.m[2][1] = this->m[2][1] - other.m[2][1];
	temp_matrix.m[2][2] = this->m[2][2] - other.m[2][2];

    return temp_matrix;
}

RotationMatrix RotationMatrix::operator * (const RotationMatrix &other)
{
	RotationMatrix temp_matrix;

	temp_matrix.m[0][0] = this->m[0][0] * other.m[0][0] + this->m[0][1] * other.m[1][0] + this->m[0][2] * other.m[2][0];
	temp_matrix.m[0][1] = this->m[0][0] * other.m[0][1] + this->m[0][1] * other.m[1][1] + this->m[0][2] * other.m[2][1];
	temp_matrix.m[0][2] = this->m[0][0] * other.m[0][2] + this->m[0][1] * other.m[1][2] + this->m[0][2] * other.m[2][2];
	temp_matrix.m[1][0] = this->m[1][0] * other.m[0][0] + this->m[1][1] * other.m[1][0] + this->m[1][2] * other.m[2][0];
	temp_matrix.m[1][1] = this->m[1][0] * other.m[0][1] + this->m[1][1] * other.m[1][1] + this->m[1][2] * other.m[2][1];
	temp_matrix.m[1][2] = this->m[1][0] * other.m[0][2] + this->m[1][1] * other.m[1][2] + this->m[1][2] * other.m[2][2];
	temp_matrix.m[2][0] = this->m[2][0] * other.m[0][0] + this->m[2][1] * other.m[1][0] + this->m[2][2] * other.m[2][0];
	temp_matrix.m[2][1] = this->m[2][0] * other.m[0][1] + this->m[2][1] * other.m[1][1] + this->m[2][2] * other.m[2][1];
	temp_matrix.m[2][2] = this->m[2][0] * other.m[0][2] + this->m[2][1] * other.m[1][2] + this->m[2][2] * other.m[2][2];

    return temp_matrix;
}

RotationMatrix &RotationMatrix::operator = (const RotationMatrix &other)	// &other contains the address of the other matrix
{
	*this = &other;
	return *this;
}

RotationMatrix &RotationMatrix::operator = (const RotationMatrix *other)	// *other is a pointer to the other matrix
{
   // Check for self assignment
   if (this != other)
   {
	   this->m[0][0] = other->m[0][0];
	   this->m[0][1] = other->m[0][1];
	   this->m[0][2] = other->m[0][2];
	   this->m[1][0] = other->m[1][0];
	   this->m[1][1] = other->m[1][1];
	   this->m[1][2] = other->m[1][2];
	   this->m[2][0] = other->m[2][0];
	   this->m[2][1] = other->m[2][1];
	   this->m[2][2] = other->m[2][2];
   }

   return *this;
}

RotationMatrix &RotationMatrix::operator += (const RotationMatrix &other)
{
	*this += &other;
	return *this;
}

RotationMatrix &RotationMatrix::operator += (const RotationMatrix *other)
{
	this->m[0][0] += other->m[0][0];
	this->m[0][1] += other->m[0][1];
	this->m[0][2] += other->m[0][2];
	this->m[1][0] += other->m[1][0];
	this->m[1][1] += other->m[1][1];
	this->m[1][2] += other->m[1][2];
	this->m[2][0] += other->m[2][0];
	this->m[2][1] += other->m[2][1];
	this->m[2][2] += other->m[2][2];
	return *this;
}

RotationMatrix &RotationMatrix::operator -= (const RotationMatrix &other)
{
	*this -= &other;
	return *this;
}

RotationMatrix &RotationMatrix::operator -= (const RotationMatrix *other)
{
	this->m[0][0] -= other->m[0][0];
	this->m[0][1] -= other->m[0][1];
	this->m[0][2] -= other->m[0][2];
	this->m[1][0] -= other->m[1][0];
	this->m[1][1] -= other->m[1][1];
	this->m[1][2] -= other->m[1][2];
	this->m[2][0] -= other->m[2][0];
	this->m[2][1] -= other->m[2][1];
	this->m[2][2] -= other->m[2][2];
	return *this;
}

RotationMatrix &RotationMatrix::operator *= (const RotationMatrix &other)
{
	*this *= &other;
	return *this;
}

RotationMatrix &RotationMatrix::operator *= (const RotationMatrix *other)
{
	RotationMatrix temp_matrix;

	temp_matrix = *this * *other;
	*this = temp_matrix;

    return *this;
}

RotationMatrix RotationMatrix::ReturnTransposed()
{
	RotationMatrix temp_matrix;

	temp_matrix.m[0][0] = this->m[0][0];
	temp_matrix.m[0][1] = this->m[1][0];
	temp_matrix.m[0][2] = this->m[2][0];
	temp_matrix.m[1][0] = this->m[0][1];
	temp_matrix.m[1][1] = this->m[1][1];
	temp_matrix.m[1][2] = this->m[2][1];
	temp_matrix.m[2][0] = this->m[0][2];
	temp_matrix.m[2][1] = this->m[1][2];
	temp_matrix.m[2][2] = this->m[2][2];

    return temp_matrix;
}

void RotationMatrix::SetToIdentity()
{
	this->m[0][0] = 1.0;
	this->m[1][0] = 0.0;
	this->m[2][0] = 0.0;
	this->m[0][1] = 0.0;
	this->m[1][1] = 1.0;
	this->m[2][1] = 0.0;
	this->m[0][2] = 0.0;
	this->m[1][2] = 0.0;
	this->m[2][2] = 1.0;
}

void RotationMatrix::SetToConstant(float constant)
{
	this->m[0][0] = constant;
	this->m[1][0] = constant;
	this->m[2][0] = constant;
	this->m[0][1] = constant;
	this->m[1][1] = constant;
	this->m[2][1] = constant;
	this->m[0][2] = constant;
	this->m[1][2] = constant;
	this->m[2][2] = constant;
}

void RotationMatrix::SetToValues(float m00, float m10, float m20, float m01, float m11, float m21, float m02, float m12, float m22)
{
	this->m[0][0] = m00;
	this->m[1][0] = m10;
	this->m[2][0] = m20;
	this->m[0][1] = m01;
	this->m[1][1] = m11;
	this->m[2][1] = m21;
	this->m[0][2] = m02;
	this->m[1][2] = m12;
	this->m[2][2] = m22;
}

