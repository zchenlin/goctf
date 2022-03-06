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

/*  \brief  AnglesAndShifts class (particle Euler angles and shifts) */

class AnglesAndShifts {

public:

	RotationMatrix	euler_matrix;

	AnglesAndShifts();
	AnglesAndShifts(float wanted_euler_phi, float wanted_euler_theta, float wanted_euler_psi, float wanted_shift_x = 0.0, float wanted_shift_y = 0.0);	// constructor with size
//	~AnglesAndShifts();							// destructor

	void GenerateEulerMatrices(float wanted_euler_phi, float wanted_euler_theta, float wanted_euler_psi);
	void GenerateRotationMatrix2D(float wanted_rotation_angle_in_degrees);
	void Init(float wanted_euler_phi_in_degrees, float wanted_euler_theta_in_degrees, float wanted_euler_psi_in_degrees, float wanted_shift_x, float wanted_shift_y);

	inline float ReturnPhiAngle() {return euler_phi;};
	inline float ReturnThetaAngle() {return euler_theta;};
	inline float ReturnPsiAngle() {return euler_psi;};
	inline float ReturnShiftX() {return shift_x;};
	inline float ReturnShiftY() {return shift_y;};

private:

	float 			euler_phi;		// in degrees
	float 			euler_theta;	// in degrees
	float 			euler_psi;		// in degrees
	float			shift_x;		// in Angstrom
	float			shift_y;		// in Angstrom
};
