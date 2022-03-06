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

#ifndef __SOCKET_CODES_H__
#define __SOCKET_CODES_H__

#define SOCKET_CODE_SIZE 16

#define SETUP_SOCKET_CODES unsigned char socket_input_buffer[SOCKET_CODE_SIZE];\
const unsigned char socket_please_identify [] = "JcFG>&P.RuC9,>za"; \
const unsigned char socket_identification [] = "eTdLfTB2kQs/{7a-"; \
const unsigned char socket_you_are_connected [] = "J82zjSwYY^-!bF>4"; \
const unsigned char socket_send_job_details [] = "gr<V>ThBp6w9fzLg"; \
const unsigned char socket_ready_to_send_job_package [] = "'8ujA!Lup%PR*!hG";
const unsigned char socket_send_job_package [] = "x3u.8?Kdvx!fPr<z";
const unsigned char socket_you_are_the_master [] = "eVmYc.3!g}}cZZs";
const unsigned char socket_you_are_a_slave [] = "U6u*:z6}W+7nV2g'";
const unsigned char socket_send_next_job [] = "\7PnJh=x;[b#f/6L";
const unsigned char socket_time_to_die [] = ")[czL7$#Sg/d4-*K";
const unsigned char socket_ready_to_send_single_job [] = "-TDv(X*kY.:d`D5:";
const unsigned char socket_send_single_job [] = "z&6GgM8/}4~H;*C6";
const unsigned char socket_i_have_an_error [] = "8TU.cDc3jr,rb[SN";
const unsigned char socket_i_have_info [] = "+5nxvY@\t.!_R#Vn";
const unsigned char socket_job_finished [] = "jNA[3!VdLdkb$LwM";
const unsigned char socket_number_of_connections [] = "Uu6tsQ,z}M''T`7f";
const unsigned char socket_all_jobs_finished [] = "aL)yaH[$3s;9Ymk6";
const unsigned char socket_job_result [] = "3F6E_.``L6YC^q[U";
const unsigned char socket_job_result_queue [] = "^}`@pF9m;{m9k=$F";


#endif

