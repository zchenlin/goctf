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

class AssetGroup {

	long number_allocated;



  public:
	
	AssetGroup();
	AssetGroup( const AssetGroup &obj); // copy contructor
	~AssetGroup();


	int id;
	long *members;
	long number_of_members;
	wxString name;

	bool can_be_picked; // Used by FindParticlesPanel to keep track of whether all images within the group are ready to be picked

	void SetName(wxString wanted_name);
	void AddMember(long number_to_add);
	void RemoveMember(long number_to_remove);
	void RemoveAll();

	void CopyFrom(AssetGroup *other_group);

	long FindMember(long member_to_find);

	AssetGroup & operator = (const AssetGroup &t);
	AssetGroup & operator = (const AssetGroup *t);

};


class AssetGroupList {

	long number_allocated;

public:

	AssetGroupList();
	~AssetGroupList();

	long number_of_groups;
	AssetGroup *groups;

	long ReturnNumberOfGroups();

	void AddGroup(wxString name);
	void AddGroup(AssetGroup *group_to_add);

	void RemoveGroup(long number_to_remove);
	void AddMemberToGroup(long wanted_group_number, long member_to_add);
	long ReturnGroupMember(long wanted_group_number, long wanted_member);
	void RemoveAssetFromExtraGroups(long wanted_asset);
	void ShiftMembersDueToAssetRemoval(long number_to_shift_after);
	void RemoveAll();
};



