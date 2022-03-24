#!/usr/bin/env python
#
# Copyright (c) 2018 The Regents of the University of Michigan
# Min Su, Ph.D.
#
# This file is part of goCTF.
#
# goCTF is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# goCTF is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with goCTF.  If not, see <https://www.gnu.org/licenses/>.
#
# This complete copyright notice must be included in any revised version of the
# source code. Additional authorship citations may be added, but existing
# author citations must be preserved.
#
#
#
# Purpose: This is to sort and split a particles.star file into individual star
# files that are associated with their corresponding micrograph.

import os
import sys
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-f", "--file", dest="input_star", help="Path to the particles.star file. [Required]")
parser.add_option("-v", "--verbose", action="store_true", dest="verbose", help="Print out debugging messages.")
options, args = parser.parse_args()

if options.input_star is None:
    print ('Missing a required option.\n')
    parser.print_help()
    sys.exit(-1)


def main():
    if not os.path.isfile(options.input_star):
        raise Exception('{} is not a file.'.format(options.input_star))

    star_fh = open(options.input_star)
    star_contents = star_fh.readlines()
    star_fh.close()

    rln_micrograph_name_found = False
    rln_coordinate_x_found = False
    rln_coordinate_y_found = False
    rln_defocus_u_found = False
    rln_defocus_v_found = False
    particle_start = 0
    particle_end = 0

    for i in range(0, len(star_contents)):
        line = star_contents[i]

        if line.find('_rlnMicrographName') > -1:
            micrograph_name_entry = int(line.split()[1].replace('#', ''))
            rln_micrograph_name_found = True
            if options.verbose is True:
                print ('Found _rlnMicrographName entry: {:d}'.format(micrograph_name_entry))

        if line.find('_rlnCoordinateX') > -1:
            coordinate_x_entry = int(line.split()[1].replace('#', ''))
            rln_coordinate_x_found = True
            if options.verbose is True:
                print ('Found _rlnCoordinateX entry: {:d}'.format(coordinate_x_entry))

        if line.find('_rlnCoordinateY') > -1:
            coordinate_y_entry = int(line.split()[1].replace('#', ''))
            rln_coordinate_y_found = True
            if options.verbose is True:
                print ('Found _rlnCoordinateY entry: {:d}'.format(coordinate_y_entry))

        if line.find('_rlnDefocusU') > -1:
            defocus_u_entry = int(line.split()[1].replace('#', ''))
            rln_defocus_u_found = True
            if options.verbose is True:
                print ('Found _rlnDefocusU entry: {:d}'.format(defocus_u_entry))

        if line.find('_rlnDefocusV') > -1:
            defocus_v_entry = int(line.split()[1].replace('#', ''))
            rln_defocus_v_found = True
            if options.verbose is True:
                print ('Found _rlnDefocusV entry: {:d}'.format(defocus_v_entry))

        if particle_start == 0 and line.find('@') > -1:
            particle_start = i
            if options.verbose is True:
                print ('First particle start from line {:d}'.format(particle_start))

        if particle_start > 0:
            if line.find('@') < 0:
                particle_end = i - 1
                if options.verbose is True:
                    print ('Last particle ends at line {:d}'.format(particle_end))
            elif i == len(star_contents) - 1:
                particle_end = i
                if options.verbose is True:
                    print ('Last particle ends at line {:d}'.format(particle_end))

        if rln_micrograph_name_found is False and i == len(star_contents) - 1:
            raise Exception('Unable to find the label _rlnMicrographName')

        if rln_coordinate_x_found is False and i == len(star_contents) - 1:
            raise Exception('Unable to find the label _rlnCoordinateX')

        if rln_coordinate_y_found is False and i == len(star_contents) - 1:
            raise Exception('Unable to find the label _rlnCoordinateY')

        if rln_defocus_u_found is False and i == len(star_contents) - 1:
            raise Exception('Unable to find the label _rlnDefocusU')

        if rln_defocus_v_found is False and i == len(star_contents) - 1:
            raise Exception('Unable to find the label _rlnDefocusV')

        if particle_start == 0 and i == len(star_contents) - 1:
            raise Exception('Unable to find any particles.')

    # Initialize a list with as many members as there are particles and set the value to 0 as a placeholder.
    li = [0 for x in range(particle_end - particle_start + 1)]
    num_particles = particle_end - particle_start + 1
    print ('Total number of particles in the input particles.star file is: {:d}'.format(num_particles))

    i = 0
    for line in star_contents:
        if line.find('@') > -1:
            li[i] = line.split()
            # print li[i][micrograph_name_entry-1]
            i = i + 1

    # star_mic_new_p = open(star_mic_new,'w')

    a = sorted(li, key=lambda a_entry: a_entry[micrograph_name_entry - 1])
    for k in range(len(a)):
        if k == 0:
            star_out_name = a[k][micrograph_name_entry - 1].replace('.mrc', '_go.star')
            print (star_out_name)
            link_src = a[k][micrograph_name_entry - 1]
            star_out_p = open(star_out_name, 'w')
            for n in range(particle_start):
                star_out_p.writelines(star_contents[n])
            sorted_line = '  '.join(a[k])
            star_out_p.writelines('{} \n'.format(sorted_line))

        if k > 0 and k < len(a) - 1:
            if a[k][micrograph_name_entry - 1] == a[k - 1][micrograph_name_entry - 1] and a[k][micrograph_name_entry - 1] != a[k + 1][micrograph_name_entry - 1]:
                sorted_line = '  '.join(a[k])
                star_out_p.writelines('{} \n'.format(sorted_line))
                star_out_p.close()
                star_out_name = a[k + 1][micrograph_name_entry - 1].replace('.mrc', '_go.star')
                print (star_out_name)
                star_out_p = open(star_out_name, 'w')
                for n in range(particle_start):
                    star_out_p.writelines(star_contents[n])
            else:
                sorted_line = '  '.join(a[k])
                star_out_p.writelines('{} \n'.format(sorted_line))

        if k == len(a) - 1:
            if a[k][micrograph_name_entry - 1] == a[k - 1][micrograph_name_entry - 1]:
                sorted_line = '  '.join(a[k])
                star_out_p.writelines('{} \n'.format(sorted_line))
                # star_out_p.close()
            else:
                star_out_name = a[k][micrograph_name_entry - 1].replace('.mrc', '_go.star')
                print (star_out_name)
                star_out_p = open(star_out_name, 'w')
                for n in range(particle_start):
                    star_out_p.writelines(star_contents[n])
                sorted_line = '  '.join(a[k])
                star_out_p.writelines('{} \n'.format(sorted_line))

    star_out_p.close()


if __name__ == '__main__':
    main()
