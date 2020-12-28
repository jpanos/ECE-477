#!/bin/bash

#######################################################################

# File: rebase.sh

# Description: A command which rebases the <base> tag on all HTML files

# 	in a given directory tree, facilitating moves

# Author: George Hadley
# Created: 8/24/15

# Last Modified: 8/24/15

#######################################################################
# Find all relevant HTML files

for f in $(find . -name '*.html' -print0 | xargs -0 grep -i -l "<BASE") 
do
	#Uncomment the following line; replace 477grp<x> with your team number (i.e. 477grp1, 477grp2, ...
	sed -i 's|base href="https://engineering.purdue.edu/ece477/StudentWebTemplate|base href="https://engineering.purdue.edu/477grp4|' ${f}
done
