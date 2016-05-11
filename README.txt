Matlab Registration - All the MATLAB code required to perform ICP with point-to-plane point cloud registration. The main code is located in registration.m . The program reads files in a folder with the file names 1.pcd, 2.pcd, 3.pcd, etc.

Registration - main.cpp contains the code for the SAC-IA with FPFH and ICP registration. Put the names of the template clouds into object_templates.txt and load the program with ./registration object_templates.txt <name of target cloud>

			   main.cpp.oldv2 contains the code for the ICP with SVD estimation. ./registration to run, it reads the files the same way the matlab program reads the files.

Apologies for the code being a mess. Some returns are pointers, others arent, etc.
