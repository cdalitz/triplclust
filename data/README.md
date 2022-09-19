TriplClust Reference Data Files
===============================

These files contain point clouds that can be used as input for the
TriplClust algorithm that is described in (cited as "IPOL paper" below):

> C. Dalitz, J. Wilberg, L. Aymans: "TriplClust: An Algorithm
> for Curve Detection in 3D Point Clouds."
> Image Processing Online 9, pp. 26-46 (2019)


Description
-----------

See the IPOL paper for a detailed description of the files and for
their origin. The data files have the exension ".dat".

Depending on the file, the following command line paramters should
be used:

    triplclust attpc.dat

    triplclust lidar.dat -k 12  -t 12

    triplclust radar.dat -a 0.003

    triplclust synthetic-clean.dat

    triplclust synthetic-noise.dat -r 1.0

    triplclust tennis.dat -k 12

For visualization of the results, you can use gnuplot, e.g.:

    triplclust attpc.dat -gnuplot | gnuplot


Author & Copyright
------------------

Christoph Dalitz 2018
Institute for Pattern Recognition
Niederrhein University of Applied Sciences
Krefeld, Germany

These files can be freely used, copied, modified and distributed under
the terms of Creative Commons license CC BY-SA 3.0. Attribution should be
done by referencing the IPOL paper.


