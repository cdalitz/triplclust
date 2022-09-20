TriplClust: Curve Detection in 3D Point Clouds
==============================================

TriplClust reads a 3D or 2D point cloud from a CSV file and assigns
each point cluster labels representing curves or noise.

This code implements the algorithm described in (cited as "IPOL paper" below):

> C. Dalitz, J. Wilberg, L. Aymans: "TriplClust: An Algorithm
> for Curve Detection in 3D Point Clouds."
> Image Processing Online 9, pp. 26-46 (2019)
> https://doi.org/10.5201/ipol.2019.234

Please cite this article when using the code. The article was published
on IPOL with version 1.3 of this code. For changes since then, see the file
*CHANGES*.


Compilation
-----------

Building the code requires cmake and a standard C++98 (or later) compiler.
We have tested the code with gcc 5.4.0, LLVM 9.0.0, and MSVC 15.7.5.

Starting from the root directory (i.e., the directory, in which this
Readme file is located), the code is compiled with ($ is the shell prompt):

    $ mkdir build
	$ cd build
	$ cmake ..
	$ make

This will create the executable "triplclust".


Usage
-----

Calling triplclust without any or with an unknown option (e.g. "-?")
will print a usage message. The meaning of the parameters controlling
the algorithm is explained in the IPOL paper.

The input file can contain 3D or 2D point coordinates, one point per line
with the coordinates separated by a delimiter character. The default delimiter
is the space character, but a different character can be specified with the
command line option "-delim <char>". Lines starting with a hash (#) are
ignored. 

Unless the option "-oprefix <prefix>" is given, the output is printed to
stdout. The default output format is a comma separated file with two header
lines (starting with #) and one point per line followed by the cluster label.
For points belonging to more than one cluster, all labels are given separated
by semicolons.

With the option "-gnuplot", the output is instead a gnuplot command that
can directly be used for visualizing the result. When the options "-gnuplot"
and "-oprefix <prefix>" are given, the result is not printed to stdout,
but into two files: CSV format to "<prefix>.csv", gnuplot command to
"<prefix>.gnuplot".

When the option "-v" is given, automatically computed default values are
additionally printed to stdout with the prefix "[Info]".

Example calls with the provided test file 'test.dat':

  1) direct visualization with gnuplot:  
     ``$ triplclust test.dat -gnuplot | gnuplot -persist``

  2) write result to files result.csv and result.gnuplot and print
     automatically computed parameters:  
     ``$ triplclust test.dat -v -oprefix result -gnuplot``

When the option "-vv" is given, data files documenting intermediate steps
of the algorithm are written: "debug_smoothed.csv" and "debug_smoothed.gnuplot"
contain the points after smoothing (Figure 2 in the IPOL paper), and
"debug_cdist.csv" contains the sequence of cluster distances of all merge
steps in the hierarchical clustering. This file can be important for finding
good stopping thresholds for the clustering. A plot similar to Figure 4 in
the IPOL paper can be created with the R script "cdist-plot.r".
Example ("$" is the shell prompt)::

    $ triplclust test.dat -vv
    $ Rscript cdist-plot.r
    cdist plot is written to 'debug_cdist.pdf'


Source Files
------------

 - ``main.cpp``  
   Main program that calls the four steps of the algorithm
   (beginning of section 2 in the IPOL paper)

 - ``pointcloud.[h|cpp]``  
   Implementation of 3D points and clouds thereof,
   and the position smoothing described in section 2.1 of the IPOL paper

 - ``triplet.[h|cpp]``
   Implementation of triplets of three points and their grouping
   (section 2.2 of the IPOL paper), and of the triplet distance
   (section 2.3.1 of the IPOL paper)

 - ``cluster.[h|cpp]``
   Implementation of the hierarchical clustering and the stopping
   criterion (section 2.3.2 of the IPOL paper)

 - ``graph.[h|cpp]``
   Implementation of the optional split up of clusters at gaps
   (section 2.4 of the IPOL paper)

 - ``dnn.[h|cpp]``
   Implementation of the characteristic length computation
   (section 3.1 of the IPOL paper)

 - ``option.[h|cpp]``
   Utilities for handling and storing command line options.

 - ``util.[h|cpp]``
   Convenience functions not fitting elsewhere.

The subdirectories ``hclust/`` and ``kdtree/`` contain the fastcluster
implementation by Daniel Müllner and the kd-tree implementation by
Christoph Dalitz.

The subdirectory ``data/`` contains the six reference point clouds discussed
in the IPOL paper.


Authors & Copyright
-------------------

Jens Wilberg, Christoph Dalitz, Lukas Aymans, 2017-2018  
Institute for Pattern Recognition  
Niederrhein University of Applied Sciences  
Krefeld, Germany

The software includes parts of the fastcluster library by Daniel Müllner,
available from http://danifold.net. See the directory ``src/hclust``
for details.
