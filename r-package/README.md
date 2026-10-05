# triplclust

R package providing an interface to the **TriplClust** algorithm for 3D point-cloud clustering.

## Build
From the repository root, build the source tarball with:

```sh
./triplclustLibR/makecrandist.sh
```

The tarball is written to `triplclustLibR/`.

## Installation
From the repository root, install the generated tarball with:

```r
install.packages("triplclustLibR/triplclust_1.0.1.tar.gz",
                 repos = NULL, type = "source")
```

## Verification
After building the C++ executable and installing the R package, run from the
repository root:

```sh
./triplclustLibR/verify.sh
```

To generate a gnuplot script and display the bundled data:

```sh
Rscript triplclustLibR/inst/scripts/use_triplclust.R data/attpc.dat -gnuplot | gnuplot --persist
```

## Basic usage
The function accepts an `n x 3` numeric matrix and returns a list of clusters.
Each list element contains the 1-based row indices of its points. A point may
occur in multiple clusters; points absent from every cluster are unassigned.

```r
library(triplclust)

data("attpc", package = "triplclust")
points <- as.matrix(attpc)
clusters <- triplclust(points)

lengths(clusters)  # number of points in each cluster
```