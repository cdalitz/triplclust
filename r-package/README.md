# triplclust

R package providing an interface to the **TriplClust** algorithm for 3D point-cloud clustering.

## Build & Install
From the sudirectory `r-package` of the repository root, build the source tarball with:

```bash makecrandist.sh
```

This creates the source tarball `triplclust_*.*.*.tar.gz` that is installed with

```R CMD INSTALL triplclust_*.*.*.tar.gz
```

## Basic usage
The main function is `triplclust` that accepts an `n x 3` numeric matrix and returns
a triplclust object with two entires: the input points (`points`) and teh assigned
clusters per point as a list `labels` of the same length as `nrows(points)`.
A point may occur in multiple clusters; points absent from every cluster have an empty
numeric label vector.

Here is an example how to apply triplclust to the included dataset `attpc`:

```r
library(triplclust)
tc <- triplclust(attpc)

# visualization with rgl
colors <- label_colors(tc)
library(rgl)
plot3d(tc$points, col=colors)

# visualization with external software gnuplot
save_gnuplot(tc, "/tmp/tc.gnuplot")
system("gnuplot -persist /tmp/tc.gnuplot")

# save result as CSV file
save.csv(tc, "result.csv")
```