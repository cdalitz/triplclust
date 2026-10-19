# triplclust

R package providing an interface to the **TriplClust** algorithm for 3D point-cloud clustering.

## Build & Install
From the subdirectory `r-package` of the repository root, build the source tarball with:

```bash
./makecrandist.sh
```

This creates the source tarball `triplclust_*.*.*.tar.gz` that is installed with

```bash
R CMD INSTALL triplclust_*.*.*.tar.gz
```

## Basic usage
The main function is `triplclust` that accepts an `n x 3` numeric matrix and
returns a triplclust object with four entries: the input points (`points`), the
assigned clusters per point as a list `labels` of the same length as
`nrow(points)`, the parameters of the run (`params`) and the merge distances of
the triplet clustering (`cdist`). A point may occur in multiple clusters; points
absent from every cluster have an empty numeric label vector.

`params` is a named list of the parameters of the run. A parameter given as a
multiple of dNN appears twice: under its plain name is the requested value and
its `_dnn` flag is `TRUE` (for example `r` is `2` and `r_dnn` is `TRUE` for
`"2dNN"`), while the same name with the suffix `_real` holds the absolute value
that was actually used. So `params$r_real` is the resulting smoothing radius and
`params$dnn_real` the characteristic length it was derived from. The `_real`
entries are resolved during the run and are only reported for debugging and
reproducibility. The meaning of every entry is documented in `?triplclust`.

`cdist` is a numeric vector with the distance of each of the n-1 merge steps of
the triplet clustering, i.e. the heights of the dendrogram in ascending order.
These are the same values the command line version writes to `debug_cdist.csv`
with `-vv`; `plot_threshold(tc)` returns that file content as a string, without
writing it.

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
save_csv(tc, "result.csv")

# content of the cdist threshold trace, as the command line version writes
# it to debug_cdist.csv with -vv. Nothing is written to disk here.
writeLines(plot_threshold(tc), "debug_cdist.csv")
```