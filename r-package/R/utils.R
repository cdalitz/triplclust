#' Convert labels to colors for visualizations.
#'
#' @param labels List containing the labels as returned by \code{triplclust()}.
#'   Alternatively a `triplclust` object can be provided as returned by
#'   \code{triplclust()}.
#'
#' For noise points (no cluster label), red is returned, for instersection
#' points (more than one cluster label), black is returned. All other cluster
#' labels are colored with a unique color generated with modulo arithmetic.
#' @return Character vector of the same length as `labels` with RGB colors.
#' @examples
#' tc <- triplclust(attpc)
#' colors <- label_colors(tc)
#' 
#' \dontrun{# visualization with rgl
#' library(rgl)
#' plot3d(tc$points, col=colors)
#' 
#' # visualization with scatterplot3d
#' library(scatterplot3d)
#' scatterplot3d(tc$points, color=colors)
#' 
#' # visualization with plot3D
#' library(plot3D)
#' points3D(tc$points[,1], tc$points[,2], tc$points[,3],
#'          colvar=1:length(colors), col=colors, colkey=F)
#' }
#' @seealso triplclust
#' @export
label_colors <- function(labels) {
  if ("triplclust" %in% class(labels)) {
    labels <- labels$labels
  }
  noise <- sapply(labels, function(x) length(x)==0)
  intersec <- sapply(labels, function(x) length(x)>1)
  other <- !(noise | intersec)
  cols <- character(length(labels))
  cols[noise] <- grDevices::rgb(1,0,0)
  cols[intersec] <- grDevices::rgb(0,0,0)
  cols[other] <- .plot_color_hex(unlist(labels[other]))
  return(cols)
}

#' Save triplclust result to gnuplot command file for plotting.
#'
#' Noise is plotted in red, points with multiple cluster memberships
#' are plotted as a separate overlap series. The plot can be shown
#' from the command line with `gunplot -persist`.
#' @param tc An object of class `triplclust` as returned by \code{triplclust()}.
#' @param file File name.
#' @seealso triplclust
#' @export
save_gnuplot <- function(tc, file) {
  if (!("triplclust" %in% class(tc))) {
    stop("tc must be of class 'triplclust'", call. = FALSE)
  }

  export_data <- .prepare_export_data(tc$points, tc$labels)
  cluster_indices <- lapply(export_data$clusters, function(indices) {
    setdiff(indices, export_data$overlap_ids)
  })
  overlap_ids <- export_data$overlap_ids
  non_clustered <- export_data$unassigned_ids
  keep_clusters <- lengths(cluster_indices) > 0L
  cluster_numbers <- export_data$cluster_ids[keep_clusters]
  cluster_indices <- cluster_indices[keep_clusters]

  axis_names <- c("x", "y", "z")
  axis_min <- apply(tc$points, 2, min)
  axis_max <- apply(tc$points, 2, max)
  axis_lower <- ifelse(axis_max > axis_min, axis_min, axis_min - 1)
  axis_upper <- ifelse(axis_max > axis_min, axis_max, axis_max + 1)
  ranges <- paste0(
    "set ", axis_names, "range [",
    formatC(axis_lower, digits = 8, format = "f"), ":",
    formatC(axis_upper, digits = 8, format = "f"), "]"
  )

  formatted_points <- paste(formatC(tc$points[, 1], digits = 8, format = "f"),
                            formatC(tc$points[, 2], digits = 8, format = "f"),
                            formatC(tc$points[, 3], digits = 8, format = "f"),
                            sep = " "
                            )
  points_block <- function(indices) {
    c(paste(formatted_points[indices], collapse = "\n"), "e")
  }

  noise_series <- if (length(non_clustered) > 0L) {
    "'-' with points lc 'red' title 'noise'"
  } else {
    character(0)
  }
  noise_blocks <- if (length(non_clustered) > 0L) {
    points_block(non_clustered)
  } else {
    character(0)
  }

  cluster_colors <- .plot_color_hex(cluster_numbers)
  cluster_series <- if (length(cluster_numbers) > 0L) {
    paste0(
      "'-' with points lc '", cluster_colors,
      "' title 'curve ", cluster_numbers, "'"
    )
  } else {
    character(0)
  }
  cluster_blocks <- unlist(
    lapply(cluster_indices, points_block),
    use.names = FALSE
  )

  overlap_series <- if (length(overlap_ids) > 0L) {
    "'-' with points lc 'black' title 'overlaps'"
  } else {
    character(0)
  }
  overlap_blocks <- if (length(overlap_ids) > 0L) {
    points_block(overlap_ids)
  } else {
    character(0)
  }

  series <- c(noise_series, cluster_series, overlap_series)
  blocks <- c(noise_blocks, cluster_blocks, overlap_blocks)

  script <- c(
    ranges,
    paste0("splot ", paste(series, collapse = ", ")),
    blocks,
    "pause mouse keypress"
  )
  f <- file(file)
  writeLines(script, f)
  close(f)
}

#' Save triplclust result as a CSV file.
#'
#' The CSV is comma separated with four columns, the x, y, z coordinates,
#' and the cluster label. For noise, the special label `-1` is used
#' and, for points belonging to multiple clusters, call clauster labels
#' are given as a semicolon-separated list.
#'
#' @param tc An object of class `triplclust` as returned by \code{triplclust()}.
#' @param file File name.
#' @seealso triplcust
#' @export
save_csv <- function(tc, file) {
  if (!("triplclust" %in% class(tc))) {
    stop("tc must be of class 'triplclust'", call. = FALSE)
  }

  export_data <- .prepare_export_data(tc$points, tc$labels)
  point_labels <- rep("-1", nrow(tc$points))
  for (point in seq_len(nrow(tc$points))) {
    cluster_ids <- export_data$point_clusters[[point]]
    if (length(cluster_ids) > 0L) {
      point_labels[[point]] <- paste(cluster_ids, collapse = ";")
    }
  }
  rows <- paste(formatC(tc$points[, 1], digits = 6, format = "f"),
    formatC(tc$points[, 2], digits = 6, format = "f"),
    formatC(tc$points[, 3], digits = 6, format = "f"),
    point_labels,
    sep = ","
  )

  f <- file(file)
  writeLines(c(
    "# Comment: curveID -1 represents noise; multiple IDs are separated by semicolons",
    "# x, y, z, curveID", rows),
    f)
  close(f)
}


.prepare_export_data <- function(points, labels) {
  if (is.list(labels)) {
    if (length(labels) != nrow(points)) {
      stop("labels must have one element per point", call. = FALSE)
    }
    point_clusters <- lapply(labels, function(ids) {
      if (!is.numeric(ids)) {
        stop("each point's cluster IDs must be numeric", call. = FALSE)
      }
      if (anyNA(ids)) {
        stop("each point's cluster IDs must not contain missing values",
             call. = FALSE)
      }
      if (any(!is.finite(ids))) {
        stop("each point's cluster IDs must be finite", call. = FALSE)
      }
      if (any(ids != floor(ids))) {
        stop("each point's cluster IDs must be integers", call. = FALSE)
      }
      if (any(ids < 1L)) {
        stop("each point's cluster IDs must be positive", call. = FALSE)
      }
      unique(as.integer(ids))
    })
  } else {
    if (!is.numeric(labels)) {
      if (!is.character(labels)) {
        stop("labels must be a point-indexed list or a per-point label vector",
             call. = FALSE)
      }
    }
    point_labels <- suppressWarnings(as.integer(labels))
    if (length(point_labels) != nrow(points)) {
      stop("labels must have one value per point", call. = FALSE)
    }
    if (anyNA(point_labels)) {
      stop("labels must contain valid integer cluster labels",
           call. = FALSE)
    }
    if (any(point_labels < -1L)) {
      stop("labels must be -1 for noise or non-negative cluster IDs",
           call. = FALSE)
    }
    point_clusters <- lapply(point_labels, function(id) {
      if (id == -1L) integer(0) else id + 1L
    })
  }

  cluster_ids <- sort(unique(unlist(point_clusters, use.names = FALSE)))
  clusters <- lapply(cluster_ids, function(cluster_id) {
    which(vapply(point_clusters, function(ids) cluster_id %in% ids,
                 logical(1)))
  })
  names(clusters) <- as.character(cluster_ids)
  overlap_ids <- which(lengths(point_clusters) > 1L)
  unassigned_ids <- which(lengths(point_clusters) == 0L)

  list(
    clusters = clusters,
    point_clusters = point_clusters,
    cluster_ids = cluster_ids,
    overlap_ids = overlap_ids,
    unassigned_ids = unassigned_ids
  )
}

.plot_color_hex <- function(cluster_index) {
  idx <- as.integer(cluster_index)
  red <- ((idx * 23L) %% 19L) / 18
  green <- ((idx * 23L) %% 7L) / 6
  blue <- ((idx * 23L) %% 3L) / 2
  tolower(grDevices::rgb(red, green, blue))
}
