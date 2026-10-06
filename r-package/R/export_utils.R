#' Convert cluster assignments to gnuplot script text
#'
#' @param points Numeric matrix with exactly three columns (x, y, z).
#' @param labels Either the point-indexed list returned by
#'   \code{triplclust()} or a per-point integer vector. In a per-point vector,
#'   `-1` marks noise and non-negative values are zero-based cluster IDs.
#' @return A character string containing a gnuplot script. Points with multiple
#'   cluster memberships are shown in a separate overlap series.
#' @export
prepare_plot <- function(points, labels) {
  if (!is.matrix(points)) {
    stop("points must be a matrix", call. = FALSE)
  }
  if (!is.numeric(points)) {
    stop("points must be numeric", call. = FALSE)
  }
  if (ncol(points) != 3L) {
    stop("points must have exactly three columns", call. = FALSE)
  }

  export_data <- .prepare_export_data(points, labels)
  cluster_indices <- lapply(export_data$clusters, function(indices) {
    setdiff(indices, export_data$overlap_ids)
  })
  overlap_ids <- export_data$overlap_ids
  non_clustered <- export_data$unassigned_ids
  keep_clusters <- lengths(cluster_indices) > 0L
  cluster_numbers <- export_data$cluster_ids[keep_clusters]
  cluster_indices <- cluster_indices[keep_clusters]

  axis_names <- c("x", "y", "z")
  axis_min <- apply(points, 2, min)
  axis_max <- apply(points, 2, max)
  axis_lower <- ifelse(axis_max > axis_min, axis_min, axis_min - 1)
  axis_upper <- ifelse(axis_max > axis_min, axis_max, axis_max + 1)
  ranges <- paste0(
    "set ", axis_names, "range [",
    formatC(axis_lower, digits = 8, format = "f"), ":",
    formatC(axis_upper, digits = 8, format = "f"), "]"
  )

  formatted_points <- paste(formatC(points[, 1], digits = 8, format = "f"),
    formatC(points[, 2], digits = 8, format = "f"),
    formatC(points[, 3], digits = 8, format = "f"),
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

  cluster_colours <- .plot_colour_hex(cluster_numbers)
  cluster_series <- if (length(cluster_numbers) > 0L) {
    paste0(
      "'-' with points lc '", cluster_colours,
      "' title 'curve ", cluster_numbers - 1L, "'"
    )
  } else {
    character(0)
  }
  cluster_blocks <- unlist(
    lapply(cluster_indices, points_block),
    use.names = FALSE
  )

  overlap_series <- if (length(overlap_ids) > 0L) {
    "'-' with points lc 'black' title 'overlap'"
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
  paste(script, collapse = "\n")
}

#' Convert cluster assignments to CSV text
#'
#' @param points Numeric matrix with exactly three columns (x, y, z).
#' @param labels Either the point-indexed list returned by `triplclust()` or a
#'   per-point integer vector (`-1` for noise, non-negative IDs for clusters).
#' @return A character string containing CSV-formatted coordinates and labels,
#'   with zero-based cluster IDs, `-1` for noise, and semicolon-separated IDs
#'   when a point belongs to multiple clusters.
#' @export
prepare_csv <- function(points, labels) {
  if (!is.matrix(points)) {
    stop("points must be a matrix", call. = FALSE)
  }
  if (!is.numeric(points)) {
    stop("points must be numeric", call. = FALSE)
  }
  if (ncol(points) != 3L) {
    stop("points must have exactly three columns", call. = FALSE)
  }

  export_data <- .prepare_export_data(points, labels)
  point_labels <- rep("-1", nrow(points))
  for (point in seq_len(nrow(points))) {
    cluster_ids <- export_data$point_clusters[[point]]
    if (length(cluster_ids) > 0L) {
      point_labels[[point]] <- paste(cluster_ids - 1L, collapse = ";")
    }
  }
  rows <- paste(formatC(points[, 1], digits = 6, format = "f"),
    formatC(points[, 2], digits = 6, format = "f"),
    formatC(points[, 3], digits = 6, format = "f"),
    point_labels,
    sep = ","
  )

  paste(c(
    "# Comment: curveID -1 represents noise; multiple IDs are separated by semicolons",
    "# x, y, z, curveID",
    rows
  ), collapse = "\n")
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

.plot_colour_hex <- function(cluster_index) {
  idx <- as.integer(cluster_index)
  red <- ((idx * 23L) %% 19L) / 18
  green <- ((idx * 23L) %% 7L) / 6
  blue <- ((idx * 23L) %% 3L) / 2
  tolower(grDevices::rgb(red, green, blue))
}
