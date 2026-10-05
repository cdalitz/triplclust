#' Convert cluster assignments to CSV text
#'
#' @param points Numeric matrix with exactly three columns (x, y, z).
#' @param labels Either a list of integer vectors returned by `triplclust()` or
#'   a per-point integer vector.
#' @return A character string containing CSV-formatted coordinates and labels,
#'   with zero-based cluster IDs, `-1` for noise, and `-2` for overlaps.
#' @export
prepare_csv <- function(points, labels) {
  if (!is.matrix(points) | !is.numeric(points) | ncol(points) != 3L) {
    stop(
      "points must be a numeric matrix with exactly three columns",
      call. = FALSE
    )
  }

  export_data <- .prepare_export_data(points, labels)
  point_labels <- rep("-1", nrow(points))
  if (length(export_data$point_clusters) > 0L) {
    point_labels[as.integer(names(export_data$point_clusters))] <-
      vapply(export_data$point_clusters, function(cluster_ids) {
        paste(cluster_ids - 1L, collapse = ";")
      }, character(1))
  }
  point_labels[export_data$overlap_ids] <- "-2"

  rows <- paste(formatC(points[, 1], digits = 6, format = "f"),
    formatC(points[, 2], digits = 6, format = "f"),
    formatC(points[, 3], digits = 6, format = "f"),
    point_labels,
    sep = ","
  )

  paste(c(
    "# Comment: curveID -1 represents noise; -2 represents overlap",
    "# x, y, z, curveID",
    rows
  ), collapse = "\n")
}

#' Convert cluster assignments to gnuplot command text
#'
#' @param points Numeric matrix with exactly three columns (x, y, z).
#' @param labels Either a list of integer vectors returned by
#'   \code{triplclust()} or a per-point integer vector. In a per-point vector,
#'   `-1` marks noise and `-2` marks overlap.
#' @return A character string containing a gnuplot script.
#' @export
prepare_plot <- function(points, labels) {
  if (!is.matrix(points) | !is.numeric(points) | ncol(points) != 3L) {
    stop(
      "points must be a numeric matrix with exactly three columns",
      call. = FALSE
    )
  }

  export_data <- .prepare_export_data(points, labels)
  cluster_indices <- export_data$clusters
  overlap_ids <- export_data$overlap_ids
  non_clustered <- export_data$unassigned_ids
  cluster_indices <- lapply(cluster_indices, function(indices) {
    setdiff(indices, overlap_ids)
  })
  cluster_numbers <- which(lengths(cluster_indices) > 0L)
  cluster_indices <- cluster_indices[cluster_numbers]

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

.prepare_plot_labels <- function(points, labels) {
  if (is.list(labels)) {
    cluster_list <- lapply(labels, as.integer)
    if (length(cluster_list) == 0L) {
      return(list())
    }
    bad <- vapply(cluster_list, function(idx) {
      length(idx) > 0L && any(idx < 1L | idx > nrow(points))
    }, logical(1))
    if (any(bad)) {
      stop("cluster indices are out of range", call. = FALSE)
    }
    return(cluster_list)
  }

  if (is.numeric(labels) | is.integer(labels) | is.character(labels)) {
    labels <- as.integer(labels)
    if (length(labels) != nrow(points)) {
      stop("labels must have one value per point", call. = FALSE)
    }
    if (any(is.na(labels))) labels[is.na(labels)] <- 0L
    return(split(seq_len(nrow(points)), labels))
  }

  stop("labels must be a list of integer vectors or a per-point label vector",
       call. = FALSE)
}

.prepare_export_data <- function(points, labels) {
  clusters <- .prepare_plot_labels(points, labels)
  overlap_ids <- noise_ids <- integer(0)

  if (!is.list(labels) && any(as.integer(labels) < 0L, na.rm = TRUE)) {
    point_labels <- as.integer(labels)
    point_labels[is.na(point_labels)] <- 0L
    overlap_ids <- which(point_labels == -2L)
    noise_ids <- which(point_labels == -1L)
    assigned_ids <- which(point_labels >= 0L)
    clusters <- split(assigned_ids, point_labels[assigned_ids])
  }

  clusters <- lapply(clusters, unique)
  point_clusters <- split(
    rep.int(seq_along(clusters), lengths(clusters)),
    unlist(clusters, use.names = FALSE)
  )
  memberships <- sapply(point_clusters, length)
  overlap_ids <- sort(unique(c(
    overlap_ids,
    as.integer(names(memberships)[memberships > 1L])
  )))
  assigned_ids <- c(as.integer(names(point_clusters)), overlap_ids)

  list(
    clusters = clusters,
    point_clusters = point_clusters,
    overlap_ids = overlap_ids,
    unassigned_ids = union(
      noise_ids,
      setdiff(seq_len(nrow(points)), assigned_ids)
    )
  )
}

.plot_colour_hex <- function(cluster_index) {
  idx <- as.integer(cluster_index)
  red <- ((idx * 23L) %% 19L) / 18
  green <- ((idx * 23L) %% 7L) / 6
  blue <- ((idx * 23L) %% 3L) / 2
  tolower(grDevices::rgb(red, green, blue))
}
