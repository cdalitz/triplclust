#' Cluster a 3D point cloud
#'
#' @param points Numeric matrix with exactly three columns (x, y, z).
#' @param r Smoothing radius. Accepts a finite numeric scalar, or a string such
#'   as `"2"`, `"2dNN"`, or `"0.5dNN"`; defaults to `2dNN`.
#' @param k Number of nearest neighbors used to generate candidate triplets.
#' @param n Minimum number of neighbors used by triplet generation.
#' @param a Collinearity tolerance in `(0, pi)`.
#' @param s Distance scale for hierarchical clustering. Accepts a finite
#'   numeric scalar, or a string such as `"0.33"`, `"0.33dNN"`, or
#'   `"1.5dNN"`; defaults to `0.33dNN`.
#' @param t Clustering threshold. Accepts a finite non-negative numeric value,
#'   or the case-insensitive strings `"auto"` / `"automatic"` to let the
#'   algorithm choose it.
#' @param dmax Optional maximum gap for splitting clusters. Accepts a finite
#'   numeric scalar, a dNN-scaled string such as `"2.5dNN"` (case-insensitive
#'   suffix), or `"none"`.
#' @param linkage Linkage method: `"single"`, `"complete"`, or `"average"`.
#' @param m Minimum cluster size retained by pruning.
#' @param verbose Verbosity level for diagnostic output.
#' @param ordered Logical flag. If `TRUE`, treat the input as an ordered point
#'   sequence, matching the CLI `-ordered` option.
#' @return A list with one integer vector per input point. Each vector contains
#'   the integer IDs of the clusters containing that point; unassigned points
#'   have an empty vector.
#' @examples
#' \dontrun{
#' data("attpc", package = "triplclust")
#' points <- as.matrix(attpc)
#' clusters <- triplclust(points)
#'
#' cluster_id <- vapply(clusters, function(ids) {
#'   if (length(ids) == 0L) NA_integer_ else ids[[1L]]
#' }, integer(1))
#' point_colors <- rep("grey70", nrow(points))
#' assigned <- !is.na(cluster_id)
#' palette <- grDevices::rainbow(max(1L, unlist(clusters)))
#' point_colors[assigned] <- palette[cluster_id[assigned]]
#'
#' rgl::open3d()
#' rgl::plot3d(points, col = point_colors, size = 5,
#'             xlab = "x", ylab = "y", zlab = "z")
#' }
#' @references Dalitz C., Wilberg J., Aymans L. (2019) TriplClust: An
#' Algorithm for Curve Detection in 3D Point Clouds. \emph{Image Processing On Line} 9:26-46, \doi{10.5201/ipol.2019.234}
#' @export
triplclust <- function(points, r = NULL, k = 19L, n = 2L, a = 0.03,
                       s = NULL, t = "auto", dmax = NULL,
                       linkage = "single", m = 5L, verbose = 0L,
                       ordered = FALSE) {
  if (is.data.frame(points)) {
    if (ncol(points) > 3) {
      if (sum(c("x","y","z") %in% names(points)) == 3) {
        points <- as.matrix(cbind(points$x, points$y, points$z))
      } else {
        points <- as.matrix(points[,1:3])
      }
    } else {
      points <- as.matrix(points)
    }
  }
  if (!is.matrix(points)) {
    stop("points must be a matrix", call. = FALSE)
  }
  if (!is.numeric(points)) {
    stop("points must be numeric", call. = FALSE)
  }
  if (ncol(points) < 2) {
    stop("points must have at least two columns", call. = FALSE)
  }
  if (ncol(points) == 2) {
    # add dummy column
    points <- cbind(points, rep(0, NROW(points)))
  }
  if (ncol(points) > 3) {
    if (sum(c("x","y","z") %in% names(points)) == 3) {
      points <- as.matrix(cbind(points$x, points$y, points$z))
    } else {
      points <- as.matrix(points[,1:3])
    }
  }
  if (nrow(points) < 3) {
    stop("points must contain at least three rows", call. = FALSE)
  }
  if (anyNA(points)) {
    stop("points must not contain missing values", call. = FALSE)
  }
  if (any(!is.finite(points))) {
    stop("points must contain only finite values", call. = FALSE)
  }
  storage.mode(points) <- "double"

  r <- .parse_distance(r, "r", 2, default_dnn = TRUE)
  s <- .parse_distance(s, "s", 0.33, default_dnn = TRUE)
  dmax <- .parse_distance(dmax, "dmax", 0, allow_none = TRUE)
  if (r$value <= 0) stop("r must be positive", call. = FALSE)
  if (s$value <= 0) stop("s must be positive", call. = FALSE)
  if (dmax$enabled && dmax$value < 0) {
    stop("dmax must be non-negative", call. = FALSE)
  }

  k <- .parse_integer(k, "k", 1)
  n <- .parse_integer(n, "n", 1)
  m <- .parse_integer(m, "m", 1)
  verbose <- .parse_integer(verbose, "verbose", 0)
  if (n > k) stop("n cannot be larger than k", call. = FALSE)
  a <- .parse_number(a, "a")
  if (a <= 0) stop("a must be greater than 0", call. = FALSE)
  if (a >= pi) stop("a must be less than pi", call. = FALSE)

  automatic <- is.character(t) && length(t) == 1L && !is.na(t) &&
    tolower(t) %in% c("auto", "automatic")
  if (automatic) {
    t <- 0
  } else {
    t <- .parse_number(t, "t")
    if (t < 0) stop("t must be non-negative", call. = FALSE)
  }
  if (!is.logical(ordered)) {
    stop("ordered must be TRUE or FALSE", call. = FALSE)
  }
  if (length(ordered) != 1L) {
    stop("ordered must be a single TRUE or FALSE value", call. = FALSE)
  }
  if (is.na(ordered)) {
    stop("ordered must not be missing", call. = FALSE)
  }

  valid_linkage <- is.character(linkage) && length(linkage) == 1L &&
    is.null(dim(linkage)) && !is.na(linkage) &&
    linkage %in% c("single", "complete", "average")
  if (!valid_linkage) {
    stop("linkage must be 'single', 'complete', or 'average'", call. = FALSE)
  }

  triplclust_rcpp( # nolint: object_usage_linter
    points, r$value, r$dnn, k, n, a,
    s$value, s$dnn, t, automatic,
    dmax$value, dmax$dnn, dmax$enabled,
    linkage, m, verbose, ordered
  )
}

.parse_distance <- function(value, name, default, default_dnn = FALSE,
                            allow_none = FALSE) {
  if (is.null(value)) {
    return(list(value = default, dnn = default_dnn,
                enabled = !allow_none))
  }
  if (is.numeric(value) && length(value) == 1L && is.null(dim(value))) {
    value <- as.double(value)
    if (!is.finite(value)) stop(name, " must be finite", call. = FALSE)
    return(list(value = value, dnn = FALSE, enabled = TRUE))
  }
  if (!is.character(value)) {
    stop(name, " must be a number or dNN string", call. = FALSE)
  }
  if (length(value) != 1L) {
    stop(name, " must be a single number or dNN string", call. = FALSE)
  }
  if (is.na(value)) {
    stop(name, " must not be missing", call. = FALSE)
  }

  value <- trimws(value)
  value_lower <- tolower(value)
  if (allow_none && identical(value_lower, "none")) {
    return(list(value = 0, dnn = FALSE, enabled = FALSE))
  }

  dnn <- grepl("dnn$", value_lower)
  if (dnn) value <- substr(value, 1L, nchar(value) - 3L)
  value <- suppressWarnings(as.numeric(value))
  if (length(value) != 1L) {
    stop(name, " must be a single finite number, optionally followed by dNN",
         call. = FALSE)
  }
  if (is.na(value)) {
    stop(name, " must contain a valid number", call. = FALSE)
  }
  if (!is.finite(value)) {
    stop(name, " must be finite", call. = FALSE)
  }
  list(value = value, dnn = dnn, enabled = TRUE)
}

.parse_integer <- function(value, name, minimum) {
  valid <- is.numeric(value) && length(value) == 1L &&
    is.null(dim(value)) && is.finite(value) && value == floor(value) &&
    value >= minimum && value <= .Machine$integer.max
  if (!valid) {
    stop(name, " must be an integer >= ", minimum, call. = FALSE)
  }
  as.integer(value)
}

.parse_number <- function(value, name) {
  valid <- is.numeric(value) && length(value) == 1L &&
    is.null(dim(value)) && is.finite(value)
  if (!valid) {
    stop(name, " must be a finite scalar number", call. = FALSE)
  }
  as.double(value)
}
