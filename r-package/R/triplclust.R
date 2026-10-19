#' Cluster a 3D point cloud with the TriplClust algorithm.
#'
#' @param points Numeric matrix or data.frame with the point coordinates,
#'   one point per line. If it has more than three columns, the columns
#'   named `"x"`, `"y"`, and `"z"` are used or, if there are no columns
#'   with these names, the first three are used. If it only has two columns,
#'   a third column is padded with zeros.
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
#' @param ordered Logical flag. If `TRUE`, treat the input as an ordered point
#'   sequence, matching the CLI `-ordered` option.
#' @return The returned value is an object of the class \code{"triplclust"}
#' containing the following components:
#' \itemize{
#'  \item points: Matrix of input points.
#'  \item labels: List with the assigned cluster labels encoded as one
#'    integer vector per input point. Each vector contains the IDs of the
#'    clusters containing that point. Unassigned points have an empty vector.
#'  \item params: Named list with the parameters of the run. A parameter given
#'    as a multiple of dNN has two entries: the plain name holds the requested
#'    value and its `_dnn` flag states that it is a multiple of dNN (for
#'    example `r` is `2` and `r_dnn` is `TRUE` for `"2dNN"`), while the
#'    `_real` suffix holds the absolute value that was actually used. The
#'    `_real` entries are resolved during the run and are only reported for
#'    debugging and reproducibility; the algorithm does not use them.
#'    \describe{
#'      \item{`r`}{smoothing radius as requested}
#'      \item{`r_dnn`}{`TRUE` if `r` was given as a multiple of dNN}
#'      \item{`r_real`}{absolute smoothing radius used}
#'      \item{`k`}{neighbours considered for triplet generation}
#'      \item{`n`}{best triplets kept per midpoint}
#'      \item{`a`}{max angle between the triplet branches}
#'      \item{`s`}{distance scale as requested}
#'      \item{`s_dnn`}{`TRUE` if `s` was given as a multiple of dNN}
#'      \item{`s_real`}{absolute distance scale used}
#'      \item{`t`}{cluster distance threshold as requested}
#'      \item{`tauto`}{`TRUE` if `t` was determined automatically}
#'      \item{`t_real`}{the absolute cluster distance threshold that was
#'        actually applied, i.e. the automatically determined one when `tauto`
#'        is `TRUE` and otherwise simply `t`; `NaN` if no triplets were
#'        generated, so that no threshold was applied at all}
#'      \item{`dmax`}{max gap within a triplet as requested}
#'      \item{`dmax_dnn`}{`TRUE` if `dmax` was given as a multiple of dNN}
#'      \item{`is_dmax`}{`TRUE` if splitting of clusters at gaps is enabled}
#'      \item{`dmax_real`}{absolute max gap used}
#'      \item{`dnn_real`}{the absolute characteristic length that was used for
#'        scaling, `0` if no parameter depended on dNN}
#'      \item{`linkage`}{linkage method}
#'      \item{`m`}{minimum cluster size}
#'      \item{`verbose`}{verbosity level, always `0` from the R interface}
#'    }
#'    Note that `k`, `n`, `a`, `linkage` and `m` are never rescaled and
#'    therefore have no `_real` counterpart.
#'  \item cdist: Numeric vector with the distance of each of the n-1 merge
#'    steps of the triplet clustering, where n is the number of generated
#'    triplets. The values are the heights of the dendrogram, in ascending
#'    order. It is empty if no triplets were generated.
#' }
#' @examples
#' tc <- triplclust(attpc)
#' # points not assigned to any cluster (noise)
#' noise <- tc$points[sapply(tc$labels, function(x) length(x)==0),]
#' # points assigned to more than one cluster (intersections)
#' intersections <- tc$points[sapply(tc$labels, function(x) length(x)>1),]
#' # the parameters that were actually used, with dNN resolved
#' tc$params
#' # requested values and the values that were actually used
#' tc$params[c("r", "r_dnn", "r_real")]
#' tc$params[c("t", "tauto", "t_real")]
#' # dendrogram heights of the triplet clustering
#' tc$cdist
#' @references Dalitz C., Wilberg J., Aymans L. (2019) TriplClust: An
#'   Algorithm for Curve Detection in 3D Point Clouds.
#'   \emph{Image Processing On Line} 9:26-46,
#'   \doi{10.5201/ipol.2019.234}
#' @seealso label_colors plot_threshold save_csv save_gnuplot
#' @export
triplclust <- function(points, r = NULL, k = 19L, n = 2L, a = 0.03,
                       s = NULL, t = "auto", dmax = NULL,
                       linkage = "single", m = 5L,
                       ordered = FALSE) {
  if (!is.matrix(points) && !is.data.frame(points)) {
    stop("points must be a matrix or data.frame", call. = FALSE)
  }
  if (ncol(points) > 3) {
    if (sum(c("x", "y", "z") %in% names(points)) == 3) {
      points <- as.matrix(cbind(points$x, points$y, points$z))
    } else {
      points <- as.matrix(points[, 1:3])
    }
  }
  if (is.data.frame(points)) {
    points <- as.matrix(points)
  }
  if (!is.numeric(points)) {
    stop("points must be numeric", call. = FALSE)
  }
  if (ncol(points) < 2) {
    stop("points must have at least two columns", call. = FALSE)
  }
  if (ncol(points) == 2) {
    # add dummy column
    points <- as.matrix(cbind(points, rep(0, NROW(points))))
  }
  if (sum(c("x", "y", "z") %in% names(points)) != 3) {
    colnames(points) <- c("x", "y", "z")
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

  result <- triplclust_rcpp( # nolint: object_usage_linter
    points, r$value, r$dnn, k, n, a,
    s$value, s$dnn, t, automatic,
    dmax$value, dmax$dnn, dmax$enabled,
    linkage, m, ordered
  )
  params <- result$params
  class(params) <- c("triplclust_params", "list")
  structure(
    list(points = points, labels = result$labels, params = params,
         cdist = result$cdist),
    class = "triplclust"
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
