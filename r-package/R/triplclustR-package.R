#' @title triplclustR – R interface to the TriplClust algorithm
#' @description
#' An implementation of the the TriplClust algorithm for detecting
#' and separating curves in 3D point clouds.
#' @name triplclustR
#' @docType package
#' @keywords internal
#' @useDynLib triplclustR, .registration = TRUE
#' @importFrom Rcpp evalCpp
"_PACKAGE"

utils::globalVariables("_triplclustR_triplclust_rcpp")
