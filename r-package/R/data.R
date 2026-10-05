#' AT-TPC Point Cloud
#'
#' Test point cloud collected using the AT-TPC at NSCL, Michigan State
#' University.
#'
#' @format A data frame with 121 rows and three numeric columns: `x`, `y`, and
#'   `z`.
#' @source AT-TPC at the NSCL, Michigan State University
#' (courtesy of Yassid Ayyad)
#' @docType data
#' @keywords datasets
#' @name attpc
NULL

#' LIDAR Point Cloud
#'
#' LIDAR point cloud from municipal and state geodata for North Rhine-Westphalia.
#' The third column contains height values.
#'
#' @format A data frame with 1000 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @source Geobasisdaten der Kommunen und des Landes NRW (2014), courtesy of
#'   Katasteramt Krefeld.
#' @docType data
#' @keywords datasets
#' @name lidar
NULL

#' Radar Point Cloud
#'
#' Radar point cloud provided by Christoph Degen. The third column contains
#' time values.
#'
#' @format A data frame with 747 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @source Christoph Degen.
#' @docType data
#' @keywords datasets
#' @name radar
NULL

#' Tennis Point Cloud
#'
#' Tennis point cloud used for real-time ball tracking.
#'
#' @format A data frame with 701 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @source V. Renò et al., “Real-time tracking of a tennis ball by combining 3D
#'   data and domain knowledge,” TISHW (2016), pp. 1-7; data courtesy of
#'   Vito Renò.
#' @docType data
#' @keywords datasets
#' @name tennis
NULL

#' Synthetic Clean Point Cloud
#'
#' Synthetically generated point cloud with four curves and no added noise.
#'
#' @format A data frame with 225 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @source Christoph Dalitz.
#' @docType data
#' @keywords datasets
#' @name synthetic_clean
NULL

#' Synthetic Noisy Point Cloud
#'
#' Synthetically generated point cloud with four curves and added noise.
#'
#' @format A data frame with 325 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @source Christoph Dalitz.
#' @docType data
#' @keywords datasets
#' @name synthetic_noise
NULL
