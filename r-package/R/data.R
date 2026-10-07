#' AT-TPC Point Cloud
#'
#' Particle tracks from an active target time projection chamber (AT-TPC)
#' measurement at the AT-TPC at NSCL, Michigan State University.
#'
#' This is an example that works with the default settings of triplclust.
#'
#' @format A data frame with 121 rows and three numeric columns: `x`, `y`, and
#'   `z`.
#' @source AT-TPC at the NSCL, Michigan State University (courtesy of Yassid Ayyad)
#' @references Dalitz C., Ayyad Y., Wilberg J., Aymans L., Bazin D., Mittig W.: Automatic trajectory recognition in Active Target Time Projection Chambers data by means of hierarchical clustering. \emph{Computer Physics Communications} 235:159-168 (2019) \doi{10.1016/j.cpc.2018.09.010}
#' @examples
#' tc <- triplclust(attpc)
#' @docType data
#' @keywords datasets
#' @name attpc
NULL

#' LIDAR Point Cloud
#'
#' Airborne LIDAR scan of a high voltage pole with about seven dots per
#' square meter. The original data in this excerpt from "Geobasisdaten der
#' Kommunen und des Landes NRW 2014" had 5813 points, from which we have
#' randomly sampled 1000 points.
#'
#' This is an example with a lower point density along the curves, so that
#' the triplcust parameter k should be set to k=12. Moreover, the automatic
#' threshold detection does not work in this case and it should be manually
#' set to t=12.
#'
#' @format A data frame with 1000 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @source Geobasisdaten der Kommunen und des Landes NRW (2014), courtesy of
#'   Katasteramt Krefeld.
#' @references Dalitz C., Wilberg J., Aymans L. (2019) TriplClust: An
#' Algorithm for Curve Detection in 3D Point Clouds. \emph{Image Processing On Line} 9:26-46, \doi{10.5201/ipol.2019.234}
#' @examples
#' tc <- triplclust(lidar, k=12, t=12)
#' @docType data
#' @keywords datasets
#' @name lidar
NULL

#' Tennis Ball Point Cloud
#'
#' Candidate points for tennis ball tracking extracted by by Vito Renò et al.
#' from video recordings of a tennis match. The resulting data contained
#' candidate points for tracking the tennis ball. From this data we have
#' selected a contiguous sequence of 701 points.
#'
#' This is an example for a low point density so that the triplcust parameter
#' k should be set to k=12.
#' 
#' @format A data frame with 701 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @source courtesy of V. Renò
#' @references Renò V., Mosca N., Nitti M., Guaragnella C., D’Orazio T., Stella E. (2016) Real-time tracking of a tennis ball by combining 3D data and domain knowledge. \emph{International Conference on Technology and Innovation in Sports, Health and Wellbeing (TISHW)} 2016:1-7 \doi{10.1109/TISHW.2016.7847774}
#' @examples
#' tc <- triplclust(tennis, k=12)
#' @docType data
#' @keywords datasets
#' @name tennis
NULL

#' Synthetic Clean Point Cloud
#'
#' Synthetically generated point cloud with four curves and no added noise.
#'
#' In this example, triplclust merges two curves because they meet tangential.
#' 
#' @format A data frame with 225 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @references Dalitz C., Wilberg J., Aymans L. (2019) TriplClust: An
#' Algorithm for Curve Detection in 3D Point Clouds. \emph{Image Processing On Line} 9:26-46, \doi{10.5201/ipol.2019.234}
#' @examples
#' tc <- triplclust(synthetic_clean)
#' @docType data
#' @keywords datasets
#' @name synthetic_clean
NULL

#' Synthetic Noisy Point Cloud
#'
#' Synthetically generated point cloud with four curves and added noise.
#'
#' In this example, the noise confuses the default smoothing of triplclust
#' and the smoothing radius r should be set to r=1.
#' 
#' @format A data frame with 325 rows and three numeric columns: `x`, `y`,
#'   and `z`.
#' @references Dalitz C., Wilberg J., Aymans L. (2019) TriplClust: An
#' Algorithm for Curve Detection in 3D Point Clouds. \emph{Image Processing On Line} 9:26-46, \doi{10.5201/ipol.2019.234}
#' @examples
#' tc <- triplclust(synthetic_noise, r=1)
#' @docType data
#' @keywords datasets
#' @name synthetic_noise
NULL
