//
// dnn.h
//     Functions for computing the characteristic length dnn
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#ifndef DNN_H
#define DNN_H
#include "pointcloud.h"

// compute first quartile of the mean squared distance from the points
double first_quartile(const PointCloud &cloud);

#endif
