//
// util.h
//     Utility functions needed here and there.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#ifndef UTIL_H
#define UTIL_H

enum Linkage { SINGLE, COMPLETE, AVERAGE };

// converts *str* to double.
double stod(const char* str);

#endif
