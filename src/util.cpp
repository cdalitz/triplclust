//
// util.cpp
//     Utility functions needed here and there.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2019-04-02
// License: see ../LICENSE
//

#include "util.h"
#include <sstream>
#include <stdexcept>

//-------------------------------------------------------------------
// converts *str* to double.
// If str is not a number a invalid_argument exception is thrown.
//-------------------------------------------------------------------
double stod(const char* s) {
  double result;

  // remove leading and trailing white space
  std::string str(s);
  str.erase(0, str.find_first_not_of("\t\r\n "));
  str.erase(str.find_last_not_of("\t\r\n ") + 1);

  // use istream for conversion
  std::istringstream iss(str);

  iss >> std::ws >> result;
  if (iss.fail() || !iss.eof()) {
    throw std::invalid_argument("not a number");
  }
  return result;
}
