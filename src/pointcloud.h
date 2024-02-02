//
// pointcloud.h
//     Classes and functions for 3D points and clouds thereof.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2024-02-02
// License: see ../LICENSE
//

#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include <cstddef>
#include <fstream>
#include <iostream>
#include <ostream>
#include <set>
#include <vector>

// 3D point class.
class Point {
 public:
  double x;
  double y;
  double z;
  std::set<size_t> cluster_ids;
  size_t index;    // only used for chronological order

  Point(){};
  Point(const std::vector<double>& point);
  Point(const std::vector<double>& point, const std::set<size_t>& cluster_ids);
  Point(double x, double y, double z);
  Point(double x, double y, double z, const std::set<size_t>& cluster_ids);
  Point(double x, double y, double z, size_t index);

  // representation of 3D point as std::vector
  std::vector<double> as_vector() const;
  // Euclidean norm
  double norm() const;
  // squared norm
  double squared_norm() const;

  friend std::ostream& operator<<(std::ostream& os, const Point& p);
  bool operator==(const Point& p) const;
  Point& operator=(const Point& other);
  // vector addition
  Point operator+(const Point& p) const;
  // vector subtraction
  Point operator-(const Point& p) const;
  // scalar product
  double operator*(const Point& p) const;
  // scalar division
  Point operator/(double c) const;
};

// scalar multiplication
Point operator*(Point x, double c);
Point operator*(double c, Point x);

// The Pointcloud is a vector of points
class PointCloud : public std::vector<Point> {
 private:
  bool points2d;
  bool ordered;   //!

 public:
  bool is2d() const;
  void set2d(bool is2d);
  bool isOrdered() const;   //!
  void setOrdered(bool isOrdered);  //!
  PointCloud();
};


// Load csv file.
void load_csv_file(const char* fname, PointCloud& cloud, const char delimiter,
                   size_t skip = 0);
// Smoothing of the PointCloud *cloud*. The result is returned in *result_cloud*
void smoothen_cloud(const PointCloud& cloud, PointCloud& result_cloud,
                    double radius);

#endif
