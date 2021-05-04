#ifndef SPRING_H
#define SPRING_H

#include <vector>

#include "CGL/CGL.h"
#include "pointMass.h"

using namespace std;

namespace CGL {

enum e_spring_type { STRUCTURAL = 0, SHEARING = 1, BENDING = 2 };

struct Spring {
  Spring() {}                                   // Final Project

  Spring(PointMass *a, PointMass *b, e_spring_type spring_type)
      : pm_a(a), pm_b(b), spring_type(spring_type) {
    rest_length = (pm_a->position - pm_b->position).norm();
  }

  Spring(PointMass *a, PointMass *b, double rest_length)
      : pm_a(a), pm_b(b), rest_length(rest_length) {}

  double rest_length;
  double max_rest_length_coefficient = 1.1;     // Final project
  double ks_coefficient = 1.0;                  // Final project
  double bending_coefficient = 0.2;             // Final project
  Vector3D length;                              // Final project

  e_spring_type spring_type;

  PointMass *pm_a;
  PointMass *pm_b;
}; // struct Spring
}
#endif /* SPRING_H */
