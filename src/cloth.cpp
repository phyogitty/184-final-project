#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

#include "nmmintrin.h" // for SSE4.2
#include "immintrin.h" // for AVX

using namespace std;

int ClothType = 0;                                 // Final Project

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

// Final Project
void modify_Structural(Spring& spr, bool vertical, int x, int y) {
    switch (ClothType)
    {
    case 2:
        if (!vertical && ((x + 1) % 6 == 0 || (x + 4) % 6 == 0)) { spr.max_rest_length_coefficient = 3.0; spr.ks_coefficient = 0.1; }
        break;
    default:
        break;
    }
}
// Final Project
void modify_Shearing(Spring& spr, bool vertical, int x, int y) {
    switch (ClothType)
    {
    default:
        break;
    }
}
// Final Project
void modify_Bending(Spring& spr, bool vertical, int x, int y) {
    switch (ClothType)
    {
    case 1:
        if ((x + 3) % 6 == 0 && (y + 3) % 6 == 0) { spr.max_rest_length_coefficient = 0.5; }
        if ((x + 0) % 6 == 0 && (y + 0) % 6 == 0) { spr.max_rest_length_coefficient = 2.0; spr.bending_coefficient = 0.1; }
        break;
    case 2:
        if (!vertical && (x + 3) % 6 == 0) { spr.max_rest_length_coefficient = 0.7; }
        break;
    case 3:
        if (vertical && (x * y) % 6 == 0) { spr.max_rest_length_coefficient = 0.8; }
        break;
    default:
        break;
    }    
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  double stepX = (double)this->width / (double)this->num_width_points;
  double stepY = (double)this->height / (double)this->num_height_points;
  bool pin;

  if (this->orientation == HORIZONTAL) {
      double yPos = 1.0;
      double zPos, xPos;
      for (int h = 0; h < this->num_height_points; h++) {
          zPos = stepY * h;
          for (int w = 0; w < this->num_width_points; w++) {
              xPos = stepX * w;
              pin = false;
              for (vector<int> vec: pinned) {
                  if (vec[0] == w && vec[1] == h) {
                      pin = true;
                      break;
                  }
              }
              point_masses.push_back(PointMass(Vector3D(xPos, yPos, zPos), pin));
          }
      }
  } else {
      double zPos, xPos, yPos;
      for (int h = 0; h < this->num_height_points; h++) {
          yPos = stepY * h;
          for (int w = 0; w < this->num_width_points; w++) {
              xPos = stepX * w;
              pin = false;
              for (vector<int> vec: pinned) {
                  if (vec[0] == w && vec[1] == h) {
                      pin = true;
                      break;
                  }
              }
              zPos = - 0.001 + (double)(rand()) / ((double)(RAND_MAX/(0.002))) ;
              point_masses.push_back(PointMass(Vector3D(xPos, yPos, zPos), pin));
          }
      }
  }

  bool left, right, top, leftLeft, topTop;
  for (int y = 0; y < this->num_height_points; y += 1) {
      top = (y == 0) ? false : true;
      topTop = (y <= 1) ? false : true;
      for (int x = 0; x < this->num_width_points; x += 1) {
          left = (x == 0) ? false : true;
          leftLeft = (x <= 1) ? false : true;
          right = (x == this->num_width_points - 1) ? false : true;

          if (left) {
              Spring spr = Spring(&point_masses[y * this->num_width_points + x], &point_masses[y * this->num_width_points + x - 1], STRUCTURAL);
              modify_Structural(spr, false, x, y);
              springs.push_back(spr);
          }
          if (top) {
              Spring spr = Spring(&point_masses[y * this->num_width_points + x], &point_masses[(y - 1) * this->num_width_points + x], STRUCTURAL);
              modify_Structural(spr, true, x, y);
              springs.push_back(spr);
          }
          if (top && left) {
              Spring spr = Spring(&point_masses[y * this->num_width_points + x], &point_masses[(y - 1) * this->num_width_points + x - 1], SHEARING);
              modify_Shearing(spr, false, x, y);
              springs.push_back(spr);
          }
          if (top && right) {
              Spring spr = Spring(&point_masses[y * this->num_width_points + x], &point_masses[(y - 1) * this->num_width_points + x + 1], SHEARING);
              modify_Shearing(spr, true, x, y);
              springs.push_back(spr);
          }
          if (leftLeft) {
              Spring spr = Spring(&point_masses[y * this->num_width_points + x], &point_masses[y * this->num_width_points + x - 2], BENDING);
              modify_Bending(spr, false, x, y);
              springs.push_back(spr);
          }
          if (topTop) {
              Spring spr = Spring(&point_masses[y * this->num_width_points + x], &point_masses[(y - 2) * this->num_width_points + x], BENDING);
              modify_Bending(spr, true, x, y);
              springs.push_back(spr);
          }          
      }
  }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  double pointMassRadius = width * height / (num_width_points + num_height_points) / 2.0;     // cm
  Vector3D gravity = mass * external_accelerations[0];                                        // g*m/(s^2)
  double airViscosity = 1.849;  // g/(cm*s)
  double stokesCoefficients = 6.0 * PI * pointMassRadius * airViscosity;
  Vector3D stokesDrag = 0.0;

  // Final Project
  size_t limit_i;
  switch (ClothType)
  {
  case 1:
      mass = mass * 10;
      break;
  case 2:
      mass = mass * 10;
      break;
  case 3:
      mass = mass * 15;
      break;
  default:
      break;
  }

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D f = 0;
  for (Vector3D a: external_accelerations) { f += mass * a; }
  
  /**/
  #pragma omp parallel for private(stokesDrag)    // stokesDrag needs to be private to each thread to avoid data race
  for (PointMass &p : point_masses) {
      if (!(external_accelerations[1] == Vector3D(0, 0, 0))) {   // Skip the stokesDrag calculation if there's no wind
          stokesDrag = stokesCoefficients * (external_accelerations[1] - p.velocity(delta_t)) * abs(dot(p.normal(), external_accelerations[1]));   // Stokes drag equation for particles
      }
      p.forces = gravity + stokesDrag;
      // TODO: need to add check for if cloth is behind object
  }
  /**/

  /*
  limit_i = 4 * (point_masses.size() / 4);
#pragma omp parallel for                                                                        // OMP Parallel
  for (size_t i = 0; i < limit_i; i += 4) {
      point_masses[i].forces = f;
      point_masses[i + 1].forces = f;
      point_masses[i + 2].forces = f;
      point_masses[i + 3].forces = f;
  }
  for (size_t i = limit_i; i < point_masses.size(); i += 1) {
      point_masses[i].forces = f;
  }
  */
      /**/
  #pragma omp parallel for                                                                        // OMP Parallel
  for (Spring& spring : springs) {
      if (spring.spring_type == STRUCTURAL && cp->enable_structural_constraints ||
          spring.spring_type == SHEARING && cp->enable_shearing_constraints ||
          spring.spring_type == BENDING && cp->enable_bending_constraints) {
          Vector3D springVect = spring.pm_a->position - spring.pm_b->position;
          // Hook's law (spring force = spring constant * (distance from each end - rest length of spring)
          Vector3D F_s = cp->ks * springVect.unit() * (springVect.norm() - spring.rest_length);
          if (spring.spring_type == BENDING)
              F_s *= 0.2; // Bending springs should be weaker
          spring.pm_a->forces -= F_s;
          spring.pm_b->forces += F_s;
      }
  }
      /**/

      /*
      limit_i = 4 * (springs.size() / 4);
#pragma omp parallel for                                                                        // OMP Parallel
      for (size_t i = 0; i < limit_i; i += 4) {
          double KS = (springs[i].spring_type == BENDING) ? cp->ks * springs[i].bending_coefficient : cp->ks;
          Vector3D force = (KS * springs[i].ks_coefficient) * ((springs[i].pm_b->position - springs[i].pm_a->position).norm() - springs[i].rest_length) * (springs[i].pm_a->position - springs[i].pm_b->position).unit();
          springs[i].pm_a->forces -= force;
          springs[i].pm_b->forces += force;

          double KS1 = (springs[i + 1].spring_type == BENDING) ? cp->ks * springs[i + 1].bending_coefficient : cp->ks;
          Vector3D force1 = (KS1 * springs[i + 1].ks_coefficient) * ((springs[i + 1].pm_b->position - springs[i + 1].pm_a->position).norm() - springs[i + 1].rest_length) * (springs[i + 1].pm_a->position - springs[i + 1].pm_b->position).unit();
          springs[i + 1].pm_a->forces -= force1;
          springs[i + 1].pm_b->forces += force1;

          double KS2 = (springs[i + 2].spring_type == BENDING) ? cp->ks * springs[i + 2].bending_coefficient : cp->ks;
          Vector3D force2 = (KS2 * springs[i + 2].ks_coefficient) * ((springs[i + 2].pm_b->position - springs[i + 2].pm_a->position).norm() - springs[i + 2].rest_length) * (springs[i + 2].pm_a->position - springs[i + 2].pm_b->position).unit();
          springs[i + 2].pm_a->forces -= force2;
          springs[i + 2].pm_b->forces += force2;

          double KS3 = (springs[i + 3].spring_type == BENDING) ? cp->ks * springs[i + 3].bending_coefficient : cp->ks;
          Vector3D force3 = (KS3 * springs[i + 3].ks_coefficient) * ((springs[i + 3].pm_b->position - springs[i + 3].pm_a->position).norm() - springs[i + 3].rest_length) * (springs[i + 3].pm_a->position - springs[i + 3].pm_b->position).unit();
          springs[i + 3].pm_a->forces -= force3;
          springs[i + 3].pm_b->forces += force3;
      }
      for (size_t i = limit_i; i < springs.size(); i += 1) {
          //Spring spring = springs[i];
          double KS = (springs[i].spring_type == BENDING) ? cp->ks * springs[i].bending_coefficient : cp->ks;
          Vector3D force = (KS * springs[i].ks_coefficient) * ((springs[i].pm_b->position - springs[i].pm_a->position).norm() - springs[i].rest_length) * (springs[i].pm_a->position - springs[i].pm_b->position).unit();
          springs[i].pm_a->forces -= force;
          springs[i].pm_b->forces += force;
      }
      */

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
    double not_damp = 1 - cp->damping / 100.0;
    double delta_t2 = delta_t * delta_t;
    double delta_t2_mass = delta_t2 / mass;

    //__m256 vec = _mm256_set_pd(0.0, 0.0, 0.0, 0.0);

    #pragma omp parallel for                                                                        // OMP Parallel
    for (PointMass& pm : point_masses) {
        if (!pm.pinned) {
            Vector3D newPos = pm.position + (pm.position - pm.last_position) * not_damp + (pm.forces * delta_t2_mass);
            pm.last_position = pm.position;
            pm.position = newPos;
        }
    }

    /*
#pragma omp parallel for                                                                        // OMP Parallel
    for (PointMass &pm: point_masses) {
        if (pm.pinned) { continue; }
        Vector3D newPos = pm.position + (pm.position - pm.last_position) * not_damp + (pm.forces * delta_t2_mass);
        pm.last_position = pm.position;
        pm.position = newPos;
    }
    */

    /*
    limit_i = 4 * (point_masses.size() / 4);
#pragma omp parallel for                                                                        // OMP Parallel
    for (size_t i = 0; i < limit_i; i += 4) {
        if (!point_masses[i].pinned) {
            Vector3D newPos = point_masses[i].position + (point_masses[i].position - point_masses[i].last_position) * not_damp + (point_masses[i].forces * delta_t2_mass);
            point_masses[i].last_position = point_masses[i].position;
            point_masses[i].position = newPos;
        }
        if (!point_masses[i + 1].pinned) {
            Vector3D newPos1 = point_masses[i + 1].position + (point_masses[i + 1].position - point_masses[i + 1].last_position) * not_damp + (point_masses[i + 1].forces * delta_t2_mass);
            point_masses[i + 1].last_position = point_masses[i + 1].position;
            point_masses[i + 1].position = newPos1;
        }
        if (!point_masses[i + 2].pinned) {
            Vector3D newPos2 = point_masses[i + 2].position + (point_masses[i + 2].position - point_masses[i + 2].last_position) * not_damp + (point_masses[i + 2].forces * delta_t2_mass);
            point_masses[i + 2].last_position = point_masses[i + 2].position;
            point_masses[i + 2].position = newPos2;
        }
        if (!point_masses[i + 3].pinned) {
            Vector3D newPos3 = point_masses[i + 3].position + (point_masses[i + 3].position - point_masses[i + 3].last_position) * not_damp + (point_masses[i + 3].forces * delta_t2_mass);
            point_masses[i + 3].last_position = point_masses[i + 3].position;
            point_masses[i + 3].position = newPos3;
        }
    }
    for (size_t i = limit_i; i < point_masses.size(); i += 1) {
        if (!point_masses[i].pinned) {
            Vector3D newPos = point_masses[i].position + (point_masses[i].position - point_masses[i].last_position) * not_damp + (point_masses[i].forces * delta_t2_mass);
            point_masses[i].last_position = point_masses[i].position;
            point_masses[i].position = newPos;
        }
    }
    */

    /*
    // TODO (Part 4): Handle self-collisions.
    // TODO (Part 3): Handle collisions with other primitives.
    build_spatial_map();    
#pragma omp parallel for                                                                        // OMP Parallel
    for (PointMass& pM : point_masses) {
        self_collide(pM, simulation_steps);
#pragma omp parallel for                                                                        // OMP Parallel
        for (CollisionObject* cObj : *collision_objects) {
            cObj->collide(pM);
        }
    }
    */
    
  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
#pragma omp parallel for                                                                        // OMP Parallel
  for (PointMass& pM : point_masses) {
      self_collide(pM, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
#pragma omp parallel for                                                                        // OMP Parallel
  for (PointMass &pM: point_masses) {
#pragma omp parallel for                                                                        // OMP Parallel
      for (CollisionObject *cObj: *collision_objects) {
          cObj->collide(pM);
      }
  }
  

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
#pragma omp parallel for                                                                        // OMP Parallel
  for (Spring &spring: springs) {
      if (spring.pm_a->pinned && spring.pm_b->pinned) { continue; }
      double dist = (spring.pm_a->position - spring.pm_b->position).norm();
      double diff = dist - spring.rest_length * spring.max_rest_length_coefficient;
      if (diff > 0) {
          Vector3D direction = (spring.pm_b->position - spring.pm_a->position).unit();
          if (spring.pm_a->pinned && !spring.pm_b->pinned) {
              spring.pm_b->position = spring.pm_b->position - diff * direction;
          }
          if (spring.pm_b->pinned && !spring.pm_a->pinned) {
              spring.pm_a->position = spring.pm_a->position + diff * direction;
          }
          if (!spring.pm_a->pinned && !spring.pm_b->pinned) {
              diff = diff / 2.0;
              spring.pm_a->position = spring.pm_a->position + diff * direction;
              spring.pm_b->position = spring.pm_b->position - diff * direction;
          }
      }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.   // OMP Parallel
  for (PointMass &pM : point_masses) {
      float key = hash_position(pM.position);
      if (map.find(key) == map.end()) {     // the key not found
        map[key] = new vector<PointMass*>();
      }       // OMP Parallel critical
      map.at(key)->push_back(&pM);
  }
}

void Cloth::self_collide(PointMass& pm, double simulation_steps) {
    // TODO (Part 4): Handle self-collision for a given point mass.
    float thick = 2 * thickness;
    int counter = 0;
    Vector3D avgCorr = 0;
    for (PointMass* cPm : *map.at(hash_position(pm.position))) {
        Vector3D vec = pm.position - cPm->position;
        float dist = vec.norm();
        if (dist != 0 && dist < thick) {
            avgCorr += (thick - dist) * vec.unit();
            counter++;
        }
    }
    if (counter > 0) {
        pm.position += avgCorr / counter / simulation_steps;
    }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float w = 3 * width / num_width_points;
  float h = 3 * height / num_height_points;
  float t = max(w, h);
  
  return pow(2, floor(pos.x / w)) * pow(3, floor(pos.y / h)) * pow(5, floor(pos.z / t));
  //return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
