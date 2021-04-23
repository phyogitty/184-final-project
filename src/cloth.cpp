#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

int ClothType = 1;                                 // Final Project

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

  // Final Project
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
  for (Vector3D a: external_accelerations) {
      f += mass * a;
  }
  for (int i = 0; i < point_masses.size(); i++) {
      point_masses[i].forces = mass * f;
  }
  if (cp->enable_bending_constraints || cp->enable_structural_constraints || cp->enable_shearing_constraints) {
      double KS, force_s;
      Vector3D force;
      for (Spring &spring: springs) {
          KS = (spring.spring_type == BENDING) ? cp->ks * spring.bending_coefficient : cp->ks;
          KS = KS * spring.ks_coefficient;
          force_s = KS * ((spring.pm_b->position - spring.pm_a->position).norm() - spring.rest_length);
          force = (spring.pm_a->position - spring.pm_b->position).unit();
          spring.pm_a->forces -= force_s * force;
          spring.pm_b->forces += force_s * force;
      }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
    double not_damp = 1 - cp->damping / 100.0;
    double delta_t2 = delta_t * delta_t;
    double delta_t2_mass = delta_t2 / mass;
    for (PointMass &pm: point_masses) {
        if (pm.pinned) { continue; }
        Vector3D newPos = pm.position + (pm.position - pm.last_position) * not_damp + (pm.forces * delta_t2_mass);
        pm.last_position = pm.position;
        pm.position = newPos;
    }


  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass& pM : point_masses) {
      self_collide(pM, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass &pM: point_masses) {
      for (CollisionObject *cObj: *collision_objects) {
          cObj->collide(pM);
      }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  double dist, diff;
  for (Spring &spring: springs) {
      if (spring.pm_a->pinned && spring.pm_b->pinned) { continue; }
      dist = (spring.pm_a->position - spring.pm_b->position).norm();
      diff = dist - spring.rest_length * spring.max_rest_length_coefficient;
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

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  float key;
  for (PointMass &pM : point_masses) {
      key = hash_position(pM.position);
      if (map.find(key) == map.end()) {     // the key not found
        map[key] = new vector<PointMass*>();
      }
      map.at(key)->push_back(&pM);
  }
}

void Cloth::self_collide(PointMass& pm, double simulation_steps) {
    // TODO (Part 4): Handle self-collision for a given point mass.
    float thick = 2 * thickness;
    int counter = 0;
    Vector3D avgCorr = 0;
    Vector3D vec;
    float hashVal = hash_position(pm.position);
    for (PointMass* cPm : *map.at(hashVal)) {
        vec = pm.position - cPm->position;
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

  float ww = floor(pos.x / w);
  float hh = floor(pos.y / h);
  float tt = floor(pos.z / t);
  
  return pow(2, ww) * pow(3, hh) * pow(5, tt);
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
