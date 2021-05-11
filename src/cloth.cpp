#pragma clang diagnostic push
#pragma ide diagnostic ignored "openmp-use-default-none"
#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "omp.h"

using namespace std;

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

void Cloth::buildGrid() {
    double horizStepSize = width / (num_width_points - 1.0);
    double vertStepSize = height / (num_height_points - 1.0);
    if (orientation == HORIZONTAL) {
        for (int y = 0; y < num_height_points; ++y) {
            for (int x = 0; x < num_width_points; ++x) {
                bool isPinned = false;
                for (vector<int> vect : pinned) {
                    if (vect[0] == x && vect[1] == y) {
                        isPinned = true;
                        break;
                    }
                }
                point_masses.emplace_back(PointMass(Vector3D(x * horizStepSize, 1.0, y * vertStepSize), isPinned));
            }
        }
    } else {
        for (int y = 0; y < num_height_points; ++y) {
            for (int x = 0; x < num_width_points; ++x) {
                bool isPinned = false;
                for (vector<int> vect : pinned) {
                    if (vect[0] == x && vect[1] == y) {
                        isPinned = true;
                        break;
                    }
                }
                double z = (rand() % 2 - 1.0) / 1000.0;
                point_masses.emplace_back(PointMass(Vector3D(x * horizStepSize, y * vertStepSize, z), isPinned));
            }
        }
    }
    for (int y = 0; y < num_height_points; ++y) {
        for (int x = 0; x < num_width_points; ++x) {
            PointMass* a = &point_masses[y * num_width_points + x];
            if (x >= 1) {
                PointMass* left = &point_masses[y * num_width_points + x - 1];
                springs.emplace_back(a, left, STRUCTURAL);
            }
            if (y >= 1) {
                PointMass* above = &point_masses[(y - 1) * num_width_points + x];
                springs.emplace_back(a, above, STRUCTURAL);
            }
            if (x >= 1 && y >= 1) {
                PointMass *upperLeft = &point_masses[(y - 1) * num_width_points + x - 1];
                springs.emplace_back(a, upperLeft, SHEARING);
            }
            if (x + 1 < num_width_points && y >= 1) {
                PointMass *upperRight = &point_masses[(y - 1) * num_width_points + x + 1];
                springs.emplace_back(a, upperRight, SHEARING);
            }
            if (x >= 2) {
                PointMass *twoLeft = &point_masses[y * num_width_points + x - 2];
                springs.emplace_back(a, twoLeft, BENDING);
            }
            if (y >= 2) {
                PointMass *twoAbove = &point_masses[(y - 2) * num_width_points + x];
                springs.emplace_back(a, twoAbove, BENDING);
            }
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    double mass = width * height * cp->density / num_width_points / num_height_points;          // g/cm^2
    double delta_t = 1.0f / frames_per_sec / simulation_steps;

    double pointMassRadius = width * height / (num_width_points + num_height_points) / 2.0;     // cm
    Vector3D gravity = mass * external_accelerations[0];                                        // g*m/(s^2)
    double airViscosity = 1.849;  // g/(cm*s)
    double stokesCoefficients = 6.0 * PI * pointMassRadius * airViscosity;
    Vector3D stokesDrag = 0.0;

    int p_size = point_masses.size(); // changed for loop for OpenMP2.0 (MSVC only supports this)
    #pragma omp parallel for private(stokesDrag)    // stokesDrag needs to be private to each thread to avoid data race
    for (int i = 0; i < p_size; i++) {
        auto& p = *(point_masses.begin() + i);
        if (!(external_accelerations[1] == Vector3D(0, 0, 0))) {   // Skip the stokesDrag calculation if there's no wind
            stokesDrag = stokesCoefficients * (external_accelerations[1] - p.velocity(delta_t)) * abs(dot(p.normal(), external_accelerations[1]));   // Stokes drag equation for particles
        }
        p.forces = gravity + stokesDrag;
        // TODO: need to add check for if cloth is behind object
        // TODO: possibly scale down wind force radially away from the center?
    }

    int s_size = springs.size();
    // Apply spring forces to each point mass using Hooke's law
    #pragma omp parallel for
    for (int i = 0; i < s_size; i++) {
        auto spring = &(springs[i]);
        Vector3D F_s = Vector3D(0, 0, 0);
        Vector3D springVect = spring->pm_a->position - spring->pm_b->position;
        // Hook's law (spring force = spring constant * (distance from each end - rest length of spring)
        if (spring->spring_type == STRUCTURAL && cp->enable_structural_constraints) {
            F_s = cp->structural_ks * springVect.unit() * (springVect.norm() - spring->rest_length);
        }
        else if (spring->spring_type == SHEARING && cp->enable_shearing_constraints) {
            F_s = cp->shearing_ks * springVect.unit() * (springVect.norm() - spring->rest_length);
        }
        else if (spring->spring_type == BENDING && cp->enable_shearing_constraints) {
            F_s = cp->bending_ks * springVect.unit() * (springVect.norm() - spring->rest_length);
        }
            spring->pm_a->forces -= F_s;
            spring->pm_b->forces += F_s;
        }

    // Apply Verlet integration/Euler forward timestep to each point mass
    #pragma omp parallel for
    for (int i = 0; i < p_size; i++) {
        auto& p = *(point_masses.begin() + i);
        if (!p.pinned) {
            Vector3D temp = p.position;
            p.position = p.position + (1 - cp->damping/100.0) * (p.position - p.last_position) + (p.forces / mass) * delta_t * delta_t;
            p.last_position = temp;
        }
    }

    // Organize our masses into a spatial map (hashmap) for efficient access
    build_spatial_map();

    // Check for and calculate collisions for each point mass
    #pragma omp parallel for
    for (int i = 0; i < p_size; i++) {
        auto& p = *(point_masses.begin() + i);
        self_collide(p, simulation_steps);
        for (CollisionObject *primitive : *collision_objects)
            primitive->collide(p);
    }

    // We don't want the springs to be able to stretch too far in a single timestep (causes instability)
    // So, our last step is to constrain the distance that connected masses can be from each other
    // based on the rest_length of the spring that connects them.
    #pragma omp parallel for
    for (int i = 0; i < s_size; i++) {
        auto spring = springs[i];
        Vector3D springVect = spring.pm_a->position - spring.pm_b->position;
        double difference = springVect.norm() - (1.10 * spring.rest_length);
        if (difference > 0) {
            Vector3D direction = springVect.unit();
            Vector3D correction = direction * difference / 2.0;
            if (!spring.pm_a->pinned && !spring.pm_b->pinned) {
                spring.pm_a->position -= correction;
                spring.pm_b->position += correction;
            } else if (!spring.pm_a->pinned) {
                spring.pm_a->position -= correction * 2.0;
            } else if (!spring.pm_b->pinned) {
                spring.pm_b->position += correction * 2.0;
            }
        }
    }
}

void Cloth::build_spatial_map() {
    for (const auto &entry : map) {
        delete(entry.second);
    }
    map.clear();
    int size = point_masses.size();
    float *hashes = new float[size]; // allocate array on heap so we can compute hashes in parallel
    #pragma omp parallel for schedule(static,8)
    for (int i = 0; i < size; i++) {
        //for (PointMass p : point_masses) {
        auto p = point_masses[i];
        //float hash = hash_position(p.position);
        hashes[i] = hash_position(p.position);
    }
    for (int i = 0; i < size; i++) {
        if (!map[hashes[i]])
            map[hashes[i]] = new vector<PointMass *>();
        map[hashes[i]]->emplace_back(&point_masses[i]);  // replaced push_back with emplace_back
    }
    delete[] hashes;
}

// Check all of the nearby points to pm. If pm is too close to another mass, we apply a corrective force
// to pm to move it away from that mass. We define nearby points to be any that are in the same 3D box volume as
// pm. We use our spatial map (hashmap) to efficiently access points that are in the same box.
void Cloth::self_collide(PointMass &pm, double simulation_steps) {
    Vector3D correction;
    int totalCorrections = 0;
    auto *points = map[hash_position(pm.position)];
    int m_size = (*points).size();
    #pragma omp parallel for
    for (int i = 0; i < m_size; i++) {
        auto p = (*points)[i];
        if (p != &pm) {
            Vector3D springVector = pm.position - p->position;
            if (springVector.norm() <= 2 * thickness) {
                correction += springVector.unit() * (2 * thickness - springVector.norm());
                totalCorrections++;
            }
        }
    }
    if (totalCorrections > 0) pm.position += (correction / totalCorrections / simulation_steps);
}

// Position hash function
float Cloth::hash_position(Vector3D pos) {
    float w = 3 * width / num_height_points, h = 3 * height / num_height_points, t = max(w, h);
    Vector3D boxDimensions{ w,h,t };
    // Convert position to nearest box coordinate
    pos.x -= fmod(pos.x, boxDimensions.x);
    pos.y -= fmod(pos.y, boxDimensions.y);
    pos.z -= fmod(pos.z, boxDimensions.z);
    // Hash using primes
    /*int hash = 13;
    hash = ((int) (hash + pos.x) << 5) - (hash + pos.x);
    hash = ((int) (hash + pos.y) << 5) - (hash + pos.y);
    return ((int) (hash + pos.z) << 5) - (hash + pos.y);*/

    return (((((13 + pos.x) * 15) + pos.y) * 17) + pos.z);
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

            PointMass* pm = &point_masses[y * num_width_points + x];

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
#pragma omp parallel for
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

#pragma clang diagnostic pop