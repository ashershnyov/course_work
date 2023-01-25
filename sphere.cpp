#include "sphere.h"
#include <cmath>
#include <igl/PI.h>

//const int kRes = 32;

Sphere::Sphere(double radius, Eigen::Vector3d origin_pos, int resolution) : radius_(radius), origin_pos_(origin_pos) {

    verts_ = Eigen::MatrixXd(resolution * resolution + 2, 3);
    faces_ = Eigen::MatrixXi(2 * resolution * (resolution), 3);

    double th = 0;
    double phi = 0;
    double dTh = igl::PI / (resolution + 2);
    double dPhi = 2 * igl::PI / resolution;
    verts_.row(0) << origin_pos(0), origin_pos(1), origin_pos(2) + radius;

    for (int ring = 0; ring < resolution; ring++) {
        th += dTh;
        for (int point = 0; point < resolution; point++) {
            phi += dPhi;
            double x = origin_pos(0) + radius * sin(th) * cos(phi);
            double y = origin_pos(1) + radius * sin(th) * sin(phi);
            double z = origin_pos(2) + radius * cos(th);
            verts_.row(ring * resolution + point + 1) << x, y, z;
        }

    }

    verts_.row(resolution * resolution + 1) << origin_pos(0), origin_pos(1), origin_pos(2) - radius;

    int counter = 0;
    for (int ring = 0; ring < resolution - 1; ring++) {
        for (int point = 0; point < resolution; point++) {
            int p_base = ring * resolution + point + 1;
            int p_down = ring * resolution + point + resolution + 1;
            int p_right = ring * resolution + point + 1 + 1;
            int p_dn_right = ring * resolution + point + 1 + resolution + 1;
            faces_.row(counter) << p_base, p_down, p_dn_right;
            counter++;
            faces_.row(counter) << p_base, p_right, p_dn_right;
            counter++;
        }
        counter -= 2;
        faces_.row(counter)(2) -= resolution;
        counter++;
        faces_.row(counter) << resolution + ring * resolution, 1 + resolution * ring, resolution + 1 + ring * resolution;
        counter++;
    }

    for (int point = 0; point < resolution - 1; point++) {
        //int p_base = 0;
        int p_down = point + 1;
        int p_dn_right = point + 2;
        faces_.row(counter) << 0, p_down, p_dn_right;
        counter++;
    }

    faces_.row(counter) << 0, resolution, 1;
    counter++;

    for (int point = 0; point < resolution - 1; point++) {
        int p_base = resolution * resolution + 1;
        int p_up = resolution * resolution + 1 - point - 1;
        int p_up_right = resolution * resolution + 1 - point - 2;
        faces_.row(counter) << p_base, p_up, p_up_right;
        counter++;
    }

    faces_.row(counter) << resolution * resolution + 1, resolution * resolution - resolution + 1, resolution * resolution;
    counter++;

    //std::cout << counter << "\n";
}

void Sphere::ResolveCollision(Spring::Point* point) {
    Eigen::Vector3d direction = point->GetPos() - origin_pos_;
    if (direction.norm() < radius_) {
        Eigen::Vector3d point_pos_update = direction.normalized() * radius_ + origin_pos_ - point->GetPos();
        point->SetPos(point->GetPos() + point_pos_update * 1);
        point->SetAccel(Eigen::Vector3d::Zero());
        point->SetVelocity(Eigen::Vector3d::Zero());
    }
}

void Sphere::UpdatePos(Eigen::Vector3d new_pos) {
    origin_pos_ = new_pos;
    for (int i = 0; i < verts_.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            verts_(0, j) += new_pos(j);
        }
    }
}

Eigen::Vector3d Sphere::GetOriginPos() {
    return origin_pos_;
}

Eigen::MatrixXd Sphere::GetVerts() {
    return verts_;
}

Eigen::MatrixXi Sphere::GetFaces() {
    return faces_;
}