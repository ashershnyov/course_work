#include "cloth.h"

Cloth::Cloth(const int hVerts, const int vVerts, const double width, const double height,
    const double mass, double stiffness) : stiffness_(stiffness) {

    verts_ = Eigen::MatrixXd(hVerts * vVerts, 3);
    for (int i = 0; i < vVerts; i++) {
        for (int j = 0; j < hVerts; j++) {
            verts_(i * hVerts + j, 0) = j * (width / hVerts);
            verts_(i * hVerts + j, 2) = -i * (height / vVerts);
            verts_(i * hVerts + j, 1) = 0.0;
            points_.push_back(new Spring::Point(mass, Eigen::Vector3d(j * (width / hVerts), 0.0, -i * (height / vVerts))));
            if (i == 0) {
                points_[points_.size() - 1]->SetPinnedStatus(true);
            }
        }
    }

    faces_ = Eigen::MatrixXi((hVerts - 1) * (vVerts - 1) * 2, 3);
    int counter = 0;
    for (int i = 0; i < vVerts - 1; i++) {
        for (int j = 0; j < hVerts - 1; j++) {
            faces_(counter, 0) = i * hVerts + j;
            faces_(counter, 1) = i * hVerts + j + 1;
            faces_(counter, 2) = i * hVerts + j + hVerts;
            counter++;

            faces_(counter, 0) = i * hVerts + j + hVerts;
            faces_(counter, 1) = i * hVerts + j + 1;
            faces_(counter, 2) = i * hVerts + j + hVerts + 1;
            counter++;
        }
    }

    counter = 0;
    for (int i = 0; i < vVerts - 1; i++) {
        for (int j = 0; j < hVerts - 1; j++) {
            int base = i * hVerts + j;
            int right = i * hVerts + j + 1;
            int down = i * hVerts + j + hVerts;
            int down_right = i * hVerts + j + hVerts + 1;
            springs_.push_back(new Spring(points_[base], points_[right], stiffness));
            points_[base]->AssignSpring(springs_[springs_.size() - 1]);
            points_[right]->AssignSpring(springs_[springs_.size() - 1]);

            springs_.push_back(new Spring(points_[base], points_[down], stiffness));
            points_[base]->AssignSpring(springs_[springs_.size() - 1]);
            points_[down]->AssignSpring(springs_[springs_.size() - 1]);

            springs_.push_back(new Spring(points_[down], points_[right], stiffness));
            points_[down]->AssignSpring(springs_[springs_.size() - 1]);
            points_[right]->AssignSpring(springs_[springs_.size() - 1]);

            springs_.push_back(new Spring(points_[down_right], points_[base], stiffness));
            points_[down_right]->AssignSpring(springs_[springs_.size() - 1]);
            points_[base]->AssignSpring(springs_[springs_.size() - 1]);
        }
    }
}

void Cloth::Update() {
    int counter = 0;
    for (int i = 0; i < verts_.rows(); i++) {
        if (points_[i]->GetPinnedStatus()) { continue; }
        points_[i]->SetAccel(points_[i]->ComputeForce() / points_[i]->GetMass() * 0.1);
        points_[i]->SetVelocity(points_[i]->GetVelocity() + points_[i]->GetAccel() * 0.1);
        counter++;
    }
    for (int i = 0; i < verts_.rows(); i++) {
        points_[i]->SetPos(points_[i]->GetPos() + points_[i]->GetVelocity() * 0.03);
        verts_(i, 0) = points_[i]->GetPos()(0);
        verts_(i, 1) = points_[i]->GetPos()(1);
        verts_(i, 2) = points_[i]->GetPos()(2);
    }
}

void Cloth::UpdatePointsPos() {
    for (int i = 0; i < verts_.rows(); i++) {
        verts_(i, 0) = points_[i]->GetPos()(0);
        verts_(i, 1) = points_[i]->GetPos()(1);
        verts_(i, 2) = points_[i]->GetPos()(2);
    }
}

void Cloth::CheckCollision(Sphere& sphere) {
    for (Spring::Point* point : points_) {
        sphere.ResolveCollision(point);
    }
}

Eigen::MatrixXd Cloth::GetVerts() const{
    return verts_;
}

Eigen::MatrixXi Cloth::GetFaces() const{
    return faces_;
}
