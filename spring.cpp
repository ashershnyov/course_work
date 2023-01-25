#include "spring.h"

Spring::Point::Point(double mass, Eigen::Vector3d position, bool is_pinned) : mass_(mass), position_(position), is_pinned_(is_pinned) {
    acceleration_ = Eigen::Vector3d(0, 0, 0);
    velocity_ = Eigen::Vector3d(0, 0, 0);
}

void Spring::Point::AssignSpring(Spring* spring) {
    springs_.push_back(spring);
}

Eigen::Vector3d Spring::Point::ComputeForce(){
    Eigen::Vector3d force(0, 0, 0);
    for (Spring* i : springs_) {
        force += i->ComputeForce(this);
    }
    force += kGravityAccel * mass_;
    return force;
}

bool Spring::Point::GetPinnedStatus() const {
    return is_pinned_;
}

void Spring::Point::SetPinnedStatus(const bool& is_pinned) {
    is_pinned_ = is_pinned;
}

double Spring::Point::GetMass() const {
    return mass_;
}

Eigen::Vector3d Spring::Point::GetAccel() const {
    return acceleration_;
}

void Spring::Point::SetAccel(Eigen::Vector3d acceleration) {
    acceleration_ = acceleration;
}

Eigen::Vector3d Spring::Point::GetVelocity() const {
    return velocity_;
}

void Spring::Point::SetVelocity(Eigen::Vector3d velocity) {
    velocity_ = velocity;
}

Eigen::Vector3d Spring::Point::GetPos() const {
    return position_;
}

void Spring::Point::SetPos(Eigen::Vector3d position) {
    position_ = position;
}

Spring::Spring(Point* point_one, Point* point_two, double stiffness) : point_one_(point_one), point_two_(point_two), stiffness_(stiffness) {
    /*length_ = sqrt(pow(point_two_->GetPos()(0) - point_one_->GetPos()(0), 2) +
                   pow(point_two_->GetPos()(1) - point_one_->GetPos()(1), 2) +
                   pow(point_two_->GetPos()(2) - point_one_->GetPos()(2), 2));*/
    length_ = (point_two->GetPos() - point_one->GetPos()).norm();
}

Eigen::Vector3d Spring::ComputeForce(Point* base_point) const{
    if (base_point != point_one_ && base_point != point_two_) throw std::exception("A provided verticle should be contained in the spring!");
    Eigen::Vector3d direction = (point_two_->GetPos() - point_one_->GetPos());
    double distance = direction.norm();
    direction.normalize();
    /* double distance = sqrt(pow((point_two_->GetPos()(0)) - point_one_->GetPos()(0), 2) +
                            pow((point_two_->GetPos()(1)) - point_one_->GetPos()(1), 2) +
                            pow((point_two_->GetPos()(2)) - point_one_->GetPos()(2), 2));*/
                            //if (distance > 1.1 * length_) { distance = 1.1 * length_; }

    double d1 = point_one_->GetVelocity().dot(direction);
    double d2 = point_two_->GetVelocity().dot(direction);
    double damp = -1.2 * d1 + d2;
    //Eigen::Vector3d damp = -0.1 * (point_one_->GetVelocity() - point_two_->GetVelocity());

    double force = -stiffness_ * (length_ - distance);
    //double force = -stiffness_ * (distance - length_) * (1);
    if (base_point == point_one_) { return direction * (force + damp); }
    else { return direction * (-force + damp); }
}