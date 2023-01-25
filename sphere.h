#ifndef COLLISION_SPHERE
#define COLLISION_SPHERE

#include <Eigen/Geometry>
#include <Eigen/Core>
#include "spring.h"

/// <summary>
/// ����� ������� �����.
/// </summary>
class Sphere {
public:
    /// <summary>
    /// ������������� �����������.
    /// </summary>
    Sphere() = default;
    /// <summary>
    /// �������� ���������������� �����������. ������� ����� �������� radius � ����� origin_pos � ���������� ����� resolution.
    /// </summary>
    /// <param name="radius">������ �����</param>
    /// <param name="origin_pos">������� ������ �����</param>
    /// <param name="resolution">��������� ������ �����</param>
    Sphere(double radius, Eigen::Vector3d origin_pos, int resolution);
    /*Sphere(double radius, Eigen::Vector3d origin_pos) : radius_(radius), origin_pos_(origin_pos) {
    }
    Sphere(double radius, Eigen::MatrixXd verts, Eigen::MatrixXi faces) : radius_(radius), verts_(verts), faces_(faces) {
    }*/

    //Sphere(Sphere&) = delete;
    //Sphere operator=(Sphere*) = delete;

    /// <summary>
    /// ������������� ����������.
    /// </summary>
    ~Sphere() = default;

    /// <summary>
    /// ����������� �������� ����� � ���������� ������.
    /// </summary>
    /// <param name="point">�����, � ������� ���������� ��������� ��������</param>
    void ResolveCollision(Spring::Point* point);
    /// <summary>
    /// ���������� ������� �����.
    /// </summary>
    /// <param name="new_pos">����� ������� ������ �����</param>
    void UpdatePos(Eigen::Vector3d new_pos);
    /// <summary>
    /// ���������� ������� ������ �����.
    /// </summary>
    /// <returns>������� ������ ����� � ���� �������</returns>
    Eigen::Vector3d GetOriginPos();
    /// <summary>
    /// ���������� ������� ������ �����.
    /// </summary>
    /// <returns>������� ������ �����</returns>
    Eigen::MatrixXd GetVerts();
    /// <summary>
    /// ���������� ������� ������������� �����.
    /// </summary>
    /// <returns>������� ������������� �����</returns>
    Eigen::MatrixXi GetFaces();

private:
    /// <summary>
    /// ������� ������ ������� �����.
    /// ���������� ��� ������������ � igl.
    /// </summary>
    Eigen::MatrixXd verts_;
    /// <summary>
    /// ������� ������������� ������� �����.
    /// ���������� ��� ������������ � igl.
    /// </summary>
    Eigen::MatrixXi faces_;
    /// <summary>
    /// ������ �����.
    /// </summary>
    double radius_;
    /// <summary>
    /// ���������� ������ �����.
    /// </summary>
    Eigen::Vector3d origin_pos_;
};

#endif