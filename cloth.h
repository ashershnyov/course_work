#ifndef MASS_SPRING_CLOTH
#define MASS_SPRING_CLOTH

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include "spring.h"
#include "sphere.h"

/// <summary>
/// ����� ������� �����.
/// </summary>
class Cloth {
public:
    /// <summary>
    /// ������������� �����������.
    /// </summary>
    Cloth() = default;
    /// <summary>
    /// ���������������� �����������. ������� ������ ����� ������� width, ������� height, ����������� ������ �� ����������� hVerts, ����������� ������ �� ��������� vVerts, � ������ ����� mass � ���������� ������ stiffness. 
    /// </summary>
    /// <param name="hVerts">���������� ������ �� ���������</param>
    /// <param name="vVerts">���������� ������ �� �����������</param>
    /// <param name="width">������ � �.�.</param>
    /// <param name="height">������ � �.�.</param>
    /// <param name="mass">����� ����� ������� � �.�.</param>
    /// <param name="stiffness">��������� ������ � �.�.</param>
    Cloth(const int hVerts, const int vVerts, const double width = 1.0, const double height = 1.0,
        const double mass = 0.3, double stiffness = 80);

    // ��������� �����������

    /// <summary>
    /// ���������������� ����������.
    /// </summary>
    ~Cloth() {
        for (Spring* i : springs_) {
            delete i;
        }
        for (Spring::Point* i : points_) {
            delete i;
        }
    }

    /// <summary>
    /// ��������� ��������� �����.
    /// �������� ������ �������� ���. ��������� �������� ��������� � ��������� ������. ��������� �������� ������� ������.
    /// </summary>
    void Update();
    /// <summary>
    /// ��������� ������� ������ � ������� verts_ � ������������ � ���������� ����� �� points_.
    /// </summary>
    void UpdatePointsPos();

    /// <summary>
    /// ��������� ��������� �� �������� ����� ������ � ������.
    /// ��� ������ ����� ����� �������� ����������� �������� �����.
    /// </summary>
    /// <param name="sphere">�����, �������� � ������� ���� ���������</param>
    void CheckCollision(Sphere& sphere);
    /// <summary>
    /// ���������� ������� ������ ������� �����.
    /// </summary>
    /// <returns>������� ������ ������� �����</returns>
    Eigen::MatrixXd GetVerts() const;
    /// <summary>
    /// ���������� ������� ������������� ������� �����.
    /// </summary>
    /// <returns>������� ������������� ������� �����</returns>
    Eigen::MatrixXi GetFaces() const;

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
    /// ������ ���������� �� ������� ������, ������������ �����.
    /// </summary>
    std::vector<Spring::Point*> points_;
    /// <summary>
    /// ������ ���������� �� ������� ������, ����������� �������.
    /// </summary>
    std::vector<Spring*> springs_;
    /// <summary>
    /// �������� ����������� ��������� ������.
    /// </summary>
    double stiffness_;
};

#endif