#ifndef MASS_SPRING_SPRING
#define MASS_SPRING_SPRING

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>

/// <summary>
/// ������ ��������� ���������� �������.
/// </summary>
const Eigen::Vector3d kGravityAccel(0, -9.81, 0);

/// <summary>
/// ����� ������� �������.
/// </summary>
class Spring {
public:
    /// <summary>
    /// ����� ������� �����.
    /// ������ � ����� �������.
    /// </summary>
    class Point {
    public:
        /// <summary>
        /// ������������� �����������.
        /// </summary>
        Point() = default;
        /// <summary>
        /// ���������������� �����������. ������� ����� ������ mass, � ����� position. ����������� �������� ������ ������ �������������� is_pinned.
        /// </summary>
        /// <param name="mass">����� ������� � �.�.</param>
        /// <param name="position">������� �������</param>
        /// <param name="is_pinned">��������� �������������� �������</param>
        Point(double mass, Eigen::Vector3d position, bool is_pinned = false);

        Point(Point&) = delete;
        Point operator=(Point&) = delete;
        // ��������� �����������

        /// <summary>
        /// ������������� ����������.
        /// </summary>
        ~Point() = default;

        /// <summary>
        /// ��������� ������� �������.
        /// </summary>
        /// <param name="spring">�������, ������� ������ ���� ������� � ��������</param>
        void AssignSpring(Spring* spring);
        /// <summary>
        /// ������������ ���� �� �����.
        /// �������� ������� ��� �� ��������, ����������� ��� ����� � �������; ��������� ����������.
        /// </summary>
        /// <returns>������ ����, ����������� �� �����.</returns>
        Eigen::Vector3d ComputeForce();

        /// <summary>
        /// ���������� ������ �������������� �����.
        /// </summary>
        /// <returns>������ �������������� �����</returns>
        bool GetPinnedStatus() const;
        /// <summary>
        /// ������������� ������ �������������� �����.
        /// </summary>
        /// <param name="is_pinned">����� �������� ������� �������������� �����</param>
        void SetPinnedStatus(const bool& is_pinned);
        /// <summary>
        /// ���������� ����� �����.
        /// </summary>
        /// <returns>����� �����</returns>
        double GetMass() const;
        /// <summary>
        /// ���������� ������ �������� ��������� �����.
        /// </summary>
        /// <returns>������ �������� ��������� �����</returns>
        Eigen::Vector3d GetAccel() const;
        /// <summary>
        /// ������������� ����� ������ ��������� �����.
        /// </summary>
        /// <param name="acceleration">����� ������ ��������� �����</param>
        void SetAccel(Eigen::Vector3d acceleration);
        /// <summary>
        /// ���������� ������� ������ �������� �����.
        /// </summary>
        /// <returns>������� ������ �������� �����</returns>
        Eigen::Vector3d GetVelocity() const;
        /// <summary>
        /// ������������� ����� ������ �������� �����.
        /// </summary>
        /// <param name="velocity">����� ������ �������� �����</param>
        void SetVelocity(Eigen::Vector3d velocity);
        /// <summary>
        /// ���������� ������� ������� ����� � ������� �������.
        /// </summary>
        /// <returns>������ � ������� �������� �����</returns>
        Eigen::Vector3d GetPos() const;
        /// <summary>
        /// ������������� ����� ������� ����� � ������� �������.
        /// </summary>
        /// <param name="position">������ � ����� �������� �����</param>
        void SetPos(Eigen::Vector3d position);

    private:
        /// <summary>
        /// ����� �������.
        /// </summary>
        double mass_;
        /// <summary>
        /// ������ �������� ��������� �������.
        /// </summary>
        Eigen::Vector3d acceleration_;
        /// <summary>
        /// ������ ������� �������� �������.
        /// </summary>
        Eigen::Vector3d velocity_;
        /// <summary>
        /// ������ ���������� �� �������������� � ����� �������.
        /// </summary>
        std::vector<Spring*> springs_;
        /// <summary>
        /// ������� ������� ����� � ������� �������.
        /// </summary>
        Eigen::Vector3d position_;
        /// <summary>
        /// ������ �������������� �����.
        /// </summary>
        bool is_pinned_ = false;
    };

    /// <summary>
    /// ������������� �����������.
    /// </summary>
    Spring() = default;
    /// <summary>
    /// ���������������� ����������� ������� �������.
    /// </summary>
    /// <param name="point_one">������ �����</param>
    /// <param name="point_two">������ �����</param>
    /// <param name="stiffness">��������� ������� � �.�.</param>
    Spring(Point* point_one, Point* point_two, double stiffness);


    Spring(Spring&) = delete;
    Spring operator=(Spring&) = delete;
    // ��������� �����������

    /// <summary>
    /// ������������� ����������.
    /// </summary>
    ~Spring() = default;

    /// <summary>
    /// ������������ ���� ��������� �� ��������.
    /// </summary>
    /// <param name="base_point">�����, ������������ ������� ���� ���������� ����</param>
    /// <returns>������ ����</returns>
    Eigen::Vector3d ComputeForce(Point* base_point) const;
private:
    /// <summary>
    /// ������ �����.
    /// </summary>
    Point* point_one_;
    /// <summary>
    /// ������ �����.
    /// </summary>
    Point* point_two_;
    /// <summary>
    /// �������� ����� �������.
    /// </summary>
    double length_;
    /// <summary>
    /// ��������� �������.
    /// </summary>
    double stiffness_;
};

#endif