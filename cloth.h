#ifndef MASS_SPRING_CLOTH
#define MASS_SPRING_CLOTH

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include "spring.h"
#include "sphere.h"

/// <summary>
/// Класс объекта ткани.
/// </summary>
class Cloth {
public:
    /// <summary>
    /// Умолчательный конструктор.
    /// </summary>
    Cloth() = default;
    /// <summary>
    /// Пользовательский конструктор. Создает объект ткани шириной width, высотой height, количеством вершин по горизонтали hVerts, количеством вершин по вертикали vVerts, с массой точек mass и жесткостью пружин stiffness. 
    /// </summary>
    /// <param name="hVerts">Количество вершин по вертикали</param>
    /// <param name="vVerts">Количество вершин по горизонтали</param>
    /// <param name="width">Ширина в у.е.</param>
    /// <param name="height">Высота в у.е.</param>
    /// <param name="mass">Масса одной вершины в у.е.</param>
    /// <param name="stiffness">Жесткость пружин в у.е.</param>
    Cloth(const int hVerts, const int vVerts, const double width = 1.0, const double height = 1.0,
        const double mass = 0.3, double stiffness = 80);

    // Запретить копирование

    /// <summary>
    /// Пользовательский деструктор.
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
    /// Обновляет состояние ткани.
    /// Вызывает методы просчета сил. Обновляет значения скоростей и ускорений вершин. Обновляет значения позиций вершин.
    /// </summary>
    void Update();
    /// <summary>
    /// Обновляет позиции вершин в матрице verts_ в соответствии с положением точек из points_.
    /// </summary>
    void UpdatePointsPos();

    /// <summary>
    /// Проверяет произошла ли коллизия между тканью и сферой.
    /// Для каждой точки ткани вызывает разрешитель коллизии сферы.
    /// </summary>
    /// <param name="sphere">Сфера, коллизию с которой надо проверить</param>
    void CheckCollision(Sphere& sphere);
    /// <summary>
    /// Возвращает матрицу вершин объекта ткани.
    /// </summary>
    /// <returns>Матрица вершин объекта ткани</returns>
    Eigen::MatrixXd GetVerts() const;
    /// <summary>
    /// Возвращает матрицу треугольгиков объекта ткани.
    /// </summary>
    /// <returns>Матрица треугольников объекта ткани</returns>
    Eigen::MatrixXi GetFaces() const;

private:
    /// <summary>
    /// Матрица вершин объекта ткани.
    /// Необходима для визуализации в igl.
    /// </summary>
    Eigen::MatrixXd verts_;
    /// <summary>
    /// Матрица треугольников объекта ткани.
    /// Необходима для визуализации в igl.
    /// </summary>
    Eigen::MatrixXi faces_;
    /// <summary>
    /// Массив указателей на объекты вершин, составляющих ткань.
    /// </summary>
    std::vector<Spring::Point*> points_;
    /// <summary>
    /// Массив указателей на объекты пружин, соединяющих вершины.
    /// </summary>
    std::vector<Spring*> springs_;
    /// <summary>
    /// Условный коэффициент жесткости пружин.
    /// </summary>
    double stiffness_;
};

#endif