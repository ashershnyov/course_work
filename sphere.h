#ifndef COLLISION_SPHERE
#define COLLISION_SPHERE

#include <Eigen/Geometry>
#include <Eigen/Core>
#include "spring.h"

/// <summary>
/// Класс объекта сферы.
/// </summary>
class Sphere {
public:
    /// <summary>
    /// Умолчательный конструктор.
    /// </summary>
    Sphere() = default;
    /// <summary>
    /// Основной пользовательский конструктор. Создает сферу радиусом radius в точке origin_pos с плотностью сетки resolution.
    /// </summary>
    /// <param name="radius">Радиус сферы</param>
    /// <param name="origin_pos">Позиция центра сферы</param>
    /// <param name="resolution">Плотность вершин сферы</param>
    Sphere(double radius, Eigen::Vector3d origin_pos, int resolution);
    /*Sphere(double radius, Eigen::Vector3d origin_pos) : radius_(radius), origin_pos_(origin_pos) {
    }
    Sphere(double radius, Eigen::MatrixXd verts, Eigen::MatrixXi faces) : radius_(radius), verts_(verts), faces_(faces) {
    }*/

    //Sphere(Sphere&) = delete;
    //Sphere operator=(Sphere*) = delete;

    /// <summary>
    /// Умолчательный деструктор.
    /// </summary>
    ~Sphere() = default;

    /// <summary>
    /// Разрешитель коллизии сферы с переданной точкой.
    /// </summary>
    /// <param name="point">Точка, с которой необходимо разрешить коллизию</param>
    void ResolveCollision(Spring::Point* point);
    /// <summary>
    /// Обновление позиции сферы.
    /// </summary>
    /// <param name="new_pos">Новая позиция центра сферы</param>
    void UpdatePos(Eigen::Vector3d new_pos);
    /// <summary>
    /// Возвращает позицию центра сферы.
    /// </summary>
    /// <returns>Позиция центра сферы в виде вектора</returns>
    Eigen::Vector3d GetOriginPos();
    /// <summary>
    /// Возвращает матрицу вершин сферы.
    /// </summary>
    /// <returns>Матрица вершин сферы</returns>
    Eigen::MatrixXd GetVerts();
    /// <summary>
    /// Возвращает матрицу треугольников сферы.
    /// </summary>
    /// <returns>Матрица треугольников сферы</returns>
    Eigen::MatrixXi GetFaces();

private:
    /// <summary>
    /// Матрица вершин объекта сферы.
    /// Необходима для визуализации в igl.
    /// </summary>
    Eigen::MatrixXd verts_;
    /// <summary>
    /// Матрица треугольников объекта сферы.
    /// Необходима для визуализации в igl.
    /// </summary>
    Eigen::MatrixXi faces_;
    /// <summary>
    /// Радиус сферы.
    /// </summary>
    double radius_;
    /// <summary>
    /// Координаты центра сферы.
    /// </summary>
    Eigen::Vector3d origin_pos_;
};

#endif