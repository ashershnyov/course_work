#ifndef MASS_SPRING_SPRING
#define MASS_SPRING_SPRING

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>

/// <summary>
/// Вектор ускорения свободного падения.
/// </summary>
const Eigen::Vector3d kGravityAccel(0, -9.81, 0);

/// <summary>
/// Класс объекта пружины.
/// </summary>
class Spring {
public:
    /// <summary>
    /// Класс объекта точки.
    /// Вложен в класс пружины.
    /// </summary>
    class Point {
    public:
        /// <summary>
        /// Умолчательный конструктор.
        /// </summary>
        Point() = default;
        /// <summary>
        /// Пользовательский конструктор. Создает точку массой mass, в точке position. Опционально возможно задать статус закрепленности is_pinned.
        /// </summary>
        /// <param name="mass">Масса вершины в у.е.</param>
        /// <param name="position">Позиция вершины</param>
        /// <param name="is_pinned">Состояние закрепленности вершины</param>
        Point(double mass, Eigen::Vector3d position, bool is_pinned = false);

        Point(Point&) = delete;
        Point operator=(Point&) = delete;
        // Запретить копирование

        /// <summary>
        /// Умолчательный деструктор.
        /// </summary>
        ~Point() = default;

        /// <summary>
        /// Назначает пружину вершине.
        /// </summary>
        /// <param name="spring">Пружина, которая должна быть связана с вершиной</param>
        void AssignSpring(Spring* spring);
        /// <summary>
        /// Просчитывает силы на точке.
        /// Вызывает просчет сил на пружинах, соединяющих эту точку с другими; учитывает гравитацию.
        /// </summary>
        /// <returns>Вектор силы, действующей на точку.</returns>
        Eigen::Vector3d ComputeForce();

        /// <summary>
        /// Возвращает статус закрепленности точки.
        /// </summary>
        /// <returns>Статус закрепленности точки</returns>
        bool GetPinnedStatus() const;
        /// <summary>
        /// Устанавливает статус закрепленности точки.
        /// </summary>
        /// <param name="is_pinned">Новое значение статуса закрепленности точки</param>
        void SetPinnedStatus(const bool& is_pinned);
        /// <summary>
        /// Возвращает массу точки.
        /// </summary>
        /// <returns>Масса точки</returns>
        double GetMass() const;
        /// <summary>
        /// Возвращает вектор текущего ускорение точки.
        /// </summary>
        /// <returns>Вектор текущего ускорения точки</returns>
        Eigen::Vector3d GetAccel() const;
        /// <summary>
        /// Устанавливает новый вектор ускорения точки.
        /// </summary>
        /// <param name="acceleration">Новый вектор ускорения точки</param>
        void SetAccel(Eigen::Vector3d acceleration);
        /// <summary>
        /// Возвращает текущий вектор скорости точки.
        /// </summary>
        /// <returns>Текущий вектор скорости точки</returns>
        Eigen::Vector3d GetVelocity() const;
        /// <summary>
        /// Устанавливает новый вектор скорости точки.
        /// </summary>
        /// <param name="velocity">Новый вектор скорости точки</param>
        void SetVelocity(Eigen::Vector3d velocity);
        /// <summary>
        /// Возвращает текущую позицию точки в формате вектора.
        /// </summary>
        /// <returns>Вектор с текущей позицией точки</returns>
        Eigen::Vector3d GetPos() const;
        /// <summary>
        /// Устанавливает новую позицию точки в формате вектора.
        /// </summary>
        /// <param name="position">Вектор с новой позицией точки</param>
        void SetPos(Eigen::Vector3d position);

    private:
        /// <summary>
        /// Масса вершины.
        /// </summary>
        double mass_;
        /// <summary>
        /// Вектор текущего ускорения вершины.
        /// </summary>
        Eigen::Vector3d acceleration_;
        /// <summary>
        /// Вектор текущей скорости вершины.
        /// </summary>
        Eigen::Vector3d velocity_;
        /// <summary>
        /// Массив указателей на присоединенные к точке пружины.
        /// </summary>
        std::vector<Spring*> springs_;
        /// <summary>
        /// Текущая позиция точки в формате вектора.
        /// </summary>
        Eigen::Vector3d position_;
        /// <summary>
        /// Статус закрепленности точки.
        /// </summary>
        bool is_pinned_ = false;
    };

    /// <summary>
    /// Умолчательный конструктор.
    /// </summary>
    Spring() = default;
    /// <summary>
    /// Пользовательский конструктор объекта пружины.
    /// </summary>
    /// <param name="point_one">Первая точка</param>
    /// <param name="point_two">Вторая точка</param>
    /// <param name="stiffness">Жесткость пружины в у.е.</param>
    Spring(Point* point_one, Point* point_two, double stiffness);


    Spring(Spring&) = delete;
    Spring operator=(Spring&) = delete;
    // Запретить копирование

    /// <summary>
    /// Умолчательный деструктор.
    /// </summary>
    ~Spring() = default;

    /// <summary>
    /// Просчитывает силы упругости на пружинах.
    /// </summary>
    /// <param name="base_point">Точка, относительно которой надо просчитать силы</param>
    /// <returns>Вектор силы</returns>
    Eigen::Vector3d ComputeForce(Point* base_point) const;
private:
    /// <summary>
    /// Первая точка.
    /// </summary>
    Point* point_one_;
    /// <summary>
    /// Вторая точка.
    /// </summary>
    Point* point_two_;
    /// <summary>
    /// Исходная длина пружины.
    /// </summary>
    double length_;
    /// <summary>
    /// Жесткость пружины.
    /// </summary>
    double stiffness_;
};

#endif