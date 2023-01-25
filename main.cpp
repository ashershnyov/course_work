#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include "spring.h"
#include "cloth.h"
#include "sphere.h"

int main(int argc, char *argv[])
{

  igl::opengl::glfw::Viewer viewer;
  Cloth a = Cloth(25, 25, 5, 2);
  Sphere b = Sphere(1, Eigen::Vector3d(2, -2, -1), 10);
  viewer.data().set_mesh(a.GetVerts(), a.GetFaces());
  //viewer.data().set_mesh(b.GetVerts(), b.GetFaces());
  //std::cout << b.GetVerts() << "\n";
  //std::cout << "\n" << b.GetFaces() << "\n\n";
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& viewer) -> bool {
      a.Update();
      a.CheckCollision(b);
      a.UpdatePointsPos();
      viewer.data().clear();
      viewer.data().set_mesh(a.GetVerts(), a.GetFaces());
      return false;
  };
  viewer.data().set_face_based(true);
  viewer.core().is_animating = true;
  viewer.core().animation_max_fps = 70;
  viewer.launch();
}
