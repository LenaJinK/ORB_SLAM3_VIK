
#include <eigen_conversions/eigen_kdl.h>

namespace tf {

void quaternionKDLToEigen(const KDL::Rotation &k, Eigen::Quaterniond &e)
{
  // // is it faster to do this?
  k.GetQuaternion(e.x(), e.y(), e.z(), e.w());
  
  // or this?
  //double x, y, z, w;
  //k.GetQuaternion(x, y, z, w);
  //e = Eigen::Quaterniond(w, x, y, z);
}

void quaternionEigenToKDL(const Eigen::Quaterniond &e, KDL::Rotation &k)
{
  k = KDL::Rotation::Quaternion(e.x(), e.y(), e.z(), e.w());  
}

namespace {
  template<typename T>
  void transformKDLToEigenImpl(const KDL::Frame &k, T &e)
  {
    // translation
    for (unsigned int i = 0; i < 3; ++i)
      e(i, 3) = k.p[i];

    // rotation matrix
    for (unsigned int i = 0; i < 9; ++i)
      e(i/3, i%3) = k.M.data[i];

    // "identity" row
    e(3,0) = 0.0;
    e(3,1) = 0.0;
    e(3,2) = 0.0;
    e(3,3) = 1.0;
  }

  template<typename T>
  void transformEigenToKDLImpl(const T &e, KDL::Frame &k)
  {
    for (unsigned int i = 0; i < 3; ++i)
      k.p[i] = e(i, 3);
    for (unsigned int i = 0; i < 9; ++i)
      k.M.data[i] = e(i/3, i%3);
  }

}

void transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e)
{
  transformKDLToEigenImpl(k, e);
}

void transformKDLToEigen(const KDL::Frame &k, Eigen::Isometry3d &e)
{
  transformKDLToEigenImpl(k, e);
}

void transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k)
{
  transformEigenToKDLImpl(e, k);
}

void transformEigenToKDL(const Eigen::Isometry3d &e, KDL::Frame &k)
{
  transformEigenToKDLImpl(e, k);
}

void twistEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Twist &k)
{
  for(int i = 0; i < 6; ++i)
    k[i] = e[i];
}

void twistKDLToEigen(const KDL::Twist &k, Eigen::Matrix<double, 6, 1> &e)
{
  for(int i = 0; i < 6; ++i)
    e[i] = k[i];
}

void vectorEigenToKDL(const Eigen::Matrix<double, 3, 1> &e, KDL::Vector &k)
{
  for(int i = 0; i < 3; ++i)
    k[i] = e[i];
}
void vectorKDLToEigen(const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e)
{
  for(int i = 0; i < 3; ++i)
    e[i] = k[i];
}

void wrenchKDLToEigen(const KDL::Wrench &k, Eigen::Matrix<double, 6, 1> &e)
{
  for(int i = 0; i < 6; ++i)
    e[i] = k[i];
}

void wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1> &e, KDL::Wrench &k)
{
  for(int i = 0; i < 6; ++i)
    k[i] = e[i];
}


} // namespace
