#pragma once
#include "eigen_kdtree.h"
#include "Eigen/Geometry"
#include <iostream>
#include <list>
#include <memory>

using Vector2f = Eigen::Vector2f;

class LASERM {
protected:
  struct PointPair{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointPair(const Vector2f& fixed_, const Vector2f& moving_):
      _fixed(fixed_),
      _moving(moving_){};
    
    PointPair(){}
    Vector2f _fixed;
    Vector2f _moving;
  };
  using PointPairVector=std::vector<PointPair, Eigen::aligned_allocator<PointPair>>;

public:
  using ContainerType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;

  LASERM(const int& size,
      int min_points_in_leaf);

  void computeCorrespondences();
  
  // for me to test
  void computeCorrespondencesFake();
  
  void optimizeCorrespondences();
  
  void run(int max_iterations);

  void draw(std::ostream& os);
  
  const Eigen::Isometry2f& X() const {return _X;}
  Eigen::Isometry2f& X()  {return _X;}

  //dr: The pose of the base frame, in some fixed (world) frame.
  const Eigen::Isometry2f& TB() const {return _TB;}
  Eigen::Isometry2f& TB()  {return _TB;}
  void updateTB() {_TB=_TB*_X;};
  const ContainerType& old() const {return _fixed;}
  ContainerType& old()  {return _fixed;}
  const ContainerType& niu() const {return _moving;}
  ContainerType& niu()  {return _moving;}
  void updateOld(){_fixed.swap(_moving);} //dr: swap is performed in constant time
  void setSet(const int& id, const int& idx, Eigen::Vector2f value){
    if(id)
      _moving[idx]=value;
    else
      _fixed[idx]=value;
  }

  //dr:end


  inline int numCorrespondences() const {return _correspondences.size();}
  inline int numKernelized() const {return _num_kernelized;}
  inline int numInliers() const {return _num_inliers;}
  inline const Eigen::Matrix<float, 3,1>& dx() const {return _dx;}
 
protected:
  using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
  
  ContainerType _fixed;
  ContainerType _moving;
  Eigen::Isometry2f _X=Eigen::Isometry2f::Identity();
  //dr: The pose of the base frame, in some fixed (world) frame. To be precise we need also to get the base_link → laser to bring from laser frame to base frame the computed _X, for this program just considered both with the same origin.
  Eigen::Isometry2f _TB=Eigen::Isometry2f::Identity();
  TreeNodeType _kd_tree;
  float _ball_radius=10.f;
  float _kernel_chi2 = 1.f;
  float _chi2_sum=0;

  PointPairVector _correspondences;
  int _num_kernelized=0;
  int _num_inliers=0;
  Eigen::Matrix<float, 3,1> _dx;
};
