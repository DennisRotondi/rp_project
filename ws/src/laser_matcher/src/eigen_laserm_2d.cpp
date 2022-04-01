#include "eigen_laserm_2d.h"
#include "Eigen/Geometry"
#include "rotations.h"
#include "Eigen/Cholesky"
#include <iostream>
#include <fstream>

using namespace std;
using ContainerType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;
using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
LASERM::LASERM(const int& size,
         int min_points_in_leaf, Eigen::Isometry2f TMF_,Eigen::Isometry2f TBF_,const int& draw): 
         _fixed(size),_moving(size), 
         _min_points_in_leaf(min_points_in_leaf), _TMF(TMF_),_TBF(TBF_),_draw(draw){
  _correspondences.reserve(std::max(_fixed.size(), _moving.size()));
}

void LASERM::computeCorrespondences() {
  int k=0;
  _correspondences.resize(_moving.size());
  for (const auto& m: _moving) {
    const auto& mt=_X*m;
    auto ft=_kd_tree->bestMatchFast(mt, _ball_radius);
    if (! ft)
      continue;
    _correspondences[k]._fixed=*ft;
    _correspondences[k]._moving=mt;
    ++k;
  }
  _correspondences.resize(k);
}
 

void LASERM::optimizeCorrespondences() {
  Eigen::Matrix<float, 3, 3> H;
  Eigen::Matrix<float, 3, 1> b;
  H.setZero();
  b.setZero();
  Eigen::Matrix<float, 2, 3> J;
  J.block<3,3>(0,0).setIdentity();
  _num_kernelized=0;
  _num_inliers=0;
  _chi2_sum=0;
  for (const auto& c: _correspondences) {
    const auto& f=c._fixed;
    const auto& m=c._moving;
    J(0,2) = -m.y();
    J(1,2) = m.x();
    Vector2f e=m-f;
    float scale=1;
    float chi=e.squaredNorm();
    _chi2_sum+=chi;
    if (e.squaredNorm()>_kernel_chi2) {
      scale=sqrt(_kernel_chi2/chi);
      ++_num_kernelized;
    } else {
      ++_num_inliers;
    }
    H.noalias()+= scale* J.transpose()*J;
    b.noalias()+= scale* J.transpose()*e;
  }
  _dx=H.ldlt().solve(-b);
  Eigen::Isometry2f dX;
  const Eigen::Matrix2f dR=Rtheta(_dx(2));
  dX.setIdentity();
  dX.linear()=dR;
  dX.translation()=_dx.block<2,1>(0,0);
  _X=dX*_X;
  cerr << "_X computed" << endl;
  cerr << _X.matrix() << endl;
}

void LASERM::run(int max_iterations) {
  //dr: create now kd_tree and not during construction, otherwise nothing works
  _kd_tree.reset(); //dr: reset the pointer
  _kd_tree = std::unique_ptr<TreeNodeType>(new TreeNodeType(_fixed.begin(), _fixed.end(), _min_points_in_leaf));
  _X=Eigen::Isometry2f::Identity();
  int current_iteration=0;
  while (current_iteration<max_iterations) {
    computeCorrespondences();
    optimizeCorrespondences();
    //dr: relative draw of those two sets
    if(_draw==1) draw(cout);
    ++current_iteration;
    cerr << "Iteration: " << current_iteration;
    cerr << " corr: " << numCorrespondences();
    cerr << " inl: " << numInliers();
    cerr << " ker: " << numKernelized();
    cerr << " chi: " << _chi2_sum << endl;
  }
}

void LASERM::draw(std::ostream& os) {
  os << "set size 1,1" << endl;
  os <<"set xzeroaxis"<< endl;
  os <<"set xtics axis"<< endl;
  os <<"set xrange [-15:15]"<< endl;
  os <<"set arrow 1 from -15,0 to -15,0"<< endl;
  os <<"set arrow 2 from  15,0 to  15,0"<< endl;
  os <<"set yzeroaxis"<< endl;
  os <<"set ytics axis"<< endl;
  os <<"set yrange [-10:10]"<< endl;
  os <<"set arrow 3 from 0,-10,0 to 0,-10"<< endl;
  os <<"set arrow 4 from 0,10,0  to 0,10"<< endl;
  os <<"set border 0"<< endl;
  os << "plot '-' w p ps 2 title \"fixed\", '-' w p ps 2 title \"moving\", '-' w l lw 1 title \"correspondences\" " << endl;
  for  (const auto& p: _fixed)
    os << (_TMF*p).transpose() << endl;
  os << "e" << endl;
  for  (const auto& p: _moving)
    os << (_TMF*_X*p).transpose() << endl;
  os << "e" << endl;
  for (const auto& c: _correspondences) {
    os << (_TMF*c._fixed).transpose() << endl;
    os << (_TMF*_X*c._moving).transpose() << endl;
    os << endl;
  }
  os << "e" << endl;
}