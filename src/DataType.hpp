#ifndef DATATYPE_HPP
#define DATATYPE_HPP

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <cmath>
using namespace std;
typedef struct vertex{
    int id;
    Eigen::Vector3d estimate;
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}Vertex;


typedef struct edge{
    int xi,xj;
    Eigen::Vector3d measure;
    Eigen::Matrix3d info; // 信息矩阵，可以用单位矩阵
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}Edge;
typedef vector<Vertex> Verts;
typedef vector<edge> Edgs;
typedef unique_ptr<Verts> Verts_ptr;
typedef unique_ptr<Edgs> Edgs_ptr;
#endif
