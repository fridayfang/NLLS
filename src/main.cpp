#include <DataType.hpp>
#include <iostream>
#include <fstream>
#include <memory>
#include <Gauss_Newton.hpp>
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
using namespace std;
using namespace Eigen;

void read_edge(Edgs_ptr& e_ptr,string& data_file_e){

    string name_e;
    int idi,idj;
    double dx,dy,dt;
    double info_xx,info_xy,info_yy,info_tt,info_xt,info_yt;
    
    ifstream fo(data_file_e);
    int num_e = 0;
    if(fo.is_open()){
        while(fo>>name_e>>idi>>idj>>dx>>dy>>dt>>info_xx>>info_xy>>info_yy>>info_tt>>info_xt>>info_yt){
            num_e++;
            if(num_e>=1){
                cout<<"reach to 6"<<endl;
            cout<<name_e<<","<<idi<<","<<idj<<","<<dx<<","<<dy<<","<<dt<<","<<info_xx<<","<<info_xy<<","<<info_yy<<","<<info_tt<<","<<info_xt<<","<<info_yt<<endl;
            }
            Edge e;
            e.xi = idi, e.xj = idj;
            e.measure = Vector3d(dx,dy,dt);
            e.info<<info_xx,info_xy,info_xt,
                info_xy,info_yy,info_yt,
                info_xt,info_yt,info_tt;
           e_ptr->push_back(e); 

        }
    }

}
void read_vertex(Verts_ptr& v_ptr, string& data_file_v){

    string name_v;
    int id;
    double dx,dy,dt;
    ifstream fo(data_file_v);
    int num_v = 0;
    if(fo.is_open()){
        while(fo>>name_v>>id>>dx>>dy>>dt){
            num_v++;
            Vertex v;
            v.id = id;
            v.estimate = Vector3d(dx,dy,dt);
            v_ptr->push_back(v);
        }
    }
}
int main(){

    string data_file_v = "../data/test_quadrat-v.dat";
    string data_file_e = "../data/test_quadrat-e.dat";

    // string name_e;
    // int idi,idj;
    // double dx,dy,dt;
    // double info_xx,info_xy,info_yy,info_tt,info_xt,info_yt;
//
    // ifstream fo(data_file_e);
    // int num_e = 0;
    // if(fo.is_open()){
        // while(!fo.eof()){
            // fo>>name_e>>idi>>idj>>dx>>dy>>dt>>info_xx>>info_xy>>info_yy>>info_tt>>info_xt>>info_yt;
            // num_e++;
            // if(num_e==3){
                // cout<<name_e<<","<<idi<<","<<idj<<","<<dx<<endl;
            // }
        // }
    // }





    Vertex v1, v2;
    v1.id = 0;
    v1.estimate = Eigen::Vector3d(0.0,0.0,0.0);
    v2.id = 1;
    v2.estimate = Eigen::Vector3d(1.0,1.0, M_PI/2.0);
    Eigen::Vector3d v3d(0,0,0);
    Eigen::Matrix3d m3d = Eigen::Matrix3d::Identity();
    Eigen::Matrix2d m2d = Eigen::Matrix2d::Identity();

    Edge e12;
    e12.xi = 0, e12.xj=1, e12.info = Eigen::Matrix3d::Identity();
    e12.measure = Eigen::Vector3d(1.0,1.0,M_PI/3.0);
    GN* gn = new  GN();

    read_edge(gn->edges_ptr, data_file_e);

    read_vertex(gn->verx_ptr, data_file_v);

    cout<<"vertex num:"<<gn->verx_ptr->size()<<endl;
    cout<<"edge num"<<gn->edges_ptr->size()<<endl;
    return 0;

    gn->verx_ptr->push_back(v1);
    gn->verx_ptr->push_back(v2);
    gn->edges_ptr->push_back(e12);

    Vector3d err;
    gn->computeError(gn->verx_ptr->at(0), gn->verx_ptr->at(1),gn->edges_ptr->at(0),err);
    Matrix3d Aij = Matrix3d::Identity();
    Matrix3d Bij = Matrix3d::Identity();
    gn->computeJacoAndError(gn->verx_ptr->at(0), gn->verx_ptr->at(1), gn->edges_ptr->at(0),err,Aij,Bij);
    cout<<Aij<<endl;
    cout<<Bij<<endl;
    // gn->computeError(v1,v2,e12,err);
    cout<<err<<endl;
    return 0;
} 
