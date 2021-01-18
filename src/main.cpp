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

    GN* gn = new  GN();

    read_edge(gn->edges_ptr, data_file_e);

    read_vertex(gn->verx_ptr, data_file_v);

    // Matrix3d Aij, Bij;
    // Vector3d err;
    // gn->computeJacoAndError(gn->verx_ptr->at(1), gn->verx_ptr->at(2),gn->edges_ptr->at(1),err,Aij,Bij);
    // cout<<"err: "<<err<<endl;
    // cout<<"Aij："<<Aij<<endl;
    // cout<<"Bij: "<<Bij<<endl;
    // cout<<"e1 info ,matrix\n"<<gn->edges_ptr->at(1).info<<endl;

    cout<<"vertex num:"<<gn->verx_ptr->size()<<endl;
    cout<<"edge num"<<gn->edges_ptr->size()<<endl;
    // gn->linearizeAndSolve();
//
    // return 0;
    gn->printResult();
    cout<<endl;
    gn->solve();
    gn->printResult();
    return 0;

} 
