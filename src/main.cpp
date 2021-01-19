#include <DataType.hpp>
#include <iostream>
#include <fstream>
#include <memory>
#include <Gauss_Newton.hpp>
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <pangolin/pangolin.h>
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

Isometry3d d2tod3(const Vector3d& pose2d){
    Isometry3d t = Isometry3d::Identity();
    AngleAxisd rotation_vector(pose2d(2), Vector3d(0,0,1));
    t.prerotate(rotation_vector.toRotationMatrix());

    t.pretranslate(Vector3d(pose2d(0),pose2d(1),0));
    return t;
}
void DrawTrajectory(vector<Vector3d, Eigen::aligned_allocator<Vector3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(5,5,200, 0, 0, 0, pangolin::AxisNegZ)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++) {
      // 画每个位姿的三个坐标轴
      Isometry3d pose = d2tod3(poses[i]);
      Vector3d Ow = pose.translation();
      cout<<"position:"<<Ow<<endl;
      Vector3d Xw = pose * (1.5 * Vector3d(1, 0, 0));
      Vector3d Yw = pose * (1.5 * Vector3d(0, 1, 0));
      Vector3d Zw = pose * (1.5 * Vector3d(0, 0, 1));
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();
    }
    // 画出连线
    for (size_t i = 0; i < poses.size()-1; i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = d2tod3( poses[i] ), p2 = d2tod3( poses[i + 1] );
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
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

    vector<Vector3d, Eigen::aligned_allocator<Vector3d>> poses;
    for(int i=0;i<gn->verx_ptr->size();i++){
        poses.push_back(gn->verx_ptr->at(i).estimate);
    }
    DrawTrajectory(poses);
    return 0;

} 
