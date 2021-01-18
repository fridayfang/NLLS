#ifndef GAUSS_NEWTON_HPP
#define GAUSS_NEWTON_HPP

#include <DataType.hpp>
#include <Eigen/LU>
#include <iostream>
//针对平面运动的非线性最小二乘
// GN 实现的是Gauss-Newton 法
using namespace std;
using namespace Eigen;
class GN{
    public:
        Verts_ptr verx_ptr;
        Edgs_ptr edges_ptr;// 访问节点和边的unique_ptr

        GN(){
            verx_ptr = unique_ptr<Verts>(new Verts());
            edges_ptr = unique_ptr<Edgs>(new Edgs());
        }
        Matrix3d v2t(const Vector3d& vec){
            Matrix3d trans;
            trans<<cos(vec(2)), (-sin(vec(2))), vec(0),
                  sin(vec(2)), cos(vec(2)), vec(1),
                  0,0,1;
            return trans;
            // return a copy of trans
        }
        

        void computeError(const Vertex& vi, const Vertex& vj, const Edge& eij, Eigen::Vector3d& error_ij){
            error_ij(2) = vj.estimate(2)-vi.estimate(2)-eij.measure(2);
            
            Matrix3d mi = v2t(vi.estimate);
            Matrix3d mj = v2t(vj.estimate);
            Matrix3d mij = v2t(eij.measure);

            Vector2d err_t= mij.block(0,0,2,2).transpose()*(mi.block(0,0,2,2).transpose()*(vj.estimate.head(2)-vi.estimate.head(2)) - eij.measure.head(2));
            error_ij.head(2) = err_t;

        }

        void computeJacoAndError(const Vertex& vi, const Vertex& vj, const Edge& eij,Vector3d& error_ij, Matrix3d& Aij, Matrix3d& Bij){
            
            error_ij(2) =  vj.estimate(2)-vi.estimate(2)-eij.measure(2);
            bound(error_ij(2));
            
            Matrix3d mi = v2t(vi.estimate);
            Matrix3d mj = v2t(vj.estimate);
            Matrix3d mij = v2t(eij.measure);

            Vector2d err_t= mij.block(0,0,2,2).transpose()*(mi.block(0,0,2,2).transpose()*(vj.estimate.head(2)-vi.estimate.head(2)) - eij.measure.head(2));
            error_ij.head(2) = err_t;

            Aij.block(0,0,2,2) = -mij.block(0,0,2,2).transpose()*mi.block(0,0,2,2).transpose();
            Matrix2d dmi;
            dmi<<mi(0,1),(-mi(0,0)),mi(0,0),mi(0,1);
            Aij.block(0,2,2,1) = mij.block(0,0,2,2)*dmi.transpose()*(mj.block(0,2,2,1) - mi.block(0,2,2,1));
            Aij(2,2) = -1, Aij(2,0) = 0, Aij(2,1) = 0;
            // compute Bij
            //
            Bij = Matrix3d::Identity();
            Bij.block(0,0,2,2) = mij.block(0,0,2,2).transpose()*mi.block(0,0,2,2).transpose();
        }

        VectorXd linearizeAndSolve(){
            MatrixXd H(verx_ptr->size()*3, verx_ptr->size()*3);
            VectorXd b(verx_ptr->size()*3);
            H.setZero();
            b.setZero();

            H.block(0,0,3,3) += Matrix3d::Identity();//fix firstb frame

            for(int i = 0; i<edges_ptr->size();i++){
                Edge eij =  edges_ptr->at(i);
                Vertex vi = verx_ptr->at(eij.xi);
                Vertex vj = verx_ptr->at(eij.xj);
                Vector3d err_ij(0,0,0);
                Matrix3d Aij = Matrix3d::Identity();
                Matrix3d Bij = Matrix3d::Identity();

                computeJacoAndError(vi,vj,eij,err_ij,Aij,Bij);

                Matrix3d Hii = Aij.transpose()*eij.info*Aij;
                Matrix3d Hij = Aij.transpose()*eij.info*Bij;
                Matrix3d Hji = Bij.transpose()*eij.info*Aij;
                Matrix3d Hjj = Bij.transpose()*eij.info*Bij;

                H.block(3*eij.xi,3*eij.xi,3,3) += Hii;
                H.block(3*eij.xi,3*eij.xj,3,3) += Hij;
                H.block(3*eij.xj,3*eij.xi,3,3) += Hji;
                H.block(3*eij.xj,3*eij.xj,3,3) +=Hjj;
                
                // b.segment(3*eij.xi,3) += ( err_ij.transpose()*eij.info*Aij ).transpose();
                // b.segment(3*eij.xj,3) += ( err_ij.transpose()*eij.info*Bij ).transpose();

                Vector3d bi,bj;
                bi = ( err_ij.transpose()*eij.info*Aij ).transpose();
                bj = (err_ij.transpose()*eij.info*Bij).transpose();
                    
                b(3*eij.xi)+=bi(0);
                b(3*eij.xi+1)+=bi(1);
                b(3*eij.xi+2)+=bi(2);

                b(3*eij.xj)+=bj(0);
                b(3*eij.xj+1)+=bj(1);
                b(3*eij.xj+2)+=bj(2);

                if(i==2){
                    cout<<"Aij"<<Aij<<endl;
                    cout<<"Bij"<<Bij<<endl;
                    cout<<"err_ij"<<err_ij<<endl;
                    cout<<"H"<<H<<endl;
                    cout<<"b"<<b<<endl;
                }
            // cout<<"H:\n"<<H<<endl;
            // cout<<"b:"<<b<<endl;


            }
            // get H,b 
            // return dx
            cout<<"H:\n"<<H<<endl;
            cout<<"b:"<<b<<endl;

            VectorXd dx = -H.lu().solve(b);
            return dx;
        }

        void bound(double& theta){
            while(theta>M_PI){
                theta-=M_PI*2;
                
            }
            while(theta<=-M_PI){
                theta+=M_PI*2;
            }
        }
        void solve(){
            VectorXd dx = linearizeAndSolve();
            cout<<"dx:\n"<<dx<<endl;
            int iters = 0;
            while(iters<100 && abs( dx.mean() )>0.01)
            {
                for(int i=0;i<verx_ptr->size(); i++){
                    verx_ptr->at(i).estimate += dx.segment(i*3,3);
                    
                    bound( verx_ptr->at(i).estimate(2) );
                }
                dx = linearizeAndSolve();
                iters++;
            }
            cout<<"after "<<iters<<" end"<<endl;
            cout<<"dx.sum() = "<<dx.sum()<<endl;

        }
        void printResult(){
            for(int i=0;i<verx_ptr->size();i++){
               cout<<verx_ptr->at(i).estimate(0)<<","<<verx_ptr->at(i).estimate(1)<<","<<verx_ptr->at(i).estimate(2)<<endl;
            }
        }

};


#endif
