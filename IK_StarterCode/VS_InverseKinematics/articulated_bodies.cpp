#include "articulated_bodies.h"
#include <iostream>

ArticulatedBodies::ArticulatedBodies(int m_input)
    : n(3), m(m_input)
{
    angles = new double[m];
    lengths = new double[m];

	xCoords = new double[m+1];
	yCoords = new double[m+1];
	zCoords = new double[m+1];

    
    e0 = new double[n];
    e1 = new double[n];
    target_e = new double[n];
    
    J.resize(n,m);

}

ArticulatedBodies::~ArticulatedBodies(){
    delete[] target_e;
    delete[] e1;
    delete[] e0;
    delete[] lengths;
    delete[] angles;
	delete[] xCoords;
	delete[] yCoords;
	delete[] zCoords;


}


void crossProduct(double* a, double* b, double* result) {
	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0]; 
}


//finds coords for endpoint
void ArticulatedBodies::updateJointCoords() {
	double rotationAngle = 0;
	e0[0] = 0;
	e0[1] = 0;
	e0[2] = 0;
	e0[3] = 0;
	xCoords[0] = 0;
	yCoords[0] = 0;
	for (unsigned int i = 0; i < m; ++i) {
		double startX = xCoords[i];
		double startY = yCoords[i];
		rotationAngle += angles[i];
		xCoords[i+1] = startX + lengths[i] * cos(rotationAngle);
		yCoords[i+1] = startY + lengths[i] * sin(rotationAngle);

		//cout << "Coord " << i << " " << xCoords[i] << "," << yCoords[i] << endl;
	}
	e1[0] = xCoords[m];
	e1[1] = yCoords[m];
	e1[2] = 0;

	//cout << "E1 " << e1[0] << "," << e1[1] << endl;
	
}

void ArticulatedBodies::updateJacobian(){

	
	double zUnit[3] = { 0, 0, 1 };
	
	/*
	for (int i = 0; i < m; ++i) {
		std::cout << angles[m] << std::endl;
		std::cout << lengths[m] << std::endl;
	}
	*/ 
    for(int i=0; i<n; i++){
        for(int j=0; j<m; j++){
			J(i, j) = 0;
        }
    }

	updateJointCoords(); 
	
	for (int col = 0; col<m; col++) {
		double effectorMinusJoint[3];
		double currCoord[3];
		
		currCoord[0] = xCoords[col];
		currCoord[1] = yCoords[col];
		currCoord[2] = 0;
		

		for (int dim = 0; dim<3; dim++) {
			effectorMinusJoint[dim] = e1[dim] - currCoord[dim];
			cout << effectorMinusJoint[dim] << endl;
		}
		double* colValue = new double[n];
		crossProduct(zUnit, effectorMinusJoint, colValue);
		
		for (int row = 0; row < n; row++) {
			J(row, col) = colValue[row];
		}
		delete colValue; 
	}

    
    ////CS248 TODO: build the Jacobian matrix

}

void ArticulatedBodies::updateEndEffector(){
    ////CS248 TODO: update the end effector

	updateJointCoords();
	
	/*
	e1[0] = -10;
	e1[1] = -10;
	e1[2] = -10;
	*/ 

}	

template<typename _Matrix_Type_>
bool ArticulatedBodies::pseudoInverse(const _Matrix_Type_ &a,
                                      _Matrix_Type_ &result)
{
    double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon();

    if(a.rows()<a.cols())
        return false;

    Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeFullU |
        Eigen::ComputeFullV);

    typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(),
        a.rows()) * svd.singularValues().array().abs().maxCoeff();

    result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() >
        tolerance).select(svd.singularValues().
        array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
    return true;
}

void ArticulatedBodies::update(float dt){
    double dx = target_e[0]-e1[0];
    double dy = target_e[1]-e1[1];
    double dz = target_e[2]-e1[2];
    
    double distance = sqrt(dx*dx+dy*dy+dz*dz);
    double v = .1;
    double threshold = v*dt;
    
    ////update dg
    if(distance>threshold){
        dx = dx/distance*v;
        dy = dy/distance*v;
        dz = dz/distance*v;
    }
    
    ////update J
    updateJacobian();

    ////update pseudoInverseJ
    MatrixXd JT = J.transpose();
    MatrixXd JJT = J*JT;
    //cout<<"JJT is: "<<endl<<JJT<<endl;
    MatrixXd inverseJJT;
    pseudoInverse(JJT,inverseJJT);
    //cout<<"inverseJJT is: "<<endl<<inverseJJT<<endl;
    MatrixXd pseudoInverseJ = JT*inverseJJT;
    //cout<<"pseudoInverseJJT is: "<<endl<<pseudoInverseJ<<endl;
    
    MatrixXd d_g(n,1);
    d_g(0,0)=dx;d_g(1,0)=dy;d_g(2,0)=dz;
    MatrixXd d_theta = pseudoInverseJ*d_g;
    //cout<<"d_theta is: "<<d_theta<<endl;
    
    ////update angles
    for(int i=0; i<m; i++){
        double dangle = d_theta(i, 0);
        angles[i] += dangle;
    }
    
    ////update end effector
    updateEndEffector();
}
