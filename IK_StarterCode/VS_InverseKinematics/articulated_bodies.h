#include "Eigen/Eigen"
#include "Eigen/SVD"
#include <math.h>

using namespace std;
using namespace Eigen;

class ArticulatedBodies {

public:
    const int n; //number of DoFs of the end effector
    int m; //number of joints, assuming each joint has one DoF
    double* angles;
    double* lengths;
    double* e0;
    double* e1;
    double* target_e;

	double* xCoords;
	double* yCoords;
	double* zCoords;

    
    MatrixXd J;
    
    ArticulatedBodies(int m_input);

    ~ArticulatedBodies();

    void updateJacobian();

	void updateJointCoords();

    void updateEndEffector();  

    template<typename _Matrix_Type_>
    bool pseudoInverse(const _Matrix_Type_ &a,
                       _Matrix_Type_ &result);

    void update(float dt);
};
