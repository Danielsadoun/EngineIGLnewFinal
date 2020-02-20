#include "Mesh_Data_Struct.h"
#include "igl/circulation.h"
#include "igl/vertex_triangle_adjacency.h"
#include <Eigen/LU>

Mesh_Data_Struct::Mesh_Data_Struct(std::string filename, Eigen::MatrixXd& data_F_normals)
{
	igl::read_triangle_mesh(filename, OV, OF);
	V = OV;
	F = OF;
	F_normals = data_F_normals;
	igl::edge_flaps(F, E, EMAP, EF, EI);
	Qit->resize(E.rows());

	C.resize(E.rows(), V.cols());
	
	Q->clear();
	QV->resize(V.rows());

	std::vector<std::vector<int>> VF;
	std::vector<std::vector<int>> VFi;

	igl::vertex_triangle_adjacency(V, F, VF, VFi);

	for (int v = 0; v < V.rows(); v++) {
		(*QV)[v] = Eigen::Matrix4d::Zero();
		for (int f = 0; f < VF[v].size(); f++) {
			Eigen::Vector3d norm = F_normals.row(VF[v][f]).normalized();
			double d = V.row(v) * norm;
			double a = norm[0];
			double b = norm[1];
			double c = norm[2];
			d *= -1;
			Eigen::Matrix4d Kp;
			Kp.row(0) = Eigen::Vector4d(a * a, a * b, a * c, a * d);
			Kp.row(1) = Eigen::Vector4d(a * b, b * b, b * c, b * d);
			Kp.row(2) = Eigen::Vector4d(a * c, c * b, c * c, c * d);
			Kp.row(3) = Eigen::Vector4d(a * d, d * b, d * c, d * d);
			(*QV)[v] += Kp;
		}
	}
	
	for (int e = 0; e < E.rows(); e++)
		{
		double cost = e;
		Eigen::RowVector3d p;

		Eigen::Matrix4d QUAD = (*QV)[E(e,0)] + (*QV)[E(e, 1)];
		Eigen::Matrix4d QUADp = Eigen::Matrix4d::Identity();
		QUADp.row(0) = QUAD.row(0);
		QUADp.row(1) = QUAD.row(1);
		QUADp.row(2) = QUAD.row(2);
		//QUADp.block(0, 0, 3, 4) = QUAD3.block(0, 0, 3, 4);
		Eigen::Vector4d temp;
		if (QUADp.fullPivLu().isInvertible()) {
			temp = QUADp.inverse() * (Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
			p[0] = temp[0];
			p[1] = temp[1];
			p[2] = temp[2];
			
		}
		else {
			p = V.row(E(e, 0));
			temp[0] = p[0];
			temp[1] = p[1];
			temp[2] = p[2];
			temp[3] = 1.0;

		}

		cost = temp.transpose() * QUAD * temp;
	   

		C.row(e) = p;
		(*Qit)[e] = Q->insert(std::pair<double, int>(cost, e)).first;
	}
	num_collapsed = 0;

}

Mesh_Data_Struct:: ~Mesh_Data_Struct() {}

