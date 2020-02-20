#ifndef TUTORIAL_SANDBOX_MESH_DATA_STRUCT_H
#define TUTORIAL_SANDBOX_MESH_DATA_STRUCT_H


#include <igl/edge_flaps.h>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <vector>


#pragma once
class Mesh_Data_Struct
{
public:
	Eigen::MatrixXd V, OV;
	Eigen::MatrixXd F_normals;
	Eigen::MatrixXi F, OF;
	// Prepare array-based edge data structures and priority queue
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	std::vector<Eigen::Matrix4d> *QV = new std::vector<Eigen::Matrix4d>();
	typedef std::set<std::pair<double, int> > PriorityQueue;
	PriorityQueue* Q = new PriorityQueue();
	std::vector<PriorityQueue::iterator >* Qit = new std::vector<PriorityQueue::iterator>();
	// If an edge were collapsed, we'd collapse it to these points:
	Eigen::MatrixXd C;
	int num_collapsed;
	Mesh_Data_Struct(std::string filename, Eigen::MatrixXd & data_F_normals); //constructor
	~Mesh_Data_Struct(); //destructor

};
#endif // !Mesh_Data_Struct_HEADER