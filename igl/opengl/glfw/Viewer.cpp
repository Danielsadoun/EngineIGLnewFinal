// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  IGL_INLINE void Viewer::init()
  {
   

  }

  //IGL_INLINE void Viewer::init_plugins()
  //{
  //  // Init all plugins
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->init(this);
  //  }
  //}

  //IGL_INLINE void Viewer::shutdown_plugins()
  //{
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->shutdown();
  //  }
  //}

  IGL_INLINE Viewer::Viewer(int num_of_cyl):
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	is_picked(false),
	num_of_cyl(num_of_cyl),
    last_picked(-1),
    scoring(0),
    last_scoring(0)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string, 
      Eigen::Vector3f direction)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      data().set_uv(UV_V,UV_F);
      data().direction = direction;

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
	
	Mesh_Data_Struct s(mesh_file_name_string, data().F_normals);//put in selected_data_index in struct_list new Mesh_Data_Struct which is the struct of data() 
	struct_list.push_back(s);
	igl::AABB<Eigen::MatrixXd, 3> tree;
	tree.init(data().V, data().F);
	tree_list.push_back(tree);

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;

    this->load_mesh_from_file(fname.c_str(), Eigen::Vector3f(0,0,0));
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible){
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //}
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  bool Viewer::simplificate(double edges_to_delete) {
	  bool something_collapsed = false;
	  std::vector<Eigen::Matrix4d> QV = *struct_list[selected_data_index].QV;
	  bool to_update = true;

	  const auto& my_cost_and_placement = [&QV, &to_update](
		  const int e,
		  const Eigen::MatrixXd& V,
		  const Eigen::MatrixXi& F,
		  const Eigen::MatrixXi& E,
		  const Eigen::VectorXi& EMAP,
		  const Eigen::MatrixXi& EF,
		  const Eigen::MatrixXi& EI,
		  double& cost,
		  Eigen::RowVectorXd& p) {

			  Eigen::Matrix4d QUAD = QV[E(e, 0)] + QV[E(e, 1)];

			  if (to_update) {
				  
				  QV[E(e, 0)] = QUAD;
				  QV[E(e, 1)] = QUAD;
			  }

			  else {
				  p.resize(3);
				  Eigen::Matrix4d QUADp = Eigen::Matrix4d::Identity();
				  QUADp.row(0) = QUAD.row(0);
				  QUADp.row(1) = QUAD.row(1);
				  QUADp.row(2) = QUAD.row(2);
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

			  }

			  to_update = false;
			 	
	  };

	  for (int i = 0; i < edges_to_delete; i++) {
		  if (!collapse_edge(
			  my_cost_and_placement, struct_list[selected_data_index].V, 
			  struct_list[selected_data_index].F, struct_list[selected_data_index].E,
			  struct_list[selected_data_index].EMAP, struct_list[selected_data_index].EF,
			  struct_list[selected_data_index].EI, *struct_list[selected_data_index].Q, 
			  *struct_list[selected_data_index].Qit, struct_list[selected_data_index].C))
		  {
			  break;
		  }
		  
		  *struct_list[selected_data_index].QV = QV;
		  to_update = true;
		  something_collapsed = true;
		  struct_list[selected_data_index].num_collapsed = struct_list[selected_data_index].num_collapsed + 1;
	  }
	  if (something_collapsed)
	  {
		  data().clear();
		  data().set_mesh(struct_list[selected_data_index].V, struct_list[selected_data_index].F);
		  data().set_face_based(true);
		  std::cout << "collapsed " << struct_list[selected_data_index].num_collapsed << " edges" << std::endl;
	  }

	  return false;

  }

  void Viewer::pos_cylinder() {
	  data().show_overlay_depth = false;
	  data().point_size = 10;
	  data().line_width = 3;

	  float max_y = data().V.colwise().maxCoeff()[1];
	  data().SetCenterOfRotation(Eigen::Vector3f(data().V.colwise().mean()[0], data().V.colwise().minCoeff()[1], data().V.colwise().mean()[2]));
	  data().MyTranslate(Eigen::Vector3f(0, 2 * max_y, 0));
	  
	  //create axis
	  /*Eigen::Vector3d M = data().V.colwise().maxCoeff();
	  Eigen::Vector3d m = data().V.colwise().minCoeff();
	  Eigen::MatrixXd center(1, 3);
	  center << 0, m(1), 0;
	  data().add_points(center, Eigen::RowVector3d(1, 0, 0));
      
      if (selected_data_index != 0) {
		  Eigen::MatrixXd V_Box(6, 3);
		  V_Box <<
			0, M(1) - length, 0,
			0, M(1) + length, 0,
			length, M(1), 0,
			-length, M(1), 0,
			0, M(1), length,
			0, M(1), -length;
		  for (int i = 0; i < 3; i++) {
			  data(selected_data_index-1).add_edges
			  (
				V_Box.row(2 * i),
				V_Box.row(2 * i + 1),
				Eigen::RowVector3d(1, 0, 0));
		  }
	  }*/ 
  }

  void Viewer::draw_box(Eigen::AlignedBox<double, 3> box, int id, bool set, Eigen::RowVector3d color) {
	  data().show_overlay_depth = false;
	  data().point_size = 10;
	  data().line_width = 1;
	  //Eigen::AlignedBox<double, 3> box = tree_list[selected_data_index].m_box;
	  Eigen::MatrixXd points(8,3);
	  points << box.corner(box.BottomLeftCeil)(0), box.corner(box.BottomLeftCeil)(1), box.corner(box.BottomLeftCeil)(2),
		  box.corner(box.BottomRightCeil)(0), box.corner(box.BottomRightCeil)(1), box.corner(box.BottomRightCeil)(2),
		  box.corner(box.BottomLeftFloor)(0), box.corner(box.BottomLeftFloor)(1), box.corner(box.BottomLeftFloor)(2),
		  box.corner(box.BottomRightFloor)(0), box.corner(box.BottomRightFloor)(1), box.corner(box.BottomRightFloor)(2),
		  box.corner(box.TopLeftCeil)(0), box.corner(box.TopLeftCeil)(1), box.corner(box.TopLeftCeil)(2),
		  box.corner(box.TopRightCeil)(0), box.corner(box.TopRightCeil)(1), box.corner(box.TopRightCeil)(2),
		  box.corner(box.TopLeftFloor)(0), box.corner(box.TopLeftFloor)(1), box.corner(box.TopLeftFloor)(2),
		  box.corner(box.TopRightFloor)(0), box.corner(box.TopRightFloor)(1), box.corner(box.TopRightFloor)(2);
	 // data(id).add_points(points, Eigen::RowVector3d(1, 0, 0));
	  
	  if (set) {
		  Eigen::MatrixXi edges(12, 2);
		  edges << 0, 1,
			  0, 2,
			  1, 3,
			  2, 3,
			  4, 5,
			  4, 6,
			  5, 7,
			  6, 7,
			  0, 4,
			  3, 7,
			  1, 5,
			  2, 6;
		  data(id).set_edges(points, edges, color);
	  }
	  else {
		  Eigen::MatrixXd p1(12, 3);
		  p1 << points.row(3)(0), points.row(3)(1), points.row(3)(2),//BRF
			  points.row(3)(0), points.row(3)(1), points.row(3)(2),//BRF
			  points.row(3)(0), points.row(3)(1), points.row(3)(2),//BRF
			  points.row(6)(0), points.row(6)(1), points.row(6)(2),//TLF
			  points.row(6)(0), points.row(6)(1), points.row(6)(2),//TLF
			  points.row(6)(0), points.row(6)(1), points.row(6)(2),//TLF
			  points.row(0)(0), points.row(0)(1), points.row(0)(2),//BLC
			  points.row(0)(0), points.row(0)(1), points.row(0)(2),//BLC
			  points.row(0)(0), points.row(0)(1), points.row(0)(2),//BLC
			  points.row(5)(0), points.row(5)(1), points.row(5)(2),//TRC
			  points.row(5)(0), points.row(5)(1), points.row(5)(2),//TRC
			  points.row(5)(0), points.row(5)(1), points.row(5)(2);//TRC

		  Eigen::MatrixXd p2(12, 3);
		  p2 << points.row(7)(0), points.row(7)(1), points.row(7)(2),//TRF
			  points.row(1)(0), points.row(1)(1), points.row(1)(2),//BRC
			  points.row(2)(0), points.row(2)(1), points.row(2)(2),//BLF
			  points.row(7)(0), points.row(7)(1), points.row(7)(2),//TRF
			  points.row(4)(0), points.row(4)(1), points.row(4)(2),//TLC
			  points.row(2)(0), points.row(2)(1), points.row(2)(2),//BLF
			  points.row(2)(0), points.row(2)(1), points.row(2)(2),//BLF
			  points.row(1)(0), points.row(1)(1), points.row(1)(2),//BRC
			  points.row(4)(0), points.row(4)(1), points.row(4)(2),//TLC
			  points.row(4)(0), points.row(4)(1), points.row(4)(2),//TLC
			  points.row(1)(0), points.row(1)(1), points.row(1)(2),//BRC
			  points.row(7)(0), points.row(7)(1), points.row(7)(2);//TRF

		  data(id).add_edges(p1, p2, color);
	  }
  }

  
 

} // end namespace
} // end namespace
}
