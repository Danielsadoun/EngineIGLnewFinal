#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
#include <stdlib.h>
#include <Windows.h>
#include <mmsystem.h>

Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	


	
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	highdpi = 1;

	xold = 0;
	yold = 0;

}

IGL_INLINE void Renderer::draw( GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);
	
	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window,width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}
	
	for (auto& core : core_list)
	{
		for (auto& mesh : scn->data_list)
		{
			if (mesh.is_visible & core.id)
			{
				if (mesh.id >= scn->num_of_cyl || mesh.id == 0) {
					core.draw(scn->MakeTrans(), mesh);
				}
				else {
					Eigen::Matrix4f my_mat = scn->MakeTrans() * MakeParents(mesh.id);
					core.draw(my_mat, mesh);
				}
			}
		}
	}

}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer, int level)
{
	scn = viewer;
	Level = level;
	g = 0.01;
	core().init(); 

	core().align_camera_center(scn->data().V, scn->data().F);

	core().viewport = Eigen::Vector4f(0, 0, 500, 800);
	
	append_core(Eigen::Vector4f(500, 0, 500, 800));
	core(1).camera_up = Eigen::Vector3f(0, 0, 1);

	//core().align_camera_center(scn->data().V, scn->data().F);

}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{
	if (scn->is_picked) {
		if (button == 1)
		{
			if (scn->selected_data_index >= scn->num_of_cyl) {
				scn->data().TranslateInSystem(scn->MakeTrans(), Eigen::Vector3f(-xrel / 400.0f, 0, 0), true);
				scn->data().TranslateInSystem(scn->MakeTrans(), Eigen::Vector3f(0, yrel / 400.0f, 0), true);
			}
			else {
				scn->data(0).TranslateInSystem(scn->MakeTrans(), Eigen::Vector3f(-xrel / 100.0f, 0, 0), true);
				scn->data(0).TranslateInSystem(scn->MakeTrans(), Eigen::Vector3f(0, yrel / 100.0f, 0), true);
			}
			
		}
		else
		{
			if (scn->selected_data_index >= scn->num_of_cyl) {
				scn->data().MyRotate(Eigen::Vector3f(0, 1, 0), xrel / 180.0f, true);
				scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), yrel / 180.0f, true);
			}
			else {
				scn->data().MyRotate(Eigen::Vector3f(0, 1, 0), xrel / 180.0f, false);
				scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), yrel / 180.0f, true);
			}
		}
	}	
	else {
		if (button == 1)
		{
			//scn->MyTranslate(Eigen::Vector3f(-xrel/100.0f, 0, 0));
			//scn->MyTranslate(Eigen::Vector3f(0, yrel/100.0f, 0));
		}
		else
		{
			//scn->RotateInSystem(scn->MakeTrans(), Eigen::Vector3f(0, 1, 0), xrel / 180.0f, true);
			//scn->RotateInSystem(scn->MakeTrans(), Eigen::Vector3f(1, 0, 0), yrel / 180.0f, true);
		}
	}
	Update_headCore();
}

Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

bool Renderer::Picking(double newx, double newy, float * z)
{
		int fid;
		//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
		Eigen::Vector3f bc;
		double x = newx;
		double y = core().viewport(3) - newy;
		Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
		igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
		//we added the makeparents in duplication for picking the correct cyl according to the draw
		view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
				* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() * MakeParents(scn->selected_data_index) * scn->data().MakeTrans();
		if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
 			core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
		{
			Eigen::MatrixXi F = scn->data().F;
			Eigen::MatrixXd V = scn->data().V;
			// find z of the face with fid
			Eigen::Vector3f p1 = (V.row(F.row(fid)(0))).cast<float>();
			Eigen::Vector3f p2 = (V.row(F.row(fid)(1))).cast<float>();
			Eigen::Vector3f p3 = (V.row(F.row(fid)(2))).cast<float>();

			Eigen::Vector4f w0(p1(0), p1(1), p1(2), 1.0f);
			Eigen::Vector4f w1(p2(0), p2(1), p2(2), 1.0f);
			Eigen::Vector4f w2(p3(0), p3(1), p3(2), 1.0f);

			w0 = view * w0;
			w1 = view * w1;
			w2 = view * w2;

			p1(0) = w0(0);
			p1(1) = w0(1);
			p1(2) = w0(2);
			p2(0) = w1(0);
			p2(1) = w1(1);
			p2(2) = w1(2);
			p3(0) = w2(0);
			p3(1) = w2(1);
			p3(2) = w2(2);

			Eigen::Vector3f p = bc(0) * p1 + bc(1) * p2 + bc(2) * p3;

			if (*z < p(2)) 
				*z = p(2);
			
			
			return true;
		}
		return false;
	
}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
	{
		if (window) {
			glfwSetWindowSize(window, w / highdpi, h / highdpi);
		}
		post_resize(window,w, h);
	}

	IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
	{
		if (core_list.size() == 1)
		{
			core().viewport = Eigen::Vector4f(0, 0, w, h);
		}
		else
		{
			
			// It is up to the user to define the behavior of the post_resize() function
			// when there are multiple viewports (through the `callback_post_resize` callback)
		}
		//for (unsigned int i = 0; i < plugins.size(); ++i)
		//{
		//	plugins[i]->post_resize(w, h);
		//}
		if (callback_post_resize)
		{
			callback_post_resize(window, w, h);
		}
	}

	IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE bool Renderer::erase_core(const size_t index)
	{
		assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
		//assert(data_list.size() >= 1);
		if (core_list.size() == 1)
		{
			// Cannot remove last viewport
			return false;
		}
		core_list[index].shut(); // does nothing
		core_list.erase(core_list.begin() + index);
		if (selected_core_index >= index && selected_core_index > 0)
		{
			selected_core_index--;
		}
		return true;
	}

	IGL_INLINE size_t Renderer::core_index(const int id) const {
		for (size_t i = 0; i < core_list.size(); ++i)
		{
			if (core_list[i].id == id)
				return i;
		}
		return 0;
	}

	IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
	{
		core_list.push_back(core()); // copies the previous active core and only changes the viewport
		core_list.back().viewport = viewport;
		core_list.back().id = next_core_id;
		next_core_id <<= 1;
		if (!append_empty)
		{
			for (auto& data : scn->data_list)
			{
				data.set_visible(true, core_list.back().id);
				data.copy_options(core(), core_list.back());
			}
		}
		selected_core_index = core_list.size() - 1;
		return core_list.back().id;
	}

	void Renderer::Animate() {
		Animate_food();
		food_detection();
		if (scn->last_picked < scn->num_of_cyl ||
			!scn->data_list[scn->last_picked].is_visible) {
			return;
		}
		Eigen::Vector3f R0 = getR(0);
		Eigen::Vector3f E = getTip();
		Eigen::Vector3f D = getDes();
		Eigen::Vector3f R0D = D - R0;
		//if the destination is far so we cant reach it we dont do nothing
		if (R0D.norm() > scn->num_of_cyl* scn->length) {
			PlaySound(TEXT("snakesss.wav"), NULL, SND_FILENAME | SND_ASYNC);
			std::cout << "The food is too far -> -5 points" << std::endl;
			scn->scoring = scn->scoring - 5;
			std::cout << "SCORING -> " << scn->scoring << std::endl;
			scn->last_picked = -1;
			return;
		}
		
		for (int i = scn->num_of_cyl - 1; i >= 0; i--) {
			Eigen::Vector3f R = getR(i);
			Eigen::Vector3f RE = (E - R);
			Eigen::Vector3f RD = (D - R);
			Eigen::Vector3f normal = RE.normalized().cross(RD.normalized());//returns the plane normal
			//dot(RE.NORMALIZE,RD.NORMALIZE)= 1*1*cos(angle between them)
			double dot = RD.normalized().dot(RE.normalized());//scalar multiplication - cos of the angle between RE and RD because normalized
			Eigen::Vector3f ED = (D - E);
			if (dot > 1)
				dot = 1;
			else if (dot < -1)
				dot = -1;
			double angle = acos(dot);
			//we want that this rotate will be around normal and not around parents axis
			scn->data(i).MyRotate(MakeParents(i).block<3, 3>(0, 0).inverse() * normal, angle / 10, false);
			E = getTip();
			//update the core of the snake head
			Update_headCore();
		}
		Eigen::Vector3f ED = (D - E);
		if (is_collide(&scn->tree_list[scn->num_of_cyl-1], &scn->tree_list[scn->last_picked])) {
			set_rotation();
			set_color_dir();
			scn->last_picked = -1;
		}
		

	}
	
	Eigen::Matrix4f Renderer::MakeParents(int meshId) {
		Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
		if (meshId >= scn->num_of_cyl || meshId == 0) {
			return mat;
		}
		for (int i = 0; i < meshId; i++) {
			mat = mat * scn->data(i).MakeTrans();
		}
		return mat;	
	}

	Eigen::Matrix3f Renderer::MakeParentsRot(int meshId) {
		Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();
		if (meshId >= scn->num_of_cyl || meshId == 0) {
			return mat;
		}
		for (int i = 0; i < meshId; i++) {
			mat = mat * scn->data(i).GetRotation();
		}
		return mat;
	}


	//cancel the makeparents 
	Eigen::Matrix3f Renderer:: MakeParentsInverse(int meshId) {
		Eigen::Matrix3f mat = scn->data(meshId).GetRotation().inverse();
		if (meshId >= scn->num_of_cyl || meshId == 0) {
			return mat;
		}
		for (int i = meshId-1 ; i >= 0; i--) {
			mat = mat * scn->data(i).GetRotation().inverse() ;
		}
		return mat;
	}

	Eigen::Vector3f Renderer::getTip() {
		Eigen::Vector4f tipCenter(scn->data(scn->num_of_cyl - 1).getCenterOfRotation()[0],
			                      scn->data(scn->num_of_cyl - 1).getCenterOfRotation()[1] + 1.6,
			                      scn->data(scn->num_of_cyl - 1).getCenterOfRotation()[2], 1);
		Eigen::Vector4f tip = MakeParents(scn->num_of_cyl - 1) * scn->data(scn->num_of_cyl - 1).MakeTrans() * tipCenter;
		return tip.head(3);
	}

	Eigen::Vector3f Renderer::getR(int meshId) {
		Eigen::Vector4f RCenter(scn->data(meshId).getCenterOfRotation()[0], scn->data(meshId).getCenterOfRotation()[1] , scn->data(meshId).getCenterOfRotation()[2], 1);
		Eigen::Vector4f R = MakeParents(meshId) * scn->data(meshId).MakeTrans() * RCenter;
		return R.head(3);
	}

	Eigen::Vector3f Renderer::getDes() {
		return (scn->data(scn->last_picked).MakeTrans() * Eigen::Vector4f(0, 0, 0, 1)).head(3);
	}

	void Renderer::set_rotation() {
		for (int i = 0; i < scn->num_of_cyl; i++) {
			float r11 = (scn->data(i).GetRotation().row(1))(1);
			float r10 = (scn->data(i).GetRotation().row(1))(0);
			float r12 = (scn->data(i).GetRotation().row(1))(2);
			float thetaY1 = 0;
			if (r11 > -1 && r11 < 1) {
				thetaY1 = atan2f(r10, -r12);
			}
			scn->data(i).MyRotate(Eigen::Vector3f(0, 1, 0), -thetaY1, true);
			if (i != scn->num_of_cyl - 1) {
				scn->data(i + 1).MyRotate(Eigen::Vector3f(0, 1, 0), thetaY1, false);
			}

		}
	}
	//A_BOX = box of the head of the snake
	bool Renderer::is_separate(Eigen::AlignedBox<double, 3> A_BOX, Eigen::AlignedBox<double, 3> B_BOX) {
		Eigen::Matrix3d A = MakeParents(scn->num_of_cyl - 1).block<3, 3>(0, 0).cast<double>() * scn->data(scn->num_of_cyl - 1).GetRotation().cast<double>();
		Eigen::Vector3d A0 = A * Eigen::Vector3d(1, 0, 0);
		Eigen::Vector3d A1 = A * Eigen::Vector3d(0, 1, 0);
		Eigen::Vector3d A2 = A * Eigen::Vector3d(0, 0, 1);
		
		Eigen::Matrix3d B = scn->data(scn->last_picked).GetRotation().cast<double>();
		Eigen::Vector3d B0 = B * Eigen::Vector3d(1, 0, 0);
		Eigen::Vector3d B1 = B * Eigen::Vector3d(0, 1, 0);
		Eigen::Vector3d B2 = B * Eigen::Vector3d(0, 0, 1);
		
		Eigen::Matrix3d C = A.transpose() * B;
		Eigen::Vector3d a = A_BOX.sizes();
		Eigen::Vector3d b = B_BOX.sizes();
		a = a / 2;
		b = b / 2;
		Eigen::Vector4d centerA = Eigen::Vector4d(A_BOX.center()[0], A_BOX.center()[1], A_BOX.center()[2], 1);
		Eigen::Vector4d centerB = Eigen::Vector4d(B_BOX.center()[0], B_BOX.center()[1], B_BOX.center()[2], 1);
		Eigen::Vector4d D4 = scn->data(scn->last_picked).MakeTrans().cast<double>() * centerB - (MakeParents(scn->num_of_cyl - 1).cast<double>() * scn->data(scn->num_of_cyl-1).MakeTrans().cast<double>() * centerA);
		Eigen::Vector3d D = D4.head(3);

		if (!(a(0) + (b(0) * abs(C.row(0)(0)) + b(1) * abs(C.row(0)(1)) + b(2) * abs(C.row(0)(2))) < abs(A0.transpose() * D))) {
			if (!(a(1) + (b(0) * abs(C.row(1)(0)) + b(1) * abs(C.row(1)(1)) + b(2) * abs(C.row(1)(2))) < abs(A1.transpose() * D))) {
				if (!(a(2) + (b(0) * abs(C.row(2)(0)) + b(1) * abs(C.row(2)(1)) + b(2) * abs(C.row(2)(2))) < abs(A2.transpose() * D))) {
					if (!(b(0) + (a(0) * abs(C.row(0)(0)) + a(1) * abs(C.row(1)(0)) + a(2) * abs(C.row(2)(0))) < abs(B0.transpose() * D))) {
						if (!(b(1) + (a(0) * abs(C.row(0)(1)) + a(1) * abs(C.row(1)(1)) + a(2) * abs(C.row(2)(1))) < abs(B1.transpose() * D))) {
							if (!(b(2) + (a(0) * abs(C.row(0)(2)) + a(1) * abs(C.row(1)(2)) + a(2) * abs(C.row(2)(2))) < abs(B2.transpose() * D))) {
								double PART1 = C.row(1)(0) * A2.transpose() * D;
								double PART2 = C.row(2)(0) * A1.transpose() * D;
								double R = abs(PART1 - PART2);
								if (!(a(1)*abs(C.row(2)(0)) + a(2)*abs(C.row(1)(0)) + b(1)*abs(C.row(0)(2)) + b(2)*abs(C.row(0)(1)) < R) ) {
									PART1 = C.row(1)(1) * A2.transpose() * D;
									PART2 = C.row(2)(1) * A1.transpose() * D;
									R = abs(PART1 - PART2);
									if (!(a(1) * abs(C.row(2)(1)) + a(2) * abs(C.row(1)(1)) + b(0) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(0)) < R)) {
										PART1 = C.row(1)(2) * A2.transpose() * D;
										PART2 = C.row(2)(2) * A1.transpose() * D;
										R = abs(PART1 - PART2);
										if (!(a(1) * abs(C.row(2)(2)) + a(2) * abs(C.row(1)(2)) + b(0) * abs(C.row(0)(1)) + b(1) * abs(C.row(0)(0)) < R)) {
											PART1 = C.row(2)(0) * A0.transpose() * D;
											PART2 = C.row(0)(0) * A2.transpose() * D;
											R = abs(PART1 - PART2);
											if (!(a(0) * abs(C.row(2)(0)) + a(2) * abs(C.row(0)(0)) + b(1) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(1)) < R)) {
												PART1 = C.row(2)(1) * A0.transpose() * D;
												PART2 = C.row(0)(1) * A2.transpose() * D;
												R = abs(PART1 - PART2);
												if (!(a(0) * abs(C.row(2)(1)) + a(2) * abs(C.row(0)(1)) + b(0) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(0)) < R)) {
													PART1 = C.row(2)(2) * A0.transpose() * D;
													PART2 = C.row(0)(2) * A2.transpose() * D;
													R = abs(PART1 - PART2);
													if (!(a(0) * abs(C.row(2)(2)) + a(2) * abs(C.row(0)(2)) + b(0) * abs(C.row(1)(1)) + b(1) * abs(C.row(1)(0)) < R)) {
														PART1 = C.row(0)(0) * A1.transpose() * D;
														PART2 = C.row(1)(0) * A0.transpose() * D;
														R = abs(PART1 - PART2);
														if (!(a(0) * abs(C.row(1)(0)) + a(1) * abs(C.row(0)(0)) + b(1) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(1)) < R)) {
															PART1 = C.row(0)(1) * A1.transpose() * D;
															PART2 = C.row(1)(1) * A0.transpose() * D;
															R = abs(PART1 - PART2);
															if (!(a(0) * abs(C.row(1)(1)) + a(1) * abs(C.row(0)(1)) + b(0) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(0)) < R)) {
																PART1 = C.row(0)(2) * A1.transpose() * D;
																PART2 = C.row(1)(2) * A0.transpose() * D;
																R = abs(PART1 - PART2);
																if (!(a(0) * abs(C.row(1)(2)) + a(1) * abs(C.row(0)(2)) + b(0) * abs(C.row(2)(1)) + b(1) * abs(C.row(2)(0)) < R)) {
																	return false;
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
		return true;
	}

	bool Renderer::is_collide(igl::AABB<Eigen::MatrixXd, 3> * A_TREE, igl::AABB<Eigen::MatrixXd, 3> * B_TREE) {
		
		if (A_TREE->is_leaf() && B_TREE->is_leaf()) {
			if (!is_separate(A_TREE->m_box, B_TREE->m_box)) {
				PlaySound(TEXT("bite.wav"), NULL, SND_FILENAME | SND_ASYNC);
				//scn->draw_box(A_TREE->m_box, scn->last_picked, false, Eigen::RowVector3d (1,1,1));
				//scn->draw_box(B_TREE->m_box, !scn->last_picked, false, Eigen::RowVector3d (1,1,1));
				return true;
			}
			else {
				return false;
			}
		}
		if (is_separate(A_TREE->m_box, B_TREE->m_box)) {
			return false;
		}
		if (A_TREE->is_leaf() && !B_TREE->is_leaf()) {
			return is_collide(A_TREE, B_TREE->m_left) ||
				is_collide(A_TREE, B_TREE->m_right);
		}
		if (!A_TREE->is_leaf() && B_TREE->is_leaf()) {
			return is_collide(A_TREE->m_left, B_TREE) ||
				is_collide(A_TREE->m_right, B_TREE);
		}
		return is_collide(A_TREE->m_left, B_TREE->m_left) ||
			is_collide(A_TREE->m_left, B_TREE->m_right) ||
			is_collide(A_TREE->m_right, B_TREE->m_left) ||
			is_collide(A_TREE->m_right, B_TREE->m_right);
		
		
	}

	void Renderer:: Animate_food() {
		for (int i = scn->num_of_cyl; i < scn->data_list.size(); i++) {
			if (scn->data_list[i].is_visible) {
				scn->data_list[i].MyTranslate(scn->data_list[i].direction);
				Eigen::Vector4f position = scn->data_list[i].MakeTrans() * Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);
				if (position(1) < 0) { // food touch the floor
					scn->data_list[i].direction << scn->data_list[i].direction(0), -scn->data_list[i].direction(1), scn->data_list[i].direction(2);
					scn->data_list[i].MyTranslate(Eigen::Vector3f(0, -position(1), 0));
				}
				else {
					scn->data_list[i].direction << scn->data_list[i].direction(0), scn->data_list[i].direction(1) - g, scn->data_list[i].direction(2);
				}
				if (position(0) > 7 || position(0) < -6.5) { // food touch the wall
					scn->data_list[i].direction << -scn->data_list[i].direction(0), scn->data_list[i].direction(1), scn->data_list[i].direction(2);
					if (position(0) > 7)
						scn->data_list[i].MyTranslate(Eigen::Vector3f(7 - position(0), 0, 0));
					else
						scn->data_list[i].MyTranslate(Eigen::Vector3f(-6.5 - position(0), 0, 0));
				}
			}
		    
		}
	}

	void Renderer::set_color_dir() {
		
		if (scn->data_list[scn->last_picked].level == 0) {
			
			scn->data_list[scn->last_picked].set_colors(Eigen::RowVector3d(0, 0.2, 0.5));
			if(scn->data_list[scn->last_picked].direction(1) > 0)
				scn->data_list[scn->last_picked].direction << 0, scn->data_list[scn->last_picked].direction(1) + 0.01 , 0;
			else
				scn->data_list[scn->last_picked].direction << 0, scn->data_list[scn->last_picked].direction(1) - 0.01, 0;
			std::cout << "Tasty but not enough, I want more! -> +5 points" << std::endl;
			scn->scoring = scn->scoring + 5;
		}
		else if (scn->data_list[scn->last_picked].level == 1) {
			
			scn->data_list[scn->last_picked].set_colors(Eigen::RowVector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0));
			if (scn->data_list[scn->last_picked].direction(1) > 0)
				scn->data_list[scn->last_picked].direction << 0.05, scn->data_list[scn->last_picked].direction(1) + 0.01, 0;
			else
				scn->data_list[scn->last_picked].direction << -0.05, scn->data_list[scn->last_picked].direction(1) - 0.01, 0;
			std::cout << "I want more from this! -> +10 points" << std::endl;
			scn->scoring = scn->scoring + 10;
		}
		else if (scn->data_list[scn->last_picked].level == 2) {
			
			scn->data_list[scn->last_picked].set_colors(Eigen::RowVector3d(0.9,0,0));
			if (scn->data_list[scn->last_picked].direction(1) > 0)
				scn->data_list[scn->last_picked].direction << scn->data_list[scn->last_picked].direction(0), scn->data_list[scn->last_picked].direction(1) + 0.01, 0;
			else
				scn->data_list[scn->last_picked].direction << scn->data_list[scn->last_picked].direction(0), scn->data_list[scn->last_picked].direction(1) - 0.01, 0;
			std::cout << "Just one more bite! -> +15 points" << std::endl;
			scn->scoring = scn->scoring + 15;
		}
		else if (scn->data_list[scn->last_picked].level == 3) {
			
			scn->data_list[scn->last_picked].set_visible(false, 1);
			scn->data_list[scn->last_picked].set_visible(false, 2);
			core(1).is_animating = false;
			core(2).is_animating = false;
			std::cout << "Delicius! -> +20 points" << std::endl;
			scn->scoring = scn->scoring + 20;
		}
		scn->data_list[scn->last_picked].level = scn->data_list[scn->last_picked].level + 1;
		std::cout << "SCORING -> " << scn->scoring << std::endl;
	}

	void Renderer::Update_headCore() {
		Eigen::Matrix3f rot = MakeParentsRot(scn->num_of_cyl - 1) * (scn->data(scn->num_of_cyl - 1).MakeTrans()).block<3, 3>(0, 0);
		core(core_list[0].id).camera_up =  rot * Eigen::Vector3f(0, 0, 1);
		core(core_list[0].id).camera_eye = rot * Eigen::Vector3f(0, -1, 0);
		Eigen::Matrix4f trans = scn->MakeTrans() * MakeParents(scn->num_of_cyl - 1) * scn->data(scn->num_of_cyl - 1).MakeTrans();
		core(core_list[0].id).camera_translation = -(trans).col(3).head(3) + (rot.col(1) * (-0.9));

	}

	bool Renderer::checkFinish() {
		for (int i = scn->num_of_cyl; i < scn->data_list.size(); i++) {
			if (scn->data_list[i].is_visible) {
				return false;
			}
		}
		return true;
	}

	void Renderer::buildLevel(int level, bool prev) {
		for (int i = scn->num_of_cyl; i < scn->data_list.size(); i++) {
			scn->data_list[i].set_visible(true, 1);
			scn->data_list[i].set_visible(true, 2);
			scn->data_list[i].level = 0;
			scn->data_list[i].set_colors(Eigen::RowVector3d(0, 0.5, 0));
			scn->data_list[i].direction = Eigen::Vector3f(0, -0.05, 0);
			
		}
		if (!prev) {
			for (int i = 0; i < level; i++) {
				if (i % 2 == 0) {
					scn->load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj", Eigen::Vector3f(0.05 * (level / 2), 0, 0));
					if ((2 * i > 7) && (2 * i <= 17.5)) {
						scn->data().MyTranslate(Eigen::Vector3f((2 * i) % 7, 2 * i, 0));
					}
					else if ((2 * i > 7) && (2 * i > 17.5)) {
						scn->data().MyTranslate(Eigen::Vector3f((2 * i) % 7, (2 * i) % 17, 0));
					}
					else if ((2 * i < -6.5)) {
						scn->data().MyTranslate(Eigen::Vector3f(-((2 * -i) % 7), (2 * -i) % 17, 0));
					}
					else if ((2 * i == 0)) {
						scn->data().MyTranslate(Eigen::Vector3f(-6, 6, 0));
					}
					else {
						scn->data().MyTranslate(Eigen::Vector3f(2 * i, 2 * i, 0));
					}
				}
				else {
					scn->load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj", Eigen::Vector3f(0, 0.05 * (level / 2), 0));
					if ((2 * i > 7) && (2 * i <= 17.5)) {
						scn->data().MyTranslate(Eigen::Vector3f((2 * i) % 7, 2 * i, 0));
					}
					else if ((2 * i > 7) && (2 * i > 17.5)) {
						scn->data().MyTranslate(Eigen::Vector3f((2 * i) % 7, (2 * i) % 17, 0));
					}
					else if ((2 * i < -6.5)) {
						scn->data().MyTranslate(Eigen::Vector3f(-((2 * -i) % 7), (2 * -i) % 17, 0));
					}
					else {
						scn->data().MyTranslate(Eigen::Vector3f(2 * i, 2 * i, 0));
					}
				}
				scn->data().direction = Eigen::Vector3f(0, -0.05, 0);
				scn->data().set_colors(Eigen::RowVector3d(0, 0.5, 0));
				scn->data().set_visible(true, 1);
				scn->data().set_visible(true, 2);
				scn->data().copy_options(core(1), core(2));
				core(1).toggle(scn->data().show_lines);
				core(2).toggle(scn->data().show_lines);

			}
		}
	}

	void Renderer::food_detection() {
		for (int i = scn->num_of_cyl; i < scn->data_list.size(); i++) {
			if (scn->data_list[i].is_visible) {
				for (int j = i + 1; j < scn->data_list.size(); j++) {
					if (scn->data_list[j].is_visible) {
						Eigen::Vector4f positioni = scn->data_list[i].MakeTrans() * Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);
						Eigen::Vector4f positionj = scn->data_list[j].MakeTrans() * Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);

						if ((positioni - positionj).norm() <= 1.3) {
							scn->data_list[i].MyTranslate((positioni.head(3) - positionj.head(3)) / 10);
							scn->data_list[j].MyTranslate(-(positioni.head(3) - positionj.head(3)) / 10);
							Eigen::Vector3f temp = scn->data_list[i].direction;
							scn->data_list[i].direction = scn->data_list[j].direction;
							scn->data_list[j].direction = temp;
						}
					}
				}
			}
		}

	}


	//IGL_INLINE void Viewer::select_hovered_core()
	//{
	//	int width_window, height_window = 800;
	//   glfwGetFramebufferSize(window, &width_window, &height_window);
	//	for (int i = 0; i < core_list.size(); i++)
	//	{
	//		Eigen::Vector4f viewport = core_list[i].viewport;

	//		if ((current_mouse_x > viewport[0]) &&
	//			(current_mouse_x < viewport[0] + viewport[2]) &&
	//			((height_window - current_mouse_y) > viewport[1]) &&
	//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
	//		{
	//			selected_core_index = i;
	//			break;
	//		}
	//	}
	//}