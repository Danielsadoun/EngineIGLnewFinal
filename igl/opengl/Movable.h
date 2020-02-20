#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTranScale();
	Eigen::Matrix4f MakeTrans();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTranScaled();

	
	void MyTranslate(Eigen::Vector3f amt/*, bool preRotation*/);
	void MyRotate(Eigen::Vector3f rotAxis,float angle, bool regular_rotate);
	void MyScale(Eigen::Vector3f amt);
	//MakeTranced returns double
	//add bool is_picked to viewer for knowing if somthing is picked or nothing
	//call this in mouseprocesing instead MyTranslate void TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preTranslate);
	//call this in mouseprocesing instead MyRotate void RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle);
	void TranslateInSystem(Eigen::Matrix4f Mat, Eigen::Vector3f amt, bool preTranslate);
	void RotateInSystem(Eigen::Matrix4f Mat, Eigen::Vector3f rotAxis, double angle, bool regular_rotate);
	
	void SetCenterOfRotation(Eigen::Vector3f newCenter);
	Eigen::Vector3f getCenterOfRotation();
	Eigen::Matrix3f GetRotation()const { return Tout.rotation().matrix();}
	Eigen::Transform<float, 3, Eigen::Affine> get_Tout() { return Tout; }
	Eigen::Transform<float, 3, Eigen::Affine> get_Tin() { return Tin; }
	void set_T(Eigen::Transform<float, 3, Eigen::Affine> Tout, Eigen::Transform<float, 3, Eigen::Affine> Tin) { Tout = Tout; Tin = Tin; }

	virtual ~Movable() {}

private:
	//Eigen::Transform<float,3,Eigen::Affine> T;
	Eigen::Transform<float, 3, Eigen::Affine> Tout, Tin;
};

