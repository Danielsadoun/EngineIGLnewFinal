#include "Movable.h"

Movable::Movable()
{
	// T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

Movable::Movable(const Movable& mov) {
	Tout = mov.Tout;
	Tin = mov.Tin;
}

Eigen::Matrix4f Movable::MakeTranScale() {
	return (Tout.matrix() * Tin.matrix());
}


Eigen::Matrix4f Movable::MakeTrans()
{
	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
	mat.col(3) << Tin.translation(), 1;
	return (Tout.matrix() * mat);

	//return T.matrix();
}

/*Eigen::Matrix4d Movable::MakeTranScaled() {
	return (Tout.matrix() * Tin.matrix()).cast<double>();
}*/


/*Eigen::Matrix4d Movable::MakeTransd() {
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;
	return (Tout.matrix() * mat).cast<double>();
}*/




void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.pretranslate(amt);
	//T.translate(amt);
}
//angle in radians
//regular_rotate is a bool that marks if we rotate sphere or scn or around x axis of cyl
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle, bool regular_rotate)
{
	if (regular_rotate) {
		Tout.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
	}
	else {
		//we cancel the rotation matrix of the curr link for rotate around the parent y axis
		Tout.rotate(Eigen::AngleAxisf(angle, (Tout.rotation().matrix()).transpose() * rotAxis.normalized()));
	}
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	Tin.scale(amt);
}

void Movable::TranslateInSystem(Eigen::Matrix4f Mat, Eigen::Vector3f amt, bool preTranslate) {
	MyTranslate(Mat.block<3, 3>(0, 0).transpose() * amt);
}

void Movable::RotateInSystem(Eigen::Matrix4f Mat, Eigen::Vector3f rotAxis, double angle, bool regular_rotate) {
	MyRotate(Mat.block<3, 3>(0, 0).transpose() * rotAxis, angle, regular_rotate);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt) {
	Tin.translate(-amt);
	Tout.translate(amt);
}

Eigen::Vector3f Movable::getCenterOfRotation() {
	return -Tin.translation();
}