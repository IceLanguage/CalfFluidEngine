#include "Quaternion.h"

#include <float.h>
#include <math.h>
using namespace CalfFluidEngine;

Quaternion::Quaternion()
{
	for (int i = 0; i < 4; ++i) data[i] = 0;
	data[0] = 1;
}

Quaternion::Quaternion(const double r, const double i, const double j, const double k)
{
	data[0] = r;
	data[1] = i;
	data[2] = j;
	data[3] = k;
}

double Quaternion::Get_r() const
{
	return data[0];
}

double Quaternion::Get_i() const
{
	return data[1];
}

double Quaternion::Get_j() const
{
	return data[2];
}

double Quaternion::Get_k() const
{
	return data[3];
}

void Quaternion::Set_r(double value)
{
	data[0] = value;
}

void Quaternion::Set_i(double value)
{
	data[1] = value;
}

void Quaternion::Set_j(double value)
{
	data[2] = value;
}

void Quaternion::Set_k(double value)
{
	data[3] = value;
}

void Quaternion::Normalize()
{
	float d = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];

	if (d < FLT_EPSILON) {
		data[0] = 1;
		return;
	}

	d = 1.0f / sqrtf(d);
	
	for (int i = 0; i < 4; ++i) data[i] *= d;
}

void Quaternion::operator*=(const Quaternion & multiplier)
{
	Quaternion q = *this;

	float r = q.Get_r() * multiplier.Get_r() - q.Get_i() * multiplier.Get_i() -
		q.Get_j() * multiplier.Get_j() - q.Get_k() * multiplier.Get_k();
	float i = q.Get_r() * multiplier.Get_i() + q.Get_i() * multiplier.Get_r() +
		q.Get_j() * multiplier.Get_k() - q.Get_k() * multiplier.Get_j();
	float j = q.Get_r() * multiplier.Get_j() + q.Get_j() * multiplier.Get_r() +
		q.Get_k() * multiplier.Get_i() - q.Get_i() * multiplier.Get_k();
	float k = q.Get_r() * multiplier.Get_k() + q.Get_k() * multiplier.Get_r() +
		q.Get_i() * multiplier.Get_j() - q.Get_j() * multiplier.Get_i();

	data[0] = r;
	data[1] = i;
	data[2] = j;
	data[3] = k;
}

void Quaternion::RotateByVector(const Vector3D & vector)
{
	Quaternion q(0, vector.x, vector.y, vector.z);
	(*this) *= q;
}

void Quaternion::AddScaledVector(const Vector3D & vector, float scale)
{
	Quaternion q(0,
		vector.x * scale,
		vector.y * scale,
		vector.z * scale);
	q *= *this;
	data[0] += q.Get_r() * 0.5f;
	data[1] += q.Get_i() * 0.5f;
	data[2] += q.Get_j() * 0.5f;
	data[3] += q.Get_k() * 0.5f;
}
