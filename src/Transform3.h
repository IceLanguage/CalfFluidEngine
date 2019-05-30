#ifndef _CalfFluidEngine_Transform3_
#define _CalfFluidEngine_Transform3_
#include <Quaternion.h>
#include <Matrix3.h>
namespace CalfFluidEngine {
	class Transform3
	{
	public:
		Transform3();
		Transform3(const Vector3D& translation, const Quaternion& orientation);
		~Transform3();

		const Vector3D& GetTranslation() const { return _translation; }
		void SetTranslation(const Vector3D& translation){ _translation = translation; }
		const Quaternion& GetOrientation() const { return _orientation; }
		void SetOrientation(const Quaternion& orientation) { _orientation = orientation; }

		Vector3D InverseTransformPoint(const Vector3D& pointInWorld) const;
		Vector3D InverseTransformDirection(const Vector3D& dirInWorld) const;
		Vector3D TransformPoint(const Vector3D& pointInLocal) const;
		Vector3D TransformDirection(const Vector3D& dirInLocal) const;
	private:
		Vector3D _translation;
		Quaternion _orientation;
		Matrix3x3D _orientationMat3;
		Matrix3x3D _inverseOrientationMat3;
	};
}
#endif
