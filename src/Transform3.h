#ifndef _CalfFluidEngine_Transform3_
#define _CalfFluidEngine_Transform3_
#include <Quaternion.h>
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
	private:
		Vector3D _translation;
		Quaternion _orientation;
	};
}
#endif
