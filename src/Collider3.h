#ifndef _CalfFluidEngine_Collider3_
#define _CalfFluidEngine_Collider3_

#include <Vector3.h>
namespace CalfFluidEngine {
	class Collider3
	{
	public:
		Collider3();
		virtual ~Collider3();
		void ResolveCollision(
			double radius,
			double restitutionCoefficient,
			Vector3D* position,
			Vector3D* velocity);
	};
}
#endif