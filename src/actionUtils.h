#include <string>

#include <iostream>
#include <iomanip>

namespace actUtils{

	struct handPose{
		float x, y, z;
		float rx, ry, rz;
		bool openHand;
		float pause;
	};

}