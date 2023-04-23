#pragma once

#include <CrsLibtmp/std_type.hpp>

namespace Nhk23Servo
{
	using namespace CRSLib::IntegerTypes;

	struct Feedback final
	{
		i16 angle{0};
		i16 speed{0};
		i16 current{0};
	};
}