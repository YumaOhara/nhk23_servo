#pragma once

#include <cmath>

#include <CRSLibtmp/std_type.hpp>
#include "feedback.hpp"

namespace Nhk23Servo
{
	using namespace CRSLib::IntegerTypes;

	// MotorState
	struct MotorState
	{
		static constexpr i32 full_angle = 8192;
		Feedback feedback{};
		i32 motor_rotation_count{0};

		void update(const Feedback& new_feedback) noexcept
		{
			if(std::abs(new_feedback.angle - feedback.angle) > full_angle / 2)
			{
				motor_rotation_count += std::signbit(new_feedback.angle - feedback.angle) ? -1 : 1;
			}

			feedback = new_feedback;
		}

		i32 get_total_angle() const noexcept
		{
			return motor_rotation_count * full_angle + feedback.angle;
		}
	};
}
