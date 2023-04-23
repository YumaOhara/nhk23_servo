#pragma once

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/Math/pid.hpp>
#include "motor_state.hpp"

namespace Nhk23Servo
{
	using namespace CRSLib::IntegerTypes;

	class Injector final
	{
		struct Constant final
		{
			float gear_ratio;
			i32 barrel_length;
			i32 idling_point_epsilon;

			Constant(const float gear_ratio) noexcept:
				gear_ratio(gear_ratio),
				barrel_length(MotorState::full_angle * gear_ratio / 2),
				idling_point_epsilon(barrel_length / 360)
			{}

			static constexpr i32 enough_slow_speed = 60;
			static constexpr i32 setting_up_speed = 60;
			static constexpr i32 current_max = 0x4;
		};

		const Constant constant;

		// ControlState
		struct Idle final{};
		struct Injecting final
		{
			i32 injection_point;
			i16 speed;
		};
		struct Stopping final{};
		struct SettingUp final{};

		std::variant<Idle, Injecting, Stopping, SettingUp> control_state{};
		
		MotorState motor_state{};

		// pid
		CRSLib::Math::Pid<i16> speed_pid;

		public:
		Injector(const float gear_ratio, const CRSLib::Math::Pid<i16>& speed_pid) noexcept:
			constant(gear_ratio),
			speed_pid(speed_pid)
		{}

		void update_motor_state(const Feedback& state) noexcept
		{
			motor_state.update(state);
		}

		void inject_start(const i16 speed) noexcept
		{
			// Idleでなければ何もしない
			if(std::holds_alternative<Idle>(control_state))
			{
				control_state.template emplace<Injecting>(motor_state.get_total_angle(), speed);
			}
		}

		private:
		struct UpdateCurrent final
		{
			Injector& injector;

			UpdateCurrent(Injector& injector) noexcept:
				injector(injector)
			{}

			i16 operator()(Idle) noexcept
			{
				return injector.calc_target_current_from_speed(0.0);
			}

			i16 operator()(const Injecting& injecting) noexcept
			{
				if(std::abs(injector.motor_state.get_total_angle() - injecting.injection_point) > injector.constant.barrel_length)
				{
					injector.control_state.template emplace<Stopping>();
					return injector.calc_target_current_from_speed(0.0);
				}
				return injector.calc_target_current_from_speed(injecting.speed);
			}

			i16 operator()(Stopping) noexcept
			{
				if(std::abs(injector.motor_state.feedback.speed) < injector.constant.enough_slow_speed)
				{
					injector.control_state.template emplace<SettingUp>();
					return injector.calc_target_current_from_speed(injector.constant.setting_up_speed);
				}
				return injector.calc_target_current_from_speed(0.0);
			}

			i16 operator()(SettingUp) noexcept
			{
				if(std::abs(injector.fixed_position() - injector.constant.barrel_length) < injector.constant.idling_point_epsilon)
				{
					injector.control_state.template emplace<Idle>();
					return injector.calc_target_current_from_speed(0.0);
				}
				return injector.calc_target_current_from_speed(injector.constant.setting_up_speed);
			}
		};

		public:
		i16 run_and_calc_target() noexcept
		{
			return std::visit(UpdateCurrent(*this), control_state);
		}

		private:
		i16 calc_target_current_from_speed(i16 target) noexcept
		{
			const auto ret = speed_pid.update(target, motor_state.feedback.speed);
			return std::max<i16>(-constant.current_max, std::min<i16>(constant.current_max, ret));
		}

		i32 fixed_position() const noexcept
		{
			return motor_state.get_total_angle() % (2 * constant.barrel_length);
		}
	};
}
