#pragma once

#include <cmath>
#include <variant>

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/utility.hpp>
#include <CRSLibtmp/Math/pid.hpp>
#include <CRSLibtmp/Can/utility.hpp>

// debug
volatile CRSLib::IntegerTypes::i32 debug_var = 0;

namespace Nhk23Servo
{
	using namespace CRSLib::IntegerTypes;

	struct MotorFeedback final
	{
		i16 angle{0};
		i16 speed{0};
		i16 current{0};

		MotorFeedback() noexcept = default;

		MotorFeedback(const CRSLib::Can::DataField::Buffer& buffer) noexcept
		{
			angle = (u8)buffer[0] | (u8)(buffer[1] << 8);
			speed = CRSLib::bit_cast<i16>((u8)buffer[2] | (u8)(buffer[3] << 8));
			current = CRSLib::bit_cast<i16>((u8)buffer[4] | (u8)(buffer[5] << 8));
		}
	};

	template<CRSLib::is_std_ratio auto gear_ratio_>
	class Injector final
	{
		static constexpr float gear_ratio = CRSLib::ratio_to_floating<float>(gear_ratio_);
		static constexpr i32 max_angle = 8192 * gear_ratio;
		static constexpr i32 half_angle = 8192 * gear_ratio / 2;
		static constexpr i32 enough_near_start_angle = max_angle / 360;
		static constexpr i16 enough_slow_speed = 60;
		static constexpr i16 setting_up_speed = 60;

		// ControlState
		struct Idle final{};
		struct Injecting final
		{
			i16 speed;
		};
		struct Stopping final{};
		struct SettingUp final{};

		std::variant<Idle, Injecting, Stopping, SettingUp> control_state{};
		
		// MotorState
		struct MotorState
		{
			MotorFeedback motor_feedback{};
			i32 motor_rotation_count{0};
			bool is_injected{false};

			void update(const MotorFeedback& state) noexcept
			{
				if(std::signbit(state.angle - motor_feedback.angle) ^ std::signbit(state.speed))
				{
					++motor_rotation_count;
				}

				if(std::abs(get_total_angle() + state.angle) > max_angle)
				{
					is_injected = true;
					motor_rotation_count = 0;
				}

				motor_feedback = state;
			}

			void reset() noexcept
			{
				motor_rotation_count = 0;
				is_injected = false;
			}

			i32 get_total_angle() const noexcept
			{
				return motor_rotation_count * 8192 + motor_feedback.angle;
			}

			bool clear_if_injected() noexcept
			{
				if(is_injected)
				{
					is_injected = false;
					return true;
				}
				return false;
			}
		} motor_state{};

		// pid
		CRSLib::Math::Pid<i16> current_pid;
		CRSLib::Math::Pid<i16> speed_pid;

		public:
		Injector(const CRSLib::Math::Pid<i16>& current_pid, const CRSLib::Math::Pid<i16>& speed_pid) noexcept
			:
			current_pid(current_pid),
			speed_pid(speed_pid)
		{}

		void update_motor_state(const MotorFeedback& state) noexcept
		{
			motor_state.update(state);
		}

		void inject_start(const i16 speed) noexcept
		{
			if(std::holds_alternative<Idle>(control_state))
			{
				control_state.template emplace<Injecting>(speed);
			}
		}

		private:
		struct UpdateCurrent final
		{
			Injector& injector;

			UpdateCurrent(Injector& injector) noexcept
				:
				injector(injector)
			{}

			i16 operator()(Idle) noexcept
			{
				return injector.calc_target_current_from_speed(0.0);
			}

			i16 operator()(const Injecting injecting) noexcept
			{
				if(injector.motor_state.clear_if_injected())
				{
					injector.control_state.template emplace<Stopping>();
					return injector.calc_target_current_from_speed(0.0);
				}
				return injector.calc_target_current_from_speed(injecting.speed);
			}

			i16 operator()(Stopping) noexcept
			{
				if(injector.motor_state.motor_feedback.speed < enough_slow_speed)
				{
					injector.control_state.template emplace<SettingUp>();
				}
				return injector.calc_target_current_from_speed(0.0);
			}

			i16 operator()(SettingUp) noexcept
			{
				if(std::abs(injector.motor_state.get_total_angle() - half_angle) < enough_near_start_angle)
				{
					injector.motor_state.reset();
					injector.control_state.template emplace<Idle>();
					return injector.calc_target_current_from_speed(0.0);
				}
				return injector.calc_target_current_from_speed(setting_up_speed);
			}
		};

		public:
		i16 update_target() noexcept
		{
			return std::visit(UpdateCurrent(*this), control_state);
		}

		private:
		i16 calc_target_current_from_speed(i16 target) noexcept
		{
			const auto ret = current_pid.update(speed_pid.update(target, motor_state.motor_feedback.speed), motor_state.motor_feedback.current);
			return std::max<i16>(-16384, std::min<i16>(16384, ret));
		}
	};
}
