#pragma once

#include <cmath>
#include <variant>

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/utility.hpp>
#include <CRSLibtmp/Math/pid.hpp>

namespace Nhk23Servo
{
	using namespace CRSLib::IntegerTypes;

	/// @todo 設定
	inline constexpr auto gear_ratio = 16;
	
	using Angle = u32;
	using Speed = i16;
	using Current = i16;

	template<u8 gear_ratio>
	class IjectorRecord final
	{
		Angle degree; // 精度 8192 * gear_ratio. 射出時に0になる。モーターのdegree * gear_ratio
		Speed speed; // モーターのspeed
		Current current; // モーターのcurrent
		bool is_injected;

		bool clear_if_injected() noexcept
		{
			if(is_injected)
			{
				is_injected = false;
				return true;
			}
			return false;
		}
	};

	class Injector final
	{
		struct Idle final{};
		struct Injecting final
		{
			 speed;
		};
		struct Stopping final{};
		struct SettingUp final{};

		CRSLib::Math::Pid<i16> current_pid;
		CRSLib::Math::Pid<i16> speed_pid;

		std::variant<Idle, Injecting, Stopping, SettingUp> state_machine{};

		Injector(const CRSLib::Math::Pid<float>& current_pid, const CRSLib::Math::Pid<float>& speed_pid, const IjectorRecord& initial_state) noexcept
			:
			current_pid(current_pid),
			speed_pid(speed_pid),
			motor_state(initial_state)
		{}

		// void inject_start(const float speed) noexcept
		// {
		// 	inject_speed.emplace<to_underlying(State::Injecting)>(speed);
		// }

		void update_motor_state(const IjectorRecord& state) noexcept
		{
			motor_state = state;
		}

		friend struct UpdateCurrent final
		{
			Injector& injector;

			UpdateCurrent(Injector& injector) noexcept
				:
				injector(injector)
			{}

			u16 operator()(Idle) const noexcept
			{
				return injector.calc_target_current_from_speed(0.0);
			}

			u16 operator()(const Injecting injecting) const noexcept
			{
				if(injector.motor_state.clear_if_injected())
				{
					injector.inject_speed.emplace<Stopping>();
					return injector.calc_target_current_from_speed(0.0);
				}
				return injector.calc_target_current_from_speed(injecting.speed);
			}

			u16 operator()(Stopping) const noexcept
			{
				if(std::abs(injector.motor_state.degree) < 1.0f / 60.f * gear_ratio_from_motor_to_injector && std::abs(injector.motor_state.speed) < 480 * 0.1)
				{
					injector.inject_speed.emplace<to_underlying(State::SettingUp)>();
				}
				return injector.calc_target_current_from_speed(0.0);
			}

			u16 operator()(SettingUp) const noexcept
			{
				if(injector.motor_state.degree < 0.1)
				{
					injector.inject_speed.emplace<to_underlying(State::Idle)>();
				}
				return injector.calc_target_current_from_speed(0.0);
			}
		};

		u16 update_target() noexcept
		{
			return std::visit(UpdateCurrent(*this), inject_speed);
		}

		private:
		u16 calc_target_current_from_speed(i16 target) noexcept
		{
			const auto ret = current_pid.update(speed_pid.update(target, motor_state.speed), motor_state.current);
			return std::max(-16384, std::min(16384, ret));
		}
	};
}