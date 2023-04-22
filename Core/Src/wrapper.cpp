#include <array>

#include "can.h"
#include "tim.h"

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/utility.hpp>
#include <CRSLibtmp/Can/Stm32/RM0008/can_bus.hpp>
#include <CRSLibtmp/Can/Stm32/RM0008/filter_manager.hpp>

#include "injector.hpp"

//PA9 TIM1_CH2
//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,???)
//???に390で右、700で正面、1100で左

using namespace CRSLib::IntegerTypes;
using namespace CRSLib::Can::Stm32::RM0008;

const char * error_msg{nullptr};

namespace Nhk23Servo
{
	void init_can_other() noexcept;

	void fifo0_callback(const ReceivedMessage& message) noexcept;
	void servo_callback(const ReceivedMessage& message) noexcept;
	void inject_callback(const ReceivedMessage& message) noexcept;

	void fifo1_callback(const ReceivedMessage& message) noexcept;
	void motor_state_callback(const ReceivedMessage& message) noexcept;

	enum Index : u8
	{
		TuskL,
		TuskR,
		Trunk
	};

	std::array<Injector, 3> injectors
	{
		Injector{20.35, CRSLib::Math::Pid<i16>{.p=1, .i=0, .d=0}},
		Injector{18.75, CRSLib::Math::Pid<i16>{.p=1, .i=0, .d=0}},
		Injector{14.85, CRSLib::Math::Pid<i16>{.p=1, .i=0, .d=0}}
	};
}

extern "C" void main_cpp()
{

	// PWMなど初期化
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	// *先に*フィルタの初期化を行う。先にCanBusを初期化すると先にNormalModeに以降してしまい、これはRM0008に違反する。
	Nhk23Servo::init_can_other();
	// 通信開始
	CanBus can_bus{can1};

	// まさか数日間動かすなんてことないだろ
	auto time = HAL_GetTick();

	while(true)
	{
		// FIFO0の受信
		{
			const auto message = can_bus.receive(Fifo::Fifo0);
			if(message) Nhk23Servo::fifo0_callback(*message);
		}

		// FIFO1の受信
		{
			const auto message = can_bus.receive(Fifo::Fifo1);
			if(message) Nhk23Servo::fifo1_callback(*message);
		}

		// C620へ電流指令値を送信
		if(const auto now = HAL_GetTick(); now - time >= 2)
		{
			CRSLib::Can::DataField data{.buffer={}, .dlc=8};
			for(u8 i = 0; auto& injector : Nhk23Servo::injectors)
			{
				const i16 target = injector.run_and_calc_target();
				std::memcpy(data.buffer + sizeof(i16) * i, &target, sizeof(i16));
				++i;
			}

			(void)can_bus.post(0x200, data);

			time = now;
		}
	}
}


namespace Nhk23Servo
{
	constexpr u32 servo_id = 0x110;
	constexpr u32 inject_speed_id_base = 0x120;  // 0x120-0x122
	constexpr u32 inject_speed_id_mask = 0x7FC;
	constexpr u32 inject_feedback_id_base = 0x130;  // 0x130-0x132
	constexpr u32 inject_feedback_id_mask = 0x7FC;
	constexpr u32 motor_state_id_base = 0x201;  // 0x201-0x203
	constexpr u32 motor_state_id_mask = 0x7FC;

	void init_can_other() noexcept
	{
		// ここでCANのMSP(ピンやクロックなど。ここまで書ききるのはキツかった...)の初期化を行う
		HAL_CAN_DeInit(&hcan);
		HAL_CAN_MspInit(&hcan);

		enum FilterName : u8
		{
			Servo,
			InjectSpeed,
			MotorState,

			N
		};

		FilterConfig filter_configs[N];
		filter_configs[Servo] = FilterConfig::make_default(Fifo::Fifo0);
		filter_configs[InjectSpeed] = FilterConfig::make_default(Fifo::Fifo0, false);
		filter_configs[MotorState] = FilterConfig::make_default(Fifo::Fifo1, false);

		// ここでフィルタの初期化を行う
		FilterManager::initialize(filter_bank_size, filter_configs);

		if(!FilterManager::set_filter(Servo, Filter{.FR1=FilterManager::make_list32(servo_id), .FR2=0}))
		{
			error_msg = "Fail to set filter for Servo";
			Error_Handler();
		}
		FilterManager::activate(Servo);

		if(!FilterManager::set_filter(InjectSpeed, FilterManager::make_mask32(inject_speed_id_base, inject_speed_id_mask)))
		{
			error_msg = "Fail to set filter for InjectSpeed";
			Error_Handler();
		}
		FilterManager::activate(InjectSpeed);

		/// @todo C620のIDを1~3に。
		if(!FilterManager::set_filter(MotorState, FilterManager::make_mask32(motor_state_id_base, motor_state_id_mask)))
		{
			error_msg = "Fail to set filter for MotorState";
			Error_Handler();
		}
		FilterManager::activate(MotorState);
	}

	//////// ここから下はコールバック関数 ////////
	/// @attention 十分に短い処理しか書かないこと。

	/// @brief fifo0のコールバック
	/// @param message
	void fifo0_callback(const ReceivedMessage& message) noexcept
	{
		if(message.id == servo_id)
		{
			servo_callback(message);
		}
		else if(inject_speed_id_base <= message.id && message.id <= inject_speed_id_base + 2)
		{
			inject_callback(message);
		}
	}

	/// @brief サーボのコールバック
	/// @param message
	void servo_callback(const ReceivedMessage& message) noexcept
	{
		switch(static_cast<Index>(message.data.buffer[0]))
		{
			case TuskL:
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1100);
			}
			break;

			case TuskR:
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,390);
			}
			break;

			case Trunk:
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,700);
			}

			default:;
		}
	}

	/// @brief インジェクターのコールバック
	/// @param message
	void inject_callback(const ReceivedMessage& message) noexcept
	{
		const auto which = static_cast<Index>(message.id - inject_speed_id_base);
		const i16 speed = (u8)message.data.buffer[1] << 8 | (u8)(message.data.buffer[0]);

		Nhk23Servo::injectors[which].inject_start(speed);
	}

	/// @brief fifo1のコールバック
	/// @param message
	void fifo1_callback(const ReceivedMessage& message) noexcept
	{
		if(0x201 <= message.id && message.id <= 0x203)
		{
			motor_state_callback(message);
		}
	}

	/// @brief モーターの状態のコールバック
	/// @param message
	void motor_state_callback(const ReceivedMessage& message) noexcept
	{
		const auto which = static_cast<Index>(message.id - motor_state_id_base);

		Feedback feedback{};
		feedback.angle = (u32)message.data.buffer[1] << 8 | (u32)(message.data.buffer[0]);
		feedback.speed = CRSLib::bit_cast<i16>((u16)((u32)message.data.buffer[3] << 8 | (u32)(message.data.buffer[2])));
		feedback.current = CRSLib::bit_cast<i16>((u16)((u32)message.data.buffer[5] << 8 | (u32)(message.data.buffer[4])));

		injectors[which].update_motor_state(feedback);
	}
}
