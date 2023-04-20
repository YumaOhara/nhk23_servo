#include <ratio>

#include "can.h"
#include "tim.h"

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/Can/Stm32/RM0008/can_bus.hpp>

#include <injector.hpp>

//PA9 TIM1_CH2
//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,???)
//???に390で右、700で正面、1100で左

using namespace CRSLib::Can::Stm32::RM0008;
using namespace Nhk23Servo;

const char * error_msg{nullptr};

namespace
{
	void init_can_other() noexcept;

	void fifo0_callback(const ReceivedMessage& message) noexcept;
	void servo_callback(const ReceivedMessage& message) noexcept;
	void inject_callback(const ReceivedMessage& message) noexcept;

	void fifo1_callback(const ReceivedMessage& message) noexcept;
	void motor_state_callback(const ReceivedMessage& message) noexcept;

	enum InjectMotor : u8
	{
		TuskL,
		TuskR,
		Trunk
	};

	Injector<std::ratio<2035, 100>{}> tusl_l
	{
		CRSLib::Math::Pid{i16{0}, i16{0}, i16{0}},
		CRSLib::Math::Pid{i16{0}, i16{0}, i16{0}}
	};

	Injector<std::ratio<1875, 100>{}> tusl_r
	{
		CRSLib::Math::Pid{i16{0}, i16{0}, i16{0}},
		CRSLib::Math::Pid{i16{0}, i16{0}, i16{0}}
	};

	Injector<std::ratio<1485, 100>{}> trunk
	{
		CRSLib::Math::Pid{i16{0}, i16{0}, i16{0}},
		CRSLib::Math::Pid{i16{0}, i16{0}, i16{0}}
	};

	constexpr u32 servo_id = 0x1A0;
	constexpr u32 inject_speed_id_base = 0x1B0;
	constexpr u32 inject_speed_id_mask = 0x7FC;
	constexpr u32 inject_feedback_id_base = 0x1B4;
	constexpr u32 inject_feedback_id_mask = 0x7F3;
	constexpr u32 motor_state_id_base = 0x201;
	constexpr u32 motor_state_id_mask = 0x7FC;
}

extern "C" void main_cpp()
{
	// PWMなど初期化
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	// *先に*フィルタの初期化を行う。先にCanBusを初期化すると先にNormalModeに以降してしまい、これはRM0008に違反する。
	init_can_other();
	// 通信開始
	CanBus can_bus{can1};

	while(true)
	{
		// FIFO0の受信
		{
			const auto message = can_bus.receive(Fifo::Fifo0);
			if(message) fifo0_callback(*message);
		}

		// FIFO1の受信
		{
			const auto message = can_bus.receive(Fifo::Fifo1);
			if(message) fifo1_callback(*message);
		}

		CRSLib::Can::DataField data{.buffer={}, .dlc=8};
		u16 tmp = tusk_l.update_target();
		std::memcpy(data.buffer, &tmp, sizeof(tmp));
		tmp = tusk_r.update_target();
		std::memcpy(data.buffer + 2, &tmp, sizeof(tmp));
		tmp = trunk.update_target();
		std::memcpy(data.buffer + 4, &tmp, sizeof(tmp));

		can_bus.post(0x200, data);
	}
}


namespace
{
	void init_can_other() noexcept
	{
		// ここでCANのMSP(ピンやクロックなど。ここまで書ききるのはキツかった...)の初期化を行う
		HAL_CAN_DeInit(&hcan);
		HAL_CAN_MspInit(&hcan);

		enum FilterName : u8
		{
			Servo,
			InjectSpeed,
			MotorState
		};

		FilterConfig filter_configs[3];
		filter_configs[Servo] = FilterConfig::make_default(Fifo::Fifo0);
		filter_configs[InjectSpeed] = FilterConfig::make_default(Fifo::Fifo0);
		filter_configs[MotorState] = FilterConfig::make_default(Fifo::Fifo1);

		// ここでフィルタの初期化を行う
		FilterManager::initialize(filter_bank_size, filter_configs);

		if(!FilterManager::set_filter(Servo, FilterManager::make_mask32(0x190, 0x7FF)))
		{
			error_msg = "Fail to set filter for Servo";
			Error_Handler();
		}
		FilterManager::activate(Servo);

		if(!FilterManager::set_filter(InjectSpeed, FilterManager::make_mask32(0x1A0, 0x7FC)))
		{
			error_msg = "Fail to set filter for InjectSpeed";
			Error_Handler();
		}
		FilterManager::activate(InjectSpeed);

		/// @todo C620のIDを1~3に。
		if(!FilterManager::set_filter(MotorState, FilterManager::make_mask32(0x201, 0x7FC)))
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
		if(message.id == 0x190)
		{
			servo_callback(message);
		}
		else if(message.id == 0x1A0)
		{
			inject_callback(message);
		}
	}

	/// @brief サーボのコールバック
	/// @param message
	void servo_callback(const ReceivedMessage& message) noexcept
	{
		switch((InjectMotor)message.data.buffer[0])
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
		}
	}

	/// @brief インジェクターのコールバック
	/// @param message
	void inject_callback(const ReceivedMessage& message) noexcept
	{
		const auto which_motor = (InjectMotor)(message.id - 0x201);
		const i16 speed = (u8)message.data.buffer[0] | (u8)(message.data.buffer[1] << 8);

		switch(which_motor)
		{
			case TuskL:
			{
				tusl_l.inject_start(speed);
			}
			break;

			case TuskR:
			{
				tusl_r.inject_start(speed);
			}
			break;

			case Trunk:
			{
				trunk.inject_start(speed);
			}
		}
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
		const auto which_motor = message.id - 0x201;

		switch(which_motor)
		{
			case TuskL:
			{
				tusl_l.update_motor_state(message.data.buffer);
			}
			break;

			case TuskR:
			{
				tusl_r.update_motor_state(message.data.buffer);
			}
			break;

			case Trunk:
			{
				trunk.update_motor_state(message.data.buffer);
			}
		}
	}
}
