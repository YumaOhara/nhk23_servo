#include "main.h"

#include <CRSLibtmp/std_type.hpp>
#include <CRSLibtmp/Can/Stm32/RM0008/can_bus.hpp>

#include <injector.hpp>

//PA9 TIM1_CH2
//__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,???)
//???に390で右、700で正面、1100で左

using namespace CRSLib::Can::Stm32::RM0008;
using namespace Nhk23Servo;

extern "C" const char * error_msg{nullptr};

namespace
{
	void init_can_other(CAN_HandleTypeDef *const hcan) noexcept;

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

	Injector injectors[3]
	{
		Injector{CRSLib::Math::Pid<float>{0.1f, 0.1f, 0.1f}, CRSLib::Math::Pid<float>{0.1f, 0.1f, 0.1f}}, // TuskL
		Injector{CRSLib::Math::Pid<float>{0.1f, 0.1f, 0.1f}, CRSLib::Math::Pid<float>{0.1f, 0.1f, 0.1f}}, // TuskR
		Injector{CRSLib::Math::Pid<float>{0.1f, 0.1f, 0.1f}, CRSLib::Math::Pid<float>{0.1f, 0.1f, 0.1f}} // Trunk
	};
}

extern "C" void main_cpp(CAN_HandleTypeDef *const hcan);

void main_cpp(CAN_HandleTypeDef *const hcan)
{
	// PWMなど初期化
	
	// *先に*フィルタの初期化を行う。先にCanBusを初期化すると先にNormalModeに以降してしまい、これはRM0008に違反する。
	init_can_other(hcan);
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
	}
}


namespace
{
	void init_can_other(CAN_HandleTypeDef *const hcan) noexcept
	{
		// ここでCANのMSP(ピンやクロックなど。ここまで書ききるのはキツかった...)の初期化を行う
		HAL_CAN_DeInit(hcan);
		HAL_CAN_MspInit(hcan);

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
		FilterManager::activate(GreetFromRos);

		if(!FilterManager::set_filter(InjectSpeed, FilterManager::make_mask32(0x1A0, 0x7FF)))
		{
			error_msg = "Fail to set filter for InjectSpeed";
			Error_Handler();
		}
		FilterManager::activate(InjectSpeed);
		constexpr u32 inject_feedback = 0x1A1;

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
		switch(message.data.buffer[0])
		{
			case TuskL:
			{
				
			}
			break;

			case TuskR:
			{

			}
			break;

			case Trunk:
			{

			}
		}
	}

	/// @brief インジェクターのコールバック
	/// @param message
	void inject_callback(const ReceivedMessage& message) noexcept
	{

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
		
		MotorState motor_state;
		motor_state.degree = message.data[0] | (message.data[1] << 8);
		motor_state.speed = message.data[2] | (message.data[3] << 8);
		motor_state.current = message.data[4] | (message.data[5] << 8);


	}
}
