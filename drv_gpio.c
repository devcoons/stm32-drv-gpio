/*!
	@file   drv_gpio.c
    @brief  <brief description here>
	@t.odo	-
	---------------------------------------------------------------------------

	MIT License
	Copyright (c) 2020 Daniele Russo, Ioannis Deligiannis, Federico Carnevale

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/
/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/

#include "drv_gpio.h"

#ifdef DRV_GPIO_ENABLED
/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

static gpio_t* gpio_interfaces[64] = {NULL};
static uint32_t gpio_interfaces_cnt = 0;

#ifdef DRV_GPIO_TIM
	static uint8_t is_timer_initialized = 0;
#endif

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

/******************************************************************************
* Definition  | Static Functions
******************************************************************************/

#ifdef DRV_GPIO_TIM

	static void MX_GPIO_TIM_Init(void)
	{
		if(HAL_TIM_Base_GetState(&DRV_GPIO_TIM_HANDLER) != HAL_TIM_STATE_RESET)
			return;

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  DRV_GPIO_TIM_HANDLER.Instance = DRV_GPIO_TIM;
	  DRV_GPIO_TIM_HANDLER.Init.Prescaler = (HAL_RCC_GetPCLK2Freq()/10000000) - 1;
	  DRV_GPIO_TIM_HANDLER.Init.CounterMode = TIM_COUNTERMODE_UP;
	  DRV_GPIO_TIM_HANDLER.Init.Period = 199;
	  DRV_GPIO_TIM_HANDLER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  DRV_GPIO_TIM_HANDLER.Init.RepetitionCounter = 0;
	  DRV_GPIO_TIM_HANDLER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	  if (HAL_TIM_Base_Init(&DRV_GPIO_TIM_HANDLER) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	  if (HAL_TIM_ConfigClockSource(&DRV_GPIO_TIM_HANDLER, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&DRV_GPIO_TIM_HANDLER, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  HAL_TIM_Base_Start_IT(&DRV_GPIO_TIM_HANDLER);
	}

#endif

	static void gpio_enable_port_clock(gpio_t* handler)
	{
		if(handler->port == GPIOA)
		{
			__HAL_RCC_GPIOA_CLK_ENABLE();
		}
		else if(handler->port == GPIOB)
		{
			__HAL_RCC_GPIOB_CLK_ENABLE();
		}
		else if(handler->port == GPIOC)
		{
			__HAL_RCC_GPIOC_CLK_ENABLE();
		}
		else if(handler->port == GPIOD)
		{
			__HAL_RCC_GPIOD_CLK_ENABLE();
		}
		else if(handler->port == GPIOE)
		{
			__HAL_RCC_GPIOE_CLK_ENABLE();
		}
		else if(handler->port == GPIOF)
		{
			__HAL_RCC_GPIOF_CLK_ENABLE();
		}
		else if(handler->port == GPIOG)
		{
			__HAL_RCC_GPIOG_CLK_ENABLE();
		}
		else if(handler->port == GPIOH)
		{
			__HAL_RCC_GPIOH_CLK_ENABLE();
		}
	#if defined(GPIOI)
		else if(handler->port == GPIOI)
		{
			__HAL_RCC_GPIOI_CLK_ENABLE();
		}
	#endif
		else if(handler->port == GPIOJ)
		{
			__HAL_RCC_GPIOJ_CLK_ENABLE();
		}
		else if(handler->port == GPIOK)
		{
			__HAL_RCC_GPIOK_CLK_ENABLE();
		}
	}

/******************************************************************************
* Definition  | Public Functions
******************************************************************************/

i_status gpio_initialize(gpio_t* handler)
{
#ifdef DRV_GPIO_TIM
	if(is_timer_initialized == 0)
	{
		MX_GPIO_TIM_Init();
		is_timer_initialized = 1;
	}

	handler->_send_custom_high = 0;
	handler->_send_custom_low = 0;
#endif

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	gpio_enable_port_clock(handler);
	switch(handler->type)
	{
		case GPIO_INPUT:
			GPIO_InitStruct.Pin = handler->pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(handler->port, &GPIO_InitStruct);
		break;
		case GPIO_OUTPUT:
			HAL_GPIO_WritePin(handler->port, handler->pin,  handler->init_state);
			GPIO_InitStruct.Pin = handler->pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
			HAL_GPIO_Init(handler->port, &GPIO_InitStruct);
		break;
		case GPIO_INTERRUPT:
			GPIO_InitStruct.Pin =  handler->pin;
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(handler->port, &GPIO_InitStruct);
		break;
		default:
		return I_ERROR;
	}

	for(register uint32_t i=0;i<gpio_interfaces_cnt;i++)
		if(gpio_interfaces[i] == handler)
			return I_OK;

	gpio_interfaces[gpio_interfaces_cnt] = handler;
	gpio_interfaces_cnt++;

	return I_OK;
}

i_status gpio_send_lowpulse(gpio_t* handler, uint32_t microseconds)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = handler->pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_WritePin(handler->port, handler->pin, GPIO_PIN_SET);
	HAL_GPIO_DeInit(handler->port, handler->pin);
	HAL_GPIO_Init(handler->port, &GPIO_InitStruct);
	handler->_send_custom_low = 1 + (microseconds / 10);
	return I_OK;
}

i_status gpio_send_highpulse(gpio_t* handler, uint32_t microseconds)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = handler->pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_WritePin(handler->port, handler->pin, GPIO_PIN_RESET);
	HAL_GPIO_DeInit(handler->port, handler->pin);
	HAL_GPIO_Init(handler->port, &GPIO_InitStruct);
	handler->_send_custom_high =1 + (microseconds / 10);
	return I_OK;
}

i_status gpio_set_state(gpio_t* handler, GPIO_PinState state)
{
	if(handler->type != GPIO_OUTPUT)
		return I_INVALID;

	HAL_GPIO_WritePin(handler->port, handler->pin, state);
	return I_OK;
}

i_status gpio_get_state(gpio_t* handler, GPIO_PinState* state)
{
	if(handler->type == GPIO_OUTPUT)
		return I_INVALID;

	*state = HAL_GPIO_ReadPin(handler->port, handler->pin);
	return I_OK;
}

void gpio_tim_complete_cb(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == DRV_GPIO_TIM)
	{
		for(register uint32_t i=0;i<gpio_interfaces_cnt;i++)
		{
			if(gpio_interfaces[i] != NULL)
			{
				if(gpio_interfaces[i]->_send_custom_low != 0)
				{
					HAL_GPIO_WritePin(gpio_interfaces[i]->port, gpio_interfaces[i]->pin, GPIO_PIN_RESET);
					gpio_interfaces[i]->_send_custom_low--;
					if(gpio_interfaces[i]->_send_custom_low == 0)
					{
						HAL_GPIO_WritePin(gpio_interfaces[i]->port, gpio_interfaces[i]->pin, GPIO_PIN_SET);
						gpio_initialize(gpio_interfaces[i]);
					}
				}
				if(gpio_interfaces[i]->_send_custom_high != 0)
				{
					HAL_GPIO_WritePin(gpio_interfaces[i]->port, gpio_interfaces[i]->pin, GPIO_PIN_SET);
					gpio_interfaces[i]->_send_custom_high--;
					if(gpio_interfaces[i]->_send_custom_high == 0)
					{
						HAL_GPIO_WritePin(gpio_interfaces[i]->port, gpio_interfaces[i]->pin, GPIO_PIN_RESET);
						gpio_initialize(gpio_interfaces[i]);
					}
				}
			}
		}
	}
}

void gpio_int_complete_cb(uint16_t GPIO_Pin)
{
	static gpio_callback_t* callback_item = NULL;

	for(register uint32_t i=0;i<gpio_interfaces_cnt;i++)
	{
		if(gpio_interfaces[i] != NULL)
		{
			if(gpio_interfaces[i]->pin == GPIO_Pin && gpio_interfaces[i]->type == GPIO_INTERRUPT)
			{
				GPIO_PinState state = HAL_GPIO_ReadPin(gpio_interfaces[i]->port, gpio_interfaces[i]->pin);
				callback_item = gpio_interfaces[i]->callbacks;
				while(callback_item!=NULL)
				{
					callback_item->callback(state);
					callback_item = callback_item->next;
				}
			}
		}
	}
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif
