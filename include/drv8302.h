#ifndef DRV8302_H__
#define DRV8302_H__
#include "../ARM_HAL.h"
#include "../Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_gpio.h"
#include "../Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_tim.h"

class Drv8302Driver {
 public:
  Drv8302Driver(GPIO_TypeDef* hall_ph_port, uint16_t hall_ph1_pin,
                uint16_t hall_ph2_pin, uint16_t hall_ph3_pin,
                uint16_t ph_u_l,
                GPIO_TypeDef* ph_u_l_port,
                uint16_t ph_v_l,
                GPIO_TypeDef* ph_v_l_port,
                uint16_t ph_w_l,
                GPIO_TypeDef* ph_w_l_port,
                uint16_t ph_u_h,
                GPIO_TypeDef* ph_u_h_port,
                uint16_t ph_v_h,
                GPIO_TypeDef* ph_v_h_port,
                uint16_t ph_w_h,
                GPIO_TypeDef* ph_w_h_port,
                uint32_t pwmtimer_ph1,
                uint32_t pwmtimer_ph2,
                uint32_t pwmtimer_ph3,
                TIM_HandleTypeDef* pwmtimer_handler
                ) {
    hall_ph_port_ = hall_ph_port;
    hall_ph1_pin_ = hall_ph1_pin;
    hall_ph2_pin_ = hall_ph2_pin;
    hall_ph3_pin_ = hall_ph3_pin;
    ph_u_l_ = ph_u_l;
    ph_v_l_ = ph_v_l;
    ph_w_l_ = ph_w_l;
    ph_u_h_ = ph_u_h;
    ph_v_h_ = ph_v_h;
    ph_w_h_ = ph_w_h;
    ph_u_l_port_ = ph_u_l_port;
    ph_v_l_port_ = ph_v_l_port;
    ph_w_l_port_ = ph_w_l_port;
    ph_u_h_port_ = ph_u_h_port;
    ph_v_h_port_ = ph_v_h_port;
    ph_w_h_port_ = ph_w_h_port;
    pwmtimer_ph1_ = pwmtimer_ph1;
    pwmtimer_ph2_ = pwmtimer_ph2;
    pwmtimer_ph3_ = pwmtimer_ph3;
    pwmtimer_handler_ = pwmtimer_handler;
    pwm_scale_ = pwmtimer_handler_->Init.Period * 0.01;
  }
  ~Drv8302Driver() {
  }
  // Warning: Run() command needs to be in fast loop inorder to drive BLDCmotor
  // cw: true for clockwise rotation and false for ccw rotation
  // duty_cycle: value between 0-100
  void Run(bool cw, unsigned char duty_cycle) {
    // update duty_cycle before applying phase
    duty_cycle_ = (uint32_t)(duty_cycle*pwm_scale_);
    if (cw) {
      ApplyPhaseCW();
    } else {
      ApplyPhaseCCW();
    }
  }

 private:
  unsigned char GetHallFB(void) {
    unsigned char state = 0;
    state = HAL_GPIO_ReadPin(hall_ph_port_, hall_ph1_pin_);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(hall_ph_port_, hall_ph2_pin_);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(hall_ph_port_, hall_ph3_pin_);
    return state;
  }

  // GPIO_Pin:  which pin to apply the pwm
  // status:    true for applying dutycycle and false for setitng duty_cycle=0
  void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState) {
    if (PinState) {
      switch (GPIO_Pin) {
        case (ph_u_l_):
          pwmtimer_ph1_ = duty_cycle_*pwmtimer_handler_->Init.Period;
          break;
        case (ph_v_l_):
          pwmtimer_ph1_ = duty_cycle_;
          break;
        case (ph_w_l_):
          pwmtimer_ph1_ = duty_cycle_;
          break;
        default:
          break;
      }
    } else {
      switch (GPIO_Pin) {
        case (ph_u_l_):
          pwmtimer_ph1_ = 0;
          break;
        case (ph_v_l_):
          pwmtimer_ph1_ = 0;
          break;
        case (ph_w_l_):
          pwmtimer_ph1_ = 0;
          break;
        default:
          break;
      }
    }
  }

  void ApplyPhaseCW() {
    unsigned char hall_state = GetHallFB();
    uint8_t duty_motor = duty_cycle_ * 1;
    switch (hall_state) {
      case 5:
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_SET);
        break;
      case 4:
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_SET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_SET);
        break;
      case 6:
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_SET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_SET);
        break;
      case 2:
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_SET);
        break;
      case 3:
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_SET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_SET);
        break;
      case 1:
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_SET);
        break;
      default:
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);

        while (1);
        // there shouldn't be any other state positions.
    }
  }

  void ApplyPhaseCCW() {
    unsigned char hall_state = GetHallFB();
    uint8_t duty_motor = duty_cycle_ * 1;
    switch (hall_state) {
      case 4:
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_SET);
        break;
      case 6:
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_SET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_SET);
        break;
      case 2:
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_SET);
        break;
      case 3:
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_SET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_SET);
        break;
      case 1:
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_SET);
        break;
      case 5:
        HAL_GPIO_WritePin(ph_u_h_port_, ph_u_h_, GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l_, GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_w_h_port_, ph_w_h_, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ph_v_h_port_, ph_v_h_, GPIO_PIN_SET);
        GPIO_pwm(ph_w_l_, GPIO_PIN_SET);
        break;
      default:
        // if code gets here it means something is wrong with hall sensor in
        // motor
        while (1);
    }
  }

  GPIO_TypeDef* hall_ph_port_;
  uint16_t hall_ph1_pin_;
  uint16_t hall_ph2_pin_;
  uint16_t hall_ph3_pin_;
  uint32_t duty_cycle_;
  uint16_t ph_u_l_;  //        GPIO_PIN_14         //N1
  uint16_t ph_v_l_;  //        GPIO_PIN_7         //N3
  uint16_t ph_w_l_;  //        GPIO_PIN_6          //N2
  uint16_t ph_u_h_;  //        GPIO_PIN_0				//P1
  uint16_t ph_v_h_;  //        GPIO_PIN_1				//P3
  uint16_t ph_w_h_;  //        GPIO_PIN_14				//P2
  GPIO_TypeDef* ph_u_l_port_;
  GPIO_TypeDef* ph_v_l_port_;
  GPIO_TypeDef* ph_w_l_port_;
  GPIO_TypeDef* ph_u_h_port_;
  GPIO_TypeDef* ph_v_h_port_;
  GPIO_TypeDef* ph_w_h_port_;
  uint32_t pwmtimer_ph1_;
  uint32_t pwmtimer_ph2_;
  uint32_t pwmtimer_ph3_;
  TIM_HandleTypeDef* pwmtimer_handler_;
  float pwm_scale_;
}
#endif  // DRV8302_H__
