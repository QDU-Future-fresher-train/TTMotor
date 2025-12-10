#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: 
  - motor_pwm_name: pwm_tim1_ch1
  - motor_in1_name: AIN1
  - motor_in2_name: AIN2
  - enc_a_name: ENCA
  - enc_b_name: ENCB
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "gpio.hpp"
#include "libxr_time.hpp"
#include "pwm.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"
#include <sys/_intsup.h>

class TTMotor : public LibXR::Application {
public:
  TTMotor(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
         const char *pwm_name, const char *in1_name, const char *in2_name,
         const char *enc_a_name, const char *enc_b_name)
      : motor_pwm_(hw.template Find<LibXR::PWM>(pwm_name)),
        motor_in1_(hw.template Find<LibXR::GPIO>(in1_name)),
        motor_in2_(hw.template Find<LibXR::GPIO>(in2_name)),
        enc_a_(hw.template Find<LibXR::GPIO>(enc_a_name)),
        enc_b_(hw.template Find<LibXR::GPIO>(enc_b_name)) {
    motor_in1_->SetConfig(
        {.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
         .pull = LibXR::GPIO::Pull::NONE});
    motor_in2_->SetConfig(
        {.direction = LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
         .pull = LibXR::GPIO::Pull::NONE});
    enc_a_->SetConfig({.direction = LibXR::GPIO::Direction::INPUT,
                       .pull = LibXR::GPIO::Pull::UP});
    enc_b_->SetConfig({.direction = LibXR::GPIO::Direction::INPUT,
                       .pull = LibXR::GPIO::Pull::UP});
    uint8_t a = static_cast<uint8_t>(enc_a_->Read());
    uint8_t b = static_cast<uint8_t>(enc_b_->Read());
    last_ab_state_ = static_cast<uint8_t>((a << 1) | b);
    motor_pwm_->SetConfig({.frequency = 1000});
    auto timer_func = LibXR::Timer::CreateTask(TimerFunc, this, 50);
    LibXR::Timer::Add(timer_func);
    LibXR::Timer::Start(timer_func);
    app.Register(*this);
  }
  static void TimerFunc(TTMotor *self) {
    auto now = LibXR::Timebase::GetMilliseconds();
    self->dt_ = (now - self->last_time_).ToSecondf();
    self->last_time_ = now;

    uint8_t a = static_cast<uint8_t>(self->enc_a_->Read());
    uint8_t b = static_cast<uint8_t>(self->enc_b_->Read());
    uint8_t ab = static_cast<uint8_t>((a << 1) | b);

    uint8_t transition = static_cast<uint8_t>((self->last_ab_state_ << 2) | ab);
    int8_t delta = 0;
    switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      delta = +1;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      delta = -1;
      break;
    default:
      delta = 0;
      break;
    }
    self->encoder_count_ += delta;
    self->last_ab_state_ = ab;

    constexpr float PPR_OUT = 1152.0f; // 输出轴一圈 1152 脉冲

    int32_t delta_count = self->encoder_count_ - self->last_count_;
    self->last_count_ = self->encoder_count_;

    float rev = static_cast<float>(delta_count) / PPR_OUT;
    self->rpm_ = (rev / self->dt_) * 60.0f;
  }

  float GetRPM() const { return rpm_; }

  void RPMControl(float target_rpm) {
    motor_pwm_->Enable();
    float out = std::clamp(target_rpm, -100.0f, 100.0f);
    if (out > 0.0f) {
      motor_in1_->Write(true);
      motor_in2_->Write(false);
      motor_pwm_->SetDutyCycle(out / 100.0f);
    } else if (out < 0.0f) {
      motor_in1_->Write(false);
      motor_in2_->Write(true);
      motor_pwm_->SetDutyCycle(-out / 100.0f);
    } else {
      motor_in1_->Write(false);
      motor_in2_->Write(false);
      motor_pwm_->SetDutyCycle(0.0f);
    }
  }

  void Relax() {
    motor_in1_->Write(false);
    motor_in2_->Write(false);
    motor_pwm_->SetDutyCycle(0.0f);
    motor_pwm_->Disable();
  }

  void OnMonitor() override {}

private:
  LibXR::PWM *motor_pwm_;
  LibXR::GPIO *motor_in1_;
  LibXR::GPIO *motor_in2_;
  LibXR::GPIO *enc_a_;
  LibXR::GPIO *enc_b_;

  float dt_;
  LibXR::MillisecondTimestamp last_time_;

  int32_t encoder_count_ = 0;
  int32_t last_count_ = 0;
  float rpm_ = 0.0f;
  uint8_t last_ab_state_ = 0;
};
