#ifndef BIPEDALV1_BUZZER_H
#define BIPEDALV1_BUZZER_H

#include <stdint.h>
#include "drivers/GPIO.h"
#include "drivers/PWM.h"
#include "drivers/Clock.h"

namespace Biped {
    class Buzzer {
    public:
        enum class Tones {
            BEEP_BEEP,
            SIREN,
            BATTERY_LOW,
            MANUAL,
        };

        Buzzer() = default;
        Buzzer(Buzzer &&buzzer) = delete;
        Buzzer(const Buzzer &buzzer) = delete;

        void initialize();
        void setDutyCycle(const float new_duty_cycle);
        void playTone(Tones tone);
        void play(unsigned int duration_ms);
        void stop();
        void update();

    private:
        Biped::PWM::PWM<Biped::PWM::Timer::TIMER3, Biped::PWM::TimerChannel::Channel3> m_pwm;
        
        uint64_t m_last_update_time = 0;
        unsigned int m_duration = 0;
        float m_duty_cycle = 0.5f;
        
        Tones m_tone = Tones::MANUAL;
        bool m_is_playing = false;
        
        // State tracking variables
        uint8_t m_sequence_step = 0;
        unsigned int m_current_freq = 1000;
        bool m_sweep_up = true;
    };
}
#endif //BIPEDALV1_BUZZER_H