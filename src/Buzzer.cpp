#include "Buzzer.h"

namespace Biped {

    void Buzzer::initialize() {
        F411::Pins::B0::enableAlternateFunction<F411::Peripherals::TIMER3>();
        m_pwm.enable();
        m_pwm.setFrequency(4000); 
        m_pwm.setDutyCycle(0);
    }

    void Buzzer::setDutyCycle(const float new_duty_cycle) {
        this->m_duty_cycle = new_duty_cycle;
        if (m_is_playing) {
            m_pwm.setDutyCycle(new_duty_cycle);
        }
    }

    void Buzzer::playTone(Tones tone) {
        m_tone = tone;
        m_is_playing = true;
        m_sequence_step = 0;
        m_last_update_time = F411::Clock::millis();

        switch (tone) {
            case Tones::BEEP_BEEP:
                m_pwm.setFrequency(4000);
                m_pwm.setDutyCycle(m_duty_cycle);
                break;
            case Tones::SIREN:
                m_current_freq = 1000;
                m_sweep_up = true;
                m_pwm.setFrequency(m_current_freq);
                m_pwm.setDutyCycle(m_duty_cycle);
                break;
            case Tones::BATTERY_LOW:
                m_pwm.setFrequency(6000);
                m_pwm.setDutyCycle(m_duty_cycle);
                break;
            default:
                break;
        }
    }

    void Buzzer::play(unsigned int duration_ms) {
        m_tone = Tones::MANUAL;
        this->m_duration = duration_ms;
        this->m_last_update_time = F411::Clock::millis();
        this->m_is_playing = true;
        m_pwm.setFrequency(4000);
        m_pwm.setDutyCycle(this->m_duty_cycle);
    }

    void Buzzer::stop() {
        this->m_is_playing = false;
        m_pwm.setDutyCycle(0);
    }

    void Buzzer::update() {
        if (!m_is_playing) return;

        const uint64_t current_time = F411::Clock::millis();

        switch (m_tone) {
            case Tones::MANUAL:
                if (current_time - m_last_update_time >= m_duration) {
                    stop();
                }
                break;

            case Tones::BEEP_BEEP:
                if (m_sequence_step == 0 && (current_time - m_last_update_time >= 100)) {
                    m_pwm.setDutyCycle(0);
                    m_last_update_time = current_time;
                    m_sequence_step++;
                } else if (m_sequence_step == 1 && (current_time - m_last_update_time >= 100)) {
                    m_pwm.setDutyCycle(m_duty_cycle);
                    m_last_update_time = current_time;
                    m_sequence_step++;
                } else if (m_sequence_step == 2 && (current_time - m_last_update_time >= 100)) {
                    stop();
                }
                break;

            case Tones::SIREN:
                if (current_time - m_last_update_time >= 5) {
                    m_last_update_time = current_time;
                    
                    if (m_sweep_up) {
                        m_current_freq += 20;
                        if (m_current_freq >= 3000) m_sweep_up = false;
                    } else {
                        m_current_freq -= 20;
                        if (m_current_freq <= 1000) m_sweep_up = true;
                    }
                    
                    m_pwm.setFrequency(m_current_freq); 
                }
                break;

            case Tones::BATTERY_LOW:
                if (m_sequence_step % 2 == 0) {
                    if (current_time - m_last_update_time >= 50) {
                        m_pwm.setDutyCycle(0);
                        m_last_update_time = current_time;
                        m_sequence_step++;
                    }
                } else {
                    unsigned int pause_duration = (m_sequence_step == 5) ? 1000 : 50; 
                    
                    if (current_time - m_last_update_time >= pause_duration) {
                        if (m_sequence_step == 5) {
                            m_sequence_step = 0;
                            stop();
                        } else {
                            m_sequence_step++;
                        }
                        m_pwm.setDutyCycle(m_duty_cycle);
                        m_last_update_time = current_time;
                    }
                }
                break;
        }
    }

}
