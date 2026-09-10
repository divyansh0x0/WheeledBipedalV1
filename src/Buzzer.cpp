#include "Buzzer.h"

namespace Biped {

    void Buzzer::initialize() {
        Biped::Pins::B0::enableAlternateFunction<Biped::Peripherals::TIMER3>();
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
        m_last_update_time = Biped::Clock::millis();

        // Initial hardware configuration for the chosen sequence
        switch (tone) {
            case Tones::BEEP_BEEP:
                m_pwm.setFrequency(4000); // Standard beep pitch
                m_pwm.setDutyCycle(m_duty_cycle);
                break;
            case Tones::SIREN:
                m_current_freq = 1000;    // Start at low pitch
                m_sweep_up = true;
                m_pwm.setFrequency(m_current_freq);
                m_pwm.setDutyCycle(m_duty_cycle);
                break;
            case Tones::BATTERY_LOW:
                m_pwm.setFrequency(6000); // High pitch alert
                m_pwm.setDutyCycle(m_duty_cycle);
                break;
            default:
                break;
        }
    }

    void Buzzer::play(unsigned int duration_ms) {
        m_tone = Tones::MANUAL;
        this->m_duration = duration_ms;
        this->m_last_update_time = Biped::Clock::millis();
        this->m_is_playing = true;
        m_pwm.setFrequency(4000);
        m_pwm.setDutyCycle(this->m_duty_cycle);
    }

    void Buzzer::stop() {
        this->m_is_playing = false;
        m_pwm.setDutyCycle(0); // Instantly silences the hardware output
    }

    void Buzzer::update() {
        if (!m_is_playing) return;

        const uint64_t current_time = Biped::Clock::millis();

        switch (m_tone) {
            case Tones::MANUAL:
                if (current_time - m_last_update_time >= m_duration) {
                    stop();
                }
                break;

            case Tones::BEEP_BEEP:
                // Step 0: Beep 1 (ON)  - 100ms
                // Step 1: Pause (OFF)  - 100ms
                // Step 2: Beep 2 (ON)  - 100ms
                // Step 3: Done
                if (m_sequence_step == 0 && (current_time - m_last_update_time >= 100)) {
                    m_pwm.setDutyCycle(0); // Hardware off
                    m_last_update_time = current_time;
                    m_sequence_step++;
                } else if (m_sequence_step == 1 && (current_time - m_last_update_time >= 100)) {
                    m_pwm.setDutyCycle(m_duty_cycle); // Hardware on
                    m_last_update_time = current_time;
                    m_sequence_step++;
                } else if (m_sequence_step == 2 && (current_time - m_last_update_time >= 100)) {
                    stop();
                }
                break;

            case Tones::SIREN:
                // Continuously sweeps the ARR (frequency) register up and down
                if (current_time - m_last_update_time >= 5) { // 5ms step interval
                    m_last_update_time = current_time;
                    
                    if (m_sweep_up) {
                        m_current_freq += 20;
                        if (m_current_freq >= 3000) m_sweep_up = false;
                    } else {
                        m_current_freq -= 20;
                        if (m_current_freq <= 1000) m_sweep_up = true;
                    }
                    
                    // Modifies the hardware Auto-Reload Register dynamically
                    m_pwm.setFrequency(m_current_freq); 
                }
                break;

            case Tones::BATTERY_LOW:
                // 3 rapid, high-pitched chirps followed by a long pause, looped indefinitely
                if (m_sequence_step % 2 == 0) {
                    // Even steps are ON (Chirps)
                    if (current_time - m_last_update_time >= 50) {
                        m_pwm.setDutyCycle(0);
                        m_last_update_time = current_time;
                        m_sequence_step++;
                    }
                } else {
                    // Odd steps are OFF (Pauses)
                    unsigned int pause_duration = (m_sequence_step == 5) ? 1000 : 50; 
                    
                    if (current_time - m_last_update_time >= pause_duration) {
                        if (m_sequence_step == 5) {
                            m_sequence_step = 0; // Reset loop after long pause
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
