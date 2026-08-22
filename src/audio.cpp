#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include "game.hpp"
#include "platform.hpp"

namespace {
constexpr float atten_near = 0.5f;  // linear distance term
constexpr float atten_far = 0.05f;  // quadratic distance term

// Motor sound synthesis
constexpr float motor_base_gain = 4000.0f;
constexpr float motor_max_speed = 0.7f;  // motor speed at which pitch, etc is maxed out
constexpr float motor_base_freq = 250.0f;
constexpr float motor_max_freq = 600.0f;
constexpr float motor_freq_slew = 1000.0f;  // max pitch change, Hz per second
constexpr float motor_speed_gain_floor = 0.1f;
constexpr float motor_speed_gain_scale = 2.0f;

constexpr float dead_noise_gain = 4000.0f;
}  // namespace

struct AudioEngine::State {
    struct MotorAudio {
        Vec<3> cam_to_motor;
        float speed;  // 0..1
        float audio_freq;
        float audio_phase;
    };
    struct DroneAudio {
        uint id;
        Drone::Kind kind;
        Drone::Status status;
        std::array<MotorAudio, 4> motors;
    };

    std::mutex mutex;
    Vec<3> cam_pos{};
    Vec<3> cam_right{1.0f, 0.0f, 0.0f};
    std::vector<DroneAudio> drones;
    std::vector<float> mix_buf;
    uint32_t rng_state = 1337;

    float next_noise() {
        rng_state = rng_state * 1664525UL + 1013904223UL;
        return (float)(int32_t)rng_state / (float)0x7fffffff;
    }
};

AudioEngine::AudioEngine() : state(std::make_unique<State>()) {}
AudioEngine::~AudioEngine() = default;

void AudioEngine::update(const GameState& game_state) {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->cam_pos = game_state.camera_pos;
    state->cam_right = rotate(game_state.camera_orientation, {0.0f, -1.0f, 0.0f});

    sync_parallel(
        game_state.drones, state->drones,
        [&](const Drone& d) {
            std::array<State::MotorAudio, 4> motors;
            for (auto m = 0u; m < 4; ++m) {
                motors[m].audio_freq = 0;
                motors[m].audio_phase = 0.5f * (state->next_noise() + 1.0f);
            }
            return State::DroneAudio{
                .id = d.id,
                .kind = d.kind,
                .status = d.status,
                .motors = motors,
            };
        },
        [&](const Drone& d, auto& da) {
            da.status = d.status;
            auto motor_offsets_m = d.type.motor_offsets_m;
            for (auto m = 0u; m < da.motors.size(); ++m) {
                da.motors[m].cam_to_motor =
                    d.position + rotate(d.orientation, motor_offsets_m[m]) - game_state.camera_pos;
                da.motors[m].speed = std::min(d.motor_speed[m], motor_max_speed) / motor_max_speed;
            }
        });
}

void AudioEngine::fill_buffer(int16_t* stream, int num_samples) {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->mix_buf.assign(num_samples * 2, 0.0f);

    for (auto& d : state->drones) {
        if (d.status != Drone::Status::Alive) continue;
        for (auto& m : d.motors) {
            float dist = length(m.cam_to_motor);

            // Gain & pan
            float gain = motor_base_gain;
            gain *= 1.0f / (1 + atten_near * dist + atten_far * dist * dist);
            gain *= std::min(1.0f, m.speed * motor_speed_gain_scale + motor_speed_gain_floor);
            float pan = 0.0f;
            if (dist > 1e-4f) {
                pan = dot(m.cam_to_motor, state->cam_right) / dist;
            }
            pan = std::clamp(pan, -1.0f, 1.0f);

            float left_gain = gain * (0.5f - 0.5f * pan);
            float right_gain = gain * (0.5f + 0.5f * pan);
            float freq_target = motor_base_freq + m.speed * (motor_max_freq - motor_base_freq);

            // Render samples
            for (int i = 0; i < num_samples; ++i) {
                constexpr float inv_sample_rate = 1.0f / float(Window::audio_sample_rate);
                constexpr float freq_step = motor_freq_slew * inv_sample_rate;
                m.audio_freq += std::clamp(freq_target - m.audio_freq, -freq_step, freq_step);
                m.audio_phase += m.audio_freq * inv_sample_rate;
                if (m.audio_phase >= 1.0f) m.audio_phase -= 1.0f;
                float saw = 2.0f * (m.audio_phase - std::floor(m.audio_phase + 0.5f));
                float sq = (m.audio_phase < 0.5f) ? 1.0f : -1.0f;
                float noise = state->next_noise();
                float sample = (0.4f * saw + 0.2f * sq + 0.4f * noise);

                state->mix_buf[i * 2 + 0] += sample * left_gain;
                state->mix_buf[i * 2 + 1] += sample * right_gain;
            }
        }
    }

    bool player_dead = std::any_of(state->drones.begin(), state->drones.end(), [](auto& d) {
        return d.kind == Drone::Kind::Player && d.status != Drone::Status::Alive;
    });
    if (player_dead) {
        for (int i = 0; i < num_samples; ++i) {
            state->mix_buf[i * 2 + 0] += state->next_noise() * dead_noise_gain;
            state->mix_buf[i * 2 + 1] += state->next_noise() * dead_noise_gain;
        }
    }

    for (int i = 0; i < num_samples * 2; ++i) {
        stream[i] = static_cast<int16_t>(std::clamp(state->mix_buf[i], -32768.0f, 32767.0f));
    }
}
