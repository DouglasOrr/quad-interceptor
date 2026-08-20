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
    struct DroneAudio {
        uint id;
        Drone::Kind kind;
        // game state
        Vec<3> position;
        Quat orientation;
        Vec<4> motor_speed;
        Drone::Status status = Drone::Status::Alive;
        // audio state
        std::array<float, 4> motor_phase{};
        std::array<float, 4> motor_freq{};
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
            State::DroneAudio da;
            da.id = d.id;
            da.kind = d.kind;
            for (float& p : da.motor_phase) {
                p = 0.5f * (state->next_noise() + 1.0f);
            }
            return da;
        },
        [&](const Drone& d, auto& da) {
            da.position = d.position;
            da.orientation = d.orientation;
            da.motor_speed = d.motor_speed;
            da.status = d.status;
        });
}

void AudioEngine::fill_buffer(int16_t* stream, int num_samples) {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->mix_buf.assign(num_samples * 2, 0.0f);

    for (auto& d : state->drones) {
        if (d.status != Drone::Status::Alive) continue;
        for (int m = 0; m < 4; ++m) {
            float speed = std::min(d.motor_speed[m], motor_max_speed) / motor_max_speed;
            Vec<3> to_motor =
                d.position + rotate(d.orientation, Drone::motor_offsets_m[m]) - state->cam_pos;
            float dist = length(to_motor);

            // Gain & pan
            float gain = motor_base_gain;
            gain *= 1.0f / (1 + atten_near * dist + atten_far * dist * dist);
            gain *= std::min(1.0f, speed * motor_speed_gain_scale + motor_speed_gain_floor);
            float pan = 0.0f;
            if (dist > 1e-4f) {
                pan = dot(to_motor, state->cam_right) / dist;
            }
            pan = std::clamp(pan, -1.0f, 1.0f);

            float left_gain = gain * (0.5f - 0.5f * pan);
            float right_gain = gain * (0.5f + 0.5f * pan);

            float target = motor_base_freq + speed * (motor_max_freq - motor_base_freq);
            float freq = d.motor_freq[m];
            float phase = d.motor_phase[m];

            for (int i = 0; i < num_samples; ++i) {
                constexpr float inv_sample_rate = 1.0f / float(Window::audio_sample_rate);
                constexpr float freq_step = motor_freq_slew * inv_sample_rate;
                freq += std::clamp(target - freq, -freq_step, freq_step);
                phase += freq * inv_sample_rate;
                if (phase >= 1.0f) phase -= 1.0f;
                float saw = 2.0f * (phase - std::floor(phase + 0.5f));
                float sq = (phase < 0.5f) ? 1.0f : -1.0f;
                float noise = state->next_noise();
                float sample = (0.4f * saw + 0.2f * sq + 0.4f * noise);

                state->mix_buf[i * 2 + 0] += sample * left_gain;
                state->mix_buf[i * 2 + 1] += sample * right_gain;
            }
            d.motor_freq[m] = freq;
            d.motor_phase[m] = phase;
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
