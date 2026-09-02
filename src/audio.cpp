#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include "game.hpp"
#include "platform.hpp"

namespace {

// General
constexpr uint sample_rate = Window::audio_sample_rate;
constexpr float inv_sample_rate = 1.0f / float(sample_rate);
constexpr float atten_near = 0.08f;  // linear distance term
constexpr float atten_far = 1e-4f;   // quadratic distance term
constexpr float dead_noise_gain = 2500.0f;

// Doppler
constexpr float speed_of_sound = 343.0f;  // m/s
constexpr float doppler_scale = 3.0f;
constexpr float doppler_min = 0.25f;
constexpr float doppler_max = 4.0f;

// Motor sound synthesis
constexpr float motor_base_gain = 4000.0f;
constexpr float motor_max_speed = 0.7f;  // motor speed at which pitch, etc is maxed out
constexpr float motor_base_freq = 250.0f;
constexpr float motor_max_freq = 500.0f;
constexpr float motor_freq_slew = 1000.0f;  // max pitch change, Hz per second
constexpr float motor_speed_gain_floor = 0.1f;
constexpr float motor_speed_gain_scale = 2.0f;

// Player motor muffling
constexpr float player_gain = 0.4f;
constexpr float player_lp_cutoff = 800.0f;  // Hz

constexpr float music_gain = 8000.0f;

struct Noise {
    uint32_t state = 1337;

    float next() {
        state = state * 1664525UL + 1013904223UL;
        return (float)(int32_t)state / (float)0x7fffffff;
    }
};

////////////////////////////////////////////////////////////////////////////////
// Effects

struct Effects {
    struct MotorAudio {
        Vec<3> cam_to_motor;
        float speed;  // 0..1
        float doppler;
        float audio_freq;
        float audio_phase;
        float lp_state;
    };
    struct DroneAudio {
        uint id;
        Drone::Kind kind;
        Drone::Status status;
        std::array<MotorAudio, 4> motors;
    };

    Vec<3> cam_pos{};
    Vec<3> cam_right{1.0f, 0.0f, 0.0f};
    Vec<3> cam_vel{};
    Vec<3> prev_cam_pos{};
    uint64_t prev_tick = 0;
    std::vector<DroneAudio> drones;
    Noise noise;

    void update(const GameState& game_state) {
        cam_pos = game_state.camera_pos;
        cam_right = rotate(game_state.camera_orientation, {0.0f, -1.0f, 0.0f});

        float dt = static_cast<float>(game_state.tick - prev_tick) * Physics::dt;
        prev_tick = game_state.tick;
        if (dt > 0.0f) {
            cam_vel = (1.0f / dt) * (game_state.camera_pos - prev_cam_pos);
            prev_cam_pos = game_state.camera_pos;
        }

        sync_parallel(
            game_state.drones, drones,
            [&](const Drone& d) {
                std::array<MotorAudio, 4> motors;
                for (auto m = 0u; m < 4; ++m) {
                    motors[m].audio_freq = 0;
                    motors[m].doppler = 1.0f;
                    motors[m].audio_phase = 0.5f * (noise.next() + 1.0f);
                }
                return DroneAudio{
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
                    auto motor_pos = d.position + rotate(d.orientation, motor_offsets_m[m]);
                    auto cam_to_motor = motor_pos - game_state.camera_pos;
                    da.motors[m].cam_to_motor = cam_to_motor;
                    da.motors[m].speed =
                        std::min(d.motor_speed[m], motor_max_speed) / motor_max_speed;

                    float dist = length(cam_to_motor);
                    if (dist > 1e-4f) {
                        Vec<3> u = (1.0f / dist) * cam_to_motor;  // listener -> source
                        float v_listener = dot(cam_vel, u);
                        float v_source = dot(d.velocity, u);
                        float doppler = (speed_of_sound + doppler_scale * v_listener) /
                                        (speed_of_sound + doppler_scale * v_source);
                        da.motors[m].doppler = std::clamp(doppler, doppler_min, doppler_max);
                    } else {
                        da.motors[m].doppler = 1.0f;
                    }
                }
            });
    }

    void render(std::vector<float>& mix_buf, float volume) {
        int samples = static_cast<int>(mix_buf.size() / 2);

        for (auto& d : drones) {
            if (d.status != Drone::Status::Alive) continue;
            for (auto& m : d.motors) {
                float dist = length(m.cam_to_motor);

                // Gain & pan
                float gain = motor_base_gain;
                gain *= 1.0f / (1 + atten_near * dist + atten_far * dist * dist);
                gain *= std::min(1.0f, m.speed * motor_speed_gain_scale + motor_speed_gain_floor);
                gain *= (d.kind == Drone::Kind::Player) ? player_gain : 1.0f;
                float pan = 0.0f;
                if (dist > 1e-4f) {
                    pan = dot(m.cam_to_motor, cam_right) / dist;
                }
                pan = std::clamp(pan, -1.0f, 1.0f);

                float left_gain = volume * gain * (0.5f - 0.5f * pan);
                float right_gain = volume * gain * (0.5f + 0.5f * pan);
                float freq_target =
                    (motor_base_freq + m.speed * (motor_max_freq - motor_base_freq)) * m.doppler;

                float lp_a = (d.kind == Drone::Kind::Player)
                                 ? 1.0f - std::exp(-2.0f * pi * player_lp_cutoff * inv_sample_rate)
                                 : 0.0f;

                // Render samples
                for (int i = 0; i < samples; ++i) {
                    constexpr float freq_step = motor_freq_slew * inv_sample_rate;
                    m.audio_freq += std::clamp(freq_target - m.audio_freq, -freq_step, freq_step);
                    m.audio_phase += m.audio_freq * inv_sample_rate;
                    if (m.audio_phase >= 1.0f) m.audio_phase -= 1.0f;
                    float saw = 2.0f * (m.audio_phase - std::floor(m.audio_phase + 0.5f));
                    float sq = (m.audio_phase < 0.5f) ? 1.0f : -1.0f;
                    float n = noise.next();
                    float sample = (0.4f * saw + 0.2f * sq + 0.4f * n);
                    if (lp_a > 0.0f) {
                        m.lp_state += lp_a * (sample - m.lp_state);
                        sample = m.lp_state;
                    }

                    mix_buf[i * 2 + 0] += sample * left_gain;
                    mix_buf[i * 2 + 1] += sample * right_gain;
                }
            }
        }

        bool player_dead = std::any_of(drones.begin(), drones.end(), [](auto& d) {
            return d.kind == Drone::Kind::Player && d.status != Drone::Status::Alive;
        });
        if (player_dead) {
            for (int i = 0; i < samples; ++i) {
                mix_buf[i * 2 + 0] += noise.next() * dead_noise_gain * volume;
                mix_buf[i * 2 + 1] += noise.next() * dead_noise_gain * volume;
            }
        }
    }
};

////////////////////////////////////////////////////////////////////////////////
// Music

struct Music {
    static float attack_decay(float t, float attack, float decay) {
        return std::max(0.0f, std::min(t / attack, 1 - (t - attack) / decay));
    }

    struct Oscillator {
        enum Type { Sine, Saw };

        Type type = Sine;
        float phase = 0;

        float operator()(float freq) {
            phase += freq * inv_sample_rate;
            phase -= std::floor(phase);
            switch (type) {
                case Type::Sine:
                    return std::sin(2 * pi * phase);
                case Type::Saw:
                    return 2.0f * (phase - std::floor(phase + 0.5f));
            }
            return 0.0f;
        }
    };

    struct Kick {
        Oscillator osc{Oscillator::Sine};
        float level = 0;
        float t = 1e3f;

        void play(float velocity) {
            level = velocity;
            t = 0;
            osc.phase = 0;
        }

        float operator()() {
            const auto duration = 0.1f;
            const auto base_freq = 200.0f;
            auto env = std::max(0.0f, 1 - t / duration);
            auto freq = base_freq * env;
            auto amp = level * env;
            t += inv_sample_rate;
            return amp * osc(freq);
        }
    };

    struct Hat {
        float t = 1e3f;
        Noise noise;

        void play() { t = 0; }

        float operator()() {
            auto amp = attack_decay(t, 0.01f, 0.08f);
            t += inv_sample_rate;
            return amp * noise.next();
        }
    };

    struct Pad {
        std::array<Oscillator, 3> oscs{Oscillator{Oscillator::Saw}, Oscillator{Oscillator::Saw},
                                       Oscillator{Oscillator::Saw}};
        float t = 1e3f;
        float freq = 0;
        float duration = 1.0f;

        void play(float f, float dur) {
            freq = f;
            duration = dur;
            t = 0;
        }

        float operator()() {
            auto amp = attack_decay(t, duration / 2, duration / 2);
            float wave = 0.0f;
            for (auto i = 0u; i < oscs.size(); ++i) {
                float detune = (float(i) - 1.0f) * 0.008f;
                wave += oscs[i](freq * (1.0f + detune));
            }
            wave /= 3.0f;
            t += inv_sample_rate;
            return amp * wave;
        }
    };

    struct Bass {
        Oscillator osc{Oscillator::Saw};
        float t = 1e3f;
        float freq = 0;
        float duration = 1.0f;
        float velocity = 1.0f;
        float lp_state = 0;

        void play(float f, float dur, float vel) {
            freq = f;
            duration = dur;
            velocity = vel;
            t = 0;
        }

        float operator()() {
            auto amp = attack_decay(t, 0.01f, duration) * (0.5f + 0.5f * velocity);
            auto wave = osc(freq);
            float cutoff = std::max(10.0f, 600.0f * (1 - t / duration) * velocity);
            float lp_a = 1.0f - std::exp(-2 * pi * cutoff * inv_sample_rate);
            lp_state += lp_a * (wave - lp_state);
            t += inv_sample_rate;
            return amp * lp_state;
        }
    };

    struct Note {
        float freq;
        float duration;
        float velocity;
    };

    static constexpr uint sub_beats = 16u;
    static constexpr uint sec_bars = 4u;
    static constexpr std::array<int, 8> minor_semitones = {0, 2, 3, 5, 7, 8, 10, 12};
    static constexpr float base_freq_A2 = 220.0f;
    static constexpr uint bass_octave_shift = 1;

    // State
    ulong sample_seed = 0;
    uint phase = 0;
    uint beat_in_bar = 0;
    uint bar_in_sec = 0;

    // Instruments
    Kick kick;
    Hat hat;
    std::array<Pad, 3> pads;
    Bass bass;

    // Tempo & Scale
    uint period;
    float sub_time;
    float bar_time;
    std::array<float, 8> scale{};

    // Tracks
    std::array<bool, sub_beats> kick_track{};
    std::array<bool, sub_beats> hat_track{};
    std::array<std::optional<Note>, sec_bars * sub_beats> bass_track{};
    std::array<std::array<float, 3>, sec_bars> pad_track{};

    // Section
    bool kick_hat_on = true;
    bool bass_on = true;
    bool pad_on = true;

    explicit Music() { reset(0); }

    void reset(uint seed) {
        // Tempo & Scale
        auto subbpm = 160u + uint(hash_u(hash(seed, 0x5a4a2a05), 0, 0.0f, 80.0f));
        period = (sample_rate * 60) / subbpm;
        sub_time = inv_sample_rate * float(period);
        bar_time = sub_beats * sub_time;
        {
            auto h = hash(seed);
            auto base_freq = base_freq_A2 * std::pow(2.0f, float(h % 12) / 12.0f) / 2;
            for (auto i = 0u; i < scale.size(); ++i)
                scale[i] = base_freq * std::pow(2.0f, float(minor_semitones[i]) / 12.0f);
        }

        // State
        phase = beat_in_bar = bar_in_sec = 0;

        // kick & hat
        kick_track.fill(false);
        hat_track.fill(false);
        for (auto i = 0u; i < sub_beats; ++i) {
            auto h = hash(seed, 0x0e965213, i);
            if (i % 4 == 0) kick_track[i] = true;
            if (i == 0) hat_track[i] = true;
            hat_track[i] |= hash_u(h, 0) < 0.5f;
            kick_track[i] |= hash_u(h, 16) < 0.25f;
        }
        // bass & pad
        {
            auto last_note = 0u;
            bass_track.fill(std::nullopt);
            bass_track[0] = {scale[0] / (1 << bass_octave_shift), sub_time, 1.0f};
            pad_track.fill(chord(0));
            auto pad_generated = 0u;
            for (auto i = 1u; i < sec_bars * sub_beats; i += 1) {
                auto bar = i / sub_beats;
                auto h = hash(seed, 0x24f65362, i);
                auto p_note = i % sub_beats == 0 ? 1.0f
                                                 : (i % 4 == 0   ? 1.0f
                                                    : i % 2 == 0 ? 0.25f
                                                                 : 0.0f);
                if (hash_u(h, 0) < p_note) {
                    auto p_jump = i % 4 == 0 ? 0.1f : 0.0f;
                    auto note = last_note;
                    if (hash_u(h, 16) < p_jump) {  // only 0,3,4,5,7
                        note = h % 5;
                        note += note >= 1;
                        note += note >= 2;
                        note += note >= 6;
                    } else if (hash_u(h, 32) < 0.5f) {
                        note = (last_note + 7) % 8;
                    } else {
                        note = (last_note + 1) % 8;
                    }
                    bass_track[i] = {scale[note] / (1 << bass_octave_shift), sub_time,
                                     hash_u(h, 48, 0.7f, 1.0f)};
                    last_note = note;
                    if (pad_generated < bar) {
                        pad_track[bar] = chord(note);
                        pad_generated = bar;
                    }
                }
            }
            auto gap_duration = 0u;
            for (auto i = int(sec_bars * sub_beats - 1); i >= 0; --i) {
                if (bass_track[i]) {
                    bass_track[i]->duration = (1 + gap_duration) * sub_time;
                    gap_duration = 0;
                } else {
                    gap_duration += 1;
                }
            }
        }
        sample_seed = hash(seed, 0x93746a7f);
    }

    std::array<float, 3> chord(uint root) {
        std::array<float, 3> freq;
        for (auto i = 0; i < 3; ++i) {
            freq[i] = scale[(root + i * 2) % 8];
        }
        return freq;
    }

    void sample_section() {
        auto u = hash_u(sample_seed, 0);
        if (u < 0.5f) {
            kick_hat_on = bass_on = pad_on = true;
        } else if (u < 0.7f) {
            kick_hat_on = pad_on = true;
            bass_on = false;
        } else if (u < 0.85f) {
            pad_on = true;
            kick_hat_on = bass_on = false;
        } else {
            kick_hat_on = true;
            bass_on = pad_on = false;
        }
    }

    void render(std::vector<float>& mix_buf, float volume) {
        for (auto i = 0u; i < mix_buf.size(); i += 2) {
            auto beat = (phase == 0);

            if (beat) {
                if (beat_in_bar == 0 && bar_in_sec == 0) {
                    sample_section();
                }
                if (kick_track[beat_in_bar]) kick.play(beat_in_bar == 0 ? 1.0f : 0.6f);
                if (hat_track[beat_in_bar]) hat.play();
                if (auto b = bass_track[bar_in_sec * sub_beats + beat_in_bar]) {
                    bass.play(b->freq, b->duration, b->velocity);
                }
                if (beat_in_bar == 0) {
                    for (auto i = 0u; i < pads.size(); ++i) {
                        pads[i].play(pad_track[bar_in_sec][i], bar_time);
                    }
                }
            }

            auto mix = kick_hat_on * (0.6f * kick() + 0.1f * hat());
            for (auto& pad : pads) {
                mix += pad_on * 0.03f * pad();
            }
            mix += bass_on * 0.15f * bass();

            mix_buf[i] += music_gain * volume * mix;
            mix_buf[i + 1] += music_gain * volume * mix;
            beat_in_bar = (beat_in_bar + beat) % sub_beats;
            bar_in_sec = (bar_in_sec + (beat && (beat_in_bar == 0))) % sec_bars;
            phase += 1;
            if (phase >= period) {
                phase = 0;
            }
            sample_seed = hash_mix(sample_seed + 1);
        }
    }
};

}  // namespace

////////////////////////////////////////////////////////////////////////////////
// AudioEngine

struct AudioEngine::State {
    std::mutex mutex;
    const Settings& settings;
    float master_volume = 1.0f;
    float music_volume = 1.0f;
    float effects_volume = 1.0f;
    std::vector<float> mix_buf;
    Effects effects;
    Music music;
    std::optional<uint64> last_seed;

    explicit State(const Settings& s) : settings(s) {}
};

AudioEngine::AudioEngine(const Settings& settings) : state(std::make_unique<State>(settings)) {}
AudioEngine::~AudioEngine() = default;

void AudioEngine::update(const GameState& game_state) {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->master_volume = state->settings.master_volume;
    state->music_volume = state->settings.music_volume;
    state->effects_volume = (game_state.paused ||
                             (game_state.finish_tick && *game_state.finish_tick <= game_state.tick))
                                ? 0.0f
                                : state->settings.effects_volume;
    if (!state->last_seed || *state->last_seed != game_state.mission.seed) {
        state->music.reset(static_cast<uint>(game_state.mission.seed));
        state->last_seed = game_state.mission.seed;
    }
    state->effects.update(game_state);
}

void AudioEngine::fill_buffer(int16_t* stream, int num_samples) {
    std::lock_guard<std::mutex> lock(state->mutex);

    state->mix_buf.assign(num_samples * 2, 0.0f);
    state->effects.render(state->mix_buf, state->effects_volume);
    state->music.render(state->mix_buf, state->music_volume);

    for (int i = 0; i < num_samples * 2; ++i) {
        gassert(std::isfinite(state->mix_buf[i]));
        stream[i] = static_cast<int16_t>(
            std::clamp(state->mix_buf[i] * state->master_volume, -32768.0f, 32767.0f));
    }
}
