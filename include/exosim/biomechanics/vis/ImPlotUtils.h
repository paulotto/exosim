/**
 * @file ImPlotUtils.h
 * @brief TODO
 * @ref \link https://github.com/epezent/implot/blob/master/implot_demo.cpp \endlink
 * @ref \link https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/ \endlink
 * @author Paul-Otto Müller
 * @date 30.09.2024
 */

/*
 * Copyright (c) 2025 Paul-Otto Müller
 *
 * https://github.com/paulotto/exosim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef EXOSIM_BIOMECHANICS_IMPLOT_UTILS_H
#define EXOSIM_BIOMECHANICS_IMPLOT_UTILS_H

#include <cmath>
#include <deque>
#include <mutex>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <optional>

#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>


namespace exosim::ImPlot {
    template<typename T>
    T RandomRange(T min, T max) {
        T scale = rand() / static_cast<T>(RAND_MAX);
        return min + scale * (max - min);
    }

    inline ImVec4 RandomColor() {
        ImVec4 col;
        col.x = RandomRange(0.0f, 1.0f);
        col.y = RandomRange(0.0f, 1.0f);
        col.z = RandomRange(0.0f, 1.0f);
        col.w = 1.0f;
        return col;
    }

    inline double RandomGauss() {
        static double V1, V2, S;
        static int phase = 0;
        double X;
        if (phase == 0) {
            do {
                const double U1 = static_cast<double>(rand()) / RAND_MAX;
                const double U2 = static_cast<double>(rand()) / RAND_MAX;
                V1 = 2 * U1 - 1;
                V2 = 2 * U2 - 1;
                S = V1 * V1 + V2 * V2;
            } while (S >= 1 || S == 0);

            X = V1 * sqrt(-2 * log(S) / S);
        } else
            X = V2 * sqrt(-2 * log(S) / S);
        phase = 1 - phase;
        return X;
    }

    template<int N>
    struct NormalDistribution {
        NormalDistribution(double mean, double sd) {
            for (int i = 0; i < N; ++i)
                Data[i] = RandomGauss() * sd + mean;
        }

        double Data[N];
    };

    // Utility structure for realtime plot
    struct ScrollingVecBuffer {
        int max_size;
        int offset;
        ImVector<ImVec2> data;

        explicit ScrollingVecBuffer(int max_size)
            : max_size(max_size) {
            offset = 0;
            data.reserve(max_size);
        }

        void AddPoint(float x, float y) {
            if (data.size() < max_size)
                data.push_back(ImVec2(x, y));
            else {
                data[offset] = ImVec2(x, y);
                offset = (offset + 1) % max_size;
            }
        }

        void Clear() {
            if (!data.empty()) {
                data.shrink(0);
                offset = 0;
            }
        }
    };

    // Utility structure for realtime plot
    template <unsigned int N = 1, class T = float>
    struct ScrollingBuffer {
        int max_size;
        int offset;
        std::vector<std::array<T, N>> data{};

        explicit ScrollingBuffer(int max_size)
            : max_size(max_size), offset(0) {
            data.reserve(max_size);
        }

        void AddPoint(std::array<T, N> a) {
            if (data.size() < max_size)
                data.push_back(a);
            else {
                data[offset] = a;
                offset = (offset + 1) % max_size;
            }
        }

        void Clear() {
            if (!data.empty()) {
                data.clear();
                offset = 0;
            }
        }

        std::optional<std::array<T, N>> GetLatestValue() const {
            if (!data.empty()) {
                return data[offset == 0 ? data.size() - 1 : offset - 1];
            }
            return std::nullopt; // No data available
        }

        int FindElement(const std::array<T, N>& element) const {
            for (size_t i = 0; i < data.size(); ++i) {
                if (data[i] == element) {
                    return static_cast<int>(i);
                }
            }
            return -1; // Element not found
        }
    };

    // Utility structure for realtime plot
    struct RollingVecBuffer {
        ImVector<ImVec2> data;
        float span;
        float t_mod_last{0.0f};
        int reserve_size;

        explicit RollingVecBuffer(float span, int reserve_size = 1000)
            : span(span), reserve_size(reserve_size) {
            data.reserve(reserve_size);
        }

        void AddPoint(float x, float y) {
            const float xmod = fmodf(x, span);
            if (!data.empty() && xmod < data.back().x)
                data.shrink(0);
            data.push_back(ImVec2(xmod, y));
        }

        void AddPoint(float t, float x, float y) {
            const float xmod = fmodf(t, span);
            if (!data.empty() && xmod < t_mod_last)
                data.shrink(0);
            data.push_back(ImVec2(x, y));
            t_mod_last = xmod;
        }

        void Clear() {
            if (!data.empty())
                data.shrink(0);
        }
    };

    // Utility structure for realtime plot
    template <unsigned int N = 1, class T = float>
    struct RollingBuffer {
        std::vector<std::array<T, N>> data{};
        float span;
        float t_mod_last{0.0f};
        int reserve_size;

        explicit RollingBuffer(float span, int reserve_size = 1000)
            : span(span), reserve_size(reserve_size) {
            data.reserve(reserve_size);
        }

        void AddPoint(std::array<T, N> a) {
            const auto t_mod = fmodf(a[0], span);
            if (!data.empty() && t_mod < t_mod_last)
                data.clear();
            data.push_back(a);
            t_mod_last = t_mod;
        }

        void Clear() {
            if (!data.empty())
                data.clear();
        }

        std::optional<std::array<T, N>> GetLatestValue() const {
            if (!data.empty()) {
                return data.back();
            }
            return std::nullopt; // No data available
        }

        int FindElement(const std::array<T, N>& element) const {
            for (size_t i = 0; i < data.size(); ++i) {
                if (data[i] == element) {
                    return static_cast<int>(i);
                }
            }
            return -1; // Element not found
        }

        void RemoveElementsFrom(const std::array<T, N>& element) {
            int index = FindElement(element);
            if (index != -1) {
                data.erase(data.begin() + index, data.end());
            }
        }
    };

    template<class T>
    class CircularBuffer {
        public:
            explicit CircularBuffer(size_t size)
                : buf_(std::unique_ptr<T[]>(new T[size])),
                  max_size_(size) {
            }

            void put(T item) {
                std::lock_guard<std::mutex> lock(mutex_);

                buf_[head_] = item;

                if (full_) {
                    tail_ = (tail_ + 1) % max_size_;
                }

                head_ = (head_ + 1) % max_size_;
                full_ = head_ == tail_;
            }

            std::optional<T> get() {
                std::lock_guard<std::mutex> lock(mutex_);

                if (empty()) {
                    return std::nullopt;
                }

                // Read data and advance the tail (we now have a free space)
                auto val = buf_[tail_];
                full_ = false;
                tail_ = (tail_ + 1) % max_size_;

                return val;
            }

            void reset() {
                std::lock_guard<std::mutex> lock(mutex_);
                head_ = tail_;
                full_ = false;
            }

            bool empty() const {
                // If head and tail are equal, we are empty
                return (!full_ && (head_ == tail_));
            }

            bool full() const {
                // If tail is ahead the head by 1, we are full
                return full_;
            }

            size_t capacity() const { return max_size_; }

            size_t size() const {
                size_t size = max_size_;

                if (!full_) {
                    if (head_ >= tail_) {
                        size = head_ - tail_;
                    } else {
                        size = max_size_ + head_ - tail_;
                    }
                }

                return size;
            }

        private:
            std::mutex mutex_;
            std::unique_ptr<T[]> buf_{};
            size_t head_ = 0;
            size_t tail_ = 0;
            const size_t max_size_;
            bool full_ = false;
    };

    // Huge data used by Time Formatting example (~500 MB allocation!)
    struct HugeTimeData {
        explicit HugeTimeData(double min) {
            Ts = new double[Size];
            Ys = new double[Size];
            for (int i = 0; i < Size; ++i) {
                Ts[i] = min + i;
                Ys[i] = GetY(Ts[i]);
            }
        }

        ~HugeTimeData() {
            delete[] Ts;
            delete[] Ys;
        }

        static double GetY(double t) {
            return 0.5 + 0.25 * sin(t / 86400 / 12) + 0.005 * sin(t / 3600);
        }

        double* Ts;
        double* Ys;
        static const int Size = 60 * 60 * 24 * 366;
    };
} // namespace exosim::ImPlot

#endif // EXOSIM_BIOMECHANICS_IMPLOT_UTILS_H
