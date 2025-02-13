/**
 * @file utils.h
 * @brief TODO
 * @author Paul-Otto Müller
 * @date 11.10.2023
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

#ifndef EXOSIM_BIOMECHANICS_UTILS_H
#define EXOSIM_BIOMECHANICS_UTILS_H

#include <ctime>
#include <mutex>
#include <deque>
#include <cstdio>
#include <chrono>
#include <vector>
#include <utility>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <type_traits>

#include <fmt/os.h>
#include <fmt/color.h>


namespace chrono::biomechanics::utils {
    /// Text styles for different message types.
    constexpr fmt::text_style INFO_MSG{fg(fmt::color::sky_blue) | fmt::emphasis::bold};
    constexpr fmt::text_style DEBUG_MSG{fg(fmt::color::white_smoke) | fmt::emphasis::italic | fmt::emphasis::faint};
    constexpr fmt::text_style ERROR_MSG{fg(fmt::color::crimson) | fmt::emphasis::bold};
    constexpr fmt::text_style WARNING_MSG{fg(fmt::color::dark_golden_rod) | fmt::emphasis::bold};

    /// Progress bar constants.
    constexpr char PBSTR[] = "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
    constexpr int PBWIDTH = 60;
    constexpr int MOVING_AVERAGE_WINDOW = 3; // Window size for the moving average

    inline std::chrono::time_point<std::chrono::steady_clock> start_time;
    inline std::deque<std::chrono::duration<double> > time_samples{}; // Stores recent time samples
    inline std::mutex progress_mutex;

    /**
     * Can be used to check if a class has a member function with a specific signature and name \p func.
     * @code{.unparsed}
     * // Check whether a class has a member function with the signature double operator()(int).
     * DEFINE_HAS_MEMBER_FUNCTION(has_member_call_operator, operator())
     * bool has_member = has_member_call_operator<T, double, int>::value
     * @endcode
     * @param name Name of the test struct.
     * @param func Name of the member function.
     */
#define DEFINE_HAS_MEMBER_FUNCTION(name, func)                                                                  \
    template<typename T, typename Ret, typename... Args>                                                        \
    struct name {                                                                                               \
        template<typename U>                                                                                    \
        static constexpr auto check(U*)                                                                         \
            -> typename std::is_same<decltype(std::declval<U>().func(std::declval<Args>()...)), Ret>::type;     \
                                                                                                                \
        template<typename>                                                                                      \
        static constexpr std::false_type check(...);                                                            \
                                                                                                                \
        static constexpr bool value = std::is_same<decltype(check<T>(0)), std::true_type>::value;               \
    };

    /**
     * Throws a \p std::invalid_argument exception with the given message if the condition is met.
     * @tparam T Type of the arguments.
     * @param cond Condition.
     * @param msg Exception message.
     * @param args Arguments to format the message.
     */
    template<typename... T>
    void throw_invalid_argument(bool cond, const std::string& msg, T&&... args) {
        if (cond) throw std::invalid_argument(fmt::format(ERROR_MSG, msg, args...));
    }

    /**
     * Throws a \p std::logic_error exception with the given message if the condition is met.
     * @tparam T Type of the arguments.
     * @param cond Condition.
     * @param msg Exception message.
     * @param args Arguments to format the message.
     */
    template<typename... T>
    void throw_logic_error(bool cond, const std::string& msg, T&&... args) {
        if (cond) throw std::logic_error(fmt::format(ERROR_MSG, msg, args...));
    }

    /**
     * Throws a \p std::runtime_error exception with the given message if the condition is met.
     * @tparam T Type of the arguments.
     * @param cond Condition.
     * @param msg Exception message.
     * @param args Arguments to format the message.
     */
    template<typename... T>
    void throw_runtime_error(bool cond, const std::string& msg, T&&... args) {
        if (cond) throw std::runtime_error(fmt::format(ERROR_MSG, msg, args...));
    }

    /**
     * Throws a \p std::out_of_range exception with the given message if the condition is met.
     * @tparam T Type of the arguments.
     * @param cond Condition.
     * @param msg Exception message.
     * @param args Arguments to format the message.
     */
    template<typename... T>
    void throw_out_of_range_error(bool cond, const std::string& msg, T&&... args) {
        if (cond) throw std::out_of_range(fmt::format(ERROR_MSG, msg, args...));
    }

    /**
     * @brief Starts the progress bar.
     */
    inline void start_progress() {
        std::lock_guard<std::mutex> lock(progress_mutex);
        start_time = std::chrono::steady_clock::now();
        time_samples.clear();
    }

    /**
     * @brief Prints a progress bar to the console.
     * Invoke \p start_progress before calling this function.
     * @param percentage The percentage of the progress (0.0 to 1.0).
     */
    inline void print_progress(double percentage) {
        using namespace std::chrono;

        // Clamp percentage between 0.0 and 1.0
        if (percentage < 0.0) percentage = 0.0;
        if (percentage > 1.0) percentage = 1.0;

        std::lock_guard<std::mutex> lock(progress_mutex);

        // Get the current time
        const auto now = steady_clock::now();

        // Calculate the elapsed time
        const auto elapsed = std::chrono::duration<double>(now - start_time);

        // Add the current sample to the deque
        time_samples.emplace_back(elapsed);

        // Keep only the most recent samples within the window size
        if (time_samples.size() > MOVING_AVERAGE_WINDOW) {
            time_samples.pop_front();
        }

        // Calculate the average elapsed time per percentage point
        double average_elapsed_perc = 0.0;
        for (const auto& sample: time_samples) {
            average_elapsed_perc += sample.count();
        }
        average_elapsed_perc /= static_cast<double>(time_samples.size());

        // Estimate the total time based on the moving average
        const double total_estimated = average_elapsed_perc / percentage;

        // Calculate the remaining time
        const double remaining_time = total_estimated - elapsed.count();

        // Convert the remaining time to hours, minutes, and seconds
        const auto remaining_hours = static_cast<int>(remaining_time) / 3600;
        const auto remaining_minutes = (static_cast<int>(remaining_time) % 3600) / 60;
        const auto remaining_seconds = static_cast<int>(remaining_time) % 60;

        const int val = static_cast<int>(percentage * 100);
        const int lpad = static_cast<int>(percentage * PBWIDTH);
        const int rpad = PBWIDTH - lpad;

        printf("\r%3d%% [%.*s%*s] - %02d:%02d:%02d", val, lpad, PBSTR, rpad, "", remaining_hours, remaining_minutes,
               remaining_seconds);
        fflush(stdout);
    }

    /**
     * @brief Measures the execution time of a given function and returns the result along with the duration.
     *
     * This function template takes a callable object (function, lambda, or functor) and its arguments,
     * measures the time taken to execute the callable, and returns the result of the callable along with the
     * execution duration in milliseconds.
     *
     * @tparam Func The type of the callable object (function, lambda, or functor).
     * @tparam Args The types of the arguments to be passed to the callable.
     *
     * @param func The callable object to be executed and measured.
     * @param args The arguments to be passed to the callable.
     *
     * @return If the callable returns a non-void type, returns a std::pair containing the result of the callable and
     *         the execution duration in milliseconds as a double. If the callable returns void, returns only the
     *         execution duration in milliseconds as a double.
     *
     * @note This function uses perfect forwarding to preserve the value categories of the callable and its arguments.
     *
     * @example
     * // Example usage with a function that returns an int
     * int square(int x) {
     *     return x * x;
     * }
     * auto [result, duration] = measure_execution_time(square, 5);
     * std::cout << "Result: " << result << ", Duration: " << duration << " ms" << std::endl;
     *
     * @example
     * // Example usage with a lambda that returns void
     * auto printMessage = [](const std::string& message) {
     *     std::cout << message << std::endl;
     * };
     * double duration = measure_execution_time(printMessage, "Hello, World!");
     * std::cout << "Duration: " << duration << " ms" << std::endl;
     */
    template<typename Func, typename... Args>
    auto measure_execution_time(Func&& func, Args&&... args) {
        const auto start = std::chrono::high_resolution_clock::now();

        if constexpr (std::is_void_v<std::invoke_result_t<Func, Args...> >) {
            // If the function returns void
            std::forward<Func>(func)(std::forward<Args>(args)...);
            const auto end = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double, std::milli> duration = end - start;
            return duration.count();
        } else {
            // If the function returns a non-void type
            auto result = std::forward<Func>(func)(std::forward<Args>(args)...);
            const auto end = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double, std::milli> duration = end - start;
            return std::make_pair(result, duration.count());
        }
    }

    /**
     * @brief Attempts to create a folder, retrying with modified names if it already exists or fails.
     *
     * This function tries to create a folder with the specified name. If the folder already exists
     * or cannot be created for some reason, the function appends a numeric suffix to the folder name
     * and retries until a unique folder is successfully created or a specified retry limit is reached.
     *
     * The function provides detailed logs for each attempt and handles errors gracefully.
     *
     * @param folder The desired name of the folder to create.
     * @param max_retries The maximum number of retries with modified folder names (default is 10).
     * @return std::string The name of the successfully created folder, or an empty string if all attempts fail.
     *
     * @note
     * - The numeric suffix format is "_1", "_2", etc., appended to the base folder name.
     * @note
     * - If the function fails after the maximum number of retries, it logs an error and returns an empty string.
     * @note
     * - The function uses `std::filesystem` for directory operations and throws exceptions for unrecoverable errors.
     *
     * @example
     * @code
     * std::string folder_name = "example_folder";
     * std::string created_folder = create_folder(folder_name);
     * if (!created_folder.empty()) {
     *     std::cout << "Folder successfully created: " << created_folder << "\n";
     * } else {
     *     std::cerr << "Failed to create folder after multiple attempts.\n";
     * }
     * @endcode
     */
    inline std::string create_folder(const std::string& folder, const int max_retries = 10) {
        namespace fs = std::filesystem;

        std::string current_folder = folder;
        int attempt = 0;

        while (attempt <= max_retries) {
            try {
                if (fs::create_directory(current_folder)) {
                    // std::cout << "Directory created successfully: " << current_folder << "\n";
                    return current_folder; // Return the name of the successfully created folder.
                }
                std::cout << "Directory already exists or could not be created: " << current_folder << "\n";
            } catch (const fs::filesystem_error& e) {
                std::cerr << "Error creating directory '" << current_folder << "': " << e.what() << '\n';
            }

            // Modify the folder name for the next attempt.
            ++attempt;
            current_folder = folder + "_" + std::to_string(attempt);
        }

        // If all attempts fail, log an error and return an empty string.
        std::cerr << "Failed to create a unique directory after " << max_retries << " attempts.\n";
        return "";
    }

    /**
     * @brief Returns the current time as a string in the format "HH:MM:SS".
     * @return The current time as a string.
     */
    inline std::string get_current_time_as_string() {
        // Get the current time
        const std::time_t t = std::time(nullptr);
        const std::tm tm = *std::localtime(&t);

        // Create a string stream to format the time
        std::ostringstream oss;
        oss << std::put_time(&tm, "%H:%M:%S");

        // Return the formatted time as a string
        return oss.str();
    }

    /**
     * @brief Attaches the current date to the given filename.
     * @param filename The filename to which the date should be attached.
     * @return The filename with the current date attached.
     */
    inline std::string attach_date_to_filename(const std::string& filename) {
        // Get the current date
        const std::time_t t = std::time(nullptr);
        const std::tm tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d");

        const std::string date_str = oss.str();

        if (const std::size_t dot_pos = filename.find_last_of('.'); dot_pos == std::string::npos) {
            // No dot found, append the date to the end
            return filename + "_" + date_str;
        } else {
            // Insert the date before the file extension
            return filename.substr(0, dot_pos) + "_" + date_str + filename.substr(dot_pos);
        }
    }

    /**
     * @brief Saves a matrix to a file in CSV format.
     * @tparam MatrixType The type of the Eigen matrix.
     * @param matrix The matrix to save.
     * @param filename The name of the file to save the matrix to.
     */
    template<typename MatrixType>
    void save_matrix_to_file(const MatrixType& matrix, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            for (int i = 0; i < matrix.rows(); ++i) {
                for (int j = 0; j < matrix.cols(); ++j) {
                    file << matrix(i, j);
                    if (j < matrix.cols() - 1) {
                        file << ",";
                    }
                }
                file << "\n";
            }
            file.close();
        } else {
            std::cerr << "[ERROR] [save_matrix_to_file] Unable to open file: " << filename << std::endl;
        }
    }

    /**
     * @brief Loads a matrix from a file in CSV format.
     * @tparam MatrixType The type of the Eigen matrix.
     * @param filename The name of the file to load the matrix from.
     * @param matrix The matrix to load the data into.
     */
    template<typename MatrixType>
    void load_matrix_from_file(const std::string& filename, MatrixType& matrix) {
        std::ifstream file(filename);
        throw_runtime_error(!file.is_open(), "[ERROR] [load_matrix_from_file] Unable to open file: {}", filename);

        std::vector<typename MatrixType::Scalar> values;
        std::string line;
        int rows = 0;
        int cols = -1;

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            int current_cols = 0;

            while (std::getline(ss, value, ',')) {
                values.push_back(static_cast<typename MatrixType::Scalar>(std::stod(value)));
                ++current_cols;
            }

            if (cols == -1) {
                cols = current_cols;
            } else if (cols != current_cols) {
                throw_runtime_error(true, "[ERROR] [load_matrix_from_file] Inconsistent number of columns in file: {}",
                                    filename);
            }

            ++rows;
        }

        file.close();

        matrix.resize(rows, cols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                matrix(i, j) = values[i * cols + j];
            }
        }
    }

    /**
     * @brief A circular buffer implementation.
     * @tparam T The type of the elements stored in the buffer.
     */
    template<typename T>
    class CircularBuffer {
        public:
            explicit CircularBuffer(size_t max_size)
                : max_size_(max_size), buffer_(max_size) {
            }

            /**
             * Add an element to the buffer.
             * @param item The element to be added.
             */
            void push(const T& item) {
                std::lock_guard<std::mutex> lock(mutex_);

                if (size_ == max_size_) {
                    // Overwrite the oldest element (advance head)
                    head_ = (head_ + 1) % max_size_;
                } else {
                    ++size_;
                }
                buffer_[tail_] = item;
                tail_ = (tail_ + 1) % max_size_;
            }

            /**
             * Remove the oldest element from the buffer.
             */
            void pop() {
                std::lock_guard<std::mutex> lock(mutex_);

                throw_out_of_range_error(empty(), "Buffer is empty");
                head_ = (head_ + 1) % max_size_;
                --size_;
            }

            /**
             * Access the oldest element.
             * @return The oldest element in the buffer.
             */
            T front() {
                std::lock_guard<std::mutex> lock(mutex_);

                throw_out_of_range_error(empty(), "Buffer is empty!");
                return buffer_[head_];
            }

            /**
             * Access the newest element.
             * @return The newest element in the buffer.
             */
            T back() {
                std::lock_guard<std::mutex> lock(mutex_);

                throw_out_of_range_error(empty(), "Buffer is empty!");
                return buffer_[(tail_ + max_size_ - 1) % max_size_];
            }

            /**
             * Access an element by index relative to the logical order.
             * @param index The index of the element to be accessed.
             * @return The element at the specified index.
             */
            T get(size_t index) {
                return this->operator[](index);
            }

            /**
             * Access an element by index relative to the logical order.
             * @param index The index of the element to be accessed.
             * @return The element at the specified index.
             */
            T& operator[](size_t index) {
                std::lock_guard<std::mutex> lock(mutex_);

                throw_out_of_range_error(index >= size_, "Index ({}) out of range!", index);
                // Map logical index to physical index in the buffer
                const size_t physical_index = (head_ + index) % max_size_;
                return buffer_[physical_index];
            }

            /**
             * Access an element by index relative to the logical order.
             * @param index The index of the element to be accessed.
             * @return The element at the specified index.
             */
            const T& operator[](size_t index) const {
                std::lock_guard<std::mutex> lock(mutex_);

                throw_out_of_range_error(index >= size_, "Index ({}) out of range!", index);
                // Map logical index to physical index in the buffer
                const size_t physical_index = (head_ + index) % max_size_;
                return buffer_[physical_index];
            }

            /**
             * Check if the buffer is empty.
             * @return \c true if the buffer is empty, \c false otherwise.
             */
            bool empty() const {
                return size_ == 0;
            }

            /**
             * Check if the buffer is full.
             * @return \c true if the buffer is full, \c false otherwise.
             */
            bool full() const {
                return size_ == max_size_;
            }

            /**
             * Get the current size of the buffer.
             * @return The number of elements in the buffer.
             */
            size_t size() const {
                return size_;
            }

            /**
             * Get the maximum size of the buffer.
             * @return The maximum number of elements the buffer can store.
             */
            size_t max_size() const {
                return max_size_;
            }

            /**
             * Clear the buffer.
             */
            void clear() {
                std::lock_guard<std::mutex> lock(mutex_);

                head_ = 0;
                tail_ = 0;
                size_ = 0;
            }

            /**
             * Search for the index of a given element.
             * @param element The element to search for.
             * @return The index of the element if found, or -1 if the element is not in the buffer.
             */
            int find_index(const T& element) const {
                std::lock_guard<std::mutex> lock(mutex_);

                for (size_t i = 0; i < size_; ++i) {
                    // Map logical index `i` to physical index
                    size_t physical_index = (head_ + i) % max_size_;
                    if (buffer_[physical_index] == element) {
                        return i; // Return logical index
                    }
                }
                return -1; // Element not found
            }

            /**
             * Move head and tail to a specific logical index.
             * @param index The index to move to.
             */
            void move_to_index(size_t index) {
                std::lock_guard<std::mutex> lock(mutex_);

                throw_out_of_range_error(index >= size_, "Index ({}) out of range for move_to_index!", index);

                // Map logical index to physical index
                const size_t new_head = (head_ + index) % max_size_;

                // Update head and tail
                head_ = new_head;
                tail_ = (head_ + size_) % max_size_;
            }

            /**
             * Change the maximum size of the buffer.
             * @param new_max_size The new maximum size of the buffer.
             */
            void resize(size_t new_max_size) {
                std::lock_guard<std::mutex> lock(mutex_);

                if (new_max_size == max_size_) {
                    return; // No change needed
                }

                std::vector<T> new_buffer(new_max_size);
                size_t new_size = std::min(size_, new_max_size);

                // Copy the most recent elements into the new buffer
                for (size_t i = 0; i < new_size; ++i) {
                    new_buffer[i] = std::move(buffer_[(head_ + i) % max_size_]);
                }

                // Update the buffer and indices
                buffer_ = std::move(new_buffer);
                max_size_ = new_max_size;
                head_ = 0;
                tail_ = new_size % max_size_;
                size_ = new_size;
            }

        private:
            mutable std::mutex mutex_;

            size_t max_size_; ///< Maximum size of the buffer
            std::vector<T> buffer_; ///< Underlying storage for the buffer
            size_t head_{0}; ///< Index of the oldest element
            size_t tail_{0}; ///< Index of the next insertion point
            size_t size_{0}; ///< Current number of elements
    };
} // namespace chrono::biomechanics::utils


#endif // EXOSIM_BIOMECHANICS_UTILS_H
