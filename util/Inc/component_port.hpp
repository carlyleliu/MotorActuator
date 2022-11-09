#ifndef __MIDDLEWARE_UTILS_COMPONENT_PORT_HPP__
#define __MIDDLEWARE_UTILS_COMPONENT_PORT_HPP__

#include <stdint.h>
#include <optional>
#include <variant>

#include <algorithm>

enum OperatorNum
{
    OPERATOR_PRESENT = 0,
    OPERATOR_PREVIOUS,
    OPERATOR_NUM,
};

template<typename T>
class InputPort;

/**
 * @brief An output port stores a value for consumption by a connecting input
 * port.
 *
 * Output ports are supposed to be reset at the beginning of a control loop
 * iteration. This ensures that connecting input ports don't use an outdated
 * value and, more importantly, ensures proper handling if the producer of the
 * value is incapable of producing the value for any reason.
 *
 * Member functions of this class are not thread-safe unless noted otherwise.
 */
template<typename T>
class OutputPort {
public:
    /**
     * @brief Initializes the output port with the specified value.
     *
     * An initialization value is required for any() to work properly.
     * present() and previous() cannot be used to fetch the
     * initialization value.
     */
    OutputPort(T val) : content_(val) {}

    /**
     * @brief Updates the underlying value of this output port.
     */
    void operator=(T value) {
        content_ = value;
        operator_idx_ = OPERATOR_PRESENT;
    }

    /**
     * @brief Marks the contained value as outdated. The value is not actually
     * deleted and can still be accessed through some of the member functions
     * of this class.
     */
    void Reset() {
        operator_idx_ = OPERATOR_PREVIOUS;
    }

    /**
     * @brief Returns the value from this control loop iteration or std::nullopt
     * if the value was not yet set during this control loop iteration.
     */
    std::optional<T> GetPresent() {
        if (operator_idx_ == OPERATOR_PRESENT) {
            return content_;
        } else {
            return std::nullopt;
        }
    }

    /**
     * @brief Returns the value from exactly the previous control loop iteration.
     *
     * If during the last iteration no value was set or the value was already
     * overwritten during this control loop iteration then this function returns
     * std::nullopt.
     */
    std::optional<T> GetPrevious() {
        if (operator_idx_ == OPERATOR_PREVIOUS) {
            return content_;
        } else {
            return std::nullopt;
        }
    }

    /**
     * @brief Returns the value contained in this output port with disregard of
     * when the value was set.
     *
     * This function is thread-safe if load/store operations of T are atomic.
     */
    std::optional<T> GetAlways() {
        return content_;
    }

private:
    enum OperatorNum operator_idx_ = OPERATOR_NUM; // Age in number of control loop iterations
    T content_;
};

/**
 * @brief An input port provides a value from the source to which it's configured.
 *
 * The source can be one of:
 *  - an internally stored value
 *  - an externally stored value (referenced by a pointer)
 *  - an external OutputPort (referenced by a pointer)
 *  - none (all queries will return std::nullopt)
 *
 * Member functions of this class are not thread-safe unless otherwise noted.
 */
template<typename T>
class InputPort {
public:
    void ConnectTo(OutputPort<T>* input_port) {
        content_ = input_port;
    }

    void ConnectTo(T* input_ptr) {
        content_ = input_ptr;
    }

    void DisConnect() {
        content_ = (OutputPort<T>*)nullptr;
    }

    std::optional<T> GetPresent() {
        if (content_.index() == 2) {
            OutputPort<T>* ptr = std::get<2>(content_);
            return ptr ? ptr->GetPresent() : std::nullopt;
        } else if (content_.index() == 1) {
            T* ptr = std::get<1>(content_);
            return ptr ? std::make_optional(*ptr) : std::nullopt;
        } else {
            return std::get<0>(content_);
        }
    }

    std::optional<T> GetAlways() {
        if (content_.index() == 2) {
            OutputPort<T>* ptr = std::get<2>(content_);
            return ptr ? ptr->GetAlways() : std::nullopt;
        } else if (content_.index() == 1) {
            T* ptr = std::get<1>(content_);
            return ptr ? std::make_optional(*ptr) : std::nullopt;
        } else {
            return std::get<0>(content_);
        }
    }

private:
    std::variant<T, T*, OutputPort<T>*> content_;
};


#endif // ! __MIDDLEWARE_UTILS_COMPONENT_PORT_HPP__
