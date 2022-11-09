#ifndef __DESIGN_PATTERNS_FSM_HPP__
#define __DESIGN_PATTERNS_FSM_HPP__

#include <algorithm>
#include <deque>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

/* system use negative number */
#define STATE_INIT_EVENT               -1
#define STATE_EXIT_EVENT               -2

namespace Fsm {

#define TRANSITION_TABLE_SIZE    128

template<typename T>
inline std::string ToString(const T &t) {
    std::stringstream ss;
    return ss << t ? ss.str() : std::string();
}

template<>
inline std::string ToString(const std::string &t) {
    return t;
}

typedef std::vector<std::string> fsm_args;
typedef std::function<void(const Fsm::fsm_args &args)> fsm_function;

/**
 * @brief descript state info
 */
struct State {
    int state_;
    Fsm::fsm_args cb_args_;

    State(const int &state = 0) : state_(state) {}

    State operator()() const {
        State self = *this;
        self.cb_args_ = {};
        return self;
    }
    template<typename T0>
    State operator()( const T0 &t0 ) const {
        State self = *this;
        self.cb_args_ = { Fsm::ToString(t0) };
        return self;
    }
    template<typename T0, typename T1>
    State operator()( const T0 &t0, const T1 &t1 ) const {
        State self = *this;
        self.cb_args_ = { Fsm::ToString(t0), Fsm::ToString(t1) };
        return self;
    }

    bool operator<(const State &other) const {
        return state_ < other.state_;
    }
    bool operator==(const State &other) const {
        return state_ == other.state_;
    }
    operator int () const {
        return state_;
    }

    template<typename ostream>
    inline friend ostream &operator<<(ostream &out, const State &t) {
        if(t.state_ >= 256) {
            out << char((t.state_ >> 24) & 0xff);
            out << char((t.state_ >> 16) & 0xff);
            out << char((t.state_ >>  8) & 0xff);
            out << char((t.state_ >>  0) & 0xff);
        } else {
            out << t.state_;
        }
        out << "(";
        std::string sep;
        for(auto &arg : t.cb_args_ ) {
            out << sep << arg;
            sep = ',';
        }
        out << ")";
        return out;
    }
};

//typedef State event;

/**
 * @brief descript transition relation
 *         from previous_state_ to current_state_ by action
 */
struct Transition {
    Fsm::State previous_state_, event, current_state_;

    template<typename ostream>
    inline friend ostream &operator<<(ostream &out, const Transition &t) {
        out << t.previous_state_ << " -> " << t.event << " -> " << t.current_state_;
        return out;
    }
};

/**
 * @brief StateMachine class
 *
 */
class StateMachine {
  public:
    StateMachine(const Fsm::State &start = 0) : state_table_(1) {
        state_table_[0] = start;
        Call(state_table_.back(), STATE_INIT_EVENT);
    }

    StateMachine(int start) : StateMachine(Fsm::State(start)) {}

    ~StateMachine() {
        // ensure State destructors are called (w/ exit)
        Root();
    }

    /* pause current State (w/ exit) and create a new active child (w/ 'enter') */
    void Child(const Fsm::State &state) {
        if(state_table_.size() && state_table_.back() == state) {
            return;
        }
        state_table_.push_back(state);
        Call(state_table_.back(), STATE_INIT_EVENT);
    }

    /* terminate current state and return to parent (if any) */
    void Parent(void) {
        if(state_table_.size()) {
            Call(state_table_.back(), STATE_EXIT_EVENT);
            state_table_.pop_back();
        }
    }

    /* terminate current state and return to parent (if any) */
    void Root(void) {
        while(Size()) {
            Parent();
        }
    }

    /* set current active state */
    void Set(const Fsm::State &state) {
        if(state_table_.size()) {
            Replace(state_table_.back(), state);
        } else {
            Child(state);
        }
    }

    /* number of children (StateMachine) */
    size_t Size() const {
        return state_table_.size();
    }

    Fsm::State GetState(signed pos = -1) const {
        signed size = (signed)(state_table_.size());
        return size ? *(state_table_.begin() + (pos >= 0 ? pos % size : size - 1 + ((pos+1) % size))) : Fsm::State();
    }
    Fsm::Transition GetLog(signed pos = -1) const {
        signed size = (signed)(transition_table_.size());
        return size ? *(transition_table_.begin() + (pos >= 0 ? pos % size : size - 1 + ((pos+1) % size))) : Fsm::Transition();
    }
    std::string GetTrigger() const {
        std::stringstream ss;
        return ss << current_event_, ss.str();
    }
    bool IsState(const Fsm::State &state) const {
        return state_table_.empty() ? false : (state_table_.back() == state);
    }

    // setup
    Fsm::fsm_function &Bind(const Fsm::State &from, const Fsm::State &to) {
        return state_map_table_[bistate_(from,to)];
    }

    // generic call
    bool Call(const Fsm::State &from, const Fsm::State &to) const {
        std::map<bistate_, Fsm::fsm_function>::const_iterator found = state_map_table_.find(bistate_(from, to));
        if (found != state_map_table_.end()) {
            transition_table_.push_back({from, current_event_, to});
            if(transition_table_.size() > TRANSITION_TABLE_SIZE ) {
                transition_table_.pop_front();
            }
            found->second(to.cb_args_);
            return true;
        }
        return false;
    }

    // user Events
    bool Event(const Fsm::State &event) {
        size_t size = this->Size();
        if(!size) {
            return false;
        }
        current_event_ = Fsm::State();
        std::deque<states_::reverse_iterator> aborted;
        for(auto it = state_table_.rbegin(); it != state_table_.rend(); ++it) {
            Fsm::State &self = *it;
            if(!Call(self,event)) {
                aborted.push_back(it);
                continue;
            }
            for(auto it = aborted.begin(), end = aborted.end(); it != end; ++it) {
                Call(**it, STATE_EXIT_EVENT);
                state_table_.erase(--(it->base()));
            }
            current_event_ = event;
            return true;
        }
        return false;
    }
    template<typename T>
    bool Event(const Fsm::State &event, const T &arg1) {
        return Event(event(arg1));
    }
    template<typename T, typename U>
    bool Event(const Fsm::State &event, const T &arg1, const U &arg2) {
        return Event(event(arg1, arg2));
    }

    // debug
    template<typename ostream>
    ostream &Debug( ostream &out ) {
        int total = transition_table_.size();
        out << "status {" << std::endl;
        std::string sep = "\t";
        for(states_::const_reverse_iterator it = state_table_.rbegin(), end = state_table_.rend(); it != end; ++it ) {
            out << sep << *it;
            sep = " -> ";
        }
        out << std::endl;
        out << "} log (" << total << " entries) {" << std::endl;
        for(int i = 0 ; i < total; ++i) {
            out << "\t" << transition_table_[i] << std::endl;
        }
        out << "}" << std::endl;
        return out;
    }

    // aliases
    bool operator()(const Fsm::State &event) {
        return Event(event);
    }
    template<typename T>
    bool operator()(const Fsm::State &event, const T &arg1) {
        return Event(event(arg1));
    }
    template<typename T, typename U>
    bool operator()(const Fsm::State &event, const T &arg1, const U &arg2) {
        return Event(event(arg1, arg2));
    }
    template<typename ostream>
    inline friend ostream &operator<<(ostream &out, const StateMachine &t) {
        return t.Debug(out), out;
    }

  protected:
    void Replace( Fsm::State &current, const Fsm::State &next ) {
        Call(current, STATE_EXIT_EVENT);
        current = next;
        Call(current, STATE_INIT_EVENT);
    }

    typedef std::pair<int, int> bistate_;
    std::map<bistate_, Fsm::fsm_function> state_map_table_;

    mutable std::deque<Fsm::Transition> transition_table_;
    std::deque<Fsm::State> state_table_;
    Fsm::State current_event_;

    typedef std::deque<Fsm::State> states_;
};

}

#endif // ! __DESIGN_PATTERNS_FSM_HPP__
