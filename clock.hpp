#pragma once

/** A simple clock abstraction for simulating time.
 * 
 * To keep things extra simple, all timestamps can be treated as seconds. */
class Clock
{
public:
    /** All timestamps can be treated as seconds.
     * 
     * Here, "Time" is used for both timestamps and timespans. */
    using Time = int;

    Clock(void) :
        now_(0)
    { }

    /// Get the current timestamp
    inline Time now(void) const
    {
        return now_;
    }

    /// Set the current timestamp
    inline void set(Time now)
    {
        now_ = now;
    }

    /// Advance the current timestamp
    inline void advance(Time delta)
    {
        now_ += delta;
    }

    /// Calculate the time elapsed (or until) an event
    inline Time elapsed(Time then) const
    {
        return now_ - then;
    }

private:
    Time now_; ///< current timestamp
};