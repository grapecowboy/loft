#pragma once

#include "clock.hpp"

#include <array>
#include <iostream>
#include <vector>

/** Each lane is identified by it's enter and exit directions.
 * 
 * Here, enter and exit refere to the direction of travel as the vehicles
 * enters or exits the intersection. */
enum Lane
{
    N_N, ///< north going straight through
    N_W, ///< north turning west
    S_S, ///< south going straight through
    S_E, ///< south turning east
    E_E, ///< east going straight through
    E_N, ///< east turning north
    W_W, ///< west going straight through
    W_S, ///< west turning south
    COUNT
};

enum class SensorState
{
    SET,  ///< Car is present
    CLEAR ///< No car is present
};

/// There is one vehicle sensor per lane
using VehicleSensors = std::array<SensorState, Lane::COUNT>;

enum class SignalState
{
    RED,
    YELLOW,
    GREEN
};

/// There is one traffic signal per lane
using TrafficSignals = std::array<SignalState, Lane::COUNT>;

/** Describes the simulator state for the timespan [start, end). */
struct SimulationTimeslice
{
    Clock::Time start;      ///< start time, inclusive
    Clock::Time end;        ///< end time, exclusive
    VehicleSensors sensors; ///< state of each vehicle sensor
};

/** A "scenario" is a list of simulation states that is replayed by the
 * simulator.
 * 
 * Ideally, scenarios should be the input to a feedback loop running within the
 * simulator, so you could do things like trigger simulation state changes from
 * events emitted by the controller. As implemented, they are replayed blindly,
 * ie. providing open-loop simulation. */
using Scenario = std::vector<SimulationTimeslice>;

/** A Rule is a tuple that contains 4-elements.
 * 
 *  A rule number from 0 to 3
 *  A min time of execution
 *  A max time of execution
 *  elapsed time of execution
 * 
*/
using Rule = int;

struct RuleLength
{
    Clock::Time min;
    Clock::Time max;
};

using RuleTimes = std::vector< RuleLength >;
using RuleTrafficSignals = std::vector< TrafficSignals >;
using LS = SignalState;

/** A simulator replays data from a given scenario.
 * 
 * To advance time in the simulator, call #advance(). The simulator reads
 * vehicle sensor data from the scenario when the simulation time is advanced.
 * 
 * The current simulation time can be retrieved via #clock(). The vehicle
 * sensor data for the current timestamp can be retrieved with #sensors().
 * 
 * The simulator also maintains the list of traffic signals for each lane. To
 * update the list, use #update_signals();
 */
class Simulator
{
public:
    Simulator(const Scenario &scenario) : 
        scenario_(scenario),
        done_(false)
    { 
        currentRule_ = 0;
        ruleTable_ =
        {
            {10, 60},
            {30,120},
            {10,30},
            {30,60}
        };

    ruleLights_ = 
    {
        {LS::RED,   LS::GREEN, LS::RED,   LS::GREEN, LS::RED,   LS::RED,   LS::RED,   LS::RED},
        {LS::GREEN, LS::RED,   LS::GREEN, LS::RED,   LS::RED,   LS::RED,   LS::RED,   LS::RED},
        {LS::RED,   LS::RED,   LS::RED,   LS::RED,   LS::RED,   LS::GREEN, LS::RED,   LS::GREEN},
        {LS::RED,   LS::RED,   LS::RED,   LS::RED,   LS::GREEN, LS::RED,   LS::GREEN, LS::RED}
    }; 

    currentRule_     = 0;
    ruleTimeLimit_   = 0;
    ruleElapsedTime_ = 0;

        for (unsigned lane = 0; lane < Lane::COUNT; ++lane)
        {
            sensors_[lane] = SensorState::CLEAR;
            signals_[lane] = SignalState::RED;
        }
        
        update_simulation(); // seed the simulator
    }

    /// Banner text
    static const std::string BANNER;
    
    /** Print the state of the simulator to the given outptut stream */
    friend std::ostream& operator<<(std::ostream &os, const Simulator &simulator);

    /// Get the simulated clock
    inline const Clock& clock(void) const
    {
        return clock_;
    }

    /** Get the simulated sensors's state at the current timestamp.
     * 
     * Vehicle sensors are updated from the loaded scenario during the call to
     * #advance(). */
    inline const VehicleSensors& sensors(void) const
    {
        return sensors_;
    }

    /** Users should call this to update the traffic lights in the simulated
     * intersection */
    inline void update_lane_signals(const TrafficSignals &signals)
    {
        signals_ = signals;
    }

    /// Returns true iff the scenario has been completed
    inline bool done(void) const
    {
        return done_;
    }

    /** Users should call this to advance the simulation time by the given delta.
     * 
     * There are no restrictions placed on the size of the time delta to
     * advance by. However, care should be taken that a suitable value is used
     * which does not skip time-slices in the loaded scenario. */
    inline void advance(Clock::Time delta)
    {
        clock_.advance(delta);
        update_simulation();

        //update_simulation();
        //clock_.advance(delta);
 

    }

    inline int getNextRule(void)
    {
        std::cout << "Current Traffic " << std::endl;
        
/*         for (auto n=sensors_.begin(); n!=sensors_.end(); ++n)
        {
            if(*n == SensorState::CLEAR)
                std::cout << "CLEAR" << "\t";
            else if (*n == SensorState::SET)
                std::cout << "SET" << "\t";
            else
                std::cout << "ERR" << "\t";                     
        } */

        if ( (sensors_[1] == SensorState::SET) || (sensors_[3]==SensorState::SET) )
            return 0;
        else if ( (sensors_[0] == SensorState::SET) || (sensors_[2]==SensorState::SET) )
            return 1;
        else if ( (sensors_[5] == SensorState::SET) || (sensors_[7]==SensorState::SET) )
            return 2;
        else if ( (sensors_[4] == SensorState::SET) || (sensors_[6]==SensorState::SET) )
            return 3;
        else       
            return 0;
    }

private:
    /** Updates the simulator state.
     * 
     * The simulation state is read from the loaded scenario for the current
     * timestamp. */
    void update_simulation(void);

    const Scenario scenario_; ///< copy of loaded scenario
    bool done_;               ///< true iff scenario completed
    Clock clock_;             ///< global simulation clock
    VehicleSensors sensors_;  ///< sensor state for given timestamp
    TrafficSignals signals_;     ///< simulated traffic signal output
    Rule currentRule_;
    RuleTimes ruleTable_;
    RuleTrafficSignals ruleLights_;
    Clock::Time ruleTimeLimit_;
    Clock::Time ruleElapsedTime_;

};