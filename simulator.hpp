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
using RuleOpposingTraffic = std::vector< std::array<int,6>  >;

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

    currentRule_        = 0;
    ruleMinTimeLimit_   = 0;
    ruleMaxTimeLimit_   = 0;
    ruleElapsedTime_    = 0;

        for (unsigned lane = 0; lane < Lane::COUNT; ++lane)
        {
            sensors_[lane] = SensorState::CLEAR;
            signals_[lane] = SignalState::RED;
        }

    opposition_ =
    {
 /*            { 0, 2, 4,5,6,7},
            { 1, 3, 4, 5, 6, 7},
            { 0, 1, 2, 3, 4, 6},
            {0, 1, 2, 3, 5, 7} */

            {Lane::N_N, Lane::S_S, Lane::E_E, Lane::E_N, Lane::W_W, Lane::W_S},
            {Lane::N_W, Lane::S_E, Lane::E_E, Lane::E_N, Lane::W_W, Lane::W_S},
            {Lane::N_N, Lane::N_W, Lane::S_S, Lane::S_E, Lane::E_E, Lane::W_W},
            {Lane::N_N, Lane::N_W, Lane::S_S, Lane::S_E, Lane::E_N, Lane::W_S}            

    };    
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
        deltaTime_ = delta;
        clock_.advance(delta);
        update_simulation();
    }

    inline int getNextRule(void)
    {   
        int opposingTrafficCount = getRuleOpposingTraffic();
        int ruleTrafficCount = getRuleTraffic();

        if (clock_.now() == 0)
        {

            if (opposingTrafficCount == 0)
            {
                ruleMinTimeLimit_   = 0;
                ruleMaxTimeLimit_   = 0;
            }
            else if ( opposingTrafficCount > 0)
            {
                ruleMinTimeLimit_   = ruleTable_[0].min;
                ruleMaxTimeLimit_   = ruleTable_[0].max;                
            }       
            
            ruleElapsedTime_ = 0;
            return 0;
        }

        if ( (currentRule_==0) && (opposingTrafficCount==0) )
        {
            ruleMinTimeLimit_   = 0;
            ruleMaxTimeLimit_   = 0;
            ruleElapsedTime_    = 0;
            return 0;
        }
        else if ( (currentRule_==1) && (opposingTrafficCount==0))
        {
            ruleMinTimeLimit_   = 0;
            ruleMaxTimeLimit_   = 0;
            ruleElapsedTime_    = 0;
            return 1;
        }
        else if ( (currentRule_==2) && (opposingTrafficCount==0))
        {
            ruleMinTimeLimit_   = 0;
            ruleMaxTimeLimit_   = 0;
            ruleElapsedTime_    = 0;
            return 2;
        }    
        else if ( (currentRule_==3) && (opposingTrafficCount==0))
        {
            ruleMinTimeLimit_   = 0;
            ruleMaxTimeLimit_   = 0;
            ruleElapsedTime_    = 0;
            return 3;
        }
        else if ( (currentRule_==0) && (opposingTrafficCount>0) )
        {
            if (ruleMinTimeLimit_==0 && ruleMaxTimeLimit_==0)
            {
                ruleMinTimeLimit_   = ruleTable_[0].min;
                ruleMaxTimeLimit_   = ruleTable_[0].max;
                ruleElapsedTime_    = 0;
            }
            else if (ruleElapsedTime_ < ruleMinTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 0;
            }
            else if (ruleElapsedTime_>=ruleMinTimeLimit_ && ruleElapsedTime_<ruleMaxTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 0;
            }
            else if (ruleElapsedTime_ >= ruleMaxTimeLimit_)
            {
                ruleMinTimeLimit_   = ruleTable_[1].min;
                ruleMaxTimeLimit_   = ruleTable_[1].max;
                ruleElapsedTime_ = 0;
                return 1;
            }
            
            return 0;
        }
        else if (  (ruleTrafficCount==0) )
        {
                int nextRule = (nextRule+1)%4;

                ruleMinTimeLimit_   = ruleTable_[ nextRule ].min;
                ruleMaxTimeLimit_   = ruleTable_[ nextRule ].max;
                ruleElapsedTime_    = 0;
                return nextRule;    
        }          
        else if ( (currentRule_==1) && (opposingTrafficCount>0) )
        {
            if (ruleMinTimeLimit_==0 && ruleMaxTimeLimit_==0)
            {
                ruleMinTimeLimit_   = ruleTable_[1].min;
                ruleMaxTimeLimit_   = ruleTable_[1].max;
                ruleElapsedTime_    = 0;
            }
            else if (ruleElapsedTime_ < ruleMinTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 1;
            }
            else if (ruleElapsedTime_>=ruleMinTimeLimit_ && ruleElapsedTime_<ruleMaxTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 1;
            }
            else if (ruleElapsedTime_ >= ruleMaxTimeLimit_)
            {
                ruleMinTimeLimit_   = ruleTable_[2].min;
                ruleMaxTimeLimit_   = ruleTable_[2].max;
                ruleElapsedTime_ = 0;
                return 2;
            }
            
            return 1;
        }
        else if ( (currentRule_==2) && (opposingTrafficCount>0) )
        {
            if (ruleMinTimeLimit_==0 && ruleMaxTimeLimit_==0)
            {
                ruleMinTimeLimit_   = ruleTable_[2].min;
                ruleMaxTimeLimit_   = ruleTable_[2].max;
                ruleElapsedTime_    = 0;
            }
            else if (ruleElapsedTime_ < ruleMinTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 2;
            }
            else if (ruleElapsedTime_>=ruleMinTimeLimit_ && ruleElapsedTime_<ruleMaxTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 2;
            }
            else if (ruleElapsedTime_ >= ruleMaxTimeLimit_)
            {
                ruleMinTimeLimit_   = ruleTable_[3].min;
                ruleMaxTimeLimit_   = ruleTable_[3].max;
                ruleElapsedTime_ = 0;
                return 3;
            }
            
            return 2;
        }
        else if ( (currentRule_==3) && (opposingTrafficCount>0) )
        {
            if (ruleMinTimeLimit_==0 && ruleMaxTimeLimit_==0)
            {
                ruleMinTimeLimit_   = ruleTable_[3].min;
                ruleMaxTimeLimit_   = ruleTable_[3].max;
                ruleElapsedTime_    = 0;
            }
            else if (ruleElapsedTime_ < ruleMinTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 3;
            }
            else if (ruleElapsedTime_>=ruleMinTimeLimit_ && ruleElapsedTime_<ruleMaxTimeLimit_)
            {
                ruleElapsedTime_+=deltaTime_;
                return 3;
            }
            else if (ruleElapsedTime_ >= ruleMaxTimeLimit_)
            {
                ruleMinTimeLimit_   = ruleTable_[0].min;
                ruleMaxTimeLimit_   = ruleTable_[0].max;
                ruleElapsedTime_ = 0;
                return 0;
            }
            
            return 3;
        }    
        else
        { //this code should never be reached
            ruleMinTimeLimit_   = 0;
            ruleMaxTimeLimit_   = 0;
            ruleElapsedTime_    = 0;       
            return 0;
        }

        //this code should never be reached
        return -1;
    }

    void applyRule(void);
    int getRuleOpposingTraffic(void);
    int getRuleTraffic(void);
    
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
    Clock::Time ruleMinTimeLimit_;
    Clock::Time ruleMaxTimeLimit_;
    Clock::Time ruleElapsedTime_;
    Clock::Time deltaTime_;
    RuleOpposingTraffic opposition_;

};