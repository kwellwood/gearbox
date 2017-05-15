/*
 * Copyright (c) 2017 Kevin Wellwood
 * All rights reserved.
 *
 * This source code is distributed under the Modified BSD License. For terms and
 * conditions, see license.txt.
 */

#include "gearbox.h"
#include <cstdio>

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

Base_Gear::Base_Gear(unsigned short int phase, unsigned short int step)
: state(Engaged)
, ratio(1)
, step((step > 0) ? step : 1)
, phase(phase)
, priority(0)
, driven(nullptr)
, next(nullptr)
{ }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

void Base_Gear::connect(Base_Gear* pinion,
                        unsigned short int ratio,
                        unsigned short int phase,
                        unsigned short int step,
                        unsigned short int priority)
{
    this->ratio = ratio;
    this->phase = phase;
    this->step = (step > 0) ? step : 1;
    this->priority = priority;

    if (pinion->driven != nullptr && pinion->driven->priority <= this->priority)
    {
        int i = 0;
        Base_Gear* g = pinion->driven;
        while (g->next != nullptr && g->next->priority <= this->priority)
        {
            i++;
            g = g->next;
        }
        this->next = g->next;
        g->next = this;
        g = pinion->driven;
        while (g != nullptr)
        {
            g = g->next;
        }
    }
    else
    {
        this->next = pinion->driven;
        pinion->driven = this;
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

void Base_Gear::engage(bool engaged)
{
    if (!engaged)
    {
        if (state == Engaged || state == Engaging)
        {
            // if on_engaged() was called and it delayed the gear engagement by one more rotation
            // (via delay_engagement() or assigning 'state' directly), the gear will still be in
            // the Engaging state and may require the corresponding on_disengaged() handler to be
            // called as well.
            state = Disengaging;
        }
    }
    else
    {
        if (state == Disengaged)
        {
            state = Engaging;
        }
        else if (state == Disengaging)
        {
            // hasn't disengaged yet, so go straight back to engaged
            state = Engaged;
        }
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

void Base_Gear::tick()
{
    if (phase + step >= ratio)
    {
        if (state == Engaging)
        {
            state = Engaged;
            on_engaged();
        }
        if (state == Engaged)
        {
            on_tick();
            on_rotation();
        }
        if (state == Disengaging)
        {
            state = Disengaged;
            on_disengaged();
        }

        phase = (phase + step) - ratio;

        Base_Gear* g = driven;
        while (g != nullptr)
        {
            g->tick();
            g = g->next;
        }
    }
    else
    {
        if (state == Engaged)
        {
            on_tick();
        }
        else if (state == Disengaging)
        {
            state = Disengaged;
            on_disengaged();
        }

        phase += step;
    }
}

//-----------------------------------------------------------------------------------------------//

class User_Class
{
public:

    User_Class()
    : gear(Gear<User_Class>(this))
    , c(0)
    { gear.handle_rotation(&User_Class::increment); }

    void increment() { printf("count is %i\n", ++c); }

    int count() const { return c; }

    Gear<User_Class> gear;

private:

    int c;
};

int main(int argc, char** argv)
{
    // This test creates a gear chain that is driven by an ISR running at 12.5 kHz. A counter to
    // track the total number of interrupts, a milliseconds counter, and a seconds counter are all
    // driven by the ISR gear. The count of seconds is implemented in a user-defined class to
    // demonstrate how to handle events.

    Counter isr;

    // counts every tick
    Counter tick_counter;
    isr.connect(&tick_counter, 1);

        // counts milliseconds, 80 microseconds at a time (realtime period of the ISR)
        Counter ms_counter;
        tick_counter.connect(&ms_counter, 1000, 0, 80);

            // an instance of User_Class has its own gear that rotations once per second, connected
            // to the millisecond counter.
            User_Class run_time;
            ms_counter.connect(&(run_time.gear), 1000);

    for (int i = 0; i < 24999; i++)
    {
        isr.tick();
    }

    printf("total_ticks:%llu, ms_counter:%llu, run_time:%i\n", tick_counter.count(), ms_counter.count(), run_time.count());

    return 0;
}
