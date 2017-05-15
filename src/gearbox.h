/*
 * Copyright (c) 2017 Kevin Wellwood
 * All rights reserved.
 *
 * This source code is distributed under the Modified BSD License. For terms and
 * conditions, see license.txt.
 */

#ifndef _WELLWOOD_GEARBOX_H_
#define _WELLWOOD_GEARBOX_H_

/*
 * Gearbox is a tree of connected gears, with the drive gear (at the root) ticking all other gears
 * connected to it and those connected to them. Like clockwork, every action is synchronized with
 * respect to every other.
 *
 * Base_Gear is a superclass that implements gear connections and engaged state. When a gear is
 * connected to another, it becomes driven by it. Each rotation of the drive gear in turn results
 * in a tick of a driven gear. A ratio determines how many times a driven gear needs to be ticked
 * before it makes a complete rotation.
 *
 * When a gear is engaged, actions are fired on every tick and at the end of a complete rotation.
 * The process of engaging a gear is synchronized with its rotation, ensuring gears become engaged
 * only at the end of a complete rotation.
 */
class Base_Gear
{
public:

    /*
     * Connects this gear to drive gear 'pinion'. 'ratio' / 'step' rotations of the drive gear
     * will produce one rotation of this gear. Phase (1 to 'ratio') will start with 'phase' steps
     * already elapsed. Phase is pre-incremented, so if phase starts at 0, the first tick() will
     * see a phase of 1. 'step' is the phase increment per tick (1 to 'ratio'): Fractional gear
     * ratios can be produced with a step greater than 1. 'priority' ranks the gear in the tick
     * sequence of all gears directly driven by 'pinion', lowest number first.
     */
    void connect(Base_Gear* pinion,
                 unsigned short int ratio,
                 unsigned short int phase = 0,
                 unsigned short int step = 1,
                 unsigned short int priority = 0);

    /*
     * This is a special purpose method to allow the engagement of a gear to be delayed for more
     * than one rotation.
     *
     * This may only be called from an on_engaged() handler, otherwise the behavior undefined.
     */
    void delay_engagement() { if (state == Engaged) state = Engaging; }

    /*
     * Begins engaging or disengaging this gear. Gears are initially are engaged by default. A
     * request to engage the gear will complete on the next rotation. A request to disengage the
     * gear will complete on the next tick (as soon as possible).
     *
     * A gear is still ticked and it still drives connected gears while it is not engaged, but its
     * tick and rotation events are suppressed.
     */
    void engage(bool engaged);

    /*
     * Returns true when the gear is fully disengaged.
     */
    bool is_disengaged() const { return state == Disengaged; }

    /*
     * Returns true when the gear is fully engaged.
     */
    bool is_engaged() const { return state == Engaged; }

    /*
     * Returns the current phase of rotation. Typically is 1 to ratio, but if the gear has a
     * fractional ratio (step > 1), its phase can be as much as ratio + step at the end of a
     * rotation.
     */
    unsigned short int get_phase() const { return phase; }

    /*
     * Returns the gear's ratio that was configured when it was connected to its drive gear.
     */
    unsigned short int get_ratio() const { return ratio; }

    /*
     * Returns the gear's step that was configured when it was connected to its drive gear.
     */    
    unsigned short int get_step() const { return step; }

    /*
     * Ticks the gear, updating its phase.
     */
    void tick();

protected:

    Base_Gear(unsigned short int phase, unsigned short int step);

    /*
     * Called when the gear becomes engaged at the end of a rotation, just before on_tick() and
     * on_rotation(). There will always be a corresponding call to on_disengaged(), if the gear is
     * layer disengaged, to allow anything enabled by this handler to also be disabled.
     */
    virtual void on_engaged() { }

    /*
     * Called on each tick of the drive gear, when the gear is engaged.
     */
    virtual void on_tick() { }

    /*
     * Called on each complete rotation, just after on_tick(), when the gear is engaged.
     */
    virtual void on_rotation() { }

    /*
     * Called when the gear becomes disengaged at the end of a rotation, just after on_tick() and
     * on_rotation().
     *
     * If the gear begins engaging, the on_disengaged() handler will be called when it becomes
     * disengaged again, even if it was never fully engaged. Thus, { engage(true); engage(false); }
     * will put the gear in the Engaging state and then immediately into the Disengaging state,
     * which will invoke on_disengaged() on the next rotation.
     */
    virtual void on_disengaged() { }

    enum Gear_State { Disengaged, Engaging, Engaged, Disengaging };

    Gear_State state;               // gear's action is triggered each rotation when it is engaged

private:

    Base_Gear(const Base_Gear& other) = delete;
    Base_Gear& operator=(const Base_Gear&) = delete;

    unsigned short int ratio;       // number of drive gear rotations to one rotation of this
    unsigned short int step;        // number of steps phase change per rotation of the drive gear
    unsigned short int phase;       // current phase (1..ratio)
    unsigned short int priority;    // order among siblings (ticked by priority in ascending order)

    Base_Gear* driven;              // linked listed of gears being driven by this
    Base_Gear* next;                // next sibling gear
};

//-----------------------------------------------------------------------------------------------//

/*
 * The template Gear class is parameterized with the class that observes it. The purpose of this
 * subclass is simply to notify the object observing the gear of its events.
 */
template <class T>
class Gear : public Base_Gear
{
public:

    typedef void (T::*Handler)();

    /*
     * Creates a new gear that will notify 'observer' of its events. 'observer' cannot be null and
     * its lifetime must extend beyond the gear's.
     *
     * Use this constructor to instantiate a gear that will be driven by another. Its starting phase
     * and step size will be determined when it is connected to a drive gear.
     */
    explicit Gear(T* observer)
    : Base_Gear(0, 1)
    , observer(observer)
    { }

    /*
     * Creates a new main drive gear (not driven by another), that will notify 'observer' of its
     * events. 'observer' cannot be null and its lifetime must extend beyond the gear's.
     */
    explicit Gear(T* observer, unsigned short int phase, unsigned short int step)
    : Base_Gear(phase, step)
    , observer(observer)
    { }

    void handle_engaged(Handler handler) { engaged_handler = handler; }
    
    void handle_disengaged(Handler handler) { disengaged_handler = handler; }
    
    void handle_tick(Handler handler) { tick_handler = handler; }

    void handle_rotation(Handler handler) { rotation_handler = handler; }

protected:

    virtual void on_engaged() override { if (engaged_handler) (observer->*engaged_handler)(); }

    virtual void on_disengaged() override { if (disengaged_handler) (observer->*disengaged_handler)(); }

    virtual void on_tick() override { if (tick_handler) (observer->*tick_handler)(); }

    virtual void on_rotation() override { if (rotation_handler) (observer->*rotation_handler)(); }

private:

    T*      observer;
    Handler engaged_handler    = nullptr;
    Handler disengaged_handler = nullptr;
    Handler tick_handler       = nullptr;
    Handler rotation_handler   = nullptr;
};

//-----------------------------------------------------------------------------------------------//

/*
 * The Counter subclass simply counts rotations while it is engaged. It does not send events to an
 * observer like the Gear class does.
 */
class Counter : public Base_Gear
{
public:

    Counter(unsigned short int phase = 0, unsigned short int step = 1)
    : Base_Gear(phase, step)
    , total(0ULL)
    { }

    /*
     * Returns the total number of gear rotations.
     */
    unsigned long long int count() const { return total; }

protected:

    virtual void on_rotation() override { total +=1; }

private:

    unsigned long long int total;
};

#endif // _WELLWOOD_GEARBOX_H_ //
