/*
 * Copyright (c) 2017 Kevin Wellwood
 * All rights reserved.
 *
 * This source code is distributed under the Modified BSD License. For terms and
 * conditions, see license.txt.
 */

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
     * Connects 'gear' to be driven by this gear. 'ratio' / 'step' rotations of this gear will
     * produce one rotation in 'gear'. It's phase (0 to 'ratio' - 1) will start at 'phase'.
     * 'step' is the phase increment per tick: Fractional gear ratios can be produced by passing
     * greater than 1. 'order' ranks 'gear' among the other gears driven by this one: The lowest
     * order gear is ticked first.
     */
    void connect(Base_Gear* gear, int ratio, int phase = 0, int step = 1, int order = 0);

    /*
     * Begins engaging or disengaging this gear. Gears are engaged by default. The operation will
     * complete at the end of the next rotation.  A disengaged gear is still ticked and it still
     * drives connected gears, but its tick and rotation events are suppressed.
     */
    void engage(bool engaged);

    /*
     * Returns the current phase of rotation. Typically is 1 to ratio, but if the gear has a
     * fractional ratio (step > 1), its phase can be as much as ratio + step at the end of a
     * rotation.
     */
    short get_phase() const { return phase; }

    /*
     * Returns the gear's ratio that was configured when it was connected to its drive gear.
     */
    short get_ratio() const { return ratio; }

    /*
     * Returns the gear's step that was configured when it was connected to its drive gear.
     */    
    short get_step() const { return step; }

    /*
     * Ticks the gear, updating its phase.
     */
    void tick();

protected:

    Base_Gear();

    /*
     * Called when the gear becomes engaged at the end of a rotation, just before on_tick() and
     * on_rotation().
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
     */
    virtual void on_disengaged() { }

    enum Gear_State { Disengaged, Engaging, Engaged, Disengaging };

    Gear_State state;               // gear's action is triggered each rotation when it is engaged

private:

    unsigned short ratio;           // number of drive gear rotations to one rotation of this
    unsigned short step;            // number of steps phase change per rotation of the drive gear
    unsigned short phase;           // current phase (0..ratio-1)
    unsigned short order;           // order among siblings (ticked in ascending order)

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
     */
    explicit Gear(T* observer)
    : observer(observer)
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

    Counter()
    : total(0ULL)
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
