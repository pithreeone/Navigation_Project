class FSMItem
{
public:
    // enumerate all states
    enum State
    {
        A = 0,
        B,
        C,
    };
    // enumerate all events
    enum Events
    {
        e1 = 0,
        e2,
        e3,
    };
private:
    State cur_state_;
    State next_state_;
    Events event_;


};