#include "./location.cpp" // importing location class; TODO: import Header file

class State {
	public:
    Location* location; // loc is the physical location this state occupies taken as a list of [x, y] coordinates (ints)
    State* parent; // parent is the adjacent state that generated this state (a State ref)
    // action is the action the parent took to reach this state, encoded as the 'turn' taken before moving forward
    // turn can be 0 - no turn, 1 - turn right, 2 - turn around, 3 - turn left, or -1 if null action (see default)
    int action;
    int cur_dir;

    // Constructor with member initializer list
    State(Location* loc, State* par = nullptr, int act = -1, int dir = 0)
        : location(loc), parent(par), action(act), cur_dir(dir) {
        if (!parent) {
            parent = this; //  if a parent state is not provided during construction, the parent pointer defaults to pointing to the current object itself.
        }
    }

    // Setters
    void set_loc(Location* loc) {
        location = loc;
    }

    void set_par(State* par) {
        parent = par;
    }

    void set_act(int act) {
        action = act;
    }

    void set_cur_dir(int dir) {
        cur_dir = dir;
    }

    // Getter for location
    Location get_location() {
        return *location;
    }

    // Getter for parent state
    State* get_parent() {
        return parent;
    }

    State get_parent_state() {
        return *parent;
    }

    // Getter for action
    int get_action() {
        return action;
    }

    // Getter for current direction
    int get_cur_dir() {
        return cur_dir;
    }

    // Check if the state is a goal state
    // Todo: make this relative for testing purposes
    bool is_goal() {
        return (location->position == std::vector<int>{7, 7} ||
                location->position == std::vector<int>{7, 8} ||
                location->position == std::vector<int>{8, 7} ||
                location->position == std::vector<int>{8, 8});
    }
};
