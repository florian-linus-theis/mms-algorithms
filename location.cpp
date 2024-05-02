// Using vector class in c++ for documentation see: https://en.cppreference.com/w/cpp/container/vector
#include <vector>
#include <iostream>
// using namespace std;


class Location {
    public:
    std::vector<bool> walls; // list of 4 booleans (north, east, south, west)
    std::vector<int> position; // order pair list [x, y]
    bool visited; // boolean if the location has been visited

    // constructor function that sets the position if a position is passed
    Location(std::vector<int> pos = {}) : position({-1, -1}), visited(false) {
        walls = {false, false, false, false};
        if (!pos.empty()) {
            position[0] = pos[0];
            position[1] = pos[1];
        }
    }

    // takes position as order pair list [x, y]
    void set_position(std::vector<int> pos) {
        position[0] = pos[0];
        position[1] = pos[1];
    }

    // takes walls as list of 4 booleans, e.g. [true, false, true, false] and updates walls property
    void set_walls(std::vector<bool> walls) {
        this->walls = walls; // this-> is used to differentiate between the class attribute and the parameter that was passed
    }

    // takes boolean and updates visited property
    void set_visited(bool vis=true) {
        visited = vis;
    }

    // takes the position of another Location Cell and return boolean whether two cells are adjacent and have no walls in between them
    bool can_move_to(Location loc) {
        return (loc.position[0] == position[0] && loc.position[1] - position[1] == +1 && !walls[0])
            || (loc.position[1] == position[1] && loc.position[0] - position[0] == +1 && !walls[1])
            || (loc.position[0] == position[0] && loc.position[1] - position[1] == -1 && !walls[2])
            || (loc.position[1] == position[1] && loc.position[0] - position[0] == -1 && !walls[3]);
    }
};
