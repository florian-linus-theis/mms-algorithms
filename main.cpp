#include <vector> // For lists/arrays
#include <iostream> // for input/output
#include <stack> // For LIFO stack
#include <queue> // For FIFO queue
#include "API.h" // Later purposes
#include <string> // For string operations

// #include "./location.cpp" // importing location class
#include "./state.cpp" // importing state class
// TODO: write header file
using namespace std;

// Initializing maze size with constexpr to set them up as compile-time-constants rather than runtime constants
constexpr int MAZE_WIDTH = 16;
constexpr int MAZE_HEIGHT = 16;

// For tracking current global direction
// 0 = North, 1 = East, 2 = South, 3 = West
// Could be solved using an enum
int cur_direction = 0;

// For tracking current global 'physical' position in maze as [x, y], initialized to [0, 0] (i.e. bottom left cell, origin)
std::vector<int> cur_position = {0, 0};

// For tracking all maze data, create a 2D vector of Locations
std::vector<std::vector<Location>> maze (MAZE_HEIGHT, std::vector<Location>(MAZE_WIDTH));

// Initialize the maze with Location objectss
// maze is a global variable so no need to pass it here
void initialize_maze(){
  for (int i = 0; i < MAZE_HEIGHT; ++i) {
      for (int j = 0; j < MAZE_WIDTH; ++j) {
          maze[i][j] = Location({i, j});
      }
  }
}

// Location object stack for tracking locations that may need to be explored during mapping
std::stack<Location> loc_stack;

// Direction stack for easy backtracking through maze when a dead end is found during mapping
std::stack<int> dir_stack;

// Action stack for processing optimal sequence of actions to find goal state
std::stack<int> act_stack; // Assuming Action is a user-defined type (wahrscheinlich hier als typ std::queue<State>)

// State object queue for unexplored nodes during breadth first search
std::queue<State> frontier; // Assuming State is a user-defined type



// Function to update position by one cell in the maze
// Move direction can only be range 0-3 with 0 being North
void update_position() {
    if (cur_direction == 0) {    // facing north
        cur_position[1] += 1;
    } else if (cur_direction == 1) {  // facing east
        cur_position[0] += 1;
    } else if (cur_direction == 2) {  // facing south
        cur_position[1] -= 1;
    } else if (cur_direction == 3) {  // facing west
        cur_position[0] -= 1;
    }
}


// Function to update direction
// Takes a turn direction (0-3) and updates current direction based on that
void update_direction(int turn_direction) {
    cur_direction = (cur_direction + turn_direction + 4) % 4; // Adding 4 ensures that the result is always positive, preventing negative values from the modulo operation.
}


// Function to get the list of walls around the current cell in the maze, e.g. [True, False, True, False] -> No walls to the front and the back of the mouse but left and right
// returns values relative to direction we come from
std::vector<bool> get_walls() {
    std::vector<bool> walls(4, false); // initializing list containing 4 walls each to be false by default

    // Check for walls in each direction
    walls[cur_direction] = API::wallFront(); // Is there a wall in front
    walls[(cur_direction + 1) % 4] = API::wallRight(); // Is there a wall to the right
    walls[(cur_direction + 2) % 4] = false; // No wall from the direction we came from also in real scenario we do not have any sensors at the back
    walls[(cur_direction + 3) % 4] = API::wallLeft(); // Is there a wall to the left

    // If it's the first square, mark the bottom wall as there
    if (cur_position == std::vector<int>{0, 0}) {
        walls[2] = true;
    }

    return walls;
}


// ---------------------------------------------------------------------
// Following Functions just for visualization in MMS:


// Marks a given square green after it has been visited
void mark_visited_api(std::vector<int> pos = {}) {
    if (pos.empty()) {
        pos = cur_position;
    }
    API::setColor(pos[0], pos[1], 'G');
    API::setText(pos[0], pos[1], "hit"); // string containing info on square
}

// Marks a given square blue when it is part of the solution path
void mark_solution_api(std::vector<int> pos = {}) {
    if (pos.empty()) {
        pos = cur_position;
    }
    API::setColor(pos[0], pos[1], 'B');
    API::setText(pos[0], pos[1], "Sol");
}

// Marks a given square cyan when it is part of the bfs
void mark_bfs_api(std::vector<int> pos = {}) {
    if (pos.empty()) {
        pos = cur_position;
    }
    API::setColor(pos[0], pos[1], 'c');
    API::setText(pos[0], pos[1], "dfs");
}

// Marks a square orange that has been used in the backtracking part of the algorithm
void mark_bktrk_api(std::vector<int> pos = {}) {
    if (pos.empty()) {
        pos = cur_position;
    }
    API::setColor(pos[0], pos[1], 'o');
    API::setText(pos[0], pos[1], "back");
}


// Function for printing to MMS console
void log(std::string str) {
    std::cerr << str << std::endl;
}


// ------------------------------------------------------------------------
// Navigation & Movement inside Maze


// Function to take all actions to move forward and update belief state
void move_forward() {
    API::moveForward();  // Move forward in maze
    update_position();  // Update current position
}

// Function to take all actions to turn left and update belief state
void turn_left() {
    API::turnLeft();
    update_direction(-1);  // We are turning left
}

// Function to take all actions to turn right and update belief state
void turn_right() {
    API::turnRight();
    update_direction(+1);  // We are turning right
}

// Function to take all actions to turn around
void turn_around() {
    turn_right();
    turn_right();
}

// Function to change current direction to a specific direction
void set_dir(int _dir) {
    if (_dir == cur_direction) {  // If already facing the correct direction
        return;
    }
    if (_dir == (cur_direction + 1) % 4) {  // If need to turn right once
        turn_right();
        return;
    }
    if (_dir == (cur_direction + 2) % 4) {  // If need to turn around
        turn_around();
        return;
    }
    turn_left();  // If need to turn left once
}

// Function to turn toward an adjacent location object
void turn_toward(Location loc) {
    int _dir = cur_direction;
    // Find direction of adjacent location
    if (cur_position[0] == loc.position[0]) {  // If two locations have the same x coordinate
        if ((cur_position[1] - loc.position[1]) == 1) {  // If mouse is "above" the next location, turn south
            _dir = 2;
        } else {  // Otherwise, mouse must be below the next location
            _dir = 0;
        }
    } else {  // Two directions have the same y coordinate
        if ((cur_position[0] - loc.position[0]) == 1) {  // If mouse is to the right of location, turn west
            _dir = 3;
        } else {  // Mouse must be to the left of the location
            _dir = 1;
        }
    }
    set_dir(_dir); // turning towards desired location
}



//------------------------------------------------------------------------------------
// Algorithm for Exploration

// Maps maze in depth-first search using loc_stack
// Here's how it works:
// It starts by checking if the current location has been visited. If not, it marks the location as visited, sets the walls, and marks the current position in the API.
// It then checks adjacent locations and pushes unvisited ones onto the location stack (loc_stack) for later exploration.
// It enters a loop where it pops locations from the stack until it finds an unvisited one or the stack becomes empty. If the stack becomes empty, it backtracks to the initial position.
// If it finds an unvisited location, it checks if it can move to that location from the current position. If yes, it turns toward the new location, saves the current direction for backtracking, and moves forward. If not, it puts the location back on the stack, backs up one square, and tries again.
// Finally, it recursively calls itself to continue exploring the maze.
// This process repeats until all reachable locations in the maze have been visited.

void dfs_map_maze() {
    Location& cur_loc = maze[cur_position[0]][cur_position[1]]; // Create a reference to the current location object for easier reference (added)

    if (!cur_loc.visited) { // If current location has not been visited
        cur_loc.set_visited(true); // Mark location as visited
        cur_loc.set_walls(get_walls()); // Set wall locations
        mark_visited_api(cur_position); // Mark current position in API

        // If there is no north wall and north location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[0] && !maze[cur_position[0]][cur_position[1] + 1].visited) {
            loc_stack.push(maze[cur_position[0]][cur_position[1] + 1]);
        }

        // If there is no east wall and east location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[1] && !maze[cur_position[0] + 1][cur_position[1]].visited) {
            loc_stack.push(maze[cur_position[0] + 1][cur_position[1]]);
        }

        // If there is no south wall and south location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[2] && !maze[cur_position[0]][cur_position[1] - 1].visited) {
            loc_stack.push(maze[cur_position[0]][cur_position[1] - 1]);
        }

        // If there is no west wall and west location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[3] && !maze[cur_position[0] - 1][cur_position[1]].visited) {
            loc_stack.push(maze[cur_position[0] - 1][cur_position[1]]);
        }
    }

    // Do-while loop to get the next available position if it exists and has not been visited already
    // recursive call inside this while-loop
    Location next_loc;
    while (true) {
        // If loc_stack is empty, backtrack to the initial position then return
        if (loc_stack.empty()) {
            // if we are not at the origin yet, we move back and try again
            if (!(cur_position == std::vector<int>{0, 0})) {
                set_dir((dir_stack.top() + 2) % 4); // Turn around and follow direction stack in reverse order
                dir_stack.pop(); // Remove top element (added)
                move_forward();
                dfs_map_maze(); // Try to move again, recursive
            }
            return;
        }
        next_loc = loc_stack.top(); // Otherwise, take locations off of the loc_stack until we get an unvisited one
        loc_stack.pop(); // removes top element
        if (!next_loc.visited) {
            break; // if the next location has not been visited so far we break out of the loop and continue
        }
    }

    // If I can move to that location from where I am, turn toward new location, save that direction, and move forward
    if (cur_loc.can_move_to(next_loc)) {
        turn_toward(next_loc);
        dir_stack.push(cur_direction); // Save current direction for backtracking on the direction stack
        move_forward();
    } else { // Put the target location back on the loc_stack, back up one square, then try again
        loc_stack.push(next_loc);
        set_dir((dir_stack.top() + 2) % 4); // Turn toward last position
        dir_stack.pop(); // Remove top element (added)
        move_forward();
    }
    dfs_map_maze(); // Try to move again
}


// ------------------------------------------------------------------------------------------------------------------------------
// Algorithm for Solution path

// Defines breadth-first-search for finding optimal route to maze center
State find_bfs_shortest_path() {
    // Initialize all locations to unvisited
    for (int i = 0; i < MAZE_HEIGHT; ++i) {
        for (int j = 0; j < MAZE_WIDTH; ++j) {
            maze[i][j].set_visited(false);
        }
    }

    log("all locations unvisited"); // (added)

    // Generate initial state: parent is self, action is null
    State first_state(&maze[0][0]);
    frontier.push(first_state); // Push first state to queue

    log ("first state pushed to frontier"); // (added)

    // While queue is not empty
    while (!frontier.empty()) {
        // Dequeue next state
        State next_state = frontier.front();
        frontier.pop();

        log("next state dequeued"); // (added)

        // Mark state location as visited
        // maze[next_state.location->position[0]][next_state.location->position[1]].set_visited(true);
        next_state.location->set_visited(true);
        mark_bfs_api(next_state.location->position); // Just for visualization purposes in the MMS

        log("location marked as visited"); // (added)

        // If it is goal, return it
        if (next_state.is_goal()) {
            log("goal state found"); // (added)
            return next_state;
        }
        
        // Hier könnte issue sein
        // Provide new references to my location and possible adjacent locations for easier reference in code below
        Location* my_loc = next_state.location;
        Location* north_loc_ptr = nullptr;
        Location* east_loc_ptr = nullptr;
        Location* south_loc_ptr = nullptr;
        Location* west_loc_ptr = nullptr;

        log("locations initialized"); // (added)
        // checks whether there are walls and adds possible connections from current square
        // due to scope of variables inside if statements, we need to declare them outside of the if statements and use pointers to reference them
        if (!my_loc->walls[0]) {
            // Location north_loc = maze[my_loc->position[0]][my_loc->position[1] + 1];
            north_loc_ptr = &maze[my_loc->position[0]][my_loc->position[1] + 1];
            log("north location added"); // (added)
        }
        if (!my_loc->walls[1]) {
            // Location east_loc = maze[my_loc->position[0] + 1][my_loc->position[1]];
            east_loc_ptr = &maze[my_loc->position[0] + 1][my_loc->position[1]];
            log("east location added"); // (added)
        }
        if (!my_loc->walls[2]) {
            //Location south_loc = maze[my_loc->position[0]][my_loc->position[1] - 1];
            south_loc_ptr = &maze[my_loc->position[0]][my_loc->position[1] - 1];
            log("west location added"); // (added)
        }
        if (!my_loc->walls[3]) {
            // Location west_loc = maze[my_loc->position[0] - 1][my_loc->position[1]];
            west_loc_ptr = &maze[my_loc->position[0] - 1][my_loc->position[1]];
            log("south location added"); // (added)
        }

        log("locations checked for walls"); // (added)

        // If the position north has not been visited and I can reach it, generate a new state representing the new
        // location, with this location as its parent, and and the proper number of turns needed to reach it
        if (north_loc_ptr != nullptr && !north_loc_ptr->visited && my_loc->can_move_to(*north_loc_ptr)) {
            // Create a new state where I move from my_location to the north
            State north_state(north_loc_ptr, &next_state, (0 - next_state.cur_dir + 4) % 4, 0);
            frontier.push(north_state); // Add it to the frontier queue
            log("north state added to frontier"); // (added)
        }
        // Similar Logic for the other directions
        if (east_loc_ptr != nullptr && !east_loc_ptr->visited && my_loc->can_move_to(*east_loc_ptr)) {
            State east_state(east_loc_ptr, &next_state, (1 - next_state.cur_dir + 4) % 4, 1);
            frontier.push(east_state);
            log("east state added to frontier"); // (added)
        }
        if (south_loc_ptr != nullptr && !south_loc_ptr->visited && my_loc->can_move_to(*south_loc_ptr)) {
            State south_state(south_loc_ptr, &next_state, (2 - next_state.cur_dir + 4) % 4, 2);
            frontier.push(south_state);
            log("south state added to frontier"); // (added)
        }
        if (west_loc_ptr != nullptr && !west_loc_ptr->visited && my_loc->can_move_to(*west_loc_ptr)) {
            State west_state(west_loc_ptr, &next_state, (3 - next_state.cur_dir + 4) % 4, 3);
            frontier.push(west_state);
            log("west state added to frontier"); // (added)
        }
        log("locations added to frontier"); // (added)

    }
}

// Takes a solution state and uses it to physically traverse the maze - this constitutes the fastest possible run
void execute_shortest_path(State sol) {
    log("Executing shortest path"); // (added)
    // while (sol.parent != &sol) {   // While I have not reached the home position
    //     log("Checking if I have reached the home position"); // (added)
    //     act_stack.push(sol.action);  // Push action to stack
    //     mark_bktrk_api(sol.location.position);  // Mark the backtrack on the maze for visualiziation
    //     sol = sol.parent;    // Traverse up to parent
    //     log("Traversing up to parent"); // (added)
    // }
    string output = "position: " + to_string(sol.location->position[0]); // (added)  
    log(output); // (added)
    string output2 = "parent: " + to_string((unsigned long long)(void**)sol.parent); + "\n"; // (added)
    log(output2); // (added)
    string output3 = "action: " + to_string(sol.action) + "\n"; // (added)
    log(output3); // (added)
    string output5 = "parent_action: " + to_string(sol.parent->action); + "\n"; // (added)
    log(output5); // (added)
    // string output6 = "parent_parent: " + to_string((unsigned long long)(void**)sol.parent->parent); + "\n"; // (added)
    // log(output6); // (added)
    string output4 = "parent_position: " + to_string(sol.parent->location->position[0]) + "\n"; // (added)    
    log(output4); // (added)
    // string output7 = "parent_cur_dir: " + to_string(sol.parent->cur_dir) + "\n"; // (added)
    // log(output7); // (added)
    while (sol.parent != &sol) {   // While I have not reached the home position
        log("Checking if I have reached the home position"); // (added)
        string output_pos = "position: " + to_string(sol.location->position[0]);
        log(output_pos); // (added)  
        if (sol.parent) {
            log("Parent is null"); // (added)
        }
        output_pos = "position: " + to_string(sol.location->position[1]);
        log(output_pos); // (added)
        output_pos = "parent_position: " + to_string(sol.parent->location->position[0]);
        log(output_pos);
        act_stack.push(sol.action);  // Push action to stack
        // log("Action pushed to stack"); //  (added)
        mark_bktrk_api(sol.location->position);  // Mark the backtrack on the maze for visualization
        // // log("Backtrack marked on maze"); // (added)
        // sol.set_loc = sol.parent->location; // Get parent object
        // // // log("that worked"); // (added)
        // sol.action = sol.parent->action; // Get action from parent object
        // sol.cur_dir = sol.parent->cur_dir; // Get current direction from parent object
        // sol.parent = sol.parent->parent; // Get parent from parent object
        // // log("that worked"); // (added)
        // // log("Traversing up to parent"); // (added)
        sol = *(sol.parent);    // Traverse up to parent
    }
    log("Backtracking complete"); // (added)
    while (!act_stack.empty()) {    // Pop off actions from the stack and execute them in the maze
        int act = act_stack.top();
        act_stack.pop();
        mark_solution_api();  // Mark my square in MMS as part of the solution on the maze for better visualization
        if (act == 1) {
            turn_right();
        } else if (act == 3) {
            turn_left();
        }
        move_forward();
    }
}



// ------------------------------------------------------------------------------------------------------------------------
// Main function

int main() {
    log("Running...");
    initialize_maze(); //initializing maze
    dfs_map_maze();
    log("DFS complete");  // Start facing north at the initial position and end back at the initial position after the maze has been mapped
    set_dir(0);      // Reset heading to north
     // Find the shortest path to solution using breadth-first search
    execute_shortest_path(find_bfs_shortest_path());   // Execute the shortest path solution once found
    log("Done!");
    return 1;
}
