#include <vector> // For lists/arrays with improved memory management
#include <iostream> // for input/output
#include <stack> // For LIFO stack
#include <queue> // For FIFO queue (kann weg)
#include "API.h" // Later purposes
#include <string> // For string operations
#include <windows.h> // For sleep Function in Ballgreifer simulation

// #include "./location.cpp" // importing location class
#include "location.h"
#include "state.h" // importing state class
using namespace std;

// Using the Ballgreifer Version or not?
constexpr bool BALLGREIFER = true;

// Initializing maze size with constexpr to set them up as compile-time-constants rather than runtime constants
constexpr int MAZE_WIDTH = 16;
constexpr int MAZE_HEIGHT = 16;

// For tracking current global direction
// 0 = North, 1 = East, 2 = South, 3 = West
int cur_direction = 0;

// For tracking current global 'physical' position in maze as [x, y], initialized to [0, 0] (i.e. bottom left cell, origin)
std::vector<int> cur_position = {0, 0};

// For tracking all maze data, create a 2D vector of Locations
std::vector<std::vector<Location>> maze (MAZE_HEIGHT, std::vector<Location>(MAZE_WIDTH));

// Initialize the maze with Location objects
// maze is a global variable so no need to pass it here
void initialize_maze(){
  for (int i = 0; i < MAZE_HEIGHT; ++i) {
      for (int j = 0; j < MAZE_WIDTH; ++j) {
          maze[i][j] = Location({i, j});
      }
  }
}


// To initialize all walls in the Ballgreifer-Area
void initialize_maze_with_ballgreifer(){
    // first cell
    maze[1][0].set_visited(true);
    maze[1][0].set_walls({false, false, true, true});
    maze[1][0].set_ballgreifer(true);
    // second cell
    maze[2][0].set_visited(true);
    maze[2][0].set_walls({false, true, true, false});
    maze[2][0].set_ballgreifer(true);
    // third cell
    maze[1][1].set_visited(true);
    maze[1][1].set_walls({false, false, false, false});
    maze[1][1].set_ballgreifer(true);
    // fourth cell
    maze[2][1].set_visited(true);
    maze[2][1].set_walls({false, true, false, false});
    maze[2][1].set_ballgreifer(true);
    // fifth cell
    maze[1][2].set_visited(true);
    maze[1][2].set_walls({true, false, false, false});
    maze[1][2].set_ballgreifer(true);
    // sixth cell
    maze[2][2].set_visited(true);
    maze[2][2].set_walls({true, true, false, false});
    maze[2][2].set_ballgreifer(true);
}

// Location object stack for tracking locations that may need to be explored during mapping
std::stack<Location> loc_stack;

// Direction stack for easy backtracking through maze when a dead end is found during mapping
std::stack<int> dir_stack;

// Action stack/vector for processing optimal sequence of actions to find goal state
std::vector<int> act_vector;

// Global array/vector to act as a queue to save all the states such that we do not get any memory leaks
std::vector<State> state_vector;

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
// If it finds an unvisited location, it checks if it can move to that location from the current position. 
// If yes, it turns toward the new location, saves the current direction for backtracking, and moves forward. If not, it puts the location back on the stack, backs up one square, and tries again.
// Finally, it recursively calls itself to continue exploring the maze.
// This process repeats until all reachable locations in the maze have been visited.

void dfs_map_maze() {
    Location& cur_loc = maze[cur_position[0]][cur_position[1]]; // Create a reference to the current location object for easier reference (added)

    if (!cur_loc.visited) { // If current location has not been visited
        cur_loc.set_visited(true); // Mark location as visited
        cur_loc.set_walls(get_walls()); // Set wall locations
        mark_visited_api(cur_position); // Mark current position in API

        // If there is no north wall and north location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[0] && !maze[cur_position[0]][cur_position[1] + 1].visited && !maze[cur_position[0]][cur_position[1] + 1].ballgreifer) {
            loc_stack.push(maze[cur_position[0]][cur_position[1] + 1]);
        }

        // If there is no east wall and east location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[1] && !maze[cur_position[0] + 1][cur_position[1]].visited && !maze[cur_position[0] + 1][cur_position[1]].ballgreifer) {
            loc_stack.push(maze[cur_position[0] + 1][cur_position[1]]);
        }

        // If there is no south wall and south location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[2] && !maze[cur_position[0]][cur_position[1] - 1].visited && !maze[cur_position[0]][cur_position[1] - 1].ballgreifer) {
            loc_stack.push(maze[cur_position[0]][cur_position[1] - 1]);
        }

        // If there is no west wall and west location is not visited, put it on loc_stack to explore later
        if (!cur_loc.walls[3] && !maze[cur_position[0] - 1][cur_position[1]].visited && !maze[cur_position[0] - 1][cur_position[1]].ballgreifer) {
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

// This code implements a breadth-first search algorithm to find the shortest path in a maze. Here's a brief explanation of how it works:
// Initialize Locations: All locations in the maze are initialized as unvisited.
// Generate Initial State: Create the initial state with the starting location of the maze and push it into global vector called state_vector.
// Main Loop:
// Loop until every state in the state_vector has been explored.
// Get the current state from the state_vector.
// Mark the current location as visited.
// If the current state is a goal state, return its position in the state_vector.
// For each possible move (north, east, south, west), check if there are no walls and the next location has not been visited. 
// If so, create a new state for that move, push it into the state_vector, and mark the new location as visited.

int find_bfs_shortest_path() {
    // Initialize all locations to unvisited
    for (int i = 0; i < MAZE_HEIGHT; ++i) {
        for (int j = 0; j < MAZE_WIDTH; ++j) {
            maze[i][j].set_visited(false);
        }
    }
    log("Marking all locations as unvisited"); // (added)

    // Generate initial state: parent is self, action is null
    State first_state(&maze[0][0]);
    // frontier.push(first_state); // Push first state to queue
    state_vector.push_back(first_state); // Push first state to our queue
    // log(first_state.to_string()); // (added

    int counter = 0; 
    // while we have not every state in the state_vector
    while (counter < state_vector.size()) {
        if (counter >= MAZE_HEIGHT*MAZE_WIDTH) { // If we have explored every possible state
            log("We have explored all states."); // (added)
            break;
        }
        State* current_state = &state_vector[counter]; // Get the first node (state) we have not yet explored from the global array (FIFO)
        current_state->location->set_visited(true);  // Mark state's location as visited
        mark_bfs_api(current_state->location->position); // Just for visualization purposes in the MMS
        
        // If it is goal, return the position of it in global state vector
        if (current_state->is_goal()) {
            log("goal state found");
            return counter;
        }

        // Provide new references to my location and position for easier reference/better readability in code below
        Location* my_loc = current_state->location;
        int pos_0 = my_loc->position[0];
        int pos_1 = my_loc->position[1];

        // checks whether there are walls and adds possible connections from current square
        // Links the next locations to the current location if there are no walls and the next location has not been visited (current location becomes parent of next location)
        if(!my_loc->walls[0] && !maze[pos_0][pos_1 + 1].visited && my_loc->can_move_to(maze[pos_0][pos_1 + 1]) && my_loc->ballgreifer == false) {
            State north_state(&maze[pos_0][pos_1 + 1], (0 - current_state->cur_dir + 4) % 4, 0, counter);
            north_state.location->set_visited(true); // Mark the location as visited
            // log(north_state.to_string()); // print to terminal
            state_vector.push_back(north_state);
        }
        if(!my_loc->walls[1] && !maze[pos_0 + 1][pos_1].visited && my_loc->can_move_to(maze[pos_0 + 1][pos_1]) && my_loc->ballgreifer == false) {
            State east_state(&maze[pos_0 + 1][pos_1], (1 - current_state->cur_dir + 4) % 4, 1, counter);
            east_state.location->set_visited(true); // Mark the location as visited
            // log(east_state.to_string()); // print to terminal
            state_vector.push_back(east_state);
        }
        if(!my_loc->walls[2] && !maze[pos_0][pos_1 - 1].visited && my_loc->can_move_to(maze[pos_0][pos_1 - 1]) && my_loc->ballgreifer == false) {
            State south_state(&maze[pos_0][pos_1 - 1], (2 - current_state->cur_dir + 4) % 4, 2, counter);
            south_state.location->set_visited(true); // Mark the location as visited
            // log(south_state.to_string()); // print to terminal
            state_vector.push_back(south_state);
        }
        if(!my_loc->walls[3] && !maze[pos_0 - 1][pos_1].visited && my_loc->can_move_to(maze[pos_0 - 1][pos_1]) && my_loc->ballgreifer == false) {
            State west_state(&maze[pos_0 - 1][pos_1], (3 - current_state->cur_dir + 4) % 4, 3, counter);
            west_state.location->set_visited(true); // Mark the location as visited
            // log(west_state.to_string()); // print to terminal
            state_vector.push_back(west_state);
        }

        counter++;     
    }
    log("No solution found");
    return -1; // If no solution is found
}

// ------------------------------------------------------------------------------------------------------------------------
// Function to grab the ball 

void grab_ball(){
    // Drive to the ball
    log("grabbing ball");
    move_forward(); move_forward(); turn_right(); move_forward();
    // Sleep for 1 second to simulate grabbing the ball
    Sleep(2000); 
    log("ball grabbed");
    // Drive back to the start
    turn_around(); move_forward(); turn_right();
}


// Takes a solution state and uses it to physically traverse the maze - this constitutes the fastest possible run
void execute_shortest_path(int solution_position) {
    if (solution_position == -1) {  // If no solution was found
        log("No solution found");
        return;
    }
    State state = state_vector[solution_position]; // Get the goal state
    while (state.action != -1) {   // While I have not reached the home position
        // log(state.to_string()); // (added)
        act_vector.push_back(state.action);  // Push action to vector
        mark_bktrk_api(state.location->position);  // Mark the backtrack on the maze for visualization
        state = state_vector[state.parent];    // Traverse up to parent
    }
    int counter = act_vector.size() - 1;
    log("Backtracking complete");
    // Accounting for the fact that we might have to grab the ball first
    if (BALLGREIFER == true) {
        grab_ball();
        counter -= 2; // So that we skip the first two actions
    }
    log("Executing shortest path");
    while (counter >= 0) {    // start at the back of actions (at origin) and execute them in the maze until we are at the first action
        int act = act_vector[counter]; // Get action from vector
        mark_solution_api();  // Mark my square in MMS as part of the solution on the maze for better visualization
        if (act == 1) {
            turn_right();
        } else if (act == 3) {
            turn_left();
        }
        move_forward();
        counter--;
    }
}



// ------------------------------------------------------------------------------------------------------------------------
// Main function

int main() { 
    log("Starting...");
    initialize_maze(); //initializing maze
    if (BALLGREIFER == true) {
        initialize_maze_with_ballgreifer();
    }
    log("Mapping the maze...");
    dfs_map_maze(); // Mapping the maze using depth-first search
    log("DFS complete"); 
    set_dir(0); // Reset heading to north
    int position_solution = find_bfs_shortest_path(); // Find the shortest path to solution using breadth-first search
    execute_shortest_path(position_solution);   // Execute the shortest path solution once found
    log("Done!");
    return 0;
}
