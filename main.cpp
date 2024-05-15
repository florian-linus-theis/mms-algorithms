#include "API.h" // for interacting with the MMS
#include "mms_interaction.h" // importing mms_interaction functions
#include "location.h" // importing location class
#include "maze.h" // importing maze functions
#include "state.h" // importing state class
#include "dfs.h" // importing depth-first-search algorithm
#include "bfs.h" // importing breadth-first-search algorithm
#include "a_star_nodes.h" // importing a_star_node class
#include "a_star_algorithm.h" // importing AStarAlgorithm class


constexpr bool BALLGREIFER = false; // Using the Ballgreifer Version or not?
constexpr int ALGORITHM = 2; // 1 for BFS, 2 for PSP (prioritize straight paths with A* algorithm)
std::vector<int> GOAL_POSITION; // Global variable to store the goal position


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
    BFSAlgorithm bfs(&maze, &GOAL_POSITION, BALLGREIFER); // Initialize BFS algorithm
    int solution_position = bfs.find_bfs_shortest_path(); // Find the shortest path using BFS algorithm (also important for a* as we need to know the goal position first)
    if (ALGORITHM == 1){
        log("Finding shortest path using BFS...");
        bfs.execute_shortest_path(solution_position); // Execute the shortest path found by the BFS algorithm
    } else if (ALGORITHM == 2){
        log("Prioritizing straight paths...");
        AStarAlgorithm a_star_algorithm(&maze, GOAL_POSITION, BALLGREIFER); // Initialize A* algorithm
        A_star_node* solution_a_star = a_star_algorithm.prioritize_straight_paths(); // Find the shortest path using A* algorithm
        a_star_algorithm.execute_shortest_path_psp(solution_a_star); // Execute the shortest path found by the A* algorithm
    }
    log("Done!");
    return 0;
}
