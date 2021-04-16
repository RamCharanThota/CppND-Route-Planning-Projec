#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.y
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node); //
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (RouteModel::Node *each_current_neighbor : current_node->neighbors)
    {                                                                                                                // looping through all current node neighbours
        each_current_neighbor->parent = current_node;                                                                // set the parent
        each_current_neighbor->h_value = CalculateHValue(each_current_neighbor);                                     // set the h value
        each_current_neighbor->g_value = (current_node->g_value) + (current_node->distance(*each_current_neighbor)); // set gavlue
        open_list.push_back(each_current_neighbor);                                                                  // added neighbor to open_list
        each_current_neighbor->visited = true;                                                                       // visited nodes as true.
    }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

//function to compare f=(g+h) values of two nodes
bool Compare(RouteModel::Node *node1, RouteModel::Node *node2)
{
    double f_val_node1 = node1->h_value + node1->g_value; // calculating fvalue of first node
    double f_val_node2 = node2->h_value + node2->g_value; // calculating fvalue of second node

    if (f_val_node1 > f_val_node2)
    {
        return true;
    }

    return false;
}

RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), Compare); // sorting openlist vector descending order
    RouteModel::Node *low_sum_node = open_list.back();      // creating and assigning the pointer to  node with lowest  f value
    open_list.pop_back();                                   // delete the node from the list

    return low_sum_node; // return the pointer to node with lowest f value
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);

    // TODO: Implement your solution here.
    while (current_node != start_node)
    {
        distance = distance + current_node->distance(*current_node->parent);// 
        current_node = current_node->parent;
        path_found.push_back(*current_node);
    }

    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;
     

    start_node->visited=true;
    open_list.push_back(start_node);
    while(open_list.size()>0){
        current_node=NextNode();
        if (current_node->distance(*end_node)==0){
            m_Model.path=ConstructFinalPath(current_node);
            return;

        }

        AddNeighbors(current_node);

    }    
    
}
