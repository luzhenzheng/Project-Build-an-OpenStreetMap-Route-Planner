#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    //from user input (start_x,start_y,end_x,end_y) to write into start_node and end_node attribute
    start_node = & (model.FindClosestNode(start_x,start_y));
    end_node = & (model.FindClosestNode(end_x,end_y));

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    //loop through all the neighbor nodes of current_node
    for(RouteModel::Node *node:current_node->neighbors)
    {
        //the parent of neighbor nodes is surely current_node
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value= current_node->g_value+node->distance(*current_node);
        open_list.emplace_back(node);
        //mark the node as visited, same as marked as closed in step 4 e in pseudo code
        node->visited=true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
bool CompareNode(RouteModel::Node* a,RouteModel::Node *b)
{
    return (a->g_value+a->h_value)>(b->g_value + b->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(this->open_list.begin(),this->open_list.end(),&CompareNode);//the node with largest f value is at the begin, the node with lowest f value is at the end of the open list
    RouteModel::Node * node = this->open_list.back();//the node with lowest f value is at the end of open list.
    open_list.pop_back();//pop out the node with lowest f value from open list, this node is selected as part of path
    return node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while (current_node != start_node)
    {
        //path_found is of type vector<Node> not vector<Node*> so you need to dereference here.
        path_found.emplace_back(*current_node);
        //sum up the distance backwards;
        //the parent attribute was already decided inside the AddNeighbors() method
        this->distance+=current_node->distance(*(current_node->parent));
        current_node =  current_node->parent;   
    }
    //dont miss the first node (start_node)
    path_found.emplace_back(*current_node);
    //the current path_found has end_node at the beginning, so have to reverse it
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

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.


    //start_node was already initialized at the constructor
    current_node = start_node;

    //initialize the open list by appending current_node(start_node) into it.
    current_node->visited=true;
    this->open_list.emplace_back(current_node);

    while (current_node!=end_node)
    {
        //add all neighbor nodes into the open_list
        AddNeighbors(current_node);
        //NextNode() looks for the node inside the open_list with the least f value and return it
        current_node = NextNode();
    }

    std::vector<RouteModel::Node> final_path = ConstructFinalPath(current_node);
    m_Model.path = final_path;

}