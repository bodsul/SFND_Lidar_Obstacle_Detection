/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <cmath>
//#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert_at_node(std::vector<float> point, int id, int split_dim, Node* &node)
	{
		if(!node) node = new Node(point, id);
		else{
			if(node->point[split_dim] > point[split_dim]) insert_at_node(point, id, (split_dim + 1)%point.size(), node->left);
			else insert_at_node(point, id, (split_dim + 1)%point.size(), node->right);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_at_node(point, id, 0, this->root);
	}

	float distance(std::vector<float> first, std::vector<float> second)
	{
		float tot = 0;
		for (int j=0; j <first.size(); ++j) tot+=(first[j]-second[j])*(first[j]-second[j]);
		return pow(tot, 0.5);
	}

	void search_tree_from_node(std::vector<float> point, float distanceTol, std::vector<int>& ids, int split_dim, Node* node)
	{
		if(node)
		{
			if(node->point[split_dim] > point[split_dim]){
				if(node->point[split_dim] - point[split_dim] > distanceTol) search_tree_from_node(point, distanceTol, ids, (split_dim + 1)%point.size(), node->left);
				else {
					if(distance(node->point, point) < distanceTol) ids.push_back(node->id);
					search_tree_from_node(point, distanceTol, ids, (split_dim + 1)%point.size(), node->left);
					search_tree_from_node(point, distanceTol, ids, (split_dim + 1)%point.size(), node->right);
				}
			}
			else{
				if(point[split_dim] - node->point[split_dim] > distanceTol) search_tree_from_node(point, distanceTol, ids, (split_dim + 1)%point.size(), node->right);
				else {
					if(distance(node->point, point) < distanceTol) ids.push_back(node->id);
					search_tree_from_node(point, distanceTol, ids, (split_dim + 1)%point.size(), node->right);
					search_tree_from_node(point, distanceTol, ids, (split_dim + 1)%point.size(), node->left);
				}				
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_tree_from_node(target, distanceTol, ids, 0, root);
		return ids;
	}

	void free_heap(Node* & node)
	{
		if(node)
		{
			if (node->left) free_heap(node->left);
			if (node->right) free_heap(node->right);
			delete node;
		}
	}
	
	~KdTree()
	{
		free_heap(root);
	}
};

std::vector<int> proximity(const std::vector<std::vector<float>>& points, int id, KdTree* tree, float distanceTol, std::unordered_set<int>& processed_points)
{
	processed_points.insert(id);
	for(int close_point_id: tree->search(points[id], distanceTol))
	{
		if(std::find(processed_points.begin(), processed_points.end(), close_point_id)==processed_points.end())
		{
			proximity(points, close_point_id, tree, distanceTol, processed_points);
		}
	}
	return std::vector<int>(processed_points.begin(), processed_points.end());
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;	
	std::unordered_set<int> processed_points;
	for (int j=0; j<points.size(); ++j){
		std::unordered_set<int> new_processed_points;
		if(std::find(processed_points.begin(), processed_points.end(), j) == processed_points.end()){
			clusters.push_back(proximity(points, j, tree, distanceTol, new_processed_points));
			processed_points.insert(new_processed_points.begin(), new_processed_points.end());
		}
	}
	return clusters;
}



