/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	//childern 
	Node* left;
	Node* right;

	//the node can create now node, arr is the point assigned to it
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

	void inserHelper (Node **node, uint depth, std::vector<float> point, int ID){ // node need to be double pointer becuse the member of the struct is a pointer

		if (*node == NULL){// create a new node
			*node = new Node(point,ID);
		}
		else {
			// now we need to check whether we should split on x-axis or y-axis
			uint cd = depth %3 ; //0 meaning split x-axis, 1 meaning split on y-axis, 2 meaning split on z-axis

			//we just need to place cd in the points array and that will take care of the spliting in x, y or z.	
			if (point[cd] < ((*node)->point[cd]) ){
				//we need make the fucntion call itself until it reach a NULL and assign the point to a new node
				inserHelper( (&(*node)->left), depth+1, point,ID);
				// the weird passing here is passing the left node of the current node by reference so we can modify it.
			}
			else{

				inserHelper( (&(*node)->right), depth+1, point,ID);

			}

		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree

		inserHelper(&root,0,point,id); 
		//what this fucntion do is: start at root and call it self until it reach a NUll and assign the point to a new node.

		// the function should create a new node and place correctly with in the root 

	}

	void searchHelper (std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids ){



		if (node!=NULL){
			//we need first to check if the point is within the bounding box
			float uper_x_boundary_box= target[0]+distanceTol;
			float lower_x_boundary_box= target[0]-distanceTol; 
			float uper_y_boundary_box= target[1]+distanceTol;
			float lower_y_boundary_box= target[1]-distanceTol;
			float uper_z_boundary_box = target[2]+distanceTol;
			float lower_z_boundary_box = target[2]-distanceTol;
			//i think this is reducance step: we can just check the distance directly
			if ( (node->point[0] >= lower_x_boundary_box) && (node->point[0] <= uper_x_boundary_box) && (node->point[1] <= uper_y_boundary_box) && (node->point[1] >= lower_y_boundary_box) && (node->point[2] <= uper_z_boundary_box) && (node->point[2] >= lower_z_boundary_box) ){

				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1])+(node->point[2]-target[2])*(node->point[2]-target[2])); 
				if ( distance <= distanceTol){
					ids.push_back(node->id);
				}
			}

			//now we need to decide whether we are going right or left. This will depends on the depth and the value of the point
			//this should be with respect to the boundary box
			int cd = depth % 3;//this will decide the spliting axis. 

			//the cd will take care of spliting the x and y
			float lower_boundary=target[cd]-distanceTol;
			float upper_boundary_box= target[cd]+distanceTol;


			if (node->point[cd] > lower_boundary){
				//we will go to the right point of the current node 
				//recursively
				searchHelper(target,node->left,depth+1,distanceTol, ids);
			}
			if (node->point[cd] < upper_boundary_box){
				searchHelper(target,node->right,depth+1,distanceTol, ids);
			}
			
			

		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		//we need to do the search recursively starting from the root. 
		
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids); // start from the  root and stops until a NULL is reached
		return ids;
	}
	

};




