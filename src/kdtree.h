/* \author Aaron Brown */
// Quiz on implementing kd tree

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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root,point,id,0);
	}

	void insertHelper(Node*& node,std::vector<float> point, int id, int depth)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(node==NULL)
		{
			node = new Node(point,id);			
		}
		else
		{			
			if(point[depth%3] < node->point[depth%3])
			{
				insertHelper(node->left,point,id, depth+1);
			}
			else
			{
				insertHelper(node->right,point,id, depth+1);
			}
		}		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		// Time segmentation process
		//auto startTime = std::chrono::steady_clock::now();
		//

		searchHelper(ids,root,target,distanceTol,0);

		//auto endTime = std::chrono::steady_clock::now();
		//auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
		//std::cout << "kd search took " << elapsedTime.count() << " microseconds" << std::endl;

		return ids;
	}
	/*
	void searchHelper(std::vector<int>& ids,Node*& node,std::vector<float> target, float distanceTol, int depth)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(node!=NULL)
		{
			if((target[depth%2]-distanceTol < node->point[depth%2])&&(target[depth%2]+distanceTol > node->point[depth%2]))
			{
				if(CalculateDistance(node->point,target,distanceTol))
				{
					ids.push_back(node->id);
				}
				
				searchHelper(ids,node->left,target,distanceTol,depth+1);
				searchHelper(ids,node->right,target,distanceTol,depth+1);
			}
			else
			{
				if(target[depth%2] < node->point[depth%2])
				{
					searchHelper(ids,node->left,target,distanceTol,depth+1);
				}
				else
				{
					searchHelper(ids,node->right,target,distanceTol,depth+1);
				}
			}
		}		
	}
	*/
	
	void searchHelper(std::vector<int>& ids,Node*& node,std::vector<float> target, float distanceTol, int depth)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(node!=NULL)
		{
			if((target[0]-distanceTol < node->point[0])&&(target[0]+distanceTol > node->point[0])
			&& (target[1]-distanceTol < node->point[1])&&(target[1]+distanceTol > node->point[1])
			&& (target[2]-distanceTol < node->point[2])&&(target[2]+distanceTol > node->point[2])
			)
			{
				if(CalculateDistance(node->point,target,distanceTol))
				{
					ids.push_back(node->id);
				}				
			}
			
			if(target[depth%3]-distanceTol < node->point[depth%3])
			{
				searchHelper(ids,node->left,target,distanceTol,depth+1);
			}
			if(target[depth%3]+distanceTol > node->point[depth%3])
			{
				searchHelper(ids,node->right,target,distanceTol,depth+1);
			}
			
		}		
	}	

	bool CalculateDistance(std::vector<float>& ref,std::vector<float>& target,float distanceTol)
	{
		float diffx = ref[0] - target[0];
		float diffy = ref[1] - target[1];
		float diffz = ref[2] - target[2];
		if(sqrt(diffx*diffx+diffy*diffy+diffz*diffz)<distanceTol) return true;
		return false;
	}
	template<typename PointT>
	void InsertPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		std::random_shuffle(cloud->points.begin(),cloud->points.end());
		for(int i =0; i < cloud->size();i++)
		{
			auto d = cloud->points[i];
			std::vector<float> data = {d.x,d.y,d.z};
			insert(data,i);
		}
	}
};




