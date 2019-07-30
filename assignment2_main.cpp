//Program: Implement Dijkstra's Algorithm
//Author: Parul Agrawal
//Date: 22/7/19
//Brief: Implementaion of Dijkstra's algorithm with following guidlines:
//	1.Three classes are implemented i.e. Graph, PriorityQueue, ShortestPath
//	2.The functionalities which are not required in code are only declared but not implemented.
//	3.Excessive commenting is intentional.
//
//Important:
//	1.For storing graph adjcency matrix is used.
//	2.Some changes are made in definitions of functions according to implementation like different no. of aruguments but the functionality is same.
//	3.Main function is present in assignment_main.cpp
//	4.Graph is considered to be undirected
//	5.Random weights are initialized
//	6.Enter the desitination node between 0 to v-1

#include<iostream>
#include<vector>
#include <cstdlib>
#include<ctime>
#include <limits>
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////
////                                            CLASS,PROPERTIES AND FUNCTION DECLARATIONS
////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




class Graph
{
	
	private:                                                //all properties are defined here
			vector<vector<int> > 	graph;                  //for storing graph
			int 					v;                      //for total no. of vertices in graph
			int						e;                      //for storing total no. of vertces in graph
	
	
	public:                                                 //all member functions are defined here
		Graph ( int, 
		        int );                                      //constructor
		int V();                                            //declared only not implemented
		int E();                                            //declared only not implemented
		bool adjacent(int,
					  int);                                 //declared only not implemented
		void deleteEdge(int,
						int);                               //declared only not implemented
		int get_node_value(int);                            //declared only not implemented
		void set_node_value(int,
		                    int);                           //declared only not implemented
		int  get_edge_value(int,
		                    int);                           //declared only not implemented
		int set_edge_value(int,
						   int,
						   int);                            //declared only not implemented
		void addEdge(int,
					 int,
					 int);                                  //declared only not implemented
		void  printGraph( );                                //prints graph on screen
		void  initialiseGraph ( float,
								int,
								int,
								float,
								float );                    //initialize graph with random values
		bool  isConnected ( int );                          //find that graph is connected or not
		vector<vector<int> > getGraph();                    //returns graph(property) 
		void dfs(int,vector<bool> &);                       //used in implementing depth first search algorithm
		
};



//priority queue is implemented as array dist
class	PriorityQueue
{
   int                    v;                                 //for stroing no. of vertices
   vector<vector<int> >   g;                                 //for storing weight matrix of graph
   vector<int>            dist;                              //for storing distances of all nodes for source
   vector<bool>           visited_node;                      //mark if node is visited or not
   vector<int>            previous_node;                     //stores the nodes visited before visiting the node corresponding to index
   
   
   public:                                                   //constructor declaration
	    PriorityQueue(	int,
	   				 	int ,
					 	const vector<vector<int> > &);
		void chgPrioirity(int);	                             //declared but not implemented
		bool contains(int);                                  //declared but not implemented
		bool insert(int);                                    //declared but not implemented
		int top();                                           //declared but not implemented
		int size();                                          //declared but not implemented
   		void	find_path();                                 //finds path of every node from source node 
   	    int		min_distance();                              //equivalent to minPriority()                            
   		vector<int> getParents();                            //returns parent of every node
   		vector<int> getDistances();                          //return distance of every node from source node
   	
};
class ShortestPath
{
	
	private:
		int				v; 						//for storing no. of vertices in graph
		int 			e;  					//for stroring no. of edges in graph
		vector<int> 	previous_node;  		//stores nodes visited before visiting the node corresponding to index value
		vector<int>     distance;       		//for storing distaces(cost) of nodes from source node
		
		
		
	public:
		
		ShortestPath ( int vIn,              	//constructor-for initiaisation of properties of class ShortestPath
					   int eIn,
					   vector<int> previous_nodeIn)
		{
			v=vIn;
			e=eIn;
			previous_node=previous_nodeIn;
		}

		void path(int);                 		//finds path(node visited) from specified source to the destination passed as argument. 
		void vertices();						//returns list of vertices in graph.declared but not implemented
		void path_size();						//returns cost associated with the path used to reach destination(argumant) .declared but not implemented
		 	
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////\
//////
//////                                                      DEFINING CLASSES AND METHODS
//////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




Graph::	Graph(	int vertices,
				int edges) : e(edges),v(vertices)
{
	graph.resize(v);
	for (int i = 0;		 i < v;		 ++i)				//adding one dimensional arrays to 2d array pointer
	{
		 graph[i].resize(v);
	}
	   
	for(int i=0;		i<v;		i++)
	{
		for(int j=0;		j<v;		j++)
		{
			graph[i][j]=0;								//initializing all graph weights to 0
		}
	}
}



void	Graph::	printGraph()
{
	for(int i=0;		i<v;		i++)
	{
		for(int j=0;		j<v;		j++)
		{	
			cout<<graph[i][j]<<" ";					//printing node values
	    }
		cout<<"\n";
	}
}




void Graph:: initialiseGraph(	float density,
						int v,
						int e,
						float lowerRange,
						float upperRange)
{
	int numOfEdgesForInitialise	= (int)e * density;					//calculation of no. of edges to be initialized according to density
	srand(time(0));                                                 //giving seed to random generator
		
	for(int i=0;  i<numOfEdgesForInitialise; i++)
	{
	 int weight	= lowerRange 	+ 	(std::rand() % (int)(upperRange-lowerRange+1)	);   //calculating random weight 
	 int v1	=	(	std::rand()	% v);                                                    //picking up random vertex
	 int v2	=	(	std::rand()	 %	v);                                                  //picking up another random vertex 
	 
	 if( v1!=v2 &&                                                                       //if edge corresponding to two randomly picked vertices is not present
	 	 !graph[v1][v2] )                                                                //also checking for self loop,then initialize for weight
		{	 
			graph[v1][v2]=weight;
	 		graph[v2][v1]=weight;	
		}
	 else
		{
		 numOfEdgesForInitialise ++;                                                    //if there is self loop or existing edge is chosen then one more chance is given to loop 
		}	
	}
}




void Graph::dfs(int i,vector<bool> &visited)
{
	visited[i] = 1;                                                                     //marking the node visited i.e. 1
	for(int j=0; j <v ;j++)
	{
		if(graph[i][j] != 0  &&  visited[j] == 0)                                       //for all nodes checking if edge exist and one vertex is not visited
		{
			dfs(j,visited);                                                             //recursively calling dfs for connected nodes
		}
	}	
}




bool Graph::isConnected(int i)
{
	vector<bool> visited( v, false);                                                    //initializing all nodes with not visited
	dfs( i, visited);                                                                
	for(int j=0;j<v;j++)                                                               
	{
		if(!visited[j])                                                                //if any node is not visited then function will return false
		{
			return false;
		}
	}
	return true;	
}
					
			
		
vector<vector<int> > 	Graph::getGraph()
{
	return graph;                                                                       //returns graph
}





void Graph::	 addEdge(	int v1,
							int v2,
							int weight)
{
	if(	graph[v1][v2]	==	0)
	{
	  	graph[v1][v2] = weight;
	  	graph[v2][v1] = weight;
	}
}






PriorityQueue::PriorityQueue(int vIn,
	   				 	     int src,
					 	     const vector<vector<int> > &graphIn):g(graphIn)
{		
	v=vIn;
	dist.resize(v,std::numeric_limits<int>::max());                                          //initially assing every distance value as infinite
	visited_node.resize(v,false);                                                            //initially marking every node not visited
	previous_node.resize(v,0);                                                               //all parents are initiallt assigned 0
	previous_node[src]=-1;                                                                   //assingning base case as -1 i.e. parent of source can be value that is not in vertices
	dist[src]=0;                                                                             //initialising distance of source to source
}







void 	PriorityQueue	::	find_path()
{ 
     int x;
	 for (int count = 0;	count < v-1;   count++)                                           
     {
        x	= 	min_distance();                                                              //finding mi distance node from priority queue
	    visited_node[x]	=	true;                                                            //marking node visited
	    for (int vertex = 0;    vertex < v;   vertex++)                                      //now for all node that are reachable from min disatnce node selected,select nodes
		{  
		   if ( 	dist[x] != std::numeric_limits<int>::max() &&                             //if min dist node is already visited 
		        	g[x][vertex] &&                                                          //and edge exist between min dist node and newly selected node
					!visited_node[vertex] &&                                                 //and the newly selected node is not visited
					dist[x]+g[x][vertex] < dist[vertex])                                     //and distance of newly visited node is less than previous value
				{
					previous_node[vertex]=x;	                                             //then update the parent of newly selected node
				    dist[vertex] = dist[x] + g[x][vertex];                                   //along with its cost value
				    
				}       	
		} 
   }
}



vector<int> PriorityQueue::getDistances()
{
	return dist;                                                                             //returns all distances from source
} 



vector<int> PriorityQueue::getParents()
{
	return previous_node;                                                                   //returns parents of all nodes
}

int  PriorityQueue    ::   min_distance()
{
	int min_index,
	    min=std::numeric_limits<int>::max();                                               //initially min distance is set to infinity then value is gradually	       
	for (int vertex = 0;		vertex < v;		 vertex++)
	{
		 if (visited_node[vertex] == false	 && 	dist[vertex] 	<= 	min)               //if node is not visited and distance is less than present min then update min
		 {
		 	min = dist[vertex],	min_index = vertex;
		 }         
	}  
   return min_index; 
}




//Class ShortestPath-Used for storing paths generated by algorithm


void ShortestPath :: path(int j)  
{
	if ( 	previous_node[j]  == - 1	)  				//If j is the source node
        return; 
  
    path (	previous_node[j]	); 					//recursive calling for tracking back the path by calling previous node of previous node.....
    cout << j << "->";
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////
//////                                                   MAIN FUNCTION()
//////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





int main()
{
	
	int 					v;
	int 					e;
	float 					density;                  
	float 					lower;                                           //lower limit for distance
	float 					upper;                                           //upper limit for distance
	
	cout <<	"Enter the no. of vertices in Graph\n";                    
	cin	>>	v;
	e = v * (v-1) / 2;                                                      //calculation of no. of total no. of edges in a fully connected graph
	
	cout <<	"Enter the density of graph\n";
	cin	>>	density;
		
	cout <<	"Enter lowest number for distance range\n";
	cin	>> lower;
	
	cout <<	"Enter highest number for distance range\n";
	cin	>>	upper;
	
	Graph *g=new Graph(v,e);                                                
	g->initialiseGraph(density,v,e,lower,upper);
	cout<<endl<<"Graph is initialised.Your graph is:"<<endl<<endl;
	g->printGraph();
	cout<<endl<<endl;
	while(1)                                                                   //will run till it not found graph is connected
	{
	 	if(!g->isConnected(0))                                                      
		{
			cout<<endl;
			cout << "Initialised graph is not connected.Please try for another graph with different vertices and density values."<<endl<<endl;
			cout <<	"Enter the no. of vertices in Graph\n";
			cin	>>	v;
			cout <<	"Enter the density of graph\n";
			cin	>>	density;
			e = v * (v-1) / 2;
			delete g;
			Graph *g=new Graph(v,e);;
			cout<<e<<endl;
			g->initialiseGraph(density,v,e,lower,upper);
			cout<<endl<<"Graph is initialised.Your graph is:"<<endl<<endl;
			g->printGraph();
			cout<<endl<<endl;
		}
		else
		{
			cout<<endl<<endl<<"Congrats!Graph is connected.We can implement Dijkstra on this graph"<<endl<<endl;
			break;
		}	
	}
	const vector<vector<int> > graph=g->getGraph();
    cout<<endl<<"Enter the source from which path has to be calculated"<<endl;
    int src;
    cin>>src;
    cout<<endl<<endl;
	PriorityQueue p(v,src,graph);                                             
	p.find_path();
	vector<int> distances=p.getDistances();
	vector<int> parents=p.getParents();
    ShortestPath s(v,e,parents);
    cout<<endl<<"Enter the destination for which path has to be calculated"<<endl;
    int destination;
    cin>>destination;
    cout<<"Cost of distance for reaching destination is "<<distances[destination]<<endl<<endl;

	if(src==destination)
	{
		cout<<"source and destinations are same"<<endl<<endl;
	}
	else
	{
		cout<<"Path is:"<<endl;
	    cout<<src<<"->";
		s.path(destination);
	    cout<<"end"<<endl<<endl;
	}
	int x=0;
	while(1)                                                                                         //will run for multiple destinations
	{
		cout<<"Do you want to find path for another destination?If yes,please 1 else 0"<<endl;
		cin>>x;
		if(x)
		{
			cout<<endl<<"Enter another destination"<<endl;
			cin>>destination;
			if(src==destination)
			{
				cout<<"source and destinations are same";
			}
			else
			{
				cout<<"Path is:"<<endl;
				cout<<src<<"->";
				s.path(destination);
				cout<<"end"<<endl<<endl;
			}
		}
		else
		{
			break;
		}
			
	}	
	return 0;
}
