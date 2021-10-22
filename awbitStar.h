/*************************************************
 * 
 * This is the project for planning of Robot
 * 
 * This header file for the new version of anytime weighted BIT*
 * 
 * Author: Yi Wang
 * 
 * Data:10/22/2021
 * 
 * **********************************************/
#ifndef AW_BIT_star_H
#define AW_BIT_star_H
#include "dubins.h"
#include "BatchInformedTrees.h"
typedef pair<int,int> pairs;
	struct hash_pair { 
        template <class T1, class T2> 
        size_t operator()(const pair<T1, T2>& p) const
        { 
           auto hash1 = hash<T1>{}(p.first); 
           auto hash2 = hash<T2>{}(p.second); 
           return hash1 ^ hash2; 
        } 
    };
class AWBITStar: public Dubins {
	public: // functions we will use
	AWBITStar();               // constructs empty Map
    ~AWBITStar();
	struct edges{
		int u, v;
		float ux,uy,vx,vy;
	};
 // prune the node which its f vlaue greatert than costGoal
	void AwBitStar();
	void loading_map();
	states state(int v);
	void Initialize_RGG();	
	void Erase_edge(int xm); 
	void building_edge_queue();   
	void prune(float costGoal);	
	float Heuristic_value(int v);
	void Erase_queue_Edge(int xm);
	void creating_vertice_queue();
	void lazy_Astar(BIT_star *bit);
	void erase_vertices_infinity();	
	void prune_rgg(float costGoal);
	float estimatedComeValue(int v);
	void edges_prune(float costGoal);
    void Extract_Trajectory(int path); 
	void sampling(int num, int count);
	bool outOfbound(float x, float y);	
	void add_vertices_back_to_sample();  
	states Random_state(int xs, int ys);
	void prune_vertices(float costGoal);
	float stateEucDistance(int u, int v);   
	float stateDistance(states u, states v);
    void ExpandVertex(int v,float costGoal); 
    float stateEucDistance(states v, states w);
    void record_path(BIT_star *bit, int m, string pt);   
    void creating_edges_queue(int v, float costGoal, string name,vector<int > kn);
    bool collision_free(Dubins::Dubins_curve path_get, states u, states v, string c);
	vector<int > GetKneighbors(int v, unordered_map<int,states> nodeContainer, string str);
    void outputQ(priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > QV);
	bool path_location(float length, char c ,float sTha, float sx, float sy, float deltalen, string Cstyle);
    void outputE(priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> > QE);
	private: // variables we will use
    bool path_find, found, collisionHappen;
    int source,goal,map_x,map_y,numNodes,range_x,range_y,kneighbors;
    float start_x,start_y,end_x,end_y,infinity,Epsilon,speed,mheading, time_changed,w;
    clock_t sT,eT;
    unordered_map<int,bool> ns;
    unordered_map<int,int> parent; // checking the parent
    unordered_map<int,float> cost;// true cost to come   
    unordered_map<int,states> rggData;     
    unordered_map<int, bool > visited; // state visited 
    unordered_map<int,states> vertices; // cost for every state
    unordered_map<int,states> oldvertices;
    int * block;
    unordered_map<int,pair<float,float>> nodes; 
    unordered_map<int,unordered_map<int,float>> adj;  
	unordered_set<pair<int,int>, hash_pair> obstacles; 
	priority_queue< float, vector<float >,greater< float> > bestCost;
    unordered_map<pair<int, int>,Dubins::Dubins_curve, hash_pair> Edge;     
    priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> > BestEvalue; //Motion trees
};
#endif
