#include "awbitStar.h"

AWBITStar::AWBITStar() {
	//INITIALIZE private elements
	w = 0;
	source =0;
	goal = 1;
	path_find = false;
	found = false;
    start_x = 0;
    start_y = 0;
    end_x   = 0;
    end_y   = 0;   
    map_x   = 0;
    map_y   = 0;
    numNodes = 0;
    speed   = 1.1;
    kneighbors = 0;
    Epsilon = 0.0000001;
    infinity = numeric_limits<float>:: infinity();
}

AWBITStar::~AWBITStar(){
};

states AWBITStar::Random_state(int x, int y) {
   states sta;
   sta.x = ((float) x)*rand()/(RAND_MAX);
   sta.y = ((float) y)*rand()/(RAND_MAX);
   sta.theta = ((float ) 2*M_PI)*rand()/(RAND_MAX);
   return sta;
}

states AWBITStar::state(int v) {
   if(rggData.find(v) != rggData.end())
   return rggData[v];
   return vertices[v];	
}
void AWBITStar::sampling(int num, int count) {	
	while(count < num) {
		if(rggData.find(count) == rggData.end()) {
			states tmp = Random_state(map_x,map_y);
			int x = tmp.x, y =tmp.y; 
			if(!block[x<<16|y]) {
		       rggData[count] = tmp;
		       //cerr << count <<" [ " <<tmp.x << "," << tmp.y << "]" << endl;
		       count++;
		   }
	    }
	 
	}
}
float AWBITStar::Heuristic_value(int v) 
{ return sqrt((state(v).x- end_x)*(state(v).x- end_x)+ (state(v).y- end_y)*(state(v).y- end_y)); }


float AWBITStar::estimatedComeValue(int v) 
{ return sqrt((start_x- state(v).x)*(start_x- state(v).x) + (start_y-state(v).y)*(start_y-state(v).y)); }

float AWBITStar::stateEucDistance(int u, int v)
{ return sqrt((state(u).x- state(v).x)*(state(u).x- state(v).x) + (state(u).y-state(v).y)*(state(u).y-state(v).y)); }

float AWBITStar::stateDistance(states u, states v)
{ return sqrt((u.x- v.x)*(u.x-v.x) + (u.y-v.y)*(u.y-v.y)); }

void AWBITStar::prune_rgg(float costGoal) {
	for(auto it = rggData.begin(); it != rggData.end();) {
		float f = estimatedComeValue(it->first) + Heuristic_value(it->first);
		if(f >= costGoal)
		   it = rggData.erase(it);
		else
		   it++;
	}
}

void AWBITStar::prune_vertices(float costGoal) {
	for(auto it = vertices.begin(); it != vertices.end();) {
		float f = estimatedComeValue(it->first) + Heuristic_value(it->first);
		if(f > costGoal)
		   it = vertices.erase(it);
		else
		   it++;
	}
}
void AWBITStar::edges_prune(float costGoal){	
	for(auto it = Edge.begin(); it != Edge.end();){
	    float fv = estimatedComeValue(it->first.first) + Heuristic_value(it->first.first);
	    float fw = estimatedComeValue(it->first.second) + Heuristic_value(it->first.second);
	    if(fv > costGoal || fw > costGoal)
	       it = Edge.erase(it);
	    else
	       it++;
	}
}
void BIT_star::add_vertices_back_to_sample(){
	for(auto it = vertices.begin(); it != vertices.end(); it++){
	    if(!visited[it->first]) {
			cost[it->first] = infinity;
			rggData[it->first] = vertices[it->first];
		}
	}
}
void BIT_star::erase_vertices_infinity(){
	for(auto it = vertices.begin(); it != vertices.end();){
	    if(!visited[it->first])
		       it = vertices.erase(it);
		else{ 
		    it++;}	    
	}
}
void BIT_star::prune(float costGoal){
     prune_rgg(costGoal);
     edges_prune(costGoal);
     prune_vertices(costGoal);
     add_vertices_back_to_sample();
     erase_vertices_infinity();
}
void BIT_star::creating_vertice_queue() {
	for(auto it = vertices.begin(); it != vertices.end();it++) {
		float var = cost[it->first] + Heuristic_value(it->first);
		BestQVvalue.push(make_pair(var,it->first));
	}
}
void BIT_star::building_edge_queue() {
	for(auto it = Edge.begin(); it != Edge.end(); it++) {
		float dist = cost[it->first.first] + stateEucDistance(it->first.first,it->first.second) + Heuristic_value(it->first.second);
		BestEvalue.push(make_pair(dist,it->first));
	}
}

vector<int> BIT_star::GetKneighbors(int v, unordered_map<int,states> nodeContainer,string str) {
	vector<int > knbors;
	priority_queue< pair<float,int>, vector<pair<float,int> >,greater<pair<float,int>> > q;
	for(auto it = nodeContainer.begin(); it != nodeContainer.end(); it++) {
		float dist = stateDistance(state(v), nodeContainer[it->first]);
		if(dist > Epsilon && it->first !=v && vertices.find(it->first) == vertices.end() && str == "sample")
		   q.push(make_pair(dist,it->first));
		else {
			if(dist > Epsilon && str == "edge") {
				q.push(make_pair(dist,it->first));
			}
		}
	}
	int index =0;
	while(!q.empty()) {		
		if(index == kneighbors)
		   break;
		knbors.push_back(q.top().second);
		index++;
		q.pop();
	}
	return knbors;
}
void BIT_star::creating_edges_queue(int v, float costGoal ,string str, vector<int > kn) {
	float ghat = estimatedComeValue(v);		
	states vv = state(v);
	for(int i =0; i < kn.size(); i++) {
		states ww = state(kn[i]);
		float sdist = stateDistance(vv,ww);
		float hhat = Heuristic_value(kn[i]);
		float dist = ghat + sdist + hhat;
		float distt = cost[v] + sdist;	
		float fdist = cost[v] + sdist + w*hhat;		
		if(dist < costGoal && str == "sample")
		BestEvalue.push(make_pair(fdist,make_pair(v,kn[i])));
		else{
			if((Edge.find(make_pair(v,kn[i])) == Edge.end()) && (dist < costGoal) && (distt < cost[kn[i]])) {
				BestEvalue.push(make_pair(fdist,make_pair(v,kn[i])));
			}
		}
	}
}
void BIT_star::ExpandVertex(int v, float costGoal){
	visited[v] = true;
	string name = "sample";
	BestQVvalue.pop();
	vector<int > kn;
	kn = GetKneighbors(v,rggData,name);
	//cerr << "testing k nearest neighbors " << kn.size() << endl;
	creating_edges_queue(v, costGoal, name, kn);
	kn.clear();
	if(oldvertices.find(v) == oldvertices.end()){
		name = "edge";
		kn = GetKneighbors(v,vertices,name);
		creating_edges_queue(v,costGoal,name,kn);
	}
}
void BIT_star::Erase_edge(int xm){
	for(auto it = Edge.begin(); it != Edge.end();) {
		if(it->first.second == xm)
		   it = Edge.erase(it);
		else
		   it++;
	}
}
void BIT_star::Erase_queue_Edge(int xm) {
	priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> > temp;
	temp = BestEvalue;
	BestEvalue = priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> >();
	while(!temp.empty()) {
		if(temp.top().second.second == xm) {
			float dist = cost[temp.top().second.first] + stateEucDistance(temp.top().second.first,xm);
			if(dist < cost[xm]) {
			   BestEvalue.push(temp.top());	
			}
		}
		else {
			 BestEvalue.push(temp.top());	
		}
		
		temp.pop();
	} 
}
void BIT_star::Creating_The_Trajectory(int xm) {
	stack<int > path;
    int cp = xm;
    while(parent.find(cp) != parent.end()) {
		path.push(cp);
		cp = parent[cp];
	}
	cerr << "testing here "  << endl;
	while(!path.empty()) {
		int id = path.top();
		Dubins dubins;
	    Dubins_curve path_get;
	    states st = state(id);
	    cerr << st.x << " " << st.y << endl;
	    states sp = state(parent[id]);
	    //path_get = Edge[make_pair(parent[id],id)];
		//dubins.Generating_dubins_path(path_get,sp,st,"pathG");
		path.pop();
	}
}


void AWBITStar::Initialize_RGG() {
	states start, goal;
	start.x = start_x,start.y = start_y;
	start.theta = ((float ) 2*M_PI)*rand()/(RAND_MAX);
	goal.x = end_x,goal.y = end_y;
	goal.theta = ((float ) 2*M_PI)*rand()/(RAND_MAX);
	rggData[0] = start;
	rggData[1] = goal;
	vertices[0] = start;
	visited[0] = true;
	cost[0] = 0;
}

AWBITStar::AwBitStar(){
      Initialize_RGG();
      int num =0, count =0;
      bool initial = true;
      int batch = 1;
      while(true) {
	     if(BestEvalue.empty()){
			 w = 3;
		    if(initial) {
				count = 2;
				cost[1] = infinity;
			} else {
				count = num;
				numNodes = 100;
			} 
			num += numNodes;
			//cerr << vertices.size() << cost[1] << endl;
			prune(cost[1]);
			//cerr << vertices.size() << cost[1]<<endl;
			sampling(num,count);
			oldvertices = vertices;
			kneighbors = 15;//2*exp(1)*log(cardv), int cardv = rggData.size() + vertices.size();
		 }
		pairs edge = BestEvalue.top().second;
		      BestEvalue.pop();
		int s = eg.first;
		int t = eg.second;
		float esEdgeVar = stateEucDistance(vm,xm);
		float esHvar = Heuristic_value(xm);
		float curC = cost[vm]; 
		float dist = curC + esEdgeVar + esHvar;
		visited[vm] = true;      
		if(dis < cost[1]) {
		   Dubins_curve path_get = Dubins_Optimal_path(state(vm),state(xm));
		   if(collision_free(path_get,state(vm),state(xm),"c")){
			    float gCur = curC+path_get.relvar;
			    if(gCur+esHvar < cost[1]) {
					if(cost.find(xm) == cost.end()){
					   cost[xm] = infinity;
					}
				    if(gCur < cost[xm]) {
					   cost[xm] = gCur;
					   if(xm == 1) {
						   cerr << "batch " << batch << ", "<< gCur << endl;
						   parent[xm] = vm;
						   initial = false;
						  //if(batch == 95)
						  //Creating_The_Trajectory(xm);
						    //exit(-1);
						    w += 0.5;
						    if(w < 1)
						       w = 1;
						    continue;
					     
					   }
					   if(vertices.find(xm) != vertices.end()) {
						   Erase_edge(xm);
					   }
					   else {
						   vertices[xm] = rggData[xm]; 
						   rggData.erase(xm);
						   BestQVvalue.push(make_pair(cost[xm],xm));
						   visited[xm] = false;
					  }
				      parent[xm] = vm;
					  Edge[make_pair(vm,xm)] = path_get;
					  Erase_queue_Edge(xm);
				  }
				} 
		  }
		} else {
			   BestEvalue = priority_queue< pair<float,pairs>, vector<pair<float,pairs> >,greater<pair<float,pairs>> >();
			   batch++;
			   //cerr << batch << endl;
			   if(batch == 20) {
			     exit(-1);
			   }
	    }    
	 } 
}
