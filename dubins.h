/*****************
 * 
 * This is the dubins path (c++) 
 * 
 * Author: Yi Wang
 * 
 * Data:10/14/2021
 * 
 * email:yw1055 at unh.cs.edu (Department of CS at University of New Hampshire)
 * 
 * If you want to use this code, please cite my github
*****************/

#ifndef Dubins_H
#define Dubins_H	
#include "BatchInformedTrees.h"	
class Dubins: public Batch_Informed_Tree_Algorithms {
          public:
          Dubins();
          ~Dubins(); 
           struct Dubins_curve{
		         double rho;
		         double relvar;
		         double len;
		         double fst_seg;
		         double sec_seg;
		         double fin_seg;
		         string path_type;
		         bool operator==(const Dubins_curve& dc) {
			        return (len == dc.len && fst_seg == dc.fst_seg && sec_seg == dc.sec_seg 
			             && fin_seg == dc.fin_seg && path_type == dc.path_type);
		         }
	       };
          struct Turing{
		         float x;
		         float y;
		         float theta;
		         bool operator==(const Turing& t) {
			     return (x == t.x && y == t.y && theta == t.theta);
		         }
	       };

	       struct segement{
		          float x;
		          float y;
		          float theta;
	       };
	       struct Dubins_set{
		          float pathLen; 
		          float lenT;
		          float lenP;
		          float lenQ;
		          string pType;
		          bool operator==(const Dubins_set& ds) {
			      return ( lenT == ds.lenT && lenP == ds.lenP && lenQ == ds.lenQ && pType == ds.pType && pathLen == ds.pathLen);
		          }
	       }; 
           struct compare{
		          bool operator()(Dubins_set const & ds, Dubins_set const & db){
			      return ds.pathLen > db.pathLen;
			      }
	       };
           float q_path(states u, states v, string pathType); 
           Dubins_curve Dubins_Optimal_path(states u, states v);
           float p_path(states u, states v, string pathType,float d);
           bool Du_collision_free(Dubins_curve duc, states start, states end);
           float t_path(states u, states v, string pathType, float d, float p);
           float DubinsPathLenght(states u, states v, string path_type, float d);  
           segement seg_generation(float t, char dir, float phi,float x, float y);
           void Generating_dubins_path(Dubins_curve duc, states u, states v,string check);
           float ta_value(float sinAplpha, float sinBeta, float cosAlpha, float cosBeta, string pathType, float d);
           void drawing_path(float length, char c ,float sTha, float sx, float sy, float deltaLen, string checkStyle); 
	       private:
	       float alpha;
	       float beta;
	       float t,p,q,tanValue,rho,angle,infinity;
	       bool ColidHap, collisionDynamic; 
};	
#endif
