/*
 * Node.h
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#ifndef NODE_H_
#define NODE_H_
#include "Params.h"
#include <array>
#include <iostream>     // std::cout

template <unsigned int DIM> class Node {
public:
	Node(float val,int depth);
	~Node();
	void									clear();
	Node* 									addChild(int i);					//create a new children at position i with value val
	bool  									childExists(int i);					//checks if child i exists
	Node* 									getChild(int i);					//returns child i - creates it if it does not exists
	double									update(bool rec,bool prune=true);					//update the current node according to its children, if rec=true, descendant are first updated (full update of the subtree)
	void 									setValue(float v){val_=v;}			//set the value val_ of the node to v
	double 									getValue(){return val_;}			//return the value val_ of the node
	bool 									isLeaf(){return isLeaf_;}			//return true if the node is a leaf
	template <unsigned int DIM2> friend std::ostream& 	operator<< (std::ostream& stream, const Node<DIM2>& n);	//print operator
	double 									getDepth(){return depth_;}			//returns the depth of the node in the tree
	void									setSampled(bool a){isSampled_=a;}
	bool									isSampled(){return isSampled_;}


private:
	double 									val_;								//average value of the children
	std::array<Node*,TwoPow<DIM>::value> 	children_;							//children array
	std::array<bool,TwoPow<DIM>::value> 	childExists_;						//existing children indicator
	bool 									isLeaf_;							//whether that node is a leaf of the tree
	int										depth_;								//depth in the tree
	bool									isSampled_;
};

#include "Node.hpp"

#endif /* NODE_H_ */
