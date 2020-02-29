/*
 * Tree.h
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#ifndef TREE_H_
#define TREE_H_

#include "State.h"
#include "Node.h"
#include "Key.h"
#include <array>
#include <forward_list>

template <unsigned int DIM> class Tree {
public:
	Tree();
	~Tree();

	void copyParams(Tree<DIM>* t);
	void clear();
	void addObstacle(Key<DIM>& k);
	void addObstacle(State<DIM>& s);  //create a node a finest resolution around s with value 1
	void updateRec(){root_->update(true);}	//recursively updates every value in the tree
	inline Node<DIM>* getRoot(){return root_;} //returns the root of the tree
	void setStateBounds(const State<DIM>& minState,const State<DIM>& maxState); //set the search space range
	inline void setMaxDepth(int depth); 	//set the maximum depth of the tree
	inline int getMaxDepth(){return maxDepth_;}		//returns the max depth
	inline Key<DIM> getRootKey(){return rootKey_;}	//returns the spatial position of the center of the search space
	inline std::array<Key<DIM>,TwoPow<DIM>::value>* getDirections(){return &directions_;}		//returns the array of directions to find children
	Node<DIM>* getNode(const Key<DIM>& k);		//find the node corresponding to key k
	int getVolume(int depth);	//returns the volume of a node at a given depth (volume at maxDepth_ is 1)
	int getSize(int depth);		//returns the size at a given depth
	int getSize(Key<DIM> k);		//returns the size of a node given tis key k
	bool getKey(const State<DIM>& s,Key<DIM>& k,bool inTree=false);		//return false for failure, or update k and return true
	bool getKey(const Key<DIM>& kt,Key<DIM>& k, bool inTree=false);		//update k to the closest match of kt
	State<DIM> getState(const Key<DIM>& k);		//return false for failure, or update k and return true
	std::forward_list<Key<DIM>> getRayKeys(const Key<DIM>& k1,const Key<DIM>& k2);		//calculate the keys of every node traversed by the ray goin from k1 to k2
	int getKeySize(const Key<DIM>& k);
	//modified Jaein :20190305 return direction in which key points to
	Key<DIM> getNextKey(Key<DIM> key_direction);
	void copyWaveletTransform(std::string filename);

private:
	Node<DIM>* root_;		//root of the tree
	int maxDepth_;				//max depth of the tree
	Key<DIM> rootKey_;				//key corresponding to the center of the search space
	State<DIM> stateMin_;		//"bottom left corner" of the search space
	State<DIM> stateInc_;	 //state increment to reach the "top right corner" of the search space from stateMin
	std::array<Key<DIM>,TwoPow<DIM>::value> directions_;		//vectors towards children (defines the children order)
	int unitKeyHash(const Key<DIM> k);		//for a key with no 0s, it returns the index of the corresponding direction
	//Jaein's wavelet copy : 20190421
	void recursiveWavelet(Node<DIM>* node, int pt, int sz, float val, int N);
	std::vector<float> wavelet_coefficients_;
};

#include "Tree.hpp"

#endif /* TREE_H_ */
