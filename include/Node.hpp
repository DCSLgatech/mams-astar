#include <algorithm>
#include <iomanip>

template <unsigned int DIM> Node<DIM>::Node(float val,int depth):val_(val),isLeaf_(true),depth_(depth) {
	childExists_.fill(false);
}

template <unsigned int DIM> Node<DIM>::~Node() {
	this->clear();
}

template <unsigned int DIM> void Node<DIM>::clear(){
	for(int i=0;i<TwoPow<DIM>::value;++i)
		if(childExists_[i]){
			delete children_[i];
			childExists_[i]=false;
		}
}

template <unsigned int DIM> Node<DIM>* Node<DIM>::addChild(int i){
	isLeaf_=false;
	childExists_[i]=true;
	children_[i]=new Node<DIM>(0.0f,depth_+1);
	return children_[i];
}

template <unsigned int DIM> bool  Node<DIM>::childExists(int i){
	return childExists_[i];
}

template <unsigned int DIM> double Node<DIM>::update(bool rec,bool prune){
	if(isLeaf()){
		return val_;
	}
	//pruning variables
	bool prunable=prune;
	double prevVal=-1;
	//update
	double sum=0.0;
	for(int i=0;i<TwoPow<DIM>::value;++i){
		if(childExists_[i]){
			if(rec){
				children_[i]->update(rec,prune);
			}
			sum+=children_[i]->getValue();
			//check if pruning possible
			if(prunable){
				if(prevVal==-1){
					prevVal=children_[i]->getValue();
				}else{
					if(prevVal != children_[i]->getValue() || !children_[i]->isLeaf()){
						prunable=false;
					}
				}
			}
		}else{
			prunable=false;
		}
	}
	val_=sum/TwoPow<DIM>::value;
	if(prunable){
		//prune
		for(int i=0;i<TwoPow<DIM>::value;++i)
			if(childExists_[i])
				delete children_[i];
		isLeaf_=true;
		childExists_.fill(false);
	}
	return val_;
}

template <unsigned int DIM> Node<DIM>* Node<DIM>::getChild(int i){
	if(childExists_[i]){
		return children_[i];
	}else{
		return addChild(i);
	}
}

//print operator
template <unsigned int DIM> std::ostream& operator<< (std::ostream& stream, const Node<DIM>& n) {
	std::streamsize width=stream.width();
	stream << std::setw(width) << n.val_ << std::endl;
	for(int i=0;i<TwoPow<DIM>::value;++i){
		if(n.childExists_[i]){
			stream <<std::setw(3)<< i << ", "<<std::setw(width+8)  <<*(n.children_[i]);
		}
	}
	stream.width(width);
	return stream;
}
