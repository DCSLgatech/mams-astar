/*
 * State.h
 *
 *  Created on: Feb 12, 2015
 *      Author: florian
 */

#ifndef STATE_H_
#define STATE_H_

#include "Params.h"
#include <array>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <initializer_list>

template<unsigned int DIM>
class State : public std::array<float, DIM> {
public:
	State(){}
	State(std::initializer_list<float> a){
		auto it=this->begin();
		for(auto f:a)
			(*it++)=f;
	}
	State(float n){std::fill(this->begin(),this->end(),n);}
	State(const State& s){std::copy(s.begin(),s.end(),this->begin());}
	State operator + ( const State& b) const{
		State c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](float a, float b){return a + b;});
		return c;
	}
	State operator - ( const State& b) const{
		State c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](float a, float b){return a - b;});
		return c;
	}
	State operator * ( const float s) const{
		State c;
		std::transform(this->begin(), this->end(), c.begin(),
				[s](float a)->float {return s*a;});
		return c;
	}
	float normSq () const{
		return std::inner_product(this->begin(), this->end(), this->begin(), 0.0f);
	}
	float norm () const{
		return sqrt(normSq());
	}
	State abs () const{
		State s(*this);
		std::for_each(s.begin(),s.end(),[](float &a){a=fabs(a);});
		return s;
	}
	float normInf() const{
		State s=this->abs();
		return *std::max_element(s.begin(),s.end());
	}
	void sort(){
		std::sort(this->begin(),this->end());
	}
	float min(){
		return *std::min_element(this->begin(),this->end());
	}
	float max(){
		return *std::max_element(this->begin(),this->end());
	}
	bool isWithin(const State bound) const{
		State test=*this-bound;
		return std::all_of(test.begin(),test.end(),[](float a){return a<0;});
	}
	//print operator
	friend std::ostream& operator<< (std::ostream& stream, const State& st) {
		stream << "(";
		for(int i=0;i<DIM-1;++i)
					stream << st[i] << ", ";
				stream << st[DIM-1] << ")";
		return stream;
	}
};

#endif /* STATE_H_ */
