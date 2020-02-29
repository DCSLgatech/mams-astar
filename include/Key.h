/*
 * Key.h
 *
 *  Created on: Feb 12, 2015
 *      Author: florian
 */

#ifndef KEY_H_
#define KEY_H_

#include "Params.h"
#include <array>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <initializer_list>
#include <cstdlib>

template<unsigned int DIM>
class Key : public std::array<int, DIM> {
public:
	Key(){}
	//modifed Jaein: added initializer
	Key(std::initializer_list<int> a){
		auto it=this->begin();
		for(auto f:a)
			(*it++)=f;
	}
	Key(const std::array<int, DIM>& a){ std::copy( a.begin(), a.end(), this->begin());}
	Key(const std::vector<long int> & v){ std::copy( v.begin(), v.end(), this->begin());}

	Key(int n){std::fill(this->begin(),this->end(),n);}
	Key(const Key& s){std::copy(s.begin(),s.end(),this->begin());}
	Key operator + ( const Key& b) const{
		Key c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](int a, int b){return a + b;});
		return c;
	}
	Key operator - ( const Key& b) const{
		Key c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](int a, int b){return a - b;});
		return c;
	}
	Key operator << (const int s) const{
		Key c;
		std::transform(this->begin(), this->end(), c.begin(),
				[s](int a){return a<<s;});
		return c;
	}
	Key operator * (const int s) const{
		Key c;
		std::transform(this->begin(), this->end(), c.begin(),
				[s](int a){return a*s;});
		return c;
	}
	float normSq () const{
		return std::inner_product(this->begin(), this->end(), this->begin(), 0);
	}
	float norm () const{
		return sqrt(normSq());
	}
	float normInf() const {
		Key c = abs();
		return *std::max_element(c.begin(), c.end());
	}

	Key abs () const{
		Key c;
		std::transform(this->begin(), this->end(), c.begin(),
				[](int a){return std::abs(a);});
		return c;
	}
	std::vector<long int> vectorize() const{
		std::vector<long int> vec_key (DIM);
		std::copy(this->begin(), this->end(), vec_key.begin());
		return vec_key;
	}

	//print operator
	friend std::ostream& operator<< (std::ostream& stream, const Key& st) {
		stream << " ";
		for(int i=0;i<DIM-1;++i)
					stream << st[i] << " ";
				stream << st[DIM-1] << " ";
		return stream;
	}

};

#endif /* KEY_H_ */
