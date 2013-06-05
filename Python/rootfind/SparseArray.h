#ifndef SPARSE_ARRAY_H
#define SPARSE_ARRAY_H

#include <map>
#include <assert.h>

/** @brief A sparse 1D array class.
 *
 * Acts just like a map of integers to values.
 */
template <class T>
class SparseArray
{
public:
  typedef std::map<int,T> Storage;
  typedef typename Storage::iterator iterator;
  typedef typename Storage::const_iterator const_iterator;

  SparseArray() :n(0) {}
  SparseArray(size_t _n) :n(_n) {}
  inline void resize(size_t _n) { n=_n; assert(isValid()); }
  inline void clear() { entries.clear(); n=0; }
  inline bool empty() const { return n==0; }
  inline size_t size() const { return n; }
  inline size_t numEntries() const { return entries.size(); }
  inline iterator begin() { return entries.begin(); }
  inline iterator end() { return entries.end(); }
  inline const_iterator begin() const { return entries.begin(); }
  inline const_iterator end() const { return entries.end(); }
  inline iterator insert(int i,const T& t) {
    std::pair<int,T> p;
    p.first=i;
    iterator res=entries.insert(p).first;
    res->second=t;
    return res;
  }
  inline iterator push_back(int i,const T& t) {
    std::pair<int,T> p;
    p.first=i;
    iterator res=entries.insert(end(),p);
    res->second=t;
    return res;
  }
  inline iterator find(int i) { return entries.find(i); }
  inline const_iterator find(int i) const { return entries.find(i); }
  inline bool erase(int i) { return entries.erase(i)!=0; }
  inline void erase(const iterator& it) { entries.erase(it); }
  inline bool isValidIndex(int i) const { return i>=0&&i<(int)n; }
  inline bool isValid() const 
  {
    for(const_iterator i=entries.begin();i!=entries.end();i++)
      if(i->first < 0 || i->first >= (int)n) return false;
    return true;
  }

  Storage entries;
  size_t n;
};


#endif
