#ifndef ANY_VALUE_H
#define ANY_VALUE_H

// polymorphic container class
// adapted from boost::any

#include <typeinfo>

class AnyValue
{
 public:
  // intializers
  AnyValue()
    : content(0)
    {}
  template<typename ValueType>
  AnyValue(const ValueType & value)
    : content(new holder<ValueType>(value))
    {}
  AnyValue(const AnyValue & other)
    : content(other.content ? other.content->clone() : 0)
    {}
  ~AnyValue() { delete content; }

  //operators
   AnyValue & swap(AnyValue & rhs) {
     std::swap(content, rhs.content);
     return *this;
   }
   template<typename ValueType>
   AnyValue & operator=(const ValueType & rhs) {
     AnyValue(rhs).swap(*this);
     return *this;
   }
   AnyValue & operator=(const AnyValue & rhs) {
     AnyValue(rhs).swap(*this);
     return *this;
   }
   bool empty() const { return !content; }
   const std::type_info & type() const {
     return content ? content->type() : typeid(void);
   }

 private:
   struct placeholder
   {
     virtual ~placeholder() { }
     virtual const std::type_info & type() const = 0;
     virtual placeholder * clone() const = 0;
   };

   template<typename ValueType>
   struct holder : public placeholder
   {
     holder(const ValueType & value) : held(value) {}
     virtual const std::type_info & type() const { return typeid(ValueType); }
     virtual placeholder * clone() const { return new holder(held); }

     ValueType held;  
   };

   template<typename ValueType> friend ValueType * AnyCast(AnyValue *);

   placeholder * content;
};

template<typename ValueType>
ValueType * AnyCast(AnyValue * operand)
{
  return operand && operand->type() == typeid(ValueType)
    ? &static_cast<AnyValue::holder<ValueType> *>(operand->content)->held
    : 0;
}

template<typename ValueType>
const ValueType * AnyCast(const AnyValue * operand)
{
  return AnyCast<ValueType>(const_cast<AnyValue *>(operand));
}

#endif
