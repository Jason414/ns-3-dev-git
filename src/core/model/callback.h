/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#ifndef CALLBACK_H
#define CALLBACK_H

#include "ptr.h"
#include "fatal-error.h"
#include "empty.h"
#include "attribute.h"
#include "attribute-helper.h"
#include "simple-ref-count.h"
#include <typeinfo>
#include <vector>
#include <functional>
#include <memory>
#include <utility>

/**
 * \file
 * \ingroup callback
 * Declaration of the various callback functions.
 */

namespace ns3 {

// Define the doxygen subgroups all at once,
// since the implementations are interleaved.

/**
 * \ingroup core
 * \defgroup callback Callbacks
 * \brief Wrap functions, objects, and arguments into self contained callbacks.
 *
 * Wrapped callbacks are at the heart of scheduling events in the
 * simulator.
 */
/**
 * \ingroup callback
 * \defgroup callbackimpl Callback Implementation
 * Callback implementation classes
 */
/**
 * \ingroup callback
 * \defgroup makecallbackmemptr MakeCallback from member function pointer
 *
 * Build Callbacks for class method members which take varying numbers
 * of arguments and potentially returning a value.
 *
 * Generally the \c MakeCallback functions are invoked with the
 * method function address first, followed by the \c this pointer:
 * \code
 *   MakeCallback ( & MyClass::Handler, this);
 * \endcode
 *
 * The version with bound arguments is represented by the
 * \c MakeBoundCallback functions:
 * \code
 *   MakeBoundCallback ( & MyClass::Handler, this, barg);
 * \endcode
 */
/**
 * \ingroup callback
 * \defgroup makecallbackfnptr MakeCallback from function pointers
 *
 * Build Callbacks for functions which take varying numbers of arguments
 * and potentially returning a value.
 */
/**
 * \ingroup callback
 * \defgroup makenullcallback MakeCallback with no arguments
 *
 * Define empty (Null) callbacks as placeholders for unset callback variables.
 */
/**
 * \ingroup callback
 * \defgroup makeboundcallback MakeBoundCallback from functions bound with up to three arguments.
 *
 * Build bound Callbacks which take varying numbers of arguments,
 * and potentially returning a value.
 *
 * \internal
 *
 * The following is experimental code. It works but we have
 * not yet determined whether or not it is really useful and whether
 * or not we really want to use it.
 */

  
/**
 * \ingroup callbackimpl
 * Abstract base class for CallbackImpl
 * Provides reference counting and equality test.
 */
class CallbackImplBase : public SimpleRefCount<CallbackImplBase>
{
public:
  /** Virtual destructor */
  virtual ~CallbackImplBase () {}
  /**
   * Equality test
   *
   * \param [in] other Callback Ptr
   * \return \c true if we are equal
   */
  virtual bool IsEqual (Ptr<const CallbackImplBase> other) const = 0;
  /**
   * Get the name of this object type.
   * \return The object type as a string.
   */
  virtual std::string GetTypeid (void) const = 0;

protected:
  /**
   * \param [in] mangled The mangled string
   * \return The demangled form of mangled
   */
  static std::string Demangle (const std::string& mangled);
  /**
   * Helper to get the C++ typeid as a string.
   *
   * \tparam T The type of the argument.
   * \returns The result of applying typeid to the template type \p T.
   */
  template <typename T>
  static std::string GetCppTypeid (void)
  {
    std::string typeName;
    try
      {
        typeName = typeid (T).name ();
        typeName = Demangle (typeName);
      }
    catch (const std::bad_typeid &e)
      {
        typeName = e.what ();
      }
    return typeName;
  }
};


/**
 * \ingroup callbackimpl
 * Abstract base class for CallbackComponent.
 * Provides equality test.
 */
class CallbackComponentBase
{
public:
  /** Virtual destructor */
  virtual ~CallbackComponentBase () {}
  /**
   * Equality test
   *
   * \param [in] other CallbackComponent Ptr
   * \return \c true if we are equal
   */
  virtual bool IsEqual (std::shared_ptr<const CallbackComponentBase> other) const = 0;
};

/**
 * \ingroup callbackimpl
 * Stores a component of a callback, i.e., the callable object
 * or a bound argument. The purpose of this class is to test
 * the equality of the components of two callbacks.
 */
template <typename T, bool isComparable = true>
class CallbackComponent : public CallbackComponentBase
{
public:
  /**
   * Constructor
   *
   * \param [in] t The value of the callback component
   */
  CallbackComponent (const T& t)
    : m_comp (t) {}

  /**
   * Equality test between the values of two components
   *
   * \param [in] other CallbackComponentBase Ptr
   * \return \c true if we are equal
   */
  bool IsEqual (std::shared_ptr<const CallbackComponentBase> other) const
  {
    auto p = std::dynamic_pointer_cast<const CallbackComponent<T>> (other);

    // other must have the same type and value as ours
    if (p == nullptr || p->m_comp != m_comp)
      {
        return false;
      }

    return true;
  }

private:
    T m_comp;             //!< the value of the callback component
};


/**
 * \ingroup callbackimpl
 * Partial specialization of class CallbackComponent with isComparable equal
 * to false. This is required to handle callable objects (such as lambdas and
 * objects returned by std::function and std::bind) that do not provide the
 * equality operator. Given that these objects cannot be compared and the only
 * purpose of the class CallbackComponent is to compare values, no object is
 * stored in this specialized class.
 */
template <typename T>
class CallbackComponent<T, false> : public CallbackComponentBase
{
public:
  /**
   * Constructor
   *
   * \param [in] t The value of the callback component
   */
  CallbackComponent (const T& t) {}

  /**
   * Equality test between functions
   *
   * \param [in] other CallbackParam Ptr
   * \return \c true if we are equal
   */
  bool IsEqual (std::shared_ptr<const CallbackComponentBase> other) const
  {
    return false;
  }
};

/// Vector of callback components
typedef std::vector<std::shared_ptr<CallbackComponentBase>> CallbackComponentVector;

/**
 * \ingroup callbackimpl
 * CallbackImpl classes with varying numbers of argument types
 *
 * @{
 */
/** CallbackImpl class. */
template <typename R, typename... UArgs>
class CallbackImpl : public CallbackImplBase {
public:

  CallbackImpl (std::function<R(UArgs...)> func,
                const CallbackComponentVector& components)
    : m_func (func), m_components (components)
  {}

  virtual ~CallbackImpl () {}

  /**
   * Get the stored function.
   * \return A const reference to the stored function.
   */
  const std::function<R(UArgs...)>& GetFunction (void) const
  { return m_func; }

  /**
   * Get the vector of callback components.
   * \return A const reference to the vector of callback components.
   */
  const CallbackComponentVector& GetComponents (void) const
  { return m_components; }

  R operator() (UArgs... uargs) const {
    return m_func (uargs...);
  }

  virtual bool IsEqual (Ptr<const CallbackImplBase> other) const
  {
    CallbackImpl<R,UArgs...> const *otherDerived =
      dynamic_cast<CallbackImpl<R,UArgs...> const *> (PeekPointer (other));

    if (otherDerived == 0)
      {
        return false;
      }

    // if the two callback implementations are made of a distinct number of
    // components, they are different
    if (m_components.size () != otherDerived->GetComponents ().size ())
      {
        return false;
      }

    // the two functions are equal if they compare equal or the shared pointers
    // point to the same locations
    if (!m_components.at (0)->IsEqual (otherDerived->GetComponents ().at (0))
        && m_components.at (0) != otherDerived->GetComponents ().at (0))
    {
      return false;
    }

    // check if the remaining components are equal one by one
    for (std::size_t i = 1; i < m_components.size (); i++)
      {
        if (!m_components.at (i)->IsEqual (otherDerived->GetComponents ().at (i)))
          {
            return false;
          }
      }

    return true;
  }

  virtual std::string GetTypeid (void) const
  {
    return DoGetTypeid ();
  }
  /** \copydoc GetTypeid(). */
  static std::string DoGetTypeid (void)
  {
    static std::vector<std::string> vec = { GetCppTypeid<R> (), GetCppTypeid<UArgs> ()... };

    static std::string id ("CallbackImpl<");
    for (auto& s : vec)
      {
        id.append (s + ",");
      }
    if (id.back () == ',')
      {
        id.pop_back ();
      }
    id.push_back ('>');

    return id;
  }

private:
  // Stores the callable object associated with this callback (as a lambda)
  std::function<R(UArgs...)> m_func;

  // Stores the original callable object and the bound arguments, if any
  std::vector<std::shared_ptr<CallbackComponentBase>> m_components;
};
/**@}*/


/**
 * \ingroup callbackimpl
 * Base class for Callback class.
 * Provides pimpl abstraction.
 */
class CallbackBase {
public:
  CallbackBase () : m_impl () {}
  /** \return The impl pointer */
  Ptr<CallbackImplBase> GetImpl (void) const { return m_impl; }
protected:
  /**
   * Construct from a pimpl
   * \param [in] impl The CallbackImplBase Ptr
   */
  CallbackBase (Ptr<CallbackImplBase> impl) : m_impl (impl) {}
  Ptr<CallbackImplBase> m_impl;         //!< the pimpl
};

/**
 * \ingroup callback
 * \brief Callback template class
 *
 * This class template implements the Functor Design Pattern.
 * It is used to declare the type of a Callback:
 *  - the first non-optional template argument represents
 *    the return type of the callback.
 *  - the remaining (optional) template arguments represent
 *    the type of the subsequent arguments to the callback.
 *  - up to nine arguments are supported.
 *
 * Callback instances are built with the \ref MakeCallback
 * template functions. Callback instances have POD semantics:
 * the memory they allocate is managed automatically, without
 * user intervention which allows you to pass around Callback
 * instances by value.
 *
 * Sample code which shows how to use this class template 
 * as well as the function templates \ref MakeCallback :
 * \include src/core/examples/main-callback.cc
 *
 * \internal
 * This code was originally written based on the techniques 
 * described in http://www.codeproject.com/cpp/TTLFunction.asp
 * It was subsequently rewritten to follow the architecture
 * outlined in "Modern C++ Design" by Andrei Alexandrescu in 
 * chapter 5, "Generalized Functors".
 *
 * This code uses:
 *   - default template parameters to saves users from having to
 *     specify empty parameters when the number of parameters
 *     is smaller than the maximum supported number
 *   - the pimpl idiom: the Callback class is passed around by 
 *     value and delegates the crux of the work to its pimpl
 *     pointer.
 *   - two pimpl implementations which derive from CallbackImpl
 *     FunctorCallbackImpl can be used with any functor-type
 *     while MemPtrCallbackImpl can be used with pointers to
 *     member functions.
 *   - a reference list implementation to implement the Callback's
 *     value semantics.
 *
 * This code most notably departs from the alexandrescu 
 * implementation in that it does not use type lists to specify
 * and pass around the types of the callback arguments.
 * Of course, it also does not use copy-destruction semantics
 * and relies on a reference list rather than autoPtr to hold
 * the pointer.
 *
 * \see attribute_Callback
 */
template<typename R, typename... UArgs>
class Callback : public CallbackBase {
public:
  Callback () {}

  /**
   * Construct from a CallbackImpl pointer
   *
   * \param [in] impl The CallbackImpl Ptr
   */
  Callback (Ptr<CallbackImpl<R,UArgs...>> const &impl)
    : CallbackBase (impl)
  {}

  /**
   * Construct from another callback and bind some arguments (if any)
   *
   * \param [in] cb The existing callback
   * \param [in] bargs The values of the bound arguments
   */
  template <typename... BArgs>
  Callback (const CallbackBase& cb, BArgs... bargs)
  {
    auto cbDerived = static_cast<CallbackImpl<R,BArgs...,UArgs...> const *> (PeekPointer (cb.GetImpl ()));

    std::function<R(BArgs...,UArgs...)> f (cbDerived->GetFunction ());

    CallbackComponentVector components (cbDerived->GetComponents ());
    components.insert (components.end (), { std::make_shared<CallbackComponent<BArgs>> (bargs)... });

    m_impl = Create<CallbackImpl<R,UArgs...>> ([f,bargs...](UArgs... uargs) -> R
                                                 { return f (bargs..., uargs...); },
                                               components);
  }

  /**
   * Construct from a function and bind some arguments (if any)
   *
   * \param [in] func The function
   * \param [in] bargs The values of the bound arguments
   *
   * \internal
   * We leverage SFINAE to have the compiler discard this constructor when the type
   * of the first argument is a class derived from CallbackBase (i.e., a Callback).
   */
  template <typename T,
            std::enable_if_t<!std::is_base_of<CallbackBase,T>::value,int> = 0,
            typename... BArgs>
  Callback (T func, BArgs... bargs)
  {
    // store the function in a std::function object
    std::function<R(BArgs...,UArgs...)> f (func);

    // The original function is comparable if it is a function pointer or
    // a pointer to a member function or a pointer to a member data.
    constexpr bool isComp = std::is_function<std::remove_pointer_t<T>>::value
                            || std::is_member_pointer<T>::value;

    CallbackComponentVector components ({ std::make_shared<CallbackComponent<T,isComp>> (func),
                                          std::make_shared<CallbackComponent<BArgs>> (bargs)... });

    m_impl = Create<CallbackImpl<R,UArgs...>> ([f,bargs...](UArgs... uargs) -> R
                                                 { return f (bargs..., uargs...); },
                                               components);
  }

private:
  /**
   * Implementation of the Bind method
   *
   * \param [in] seq A compile-time integer sequence
   * \param [in] bargs The values of the arguments to bind
   * \return The bound callback
   *
   * \internal
   * The integer sequence is 0..N-1, where N is the number of arguments left unbound.
   * C++14 enables automatic return type deduction, which would make this code
   * more readable.
   */
  template <std::size_t... I, typename... BoundArgs>
  auto BindImpl (std::index_sequence<I...> seq, BoundArgs... bargs)
  {
    return Callback<R,typename std::tuple_element<sizeof...(bargs)+I,std::tuple<UArgs...>>::type...>
                   (*this, bargs...);
  }

public:

  /**
   * Bind a variable number of arguments
   *
   * \param [in] bargs The values of the arguments to bind
   * \return The bound callback
   */
  template <typename... BoundArgs>
  auto Bind (BoundArgs... bargs)
  {
    return BindImpl (std::make_index_sequence<sizeof...(UArgs) - sizeof...(BoundArgs)>{},
                     std::forward<BoundArgs> (bargs)...);
  }

  /**
   * Check for null implementation
   *
   * \return \c true if I don't have an implementation
   */
  bool IsNull (void) const {
    return (DoPeekImpl () == 0) ? true : false;
  }
  /** Discard the implementation, set it to null */
  void Nullify (void) {
    m_impl = 0;
  }

  /**
   * Functor with varying numbers of arguments
   * @{
   */
  /** \return Callback value */
  R operator() (UArgs... uargs) const {
    return (*(DoPeekImpl ()))(uargs...);
  }
  /**@}*/

  /**
   * Equality test.
   *
   * \param [in] other Callback
   * \return \c true if we are equal
   */
  bool IsEqual (const CallbackBase &other) const {
    return m_impl->IsEqual (other.GetImpl ());
  }

  /**
   * Check for compatible types
   *
   * \param [in] other Callback Ptr
   * \return \c true if other can be dynamic_cast to my type
   */
  bool CheckType (const CallbackBase & other) const {
    return DoCheckType (other.GetImpl ());
  }
  /**
   * Adopt the other's implementation, if type compatible
   *
   * \param [in] other Callback
   * \returns \c true if \p other was type-compatible and could be adopted.
   */
  bool Assign (const CallbackBase &other) {
    return DoAssign (other.GetImpl ());
  }
private:
  /** \return The pimpl pointer */
  CallbackImpl<R,UArgs...> *DoPeekImpl (void) const {
    return static_cast<CallbackImpl<R,UArgs...> *> (PeekPointer (m_impl));
  }
  /**
   * Check for compatible types
   *
   * \param [in] other Callback Ptr
   * \return \c true if other can be dynamic_cast to my type
   */
  bool DoCheckType (Ptr<const CallbackImplBase> other) const {
    if (other != 0 &&
        dynamic_cast<const CallbackImpl<R,UArgs...> *> (PeekPointer (other)) != 0)
      {
        return true;
      }
    else if (other == 0)
      {
        return true;
      }
    else
      {
        return false;
      }
  }
  /** \copydoc Assign */
  bool DoAssign (Ptr<const CallbackImplBase> other) {
    if (!DoCheckType (other))
      {
        std::string othTid = other->GetTypeid ();
        std::string myTid = CallbackImpl<R,UArgs...>::DoGetTypeid ();
        NS_FATAL_ERROR_CONT ("Incompatible types. (feed to \"c++filt -t\" if needed)" << std::endl <<
                        "got=" << othTid << std::endl <<
                        "expected=" << myTid);
        return false;
      }
    m_impl = const_cast<CallbackImplBase *> (PeekPointer (other));
    return true;
  }
};


/**
 * Inequality test.
 *
 * \param [in] a Callback
 * \param [in] b Callback
 *
 * \return \c true if the Callbacks are not equal
 */
template <typename R, typename... Ts>
bool operator != (Callback<R,Ts...> a, Callback<R,Ts...> b)
{
  return !a.IsEqual (b);
}

/**
 * \ingroup makecallbackmemptr
 * @{
 */
/**
 * \param [in] memPtr Class method member pointer
 * \param [in] objPtr Class instance
 * \return A wrapper Callback
 * 
 * Build Callbacks for class method members which take varying numbers of arguments
 * and potentially returning a value.
 */     
template <typename T, typename OBJ, typename R, typename... Ts>
Callback<R,Ts...> MakeCallback (R (T::*memPtr)(Ts...), OBJ objPtr) {
  return Callback<R,Ts...> (memPtr, objPtr);
}
template <typename T, typename OBJ, typename R, typename... Ts>
Callback<R,Ts...> MakeCallback (R (T::*memPtr)(Ts...) const, OBJ objPtr) {
  return Callback<R,Ts...> (memPtr, objPtr);
}
/**@}*/

/**
 * \ingroup makecallbackfnptr
 * @{
 */
/**
 * \param [in] fnPtr Function pointer
 * \return A wrapper Callback
 * 
 * Build Callbacks for functions which take varying numbers of arguments
 * and potentially returning a value.
 */
template <typename R, typename... Ts>
Callback<R,Ts...> MakeCallback (R (*fnPtr)(Ts...)) {
  return Callback<R,Ts...> (fnPtr);
}
/**@}*/

/**
 * \ingroup makenullcallback
 * @{
 */
/**
 * \return A wrapper Callback
 *
 * Build null Callbacks which take no arguments,
 * for varying number of template arguments,
 * and potentially returning a value.
 */     
template <typename R, typename... Ts>
Callback<R,Ts...> MakeNullCallback (void) {
  return Callback<R,Ts...> ();
}
/**@}*/


/**
 * \ingroup makeboundcallback
 * @{
 * Make Callbacks with varying number of bound arguments.
 * \param [in] fnPtr Function pointer
 * \param [in] bargs Bound arguments
 * \return A bound Callback
 */   
template <typename R, typename... Args, typename... BArgs>
auto MakeBoundCallback (R (*fnPtr)(Args...), BArgs... bargs)
{
  return Callback<R,Args...> (fnPtr).Bind (bargs...);
}

/**
 * \param [in] memPtr Class method member pointer
 * \param [in] objPtr Class instance
 * \param [in] bargs Bound arguments
 * \return A wrapper Callback
 *
 * Build Callbacks for class method members which take varying numbers of arguments
 * and potentially returning a value.
 */
template <typename T, typename OBJ, typename R, typename... Args, typename... BArgs>
auto MakeCallback (R (T::*memPtr)(Args...), OBJ objPtr, BArgs... bargs)
{
  return Callback<R,Args...> (memPtr, objPtr).Bind (bargs...);
}

template <typename T, typename OBJ, typename R, typename... Args, typename... BArgs>
auto MakeCallback (R (T::*memPtr)(Args...) const, OBJ objPtr, BArgs... bargs)
{
  return Callback<R,Args...> (memPtr, objPtr).Bind (bargs...);
}
/**@}*/

} // namespace ns3

namespace ns3 {

class CallbackValue : public AttributeValue
{
public:
  /** Constructor */
  CallbackValue ();
  /**
   * Copy constructor
   * \param [in] base Callback to copy
   */
  CallbackValue (const CallbackBase &base);
  /** Destructor */
  virtual ~CallbackValue ();
  /** \param [in] base The CallbackBase to use */
  void Set (CallbackBase base);
  /**
   * Give value my callback, if type compatible
   *
   * \param [out] value Destination callback
   * \returns \c true if successful
   */
  template <typename T>
  bool GetAccessor (T &value) const;
  /** \return A copy of this CallBack */
  virtual Ptr<AttributeValue> Copy (void) const;
  /**
   * Serialize to string
   * \param [in] checker The checker to validate with
   * \return Serialized form of this Callback.
   */
  virtual std::string SerializeToString (Ptr<const AttributeChecker> checker) const;
  /**
   * Deserialize from string (not implemented)
   *
   * \param [in] value Source string
   * \param [in] checker Checker to validate with
   * \return \c true if successful
   */
  virtual bool DeserializeFromString (std::string value, Ptr<const AttributeChecker> checker);
private:
  CallbackBase m_value;                 //!< the CallbackBase
};

ATTRIBUTE_ACCESSOR_DEFINE (Callback);
ATTRIBUTE_CHECKER_DEFINE (Callback);

} // namespace ns3

namespace ns3 {

template <typename T>
bool CallbackValue::GetAccessor (T &value) const
{
  if (value.CheckType (m_value))
    {
      if (!value.Assign (m_value))
        NS_FATAL_ERROR_NO_MSG ();
      return true;
    }
  return false;
}

} // namespace ns3


#endif /* CALLBACK_H */
