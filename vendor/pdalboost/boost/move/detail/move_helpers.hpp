//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright Ion Gaztanaga 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/move for documentation.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef BOOST_MOVE_MOVE_HELPERS_HPP
#define BOOST_MOVE_MOVE_HELPERS_HPP

#ifndef BOOST_CONFIG_HPP
#  include <boost/config.hpp>
#endif
#
#if defined(BOOST_HAS_PRAGMA_ONCE)
#  pragma once
#endif

#include <boost/move/utility_core.hpp>
#include <boost/move/detail/meta_utils.hpp>

#if defined(BOOST_NO_CXX11_RVALUE_REFERENCES)

#define BOOST_MOVE_CATCH_CONST(U)  \
   typename ::pdalboost::move_detail::if_< ::pdalboost::move_detail::is_class<U>, BOOST_CATCH_CONST_RLVALUE(U), const U &>::type
#define BOOST_MOVE_CATCH_RVALUE(U)\
   typename ::pdalboost::move_detail::if_< ::pdalboost::move_detail::is_class<U>, BOOST_RV_REF(U), ::pdalboost::move_detail::nat>::type
#define BOOST_MOVE_CATCH_FWD(U) BOOST_FWD_REF(U)
#else
#define BOOST_MOVE_CATCH_CONST(U)  const U &
#define BOOST_MOVE_CATCH_RVALUE(U) U &&
#define BOOST_MOVE_CATCH_FWD(U)    U &&
#endif

////////////////////////////////////////
//
// BOOST_MOVE_CONVERSION_AWARE_CATCH
//
////////////////////////////////////////

#ifdef BOOST_NO_CXX11_RVALUE_REFERENCES
   #define BOOST_MOVE_CONVERSION_AWARE_CATCH_COMMON(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION)\
      RETURN_VALUE PUB_FUNCTION(BOOST_MOVE_CATCH_CONST(TYPE) x)\
      {  return FWD_FUNCTION(static_cast<const TYPE&>(x)); }\
      \
      RETURN_VALUE PUB_FUNCTION(BOOST_MOVE_CATCH_RVALUE(TYPE) x) \
      {  return FWD_FUNCTION(::pdalboost::move(x));  }\
      \
      RETURN_VALUE PUB_FUNCTION(TYPE &x)\
      {  return FWD_FUNCTION(const_cast<const TYPE &>(x)); }\
   //
   #if defined(BOOST_MOVE_HELPERS_RETURN_SFINAE_BROKEN)
      #define BOOST_MOVE_CONVERSION_AWARE_CATCH(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION)\
         BOOST_MOVE_CONVERSION_AWARE_CATCH_COMMON(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION)\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         RETURN_VALUE PUB_FUNCTION(const BOOST_MOVE_TEMPL_PARAM &u,\
            typename ::pdalboost::move_detail::enable_if_and\
                                 < ::pdalboost::move_detail::nat \
                                 , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM>\
                                 , ::pdalboost::move_detail::is_class<TYPE>\
                                 , ::pdalboost::has_move_emulation_disabled<BOOST_MOVE_TEMPL_PARAM>\
                                 >::type* = 0)\
         { return FWD_FUNCTION(u); }\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         RETURN_VALUE PUB_FUNCTION(const BOOST_MOVE_TEMPL_PARAM &u,\
            typename ::pdalboost::move_detail::disable_if_or\
                              < ::pdalboost::move_detail::nat \
                              , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM> \
                              , ::pdalboost::move_detail::and_ \
                                 < ::pdalboost::move_detail::is_rv<BOOST_MOVE_TEMPL_PARAM> \
                                 , ::pdalboost::move_detail::is_class<BOOST_MOVE_TEMPL_PARAM> \
                                 > \
                              >::type* = 0)\
         {\
            TYPE t(u);\
            return FWD_FUNCTION(::pdalboost::move(t));\
         }\
      //
   #else
      #define BOOST_MOVE_CONVERSION_AWARE_CATCH(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION)\
         BOOST_MOVE_CONVERSION_AWARE_CATCH_COMMON(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION)\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         typename ::pdalboost::move_detail::enable_if_and\
                                       < RETURN_VALUE \
                                       , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM>\
                                       , ::pdalboost::move_detail::is_class<TYPE>\
                                       , ::pdalboost::has_move_emulation_disabled<BOOST_MOVE_TEMPL_PARAM>\
                                       >::type\
            PUB_FUNCTION(const BOOST_MOVE_TEMPL_PARAM &u)\
         { return FWD_FUNCTION(u); }\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         typename ::pdalboost::move_detail::disable_if_or\
                           < RETURN_VALUE \
                           , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM> \
                           , ::pdalboost::move_detail::and_ \
                              < ::pdalboost::move_detail::is_rv<BOOST_MOVE_TEMPL_PARAM> \
                              , ::pdalboost::move_detail::is_class<BOOST_MOVE_TEMPL_PARAM> \
                              > \
                           >::type\
            PUB_FUNCTION(const BOOST_MOVE_TEMPL_PARAM &u)\
         {\
            TYPE t(u);\
            return FWD_FUNCTION(::pdalboost::move(t));\
         }\
      //
   #endif
#elif (defined(_MSC_VER) && (_MSC_VER == 1600))

   #define BOOST_MOVE_CONVERSION_AWARE_CATCH(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION)\
      RETURN_VALUE PUB_FUNCTION(BOOST_MOVE_CATCH_CONST(TYPE) x)\
      {  return FWD_FUNCTION(static_cast<const TYPE&>(x)); }\
      \
      RETURN_VALUE PUB_FUNCTION(BOOST_MOVE_CATCH_RVALUE(TYPE) x) \
      {  return FWD_FUNCTION(::pdalboost::move(x));  }\
      \
      template<class BOOST_MOVE_TEMPL_PARAM>\
      typename ::pdalboost::move_detail::enable_if_c\
                        < !::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM>::value\
                        , RETURN_VALUE >::type\
      PUB_FUNCTION(const BOOST_MOVE_TEMPL_PARAM &u)\
      {\
         TYPE t(u);\
         return FWD_FUNCTION(::pdalboost::move(t));\
      }\
   //

#else    //BOOST_NO_CXX11_RVALUE_REFERENCES

   #define BOOST_MOVE_CONVERSION_AWARE_CATCH(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION)\
      RETURN_VALUE PUB_FUNCTION(BOOST_MOVE_CATCH_CONST(TYPE) x)\
      {  return FWD_FUNCTION(static_cast<const TYPE&>(x)); }\
      \
      RETURN_VALUE PUB_FUNCTION(BOOST_MOVE_CATCH_RVALUE(TYPE) x) \
      {  return FWD_FUNCTION(::pdalboost::move(x));  }\
   //

#endif   //BOOST_NO_CXX11_RVALUE_REFERENCES

////////////////////////////////////////
//
// BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG
//
////////////////////////////////////////

#ifdef BOOST_NO_CXX11_RVALUE_REFERENCES
   #define BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG_COMMON(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION, ARG1, UNLESS_CONVERTIBLE_TO)\
      RETURN_VALUE PUB_FUNCTION(ARG1 arg1, BOOST_MOVE_CATCH_CONST(TYPE) x)\
      {  return FWD_FUNCTION(arg1, static_cast<const TYPE&>(x)); }\
      \
      RETURN_VALUE PUB_FUNCTION(ARG1 arg1, BOOST_MOVE_CATCH_RVALUE(TYPE) x) \
      {  return FWD_FUNCTION(arg1, ::pdalboost::move(x));  }\
      \
      RETURN_VALUE PUB_FUNCTION(ARG1 arg1, TYPE &x)\
      {  return FWD_FUNCTION(arg1, const_cast<const TYPE &>(x)); }\
   //
   #if defined(BOOST_MOVE_HELPERS_RETURN_SFINAE_BROKEN)
      #define BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION, ARG1, UNLESS_CONVERTIBLE_TO)\
         BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG_COMMON(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION, ARG1, UNLESS_CONVERTIBLE_TO)\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         RETURN_VALUE PUB_FUNCTION(ARG1 arg1, const BOOST_MOVE_TEMPL_PARAM &u,\
            typename ::pdalboost::move_detail::enable_if_and\
                              < ::pdalboost::move_detail::nat \
                              , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM>\
                              , ::pdalboost::has_move_emulation_disabled<BOOST_MOVE_TEMPL_PARAM>\
                              >::type* = 0)\
         { return FWD_FUNCTION(arg1, u); }\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         RETURN_VALUE PUB_FUNCTION(ARG1 arg1, const BOOST_MOVE_TEMPL_PARAM &u,\
            typename ::pdalboost::move_detail::disable_if_or\
                              < void \
                              , ::pdalboost::move_detail::is_rv<BOOST_MOVE_TEMPL_PARAM>\
                              , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM>\
                              , ::pdalboost::move_detail::is_convertible<BOOST_MOVE_TEMPL_PARAM, UNLESS_CONVERTIBLE_TO>\
                              >::type* = 0)\
         {\
            TYPE t(u);\
            return FWD_FUNCTION(arg1, ::pdalboost::move(t));\
         }\
      //
   #else
      #define BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION, ARG1, UNLESS_CONVERTIBLE_TO)\
         BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG_COMMON(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION, ARG1, UNLESS_CONVERTIBLE_TO)\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         typename ::pdalboost::move_detail::enable_if_and\
                           < RETURN_VALUE \
                           , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM>\
                           , ::pdalboost::has_move_emulation_disabled<BOOST_MOVE_TEMPL_PARAM>\
                           >::type\
            PUB_FUNCTION(ARG1 arg1, const BOOST_MOVE_TEMPL_PARAM &u)\
         { return FWD_FUNCTION(arg1, u); }\
         \
         template<class BOOST_MOVE_TEMPL_PARAM>\
         typename ::pdalboost::move_detail::disable_if_or\
                           < RETURN_VALUE \
                           , ::pdalboost::move_detail::is_rv<BOOST_MOVE_TEMPL_PARAM>\
                           , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM>\
                           , ::pdalboost::move_detail::is_convertible<BOOST_MOVE_TEMPL_PARAM, UNLESS_CONVERTIBLE_TO>\
                           >::type\
            PUB_FUNCTION(ARG1 arg1, const BOOST_MOVE_TEMPL_PARAM &u)\
         {\
            TYPE t(u);\
            return FWD_FUNCTION(arg1, ::pdalboost::move(t));\
         }\
      //
   #endif

#elif (defined(_MSC_VER) && (_MSC_VER == 1600))

   #define BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION, ARG1, UNLESS_CONVERTIBLE_TO)\
      RETURN_VALUE PUB_FUNCTION(ARG1 arg1, BOOST_MOVE_CATCH_CONST(TYPE) x)\
      {  return FWD_FUNCTION(arg1, static_cast<const TYPE&>(x)); }\
      \
      RETURN_VALUE PUB_FUNCTION(ARG1 arg1, BOOST_MOVE_CATCH_RVALUE(TYPE) x) \
      {  return FWD_FUNCTION(arg1, ::pdalboost::move(x));  }\
      \
      template<class BOOST_MOVE_TEMPL_PARAM>\
      typename ::pdalboost::move_detail::disable_if_or\
                        < RETURN_VALUE \
                        , ::pdalboost::move_detail::is_same<TYPE, BOOST_MOVE_TEMPL_PARAM> \
                        , ::pdalboost::move_detail::is_convertible<BOOST_MOVE_TEMPL_PARAM, UNLESS_CONVERTIBLE_TO> \
                        >::type\
      PUB_FUNCTION(ARG1 arg1, const BOOST_MOVE_TEMPL_PARAM &u)\
      {\
         TYPE t(u);\
         return FWD_FUNCTION(arg1, ::pdalboost::move(t));\
      }\
   //

#else

   #define BOOST_MOVE_CONVERSION_AWARE_CATCH_1ARG(PUB_FUNCTION, TYPE, RETURN_VALUE, FWD_FUNCTION, ARG1, UNLESS_CONVERTIBLE_TO)\
      RETURN_VALUE PUB_FUNCTION(ARG1 arg1, BOOST_MOVE_CATCH_CONST(TYPE) x)\
      {  return FWD_FUNCTION(arg1, static_cast<const TYPE&>(x)); }\
      \
      RETURN_VALUE PUB_FUNCTION(ARG1 arg1, BOOST_MOVE_CATCH_RVALUE(TYPE) x) \
      {  return FWD_FUNCTION(arg1, ::pdalboost::move(x));  }\
   //

#endif

#endif //#ifndef BOOST_MOVE_MOVE_HELPERS_HPP
