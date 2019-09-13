// Generated by gencpp from file simulator/simulator_parameters.msg
// DO NOT EDIT!


#ifndef SIMULATOR_MESSAGE_SIMULATOR_PARAMETERS_H
#define SIMULATOR_MESSAGE_SIMULATOR_PARAMETERS_H

#include <ros/service_traits.h>


#include <simulator/simulator_parametersRequest.h>
#include <simulator/simulator_parametersResponse.h>


namespace simulator
{

struct simulator_parameters
{

typedef simulator_parametersRequest Request;
typedef simulator_parametersResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct simulator_parameters
} // namespace simulator


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::simulator::simulator_parameters > {
  static const char* value()
  {
    return "666381c29fe97dd3096923cea005a173";
  }

  static const char* value(const ::simulator::simulator_parameters&) { return value(); }
};

template<>
struct DataType< ::simulator::simulator_parameters > {
  static const char* value()
  {
    return "simulator/simulator_parameters";
  }

  static const char* value(const ::simulator::simulator_parameters&) { return value(); }
};


// service_traits::MD5Sum< ::simulator::simulator_parametersRequest> should match 
// service_traits::MD5Sum< ::simulator::simulator_parameters > 
template<>
struct MD5Sum< ::simulator::simulator_parametersRequest>
{
  static const char* value()
  {
    return MD5Sum< ::simulator::simulator_parameters >::value();
  }
  static const char* value(const ::simulator::simulator_parametersRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::simulator::simulator_parametersRequest> should match 
// service_traits::DataType< ::simulator::simulator_parameters > 
template<>
struct DataType< ::simulator::simulator_parametersRequest>
{
  static const char* value()
  {
    return DataType< ::simulator::simulator_parameters >::value();
  }
  static const char* value(const ::simulator::simulator_parametersRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::simulator::simulator_parametersResponse> should match 
// service_traits::MD5Sum< ::simulator::simulator_parameters > 
template<>
struct MD5Sum< ::simulator::simulator_parametersResponse>
{
  static const char* value()
  {
    return MD5Sum< ::simulator::simulator_parameters >::value();
  }
  static const char* value(const ::simulator::simulator_parametersResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::simulator::simulator_parametersResponse> should match 
// service_traits::DataType< ::simulator::simulator_parameters > 
template<>
struct DataType< ::simulator::simulator_parametersResponse>
{
  static const char* value()
  {
    return DataType< ::simulator::simulator_parameters >::value();
  }
  static const char* value(const ::simulator::simulator_parametersResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SIMULATOR_MESSAGE_SIMULATOR_PARAMETERS_H