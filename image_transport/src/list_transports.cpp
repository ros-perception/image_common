#include "image_transport/publisher_plugin.h"
#include "image_transport/subscriber_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <map>

using namespace image_transport;
using namespace pluginlib;

enum PluginStatus {SUCCESS, CREATE_FAILURE, LIB_LOAD_FAILURE, DOES_NOT_EXIST};

struct TransportDesc
{
  TransportDesc()
    : pub_status(DOES_NOT_EXIST), sub_status(DOES_NOT_EXIST)
  {}
  
  std::string pub_name;
  PluginStatus pub_status;
  std::string sub_name;
  PluginStatus sub_status;
};

template< typename Loader >
void printPluginStatus(Loader& loader, const std::string& name, PluginStatus status)
{
  if (status == DOES_NOT_EXIST) {
    printf("Does not exist\n");
    return;
  }
  else if (status == LIB_LOAD_FAILURE)
    printf("Failed to load library\n");
  else if (status == CREATE_FAILURE)
    printf("Failed to create instance\n");
  else if (status == SUCCESS)
    printf("Loaded successfully\n");

  printf("\t%s\n", loader.getClassDescription(name).c_str());
}

int main(int argc, char** argv)
{
  ClassLoader<PublisherPlugin> pub_loader("image_transport", "image_transport::PublisherPlugin");
  ClassLoader<SubscriberPlugin> sub_loader("image_transport", "image_transport::SubscriberPlugin");
  typedef std::map<std::string, TransportDesc> StatusMap;
  StatusMap transports;

  BOOST_FOREACH(const std::string& lookup_name, pub_loader.getDeclaredClasses()) {
    std::string transport_name = boost::erase_last_copy(lookup_name, "_pub");
    transports[transport_name].pub_name = lookup_name;
    if (pub_loader.loadLibraryForClass(lookup_name)) {
      try {
        PublisherPlugin* pub = pub_loader.createClassInstance(lookup_name);
        transports[transport_name].pub_status = SUCCESS;
        delete pub;
      }
      catch (const std::runtime_error& e) {
        transports[transport_name].pub_status = CREATE_FAILURE;
      }
    }
    else {
      transports[transport_name].pub_status = LIB_LOAD_FAILURE;
    }
  }

  BOOST_FOREACH(const std::string& lookup_name, sub_loader.getDeclaredClasses()) {
    std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
    transports[transport_name].sub_name = lookup_name;
    if (sub_loader.loadLibraryForClass(lookup_name)) {
      try {
        SubscriberPlugin* sub = sub_loader.createClassInstance(lookup_name);
        transports[transport_name].sub_status = SUCCESS;
        delete sub;
      }
      catch (const std::runtime_error& e) {
        transports[transport_name].sub_status = CREATE_FAILURE;
      }
    }
    else {
      transports[transport_name].sub_status = LIB_LOAD_FAILURE;
    }
  }

  bool problem_package = false;
  printf("Declared transports:\n");
  BOOST_FOREACH(const StatusMap::value_type& value, transports) {
    printf(value.first.c_str());
    if ((value.second.pub_status == CREATE_FAILURE || value.second.pub_status == LIB_LOAD_FAILURE) ||
        (value.second.sub_status == CREATE_FAILURE || value.second.sub_status == LIB_LOAD_FAILURE)) {
      printf(" (*)");
      problem_package = true;
    }
    printf("\n");
  }
  if (problem_package)
    printf("(*) Problem loading, see details\n");

  printf("\nDetails:\n");
  BOOST_FOREACH(const StatusMap::value_type& value, transports) {
    std::string package = pub_loader.getClassPackage(value.second.pub_name);
    printf("----------\n");
    printf("\"%s\"\n", value.first.c_str());
    printf("Provided by package: %s\n", package.c_str());
    printf("Publisher: ");
    printPluginStatus(pub_loader, value.second.pub_name, value.second.pub_status);
    printf("Subscriber: ");
    printPluginStatus(sub_loader, value.second.sub_name, value.second.sub_status);
    if (value.second.pub_status >= LIB_LOAD_FAILURE && value.second.sub_status >= LIB_LOAD_FAILURE)
      printf("Try 'rosmake %s'\n", package.c_str());
  }
  
  return 0;
}
