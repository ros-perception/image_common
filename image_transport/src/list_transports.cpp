#include "image_transport/publisher_plugin.h"
#include "image_transport/subscriber_plugin.h"
#include <pluginlib/class_loader.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <map>

using namespace image_transport;
using namespace pluginlib;

enum PluginStatus {SUCCESS, CREATE_FAILURE, LIB_LOAD_FAILURE, DOES_NOT_EXIST};

/// \cond
struct TransportDesc
{
  TransportDesc()
    : pub_status(DOES_NOT_EXIST), sub_status(DOES_NOT_EXIST)
  {}

  std::string package_name;
  std::string pub_name;
  PluginStatus pub_status;
  std::string sub_name;
  PluginStatus sub_status;
};
/// \endcond

int main(int argc, char** argv)
{
  ClassLoader<PublisherPlugin> pub_loader("image_transport", "image_transport::PublisherPlugin");
  ClassLoader<SubscriberPlugin> sub_loader("image_transport", "image_transport::SubscriberPlugin");
  typedef std::map<std::string, TransportDesc> StatusMap;
  StatusMap transports;

  BOOST_FOREACH(const std::string& lookup_name, pub_loader.getDeclaredClasses()) {
    std::string transport_name = boost::erase_last_copy(lookup_name, "_pub");
    transports[transport_name].pub_name = lookup_name;
    transports[transport_name].package_name = pub_loader.getClassPackage(lookup_name);
    try {
      PublisherPlugin* pub = pub_loader.createClassInstance(lookup_name);
      transports[transport_name].pub_status = SUCCESS;
      delete pub;
    }
    catch (const LibraryLoadException& e) {
      transports[transport_name].pub_status = LIB_LOAD_FAILURE;
    }
    catch (const CreateClassException& e) {
      transports[transport_name].pub_status = CREATE_FAILURE;
    }
  }

  BOOST_FOREACH(const std::string& lookup_name, sub_loader.getDeclaredClasses()) {
    std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
    transports[transport_name].sub_name = lookup_name;
    transports[transport_name].package_name = sub_loader.getClassPackage(lookup_name);
    try {
      SubscriberPlugin* sub = sub_loader.createClassInstance(lookup_name);
      transports[transport_name].sub_status = SUCCESS;
      delete sub;
    }
    catch (const LibraryLoadException& e) {
      transports[transport_name].sub_status = LIB_LOAD_FAILURE;
    }
    catch (const CreateClassException& e) {
      transports[transport_name].sub_status = CREATE_FAILURE;
    }
  }

  bool problem_package = false;
  printf("Declared transports:\n");
  BOOST_FOREACH(const StatusMap::value_type& value, transports) {
    const TransportDesc& td = value.second;
    printf("%s", value.first.c_str());
    if ((td.pub_status == CREATE_FAILURE || td.pub_status == LIB_LOAD_FAILURE) ||
        (td.sub_status == CREATE_FAILURE || td.sub_status == LIB_LOAD_FAILURE)) {
      printf(" (*): Not available. Try 'rosmake %s'.", td.package_name.c_str());
      problem_package = true;
    }
    printf("\n");
  }
#if 0
  if (problem_package)
    printf("(*) \n");
#endif

  printf("\nDetails:\n");
  BOOST_FOREACH(const StatusMap::value_type& value, transports) {
    const TransportDesc& td = value.second;
    printf("----------\n");
    printf("\"%s\"\n", value.first.c_str());
    if (td.pub_status == CREATE_FAILURE || td.sub_status == CREATE_FAILURE) {
      printf("*** Plugins are built, but could not be loaded. The package may need to be rebuilt or may not be compatible with this release of image_common. ***\n");
    }
    else if (td.pub_status == LIB_LOAD_FAILURE || td.sub_status == LIB_LOAD_FAILURE) {
      printf("*** Plugins are not built. ***\n");
    }
    printf(" - Provided by package: %s\n", td.package_name.c_str());
    if (td.pub_status == DOES_NOT_EXIST)
      printf(" - No publisher provided\n");
    else
      printf(" - Publisher: %s\n", pub_loader.getClassDescription(td.pub_name).c_str());
    if (td.sub_status == DOES_NOT_EXIST)
      printf(" - No subscriber provided\n");
    else
      printf(" - Subscriber: %s\n", sub_loader.getClassDescription(td.sub_name).c_str());
  }
  
  return 0;
}
