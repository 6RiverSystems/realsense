#include <RealsenseDriver.h>
#include <nodelet/nodelet.h>


namespace srs
{

class RealsenseDriverNodelet : public nodelet::Nodelet
{
public:
  RealsenseDriverNodelet()  {};

  ~RealsenseDriverNodelet() {}

private:
  virtual void onInit()
  {
    rs.reset(new RealsenseDriver());
    ROS_ERROR("I am in RS nodelet");
  };

  boost::shared_ptr<RealsenseDriver> rs;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(srsnode_realsense_to_laserscan, RealsenseDriverNodelet, srs::RealsenseDriverNodelet, nodelet::Nodelet);

