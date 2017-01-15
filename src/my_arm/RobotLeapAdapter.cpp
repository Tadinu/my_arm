#include "RobotLeapAdapter.h"

RobotLeapAdapter::~RobotLeapAdapter() {
    // Remove the sample listener when done
    _controller.removeListener(_camera_listener);

    // Remove the sample listener when done
    _controller.removeListener(_hands_listener);
}

void RobotLeapAdapter::initLeapMotion()
{
    // Have the sample listener receive events from the controller
    _controller.addListener(_camera_listener);
    _controller.addListener(_hands_listener);
    _controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));

    return;
}
