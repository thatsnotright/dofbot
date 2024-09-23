#include <gmock/gmock.h>
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/actuator.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system.hpp"

class TestInstantiationHardwares : public ::testing::Test
{
protected:
  static void SetUpTestCase() {}
};

TEST_F(TestInstantiationHardwares, build_actuator) { hardware_interface::Actuator anActuator; }



