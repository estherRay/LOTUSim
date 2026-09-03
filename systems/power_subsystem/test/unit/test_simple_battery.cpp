```cpp
#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>

#include "power_subsystem/power_provider/simple_battery.hpp"

namespace lotusim::gazebo {
namespace {

class SimpleBatteryTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }

        node_ = std::make_shared<rclcpp::Node>("simple_battery_test");

        sdf_ = std::make_shared<sdf::Element>();
        sdf::initFile("root", sdf_);

        sdf_->AddElementDescription(
            sdf::ElementPtr(new sdf::Element(*sdf_)));
    }

    void TearDown() override
    {
        node_.reset();
        sdf_.reset();

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    sdf::ElementPtr makeBatterySdf(
        float capacityAh = 100.0f,
        float initialSoc = 1.0f,
        float voltageMin = 36.0f,
        float voltageNominal = 48.0f)
    {
        sdf::SDF batterySdf;

        const std::string xml = R"(
            <sdf version="1.10">
                <model name="test_model">
                    <plugin name="test">
                        <lotusim_power>
                            <name>test_battery</name>
                            <type>simple_battery</type>
                            <capacity_ah>100</capacity_ah>
                            <initial_soc>1.0</initial_soc>
                            <voltage_min>36.0</voltage_min>
                            <voltage_nominal>48.0</voltage_nominal>
                        </lotusim_power>
                    </plugin>
                </model>
            </sdf>
        )";

        EXPECT_TRUE(batterySdf.SetFromString(xml));

        auto model = batterySdf.Root()->GetElement("model");
        auto plugin = model->GetElement("plugin");
        auto power = plugin->GetElement("lotusim_power");

        power->GetElement("capacity_ah")->Set(capacityAh);
        power->GetElement("initial_soc")->Set(initialSoc);
        power->GetElement("voltage_min")->Set(voltageMin);
        power->GetElement("voltage_nominal")->Set(voltageNominal);

        return power;
    }

    std::shared_ptr<spdlog::logger> makeLogger()
    {
        return spdlog::default_logger();
    }

    std::shared_ptr<rclcpp::Node> node_;
    sdf::ElementPtr sdf_;
};

TEST_F(SimpleBatteryTest, InitializesWithExpectedState)
{
    auto sdf = makeBatterySdf();

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    EXPECT_FLOAT_EQ(battery.getStateOfCharge(), 1.0f);
    EXPECT_FLOAT_EQ(battery.voltage(), 48.0f);
    EXPECT_FALSE(battery.isDepleted());
    EXPECT_EQ(battery.powerLevel(), PowerLevel::NORMAL);
}

TEST_F(SimpleBatteryTest, InitializesAtPartialStateOfCharge)
{
    auto sdf = makeBatterySdf(
        100.0f,
        0.5f,
        36.0f,
        48.0f);

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    EXPECT_FLOAT_EQ(battery.getStateOfCharge(), 0.5f);
    EXPECT_FLOAT_EQ(battery.voltage(), 42.0f);
}

TEST_F(SimpleBatteryTest, LoadDecreasesStateOfCharge)
{
    auto sdf = makeBatterySdf();

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    battery.receiveLoad(10.0f, 1.0f);

    EXPECT_FLOAT_EQ(battery.getStateOfCharge(), 0.9f);
}

TEST_F(SimpleBatteryTest, LoadDecreasesVoltage)
{
    auto sdf = makeBatterySdf();

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    battery.receiveLoad(10.0f, 1.0f);

    // SOC = 0.9
    // V = 36 + 0.9 * (48 - 36) = 46.8 V
    EXPECT_FLOAT_EQ(battery.voltage(), 46.8f);
}

TEST_F(SimpleBatteryTest, ZeroLoadDoesNotChangeState)
{
    auto sdf = makeBatterySdf();

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    battery.receiveLoad(0.0f, 10.0f);

    EXPECT_FLOAT_EQ(battery.getStateOfCharge(), 1.0f);
    EXPECT_FLOAT_EQ(battery.voltage(), 48.0f);
}

TEST_F(SimpleBatteryTest, NegativeLoadChargesBattery)
{
    auto sdf = makeBatterySdf(
        100.0f,
        0.5f,
        36.0f,
        48.0f);

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    battery.receiveLoad(-10.0f, 1.0f);

    EXPECT_FLOAT_EQ(battery.getStateOfCharge(), 0.6f);
    EXPECT_FLOAT_EQ(battery.voltage(), 43.2f);
}

TEST_F(SimpleBatteryTest, BatteryCannotExceedFullCharge)
{
    auto sdf = makeBatterySdf();

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    battery.receiveLoad(-100.0f, 10.0f);

    EXPECT_FLOAT_EQ(battery.getStateOfCharge(), 1.0f);
    EXPECT_FLOAT_EQ(battery.voltage(), 48.0f);
}

TEST_F(SimpleBatteryTest, BatteryCannotGoBelowZero)
{
    auto sdf = makeBatterySdf();

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    battery.receiveLoad(1000.0f, 10.0f);

    EXPECT_FLOAT_EQ(battery.getStateOfCharge(), 0.0f);
    EXPECT_FLOAT_EQ(battery.voltage(), 36.0f);
    EXPECT_TRUE(battery.isDepleted());
    EXPECT_EQ(battery.powerLevel(), PowerLevel::DEPLETED);
}

TEST_F(SimpleBatteryTest, AvailablePowerUsesRemainingCharge)
{
    auto sdf = makeBatterySdf();

    SimpleBattery battery(
        "test_battery",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    // 100 Ah × 48 V
    EXPECT_FLOAT_EQ(battery.availablePowerW(), 4800.0f);

    battery.receiveLoad(50.0f, 1.0f);

    // 50 Ah remaining, voltage = 42 V
    EXPECT_FLOAT_EQ(battery.availablePowerW(), 2100.0f);
}

}  // namespace
}  // namespace lotusim::gazebo
```
