```cpp
#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>

#include "power_subsystem/power_provider/simple_generator.hpp"

namespace lotusim::gazebo {
namespace {

class SimpleGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }

        node_ = std::make_shared<rclcpp::Node>("simple_generator_test");
    }

    void TearDown() override
    {
        node_.reset();

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    sdf::ElementPtr makeGeneratorSdf(
        float fuelCapacity = 500.0f,
        float fuelStart = 400.0f,
        float ratedOutput = 5000.0f,
        float efficiency = 0.35f,
        float voltage = 48.0f,
        const std::string& fuelType = "diesel")
    {
        sdf::SDF generatorSdf;

        const std::string xml = R"(
            <sdf version="1.10">
                <model name="test_model">
                    <plugin name="test">
                        <lotusim_power>
                            <name>test_generator</name>
                            <type>simple_generator</type>
                            <fuel_type>diesel</fuel_type>
                            <fuel_capacity>500</fuel_capacity>
                            <fuel_level_start>400</fuel_level_start>
                            <rated_output_w>5000</rated_output_w>
                            <efficiency>0.35</efficiency>
                            <voltage_nominal>48</voltage_nominal>
                        </lotusim_power>
                    </plugin>
                </model>
            </sdf>
        )";

        EXPECT_TRUE(generatorSdf.SetFromString(xml));

        auto model = generatorSdf.Root()->GetElement("model");
        auto plugin = model->GetElement("plugin");
        auto power = plugin->GetElement("lotusim_power");

        power->GetElement("fuel_capacity")->Set(fuelCapacity);
        power->GetElement("fuel_level_start")->Set(fuelStart);
        power->GetElement("rated_output_w")->Set(ratedOutput);
        power->GetElement("efficiency")->Set(efficiency);
        power->GetElement("voltage_nominal")->Set(voltage);
        power->GetElement("fuel_type")->Set(fuelType);

        return power;
    }

    std::shared_ptr<spdlog::logger> makeLogger()
    {
        return spdlog::default_logger();
    }

    std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(SimpleGeneratorTest, InitializesWithExpectedState)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    EXPECT_FLOAT_EQ(generator.getStateOfCharge(), 0.8f);
    EXPECT_FLOAT_EQ(generator.voltage(), 48.0f);
    EXPECT_FLOAT_EQ(generator.availablePowerW(), 4000.0f);

    EXPECT_FALSE(generator.isDepleted());
    EXPECT_EQ(generator.powerLevel(), PowerLevel::NORMAL);
}

TEST_F(SimpleGeneratorTest, ZeroLoadDoesNotConsumeFuel)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    generator.receiveLoad(0.0f, 100.0f);

    EXPECT_FLOAT_EQ(generator.getStateOfCharge(), 0.8f);
    EXPECT_FLOAT_EQ(generator.availablePowerW(), 4000.0f);
}

TEST_F(SimpleGeneratorTest, LoadConsumesFuel)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    const float initialFuelRatio = generator.getStateOfCharge();

    generator.receiveLoad(10.0f, 1.0f);

    EXPECT_LT(generator.getStateOfCharge(), initialFuelRatio);
}

TEST_F(SimpleGeneratorTest, FuelConsumptionMatchesExpectedValue)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    // 10 A × 48 V = 480 W
    //
    // fuel = 480 × 1 /
    //        (0.35 × 34,920,000)
    const float expectedFuelConsumed =
        (480.0f * 1.0f) / (0.35f * 34'920'000.0f);

    const float expectedFuelRemaining =
        400.0f - expectedFuelConsumed;

    const float expectedFuelRatio =
        expectedFuelRemaining / 500.0f;

    generator.receiveLoad(10.0f, 1.0f);

    EXPECT_NEAR(
        generator.getStateOfCharge(),
        expectedFuelRatio,
        1e-6f);
}

TEST_F(SimpleGeneratorTest, AvailablePowerDependsOnFuelRatio)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    // 400 / 500 = 0.8
    // 5000 × 0.8 = 4000 W
    EXPECT_FLOAT_EQ(generator.availablePowerW(), 4000.0f);

    generator.receiveLoad(1000.0f, 100000.0f);

    EXPECT_FLOAT_EQ(generator.availablePowerW(), 0.0f);
    EXPECT_TRUE(generator.isDepleted());
}

TEST_F(SimpleGeneratorTest, VoltageIsNominalWhileFuelled)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    EXPECT_FLOAT_EQ(generator.voltage(), 48.0f);
}

TEST_F(SimpleGeneratorTest, VoltageIsZeroWhenDepleted)
{
    auto sdf = makeGeneratorSdf(
        100.0f,
        0.0f);

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    EXPECT_FLOAT_EQ(generator.voltage(), 0.0f);
    EXPECT_TRUE(generator.isDepleted());
    EXPECT_EQ(generator.powerLevel(), PowerLevel::DEPLETED);
}

TEST_F(SimpleGeneratorTest, CannotConsumeFuelWhenAlreadyDepleted)
{
    auto sdf = makeGeneratorSdf(
        100.0f,
        0.0f);

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    generator.receiveLoad(100.0f, 100.0f);

    EXPECT_FLOAT_EQ(generator.getStateOfCharge(), 0.0f);
    EXPECT_FLOAT_EQ(generator.availablePowerW(), 0.0f);
}

TEST_F(SimpleGeneratorTest, CanCalculateSurplusChargingCurrent)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    // Available power = 4000 W
    // Bus demand = 10 A × 48 V = 480 W
    // Surplus = 3520 W
    // Surplus current = 3520 / 48
    const float expected = 3520.0f / 48.0f;

    EXPECT_NEAR(
        generator.surplusChargingCurrent(10.0f),
        expected,
        1e-5f);
}

TEST_F(SimpleGeneratorTest, NoSurplusWhenDemandExceedsAvailablePower)
{
    auto sdf = makeGeneratorSdf();

    SimpleGenerator generator(
        "test_generator",
        "test_vessel",
        sdf,
        node_,
        makeLogger());

    // Available = 4000 W
    // Demand = 100 A × 48 V = 4800 W
    EXPECT_FLOAT_EQ(
        generator.surplusChargingCurrent(100.0f),
        0.0f);
}

}  // namespace
}  // namespace lotusim::gazebo
```
