#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "roborun/cli_evidence_barrier.h"

namespace roborun {
namespace {

class GuiEvidenceBarrierTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const ::testing::TestInfo* test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    directory_ = std::filesystem::current_path() /
                 (std::string("gui-evidence-barrier-test-") + test_info->name());
    std::filesystem::remove_all(directory_);
    std::filesystem::create_directories(directory_);
  }

  void TearDown() override { std::filesystem::remove_all(directory_); }

  EvidenceBarrierOptions Options() const {
    return EvidenceBarrierOptions{
        directory_ / "ready.json", directory_ / "continue.json",   "normal",
        "normal-run-123",          std::chrono::milliseconds(500), std::chrono::milliseconds(5)};
  }

  std::filesystem::path directory_;
};

TEST_F(GuiEvidenceBarrierTest, PublishesBoundRecordAndWaitsForMatchingCaptureConfirmation) {
  const EvidenceBarrierOptions options = Options();
  std::thread capture([&options]() {
    while (!std::filesystem::exists(options.ready_path)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::filesystem::copy_file(options.ready_path, options.continue_path);
  });

  const EvidenceBarrierResult result = PublishEvidenceAndWait(options, 42);
  capture.join();

  EXPECT_TRUE(result.succeeded) << result.message;
  std::ifstream ready(options.ready_path);
  std::stringstream contents;
  contents << ready.rdbuf();
  EXPECT_NE(contents.str().find("\"schema\":\"roborun.gui.evidence-ready.v1\""), std::string::npos);
  EXPECT_NE(contents.str().find("\"scenario\":\"normal\""), std::string::npos);
  EXPECT_NE(contents.str().find("\"scenario_id\":\"normal-run-123\""), std::string::npos);
  EXPECT_NE(contents.str().find("\"generation\":42"), std::string::npos);
}

TEST_F(GuiEvidenceBarrierTest, TimesOutInsteadOfLeavingTheSimulatorFrozenForever) {
  EvidenceBarrierOptions options = Options();
  options.timeout = std::chrono::milliseconds(20);

  const EvidenceBarrierResult result = PublishEvidenceAndWait(options, 7);

  EXPECT_FALSE(result.succeeded);
  EXPECT_NE(result.message.find("timed out"), std::string::npos);
}

TEST_F(GuiEvidenceBarrierTest, DoesNotAcceptAStaleCaptureConfirmation) {
  EvidenceBarrierOptions options = Options();
  options.timeout = std::chrono::milliseconds(20);
  std::ofstream(options.continue_path) << "stale confirmation\n";

  const EvidenceBarrierResult result = PublishEvidenceAndWait(options, 9);

  EXPECT_FALSE(result.succeeded);
}

}  // namespace
}  // namespace roborun
