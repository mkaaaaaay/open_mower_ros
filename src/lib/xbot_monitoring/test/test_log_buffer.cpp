#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "../src/LogBuffer.h"

namespace {
// what logs.recent would answer for one WARN line with this text
std::string kept(const std::string& text) {
  LogBuffer buffer(10);
  rosgraph_msgs::Log log;
  log.level = rosgraph_msgs::Log::WARN;
  log.name = "/test";
  log.msg = text;
  buffer.add(log);
  const auto lines = buffer.get(-1, rosgraph_msgs::Log::WARN, 10);
  EXPECT_EQ(lines.size(), 1u);
  return lines.empty() ? "" : lines[0]["msg"].get<std::string>();
}

bool validUtf8(const std::string& s) {
  try {
    json(s).dump();
    return true;
  } catch (const json::type_error&) {
    return false;
  }
}

bool contains(const std::string& s, const std::string& part) {
  return s.find(part) != std::string::npos;
}
}  // namespace

TEST(LogBuffer, BlanksLoginsInUrls) {
  EXPECT_EQ(kept("ntrip://user:pw@caster:2101/M"), "ntrip://***@caster:2101/M");
  // @ and / in the password
  EXPECT_EQ(kept("http://u:p@ss/w@host:2101/M"), "http://***@host:2101/M");
}

TEST(LogBuffer, BlanksKeysWithAPrefix) {
  const auto out = kept(R"(ntrip_password=s1 mqtt_password: s2 OM_NTRIP_PASSWORD=s3 "api_key": "s4" Bearer s5)");
  for (const char* secret : {"s1", "s2", "s3", "s4", "s5"}) EXPECT_FALSE(contains(out, secret)) << out;
}

TEST(LogBuffer, LeavesOrdinaryTextAlone) {
  for (const std::string text : {"tokens_used=5", "GPS quality: 0.93", "Timeout in PRE_ROTATE phase", ""}) {
    EXPECT_EQ(kept(text), text);
  }
}

TEST(LogBuffer, CutsLongLinesWithoutCrashing) {
  // std::regex overflowed the stack on these
  const auto token = kept(std::string(100000, 'x') + " token=abc");
  EXPECT_LE(token.size(), 2003u);
  const auto url = kept("ntrip://user:pw@host/" + std::string(100000, 'y'));
  EXPECT_FALSE(contains(url, "user:pw"));
  EXPECT_LE(url.size(), 2003u);
}

TEST(LogBuffer, UrlRunningIntoTheCutShowsNoLogin) {
  const auto out = kept(std::string(1980, 'a') + " http://user:geheim@host:2101/M");
  EXPECT_FALSE(contains(out, "user")) << out;
  EXPECT_FALSE(contains(out, "geheim")) << out;
  EXPECT_EQ(out.substr(out.size() - 10), "http://...");
}

// a multi byte character at every position around the cut: the answer stays valid utf-8
TEST(LogBuffer, CutsAtACharacterBoundary) {
  for (const std::string ch : {"\xC3\xBC", "\xE2\x82\xAC", "\xF0\x9F\xA6\x94"}) {  // ü € hedgehog
    for (size_t pos = 1990; pos <= 2000; pos++) {
      const auto out = kept(std::string(pos, 'a') + ch + std::string(400, 'r'));
      EXPECT_TRUE(validUtf8(out)) << "at " << pos;
      EXPECT_EQ(out.substr(out.size() - 3), "...") << "at " << pos;
    }
  }
}

// a secret at every position around the cut: no part of it shows
TEST(LogBuffer, SecretAcrossTheCut) {
  const std::string secret = "SECRETVALUE123";
  for (const std::string form : {"http://user:" + secret + "@host:2101/M", "password=" + secret,
                                 "\"token\": \"" + secret + "\"", "Bearer " + secret}) {
    for (size_t pos = 1960; pos <= 2000; pos++) {
      const auto out = kept(std::string(pos, 'a') + " " + form + " tail");
      for (size_t n = 4; n <= secret.size(); n++) {
        EXPECT_FALSE(contains(out, secret.substr(0, n))) << form << " at " << pos << ": ..." << out.substr(1950);
      }
    }
  }
}

TEST(LogBuffer, RawBytesDontBreakTheAnswer) {
  LogBuffer buffer(10);
  rosgraph_msgs::Log log;
  log.level = rosgraph_msgs::Log::WARN;
  log.msg = "before \xFF\xFE after";
  buffer.add(log);
  const auto answer = buffer.get(-1, rosgraph_msgs::Log::WARN, 10);
  // like the rpc provider writes it
  const auto text = answer.dump(-1, ' ', false, json::error_handler_t::replace);
  EXPECT_TRUE(contains(text, "\xEF\xBF\xBD"));
  EXPECT_TRUE(contains(text, "before") && contains(text, "after"));
}

TEST(LogBuffer, OnlyContinuationBytes) {
  EXPECT_EQ(kept(std::string(3000, '\x80')), "...");
}

// worst cases for the regexes stay fast on the 2000 characters that are left
TEST(LogBuffer, WorstCaseRuntime) {
  const auto start = std::chrono::steady_clock::now();
  std::string many_urls;
  while (many_urls.size() < 100000) many_urls += "://";
  kept(many_urls);
  kept(std::string(100000, 'p') + "=x");
  kept(std::string(100000, '@'));
  const auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
  EXPECT_LT(ms, 2000);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
