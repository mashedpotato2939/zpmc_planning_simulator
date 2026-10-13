// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vehicle_agent/security/clock_skew_window.hpp"

#include <gtest/gtest.h>

using vehicle_agent::ClockSkewWindow;

TEST(ClockSkewWindow, AcceptsExactMatch)
{
  ClockSkewWindow w(5000);
  EXPECT_TRUE(w.accepts(1000, 1000));
  EXPECT_EQ(w.classify(1000, 1000), ClockSkewWindow::Reason::OK);
}

TEST(ClockSkewWindow, AcceptsWithinTolerance)
{
  ClockSkewWindow w(5000);
  EXPECT_TRUE(w.accepts(995, 1000));   // 5ms old
  EXPECT_TRUE(w.accepts(1005, 1000));  // 5ms ahead
  EXPECT_TRUE(w.accepts(0, 5000));     // exactly tolerance
  EXPECT_TRUE(w.accepts(10000, 5000)); // exactly tolerance, future
}

TEST(ClockSkewWindow, RejectsTooOld)
{
  ClockSkewWindow w(1000);
  EXPECT_FALSE(w.accepts(0, 2000));
  EXPECT_EQ(w.classify(0, 2000), ClockSkewWindow::Reason::TOO_OLD);
  EXPECT_FALSE(w.accepts(500, 2000));
  EXPECT_EQ(w.classify(500, 2000), ClockSkewWindow::Reason::TOO_OLD);
}

TEST(ClockSkewWindow, RejectsTooNew)
{
  ClockSkewWindow w(1000);
  EXPECT_FALSE(w.accepts(5000, 1000));
  EXPECT_EQ(w.classify(5000, 1000), ClockSkewWindow::Reason::TOO_NEW);
  EXPECT_FALSE(w.accepts(2001, 1000));
  EXPECT_EQ(w.classify(2001, 1000), ClockSkewWindow::Reason::TOO_NEW);
}

TEST(ClockSkewWindow, ZeroToleranceOnlyExact)
{
  ClockSkewWindow w(0);
  EXPECT_TRUE(w.accepts(100, 100));
  EXPECT_FALSE(w.accepts(99, 100));
  EXPECT_FALSE(w.accepts(101, 100));
}

TEST(ClockSkewWindow, ToleranceAccessor)
{
  ClockSkewWindow w(12345);
  EXPECT_EQ(w.toleranceMs(), 12345u);
}
