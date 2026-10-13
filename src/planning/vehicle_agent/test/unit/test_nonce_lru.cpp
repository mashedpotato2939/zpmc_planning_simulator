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

#include "vehicle_agent/security/nonce_lru.hpp"

#include <gtest/gtest.h>

#include <string>

using vehicle_agent::NonceLRU;

TEST(NonceLRU, FirstSeenIsNotReplay)
{
  NonceLRU lru(100, 60'000);
  EXPECT_FALSE(lru.seenAndRecord("n1", 1000));
  EXPECT_EQ(lru.size(), 1u);
  EXPECT_TRUE(lru.contains("n1"));
}

TEST(NonceLRU, ReplayDetectedWithinTtl)
{
  NonceLRU lru(100, 60'000);
  EXPECT_FALSE(lru.seenAndRecord("nA", 1000));
  EXPECT_TRUE(lru.seenAndRecord("nA", 2000));
  EXPECT_TRUE(lru.seenAndRecord("nA", 60'500));
}

TEST(NonceLRU, ExpiredEntryPrunedSoNonReplay)
{
  NonceLRU lru(100, 1000);
  EXPECT_FALSE(lru.seenAndRecord("nX", 100));
  EXPECT_TRUE(lru.seenAndRecord("nX", 500));    // still in window
  // ts advances beyond ttl since first record (1100 > 100 + 1000)
  EXPECT_FALSE(lru.seenAndRecord("nX", 1601))
    << "should have been pruned, treated as fresh";
}

TEST(NonceLRU, CapacityEvictsOldest)
{
  NonceLRU lru(3, 60'000);
  lru.seenAndRecord("a", 1);
  lru.seenAndRecord("b", 2);
  lru.seenAndRecord("c", 3);
  EXPECT_EQ(lru.size(), 3u);
  // Insert a 4th -> "a" should fall out.
  lru.seenAndRecord("d", 4);
  EXPECT_EQ(lru.size(), 3u);
  EXPECT_FALSE(lru.contains("a"));
  EXPECT_TRUE(lru.contains("b"));
  EXPECT_TRUE(lru.contains("c"));
  EXPECT_TRUE(lru.contains("d"));
}

TEST(NonceLRU, ReplayMovesToBackPreservingItFromEviction)
{
  NonceLRU lru(3, 60'000);
  lru.seenAndRecord("a", 1);
  lru.seenAndRecord("b", 2);
  lru.seenAndRecord("c", 3);
  // Refresh "a" so "b" becomes oldest.
  EXPECT_TRUE(lru.seenAndRecord("a", 4));
  lru.seenAndRecord("d", 5);
  EXPECT_TRUE(lru.contains("a"));
  EXPECT_FALSE(lru.contains("b"));
}

TEST(NonceLRU, ZeroTtlDisablesTimePrune)
{
  NonceLRU lru(100, 0);
  EXPECT_FALSE(lru.seenAndRecord("k", 1));
  EXPECT_TRUE(lru.seenAndRecord("k", 9999999999ULL));
}

TEST(NonceLRU, ZeroCapacityCoercedToOne)
{
  NonceLRU lru(0, 60'000);
  EXPECT_FALSE(lru.seenAndRecord("a", 1));
  EXPECT_FALSE(lru.seenAndRecord("b", 2));
  EXPECT_EQ(lru.size(), 1u);
  EXPECT_FALSE(lru.contains("a"));
  EXPECT_TRUE(lru.contains("b"));
}
