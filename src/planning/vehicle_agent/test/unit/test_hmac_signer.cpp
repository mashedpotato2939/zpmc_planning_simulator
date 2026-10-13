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

#include "vehicle_agent/security/hmac_signer.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

using vehicle_agent::HmacSigner;

TEST(HmacSigner, DeterministicForSameInputs)
{
  HmacSigner a("secret-key-1");
  const auto s1 = a.sign("payload", 123456, "nonce-1");
  const auto s2 = a.sign("payload", 123456, "nonce-1");
  EXPECT_EQ(s1, s2);
  EXPECT_EQ(s1.size(), 64u);   // 32 bytes -> 64 hex chars
  for (char c : s1) {
    EXPECT_TRUE((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))
      << "non-hex char in signature: " << c;
  }
}

TEST(HmacSigner, SignVerifyRoundtrip)
{
  HmacSigner a("k1");
  const auto sig = a.sign("hello", 1000, "nonce-A");
  EXPECT_TRUE(a.verify("hello", 1000, "nonce-A", sig));
}

TEST(HmacSigner, RejectsTamperedPayload)
{
  HmacSigner a("k1");
  const auto sig = a.sign("hello", 1000, "nonce-A");
  EXPECT_FALSE(a.verify("hellO", 1000, "nonce-A", sig));
  EXPECT_FALSE(a.verify("hello!", 1000, "nonce-A", sig));
  EXPECT_FALSE(a.verify("", 1000, "nonce-A", sig));
}

TEST(HmacSigner, RejectsTamperedTimestamp)
{
  HmacSigner a("k1");
  const auto sig = a.sign("hello", 1000, "nonce-A");
  EXPECT_FALSE(a.verify("hello", 1001, "nonce-A", sig));
  EXPECT_FALSE(a.verify("hello", 0, "nonce-A", sig));
}

TEST(HmacSigner, RejectsTamperedNonce)
{
  HmacSigner a("k1");
  const auto sig = a.sign("hello", 1000, "nonce-A");
  EXPECT_FALSE(a.verify("hello", 1000, "nonce-B", sig));
  EXPECT_FALSE(a.verify("hello", 1000, "", sig));
}

TEST(HmacSigner, DifferentKeysProduceDifferentMacs)
{
  HmacSigner a("key-A");
  HmacSigner b("key-B");
  const auto sa = a.sign("payload", 1, "n");
  const auto sb = b.sign("payload", 1, "n");
  EXPECT_NE(sa, sb);
  EXPECT_FALSE(b.verify("payload", 1, "n", sa));
}

TEST(HmacSigner, EmptyKeyThrows)
{
  EXPECT_THROW(HmacSigner(""), std::invalid_argument);
}

TEST(HmacSigner, VerifyShortSignatureRejected)
{
  HmacSigner a("k");
  EXPECT_FALSE(a.verify("p", 1, "n", "deadbeef"));
  EXPECT_FALSE(a.verify("p", 1, "n", ""));
}
