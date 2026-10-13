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

#include "vehicle_agent/app/mission_state_machine.hpp"

#include <chrono>
#include <utility>

namespace vehicle_agent
{

namespace
{
uint64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
}  // namespace

MissionStateMachine::MissionStateMachine(Sinks sinks) : sinks_(std::move(sinks)) {}

ir::MissionState MissionStateMachine::state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return state_;
}

std::optional<ir::Mission> MissionStateMachine::activeMission() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return active_;
}

void MissionStateMachine::transition(ir::MissionState next, const std::string & detail)
{
  state_ = next;
  emitStatus(detail);
}

void MissionStateMachine::emitStatus(const std::string & detail)
{
  ir::MissionStatus s;
  s.mission_id = active_ ? active_->id : "";
  s.state = state_;
  s.stamp_ms = now_ms();
  s.progress = progress_;
  s.detail = detail;
  if (sinks_.on_status) sinks_.on_status(s);
}

void MissionStateMachine::handle(const ir::Event & ev)
{
  std::lock_guard<std::mutex> lock(mtx_);
  switch (ev.kind) {
    case ir::EventKind::MISSION_RECEIVED:
      onMissionReceived(std::get<ir::Mission>(ev.payload));
      break;
    case ir::EventKind::COMMAND_RECEIVED:
      onCommand(std::get<ir::Command>(ev.payload));
      break;
    case ir::EventKind::CHECK_COMPLETE:
      onCheckComplete(std::get<ir::MissionCheckResult>(ev.payload));
      break;
    case ir::EventKind::EXECUTION_PROGRESS: {
      const auto & p = std::get<ir::ExecutionProgress>(ev.payload);
      if (active_ && active_->id == p.mission_id) {
        progress_ = p.progress;
        emitStatus("progress");
      }
      break;
    }
    case ir::EventKind::EXECUTION_COMPLETE:
      onExecutionComplete(std::get<ir::ExecutionComplete>(ev.payload).mission_id);
      break;
    case ir::EventKind::EXECUTION_FAILED: {
      const auto & f = std::get<ir::ExecutionFailed>(ev.payload);
      onExecutionFailed(f.mission_id, f.reason);
      break;
    }
    case ir::EventKind::TELEMETRY_UPDATE:
    case ir::EventKind::TICK:
      // Ignored at the state-machine layer.
      break;
  }
}

void MissionStateMachine::onMissionReceived(const ir::Mission & m)
{
  using S = ir::MissionState;
  switch (state_) {
    case S::IDLE:
    case S::DONE:
    case S::FAILED:
    case S::PREEMPTED:
      active_ = m;
      progress_ = 0.0;
      transition(S::RECEIVED, "mission accepted");
      // Synchronous validate stub: pass through. Sprint 1+ adapter pipelines
      // will inject CHECK_COMPLETE explicitly; for the loopback path the
      // mission is well-formed by construction so we move straight on.
      transition(S::VALIDATING, "validating");
      transition(S::EXECUTING, "execution begin");
      if (sinks_.on_ack) {
        ir::Ack ack{m.id, ir::AckStatus::OK, "accepted", now_ms()};
        sinks_.on_ack(ack);
      }
      break;
    case S::EXECUTING:
    case S::PAUSED:
      // Same-id re-push is a data-plane refresh of the active mission's
      // waypoints (the FMS pushes RouteNotify on-change for the same route_id
      // per ICD §8.63). Update the latched route in place, no FSM transition,
      // no ack — va_route_bridge consumes the new geometry on its next window
      // tick.
      if (active_ && active_->id == m.id) {
        active_->waypoints = m.waypoints;
        active_->seq = m.seq;
        active_->issued_at_ms = m.issued_at_ms;
        break;
      }
      // Different id: preempt active and promote the new one.
      if (sinks_.on_ack && active_) {
        ir::Ack preempt_ack{active_->id, ir::AckStatus::PREEMPTED, "preempted by new mission", now_ms()};
        sinks_.on_ack(preempt_ack);
      }
      pending_ = m;
      transition(S::PREEMPTED, "preempted");
      active_ = *pending_;
      pending_.reset();
      progress_ = 0.0;
      transition(S::RECEIVED, "queued mission promoted");
      transition(S::VALIDATING, "validating");
      transition(S::EXECUTING, "execution begin");
      if (sinks_.on_ack) {
        ir::Ack ack{m.id, ir::AckStatus::OK, "accepted", now_ms()};
        sinks_.on_ack(ack);
      }
      break;
    case S::ABORTING:
    case S::RECEIVED:
    case S::VALIDATING:
      // Reject — adapter retains for retry.
      if (sinks_.on_ack) {
        ir::Ack ack{m.id, ir::AckStatus::REJECTED, "busy", now_ms()};
        sinks_.on_ack(ack);
      }
      break;
  }
}

void MissionStateMachine::onCommand(const ir::Command & c)
{
  using S = ir::MissionState;
  using CK = ir::CommandKind;
  switch (c.kind) {
    case CK::PAUSE:
      if (state_ == S::EXECUTING) transition(S::PAUSED, "paused");
      break;
    case CK::RESUME:
      if (state_ == S::PAUSED) transition(S::EXECUTING, "resumed");
      break;
    case CK::ABORT:
    case CK::ESTOP:
      if (active_) {
        transition(S::ABORTING, c.kind == CK::ESTOP ? "estop" : "abort requested");
        transition(S::FAILED, c.kind == CK::ESTOP ? "estop complete" : "aborted");
        if (sinks_.on_ack) {
          ir::Ack ack{active_->id, ir::AckStatus::FAILED, "aborted", now_ms()};
          sinks_.on_ack(ack);
        }
      }
      break;
    case CK::CLEAR_FAULT:
      if (state_ == S::FAILED) {
        active_.reset();
        progress_ = 0.0;
        transition(S::IDLE, "fault cleared");
      }
      break;
  }
}

void MissionStateMachine::onCheckComplete(const ir::MissionCheckResult & r)
{
  using S = ir::MissionState;
  if (state_ != S::VALIDATING) return;
  if (!active_ || active_->id != r.mission_id) return;
  if (r.accepted) {
    transition(S::EXECUTING, "validated");
  } else {
    std::string reason = r.reasons.empty() ? "validation failed" : r.reasons.front();
    transition(S::FAILED, reason);
  }
}

void MissionStateMachine::onExecutionComplete(const std::string & mission_id)
{
  using S = ir::MissionState;
  if (state_ != S::EXECUTING) return;
  if (!active_ || active_->id != mission_id) return;
  progress_ = 1.0;
  transition(S::DONE, "complete");
}

void MissionStateMachine::onExecutionFailed(const std::string & mission_id, const std::string & reason)
{
  using S = ir::MissionState;
  if (!active_ || active_->id != mission_id) return;
  if (state_ == S::DONE || state_ == S::FAILED) return;
  transition(S::FAILED, reason);
}

}  // namespace vehicle_agent
