#include "hacdcpf/market/market_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace hacdcpf::market {
namespace {

double cost_secant_slope(const Generator& generator,
                         double left,
                         double right) {
  return generator.cost_c1 + std::max(0.0, generator.cost_c2) * (left + right);
}

bool policy_uses_markup(BehaviorPolicyType type) {
  return type == BehaviorPolicyType::FixedMarkup ||
         type == BehaviorPolicyType::MarkupAndWithholding;
}

bool policy_uses_withholding(BehaviorPolicyType type) {
  return type == BehaviorPolicyType::CapacityWithholding ||
         type == BehaviorPolicyType::MarkupAndWithholding;
}

std::string default_participant_id(const Generator& generator, int position) {
  if (generator.index != 0) {
    return "generator:" + std::to_string(generator.index);
  }
  return "generator_position:" + std::to_string(position);
}

}  // namespace

OfferSubmission submit_participant_offers(
    const HybridPowerSystem& system,
    const std::vector<MarketParticipant>& requested_participants,
    int segment_count) {
  const int K = std::max(1, segment_count);
  OfferSubmission submission;
  submission.participants = requested_participants;

  std::unordered_set<std::string> participant_ids;
  std::unordered_map<int, size_t> owner_by_generator;
  for (size_t p = 0; p < submission.participants.size(); ++p) {
    auto& participant = submission.participants[p];
    if (participant.participant_id.empty()) {
      throw std::invalid_argument("market participant id cannot be empty");
    }
    if (!participant_ids.insert(participant.participant_id).second) {
      throw std::invalid_argument("duplicate market participant id: " +
                                  participant.participant_id);
    }
    if (participant.participant_name.empty()) {
      participant.participant_name = participant.participant_id;
    }
    for (int position : participant.generator_positions) {
      if (position < 0 ||
          position >= static_cast<int>(system.ac.generators.size())) {
        throw std::invalid_argument("participant " + participant.participant_id +
                                    " references an invalid generator position");
      }
      if (!system.ac.generators[static_cast<size_t>(position)].in_service) {
        throw std::invalid_argument("participant " + participant.participant_id +
                                    " owns an out-of-service generator position");
      }
      if (!owner_by_generator.emplace(position, p).second) {
        throw std::invalid_argument("generator position " +
                                    std::to_string(position) +
                                    " is owned by more than one participant");
      }
    }
  }

  // Complete the ownership contract explicitly: every active generator gets a
  // cost-based participant when it was not assigned by the caller.
  for (int position = 0;
       position < static_cast<int>(system.ac.generators.size()); ++position) {
    const auto& generator = system.ac.generators[static_cast<size_t>(position)];
    if (!generator.in_service || owner_by_generator.count(position) != 0) continue;
    MarketParticipant participant;
    const std::string base_id = default_participant_id(generator, position);
    participant.participant_id = base_id;
    int suffix = 1;
    while (participant_ids.count(participant.participant_id) != 0) {
      participant.participant_id =
          base_id + "_auto_" + std::to_string(suffix++);
    }
    participant_ids.insert(participant.participant_id);
    participant.participant_name = generator.name.empty()
        ? participant.participant_id
        : generator.name;
    participant.generator_positions = {position};
    const size_t owner = submission.participants.size();
    submission.participants.push_back(std::move(participant));
    owner_by_generator.emplace(position, owner);
    if (!requested_participants.empty()) {
      submission.warnings.push_back(
          "Generator position " + std::to_string(position) +
          " was unowned and received a default cost-based participant.");
    }
  }

  for (int position = 0;
       position < static_cast<int>(system.ac.generators.size()); ++position) {
    const auto& generator = system.ac.generators[static_cast<size_t>(position)];
    if (!generator.in_service) continue;
    const auto owner_it = owner_by_generator.find(position);
    if (owner_it == owner_by_generator.end()) {
      throw std::logic_error("active generator ownership completion failed");
    }
    const auto& participant = submission.participants[owner_it->second];
    const auto& policy = participant.behavior;
    const double raw_markup = policy_uses_markup(policy.type)
        ? policy.energy_markup_fraction
        : 0.0;
    const double raw_commitment_markup = policy_uses_markup(policy.type)
        ? policy.commitment_markup_fraction
        : 0.0;
    const double raw_withholding = policy_uses_withholding(policy.type)
        ? policy.capacity_withholding_fraction
        : 0.0;
    const double markup = std::clamp(raw_markup, 0.0, 10.0);
    const double commitment_markup =
        std::clamp(raw_commitment_markup, 0.0, 10.0);
    const double withholding = std::clamp(raw_withholding, 0.0, 0.95);
    if (markup != raw_markup || commitment_markup != raw_commitment_markup ||
        withholding != raw_withholding) {
      submission.warnings.push_back(
          "Behaviour parameters were clamped for participant " +
          participant.participant_id + ".");
    }

    GeneratorOffer offer;
    offer.participant_id = participant.participant_id;
    offer.generator_position = position;
    offer.generator_index = generator.index;
    offer.generator_name = generator.name.empty()
        ? "Generator " + std::to_string(generator.index)
        : generator.name;
    offer.minimum_output_mw = std::max(0.0, generator.pmin_mw);
    offer.physical_maximum_output_mw =
        std::max(offer.minimum_output_mw, generator.pmax_mw);
    const double flexible_capacity =
        offer.physical_maximum_output_mw - offer.minimum_output_mw;
    offer.offered_maximum_output_mw = offer.minimum_output_mw +
        flexible_capacity * (1.0 - withholding);
    offer.minimum_output_cost_per_hour =
        (generator.cost_c2 * offer.minimum_output_mw * offer.minimum_output_mw +
         generator.cost_c1 * offer.minimum_output_mw) * (1.0 + markup);
    offer.no_load_price_per_hour =
        generator.cost_c0 * (1.0 + commitment_markup);
    offer.startup_price = generator.startup_cost * (1.0 + commitment_markup);
    offer.shutdown_price = generator.shutdown_cost * (1.0 + commitment_markup);
    offer.upward_reserve_price_per_mwh =
        std::max(0.0, policy.upward_reserve_price_per_mwh);

    const double width = flexible_capacity * (1.0 - withholding) /
                         static_cast<double>(K);
    offer.energy_segments.reserve(static_cast<size_t>(K));
    for (int k = 0; k < K; ++k) {
      const double left = offer.minimum_output_mw + width * static_cast<double>(k);
      const double price = cost_secant_slope(generator, left, left + width) *
                           (1.0 + markup);
      offer.energy_segments.push_back({width, price});
    }
    submission.offers.push_back(offer);
    submission.actions.push_back(BehaviorAction{
        participant.participant_id,
        position,
        generator.index,
        markup,
        withholding,
        offer.physical_maximum_output_mw,
        offer.offered_maximum_output_mw,
        offer.physical_maximum_output_mw - offer.offered_maximum_output_mw});
  }
  return submission;
}

std::vector<GeneratorOffer> make_cost_based_offers(
    const HybridPowerSystem& system,
    int segment_count) {
  return submit_participant_offers(system, {}, segment_count).offers;
}

}  // namespace hacdcpf::market
