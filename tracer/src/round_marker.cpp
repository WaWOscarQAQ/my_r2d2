// Round-boundary marker CLI for the fuzzing harness.
//
// Attaches to an already-initialized tracer shared memory object and
// appends one RoundBoundary record carrying the given round id. The paper
// analyzes "the current callback trace" after each payload execution but
// does not disclose a boundary event format; delimiting rounds with an
// in-ring marker is a reproduction choice that lets the Rust reader
// attribute asynchronously written events to the round they belong to.

#include "tracer/tracers.h"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: round_marker <shm_name> <round_id>" << std::endl;
    return 2;
  }
  const std::string shm_name = argv[1];
  std::uint32_t round_id;
  try {
    const unsigned long parsed = std::stoul(argv[2]);
    round_id = static_cast<std::uint32_t>(parsed);
  } catch (const std::exception&) {
    std::cerr << "round_marker: invalid round id: " << argv[2] << std::endl;
    return 2;
  }

  try {
    tracer::attach(shm_name.c_str());
    tracer::round_boundary(round_id, tracer::now_ns());
  } catch (const std::exception& error) {
    std::cerr << "round_marker: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
