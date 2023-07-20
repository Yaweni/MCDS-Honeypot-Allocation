
#ifndef STOCHASTICGAMESCPP_FAST_PROFILE_H
#define STOCHASTICGAMESCPP_FAST_PROFILE_H

#include <string>
#include <memory>

#define CONCAT_INTERNAL(a, b) a ## b
#define CONCAT(a, b) CONCAT_INTERNAL(a, b)

/**
 * Idea:
 *   - construct profiling_handle and initialize the profiling block
 *   - destroy profiling_handle at the end of profiling block to terminate profiling
 *   - if CODE uses return statement, the profiling block is terminated by destruction of profiling_handle on stack
 */

#ifdef PROFILING
#define PROFILED(NAME) std::unique_ptr<profiling_handle> CONCAT(prof, __LINE__)(new profiling_handle(NAME));
#define PROFILE(NAME, CODE) \
  std::unique_ptr<profiling_handle> CONCAT(prof, __LINE__)(new profiling_handle(NAME)); \
  CODE \
  CONCAT(prof, __LINE__).reset(nullptr);
#define PROFILING_INFO(OUT) profiling_info(OUT)
#else
#define PROFILED(NAME)
#define PROFILE(NAME, CODE) CODE
#define PROFILING_INFO(OUT)
#endif

class profiling_handle {
public:
    profiling_handle(std::string key);
    ~profiling_handle();
};

void profiling_info(std::ostream& out);

#endif //STOCHASTICGAMESCPP_FAST_PROFILE_H
#pragma once
