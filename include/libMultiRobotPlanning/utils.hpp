#pragma once

static constexpr bool kProduction = false;
static constexpr int kAssertFailReturnCode = 1;

#ifndef CHECK
#define CHECK(exp)                                                      \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" failed!" << std::endl;                             \
    exit(kAssertFailReturnCode);                                        \
  }
#endif

#ifndef CHECK_EQ
#define CHECK_EQ(exp1, exp2) CHECK_PRINT_VALS((exp1) == (exp2), exp1, exp2);
#endif

#ifndef CHECK_NE
#define CHECK_NE(exp1, exp2) CHECK_PRINT_VALS((exp1) != (exp2), exp1, exp2);
#endif

#ifndef CHECK_GT
#define CHECK_GT(exp1, exp2) CHECK_PRINT_VALS((exp1) > (exp2), exp1, exp2);
#endif

#ifndef CHECK_LT
#define CHECK_LT(exp1, exp2) CHECK_PRINT_VALS((exp1) < (exp2), exp1, exp2);
#endif

#ifndef CHECK_GE
#define CHECK_GE(exp1, exp2) CHECK_PRINT_VALS((exp1) >= (exp2), exp1, exp2);
#endif

#ifndef CHECK_LE
#define CHECK_LE(exp1, exp2) CHECK_PRINT_VALS((exp1) <= (exp2), exp1, exp2);
#endif

#define CHECK_MSG(exp, msg)                                             \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" failed with message: " << msg << std::endl;        \
    exit(kAssertFailReturnCode);                                        \
  }

#define CHECK_PRINT_VAL(exp, val)                                       \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" (value: " << (val) << ")  failed!" << std::endl;   \
    exit(kAssertFailReturnCode);                                        \
  }

#define CHECK_PRINT_VALS(exp, val1, val2)                               \
  if (!(exp)) {                                                         \
    std::cerr << __FILE__ << ":" << __LINE__ << " Assertion \"" << #exp \
              << "\" (value: " << (val1) << " vs value: " << (val2)     \
              << ")  failed!" << std::endl;                             \
    exit(kAssertFailReturnCode);                                        \
  }

#define FINITE(exp) CHECK_PRINT_VAL(std::isfinite(exp), (exp))

#define FINITE_MSG(exp, msg) CHECK_MSG(std::isfinite(exp), msg)

#define FINITE_VEC(exp) CHECK_PRINT_VAL((exp).allFinite(), (exp));

#define NP_CHECK(exp) \
  if (!kProduction) { \
    CHECK(exp);       \
  }

#define NP_CHECK_MSG(exp, msg) \
  if (!kProduction) {          \
    CHECK_MSG(exp, msg);       \
  }

#define NP_CHECK_VAL(exp, val) \
  if (!kProduction) {          \
    CHECK_PRINT_VAL(exp, val); \
  }

#define NP_CHECK_EQ(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_EQ(exp1, exp2);       \
  }

#define NP_CHECK_NE(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_NE(exp1, exp2);       \
  }

#define NP_CHECK_GT(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_GT(exp1, exp2);       \
  }

#define NP_CHECK_LT(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_LT(exp1, exp2);       \
  }

#define NP_CHECK_GE(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_GE(exp1, exp2);       \
  }

#define NP_CHECK_LE(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_LE(exp1, exp2);       \
  }

#define NP_FINITE(exp) \
  if (!kProduction) {  \
    FINITE(exp);       \
  }

#define NP_FINITE_MSG(exp, msg) \
  if (!kProduction) {           \
    FINITE_MSG(exp, msg);       \
  }

#define NP_FINITE_VEC(exp) \
  if (!kProduction) {      \
    FINITE_VEC(exp);       \
  }

#define NP_NOT_NULL(exp)     \
  if (!kProduction) {        \
    CHECK((exp) != nullptr); \
  }
  