#ifndef TEST_H
#define TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure for a test scenario.
 * Each test defines an init() and update() function.
 */
typedef struct {
    const char *name;             ///< Name of the test (for logging/debug)
    void (*init)(void);           ///< Called once at test start
    void (*update)(float dt_s);   ///< Called each control cycle (dt in seconds)
} Test_t;

/**
 * @brief Set and get the currently active test.
 */
void Test_SetActive(const Test_t *test);
const Test_t *Test_GetActive(void);

/**
 * @brief Initialize test system (optional).
 */
void Test_InitSystem(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_H */
