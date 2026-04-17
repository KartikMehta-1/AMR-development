#include "test.h"

/* Active test pointer */
static const Test_t *active_test = 0;

void Test_SetActive(const Test_t *test)
{
    active_test = test;
    if (active_test && active_test->init)
        active_test->init();
}

const Test_t *Test_GetActive(void)
{
    return active_test;
}

void Test_InitSystem(void)
{
    active_test = 0;
}
