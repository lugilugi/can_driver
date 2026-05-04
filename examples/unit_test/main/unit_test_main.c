#include <stdio.h>

#include "unity.h"
#include "unity_test_runner.h"

void app_main(void)
{
    printf("\nCAN Unity tests ready.\n");
    printf("Press Enter to list tests, then run [can] or an index.\n");
    unity_run_menu();
}
