#include "platform/ubuntu/main.h"
#include <stdio.h>
#include <string.h>
#include "application.hpp"

int main(void) {
    printf("Hello World!\n");
    application_entry_point();
    printf("Done!\n");
    return 0;
}
