#include "kernel/types.h"
#include "kernel/stat.h"
#include "user/user.h"

int
main(int argc, char *argv[])
{
    int algo = atoi(argv[1]);
    int is_preemptive = atoi(argv[2]);
    int a = atoi(argv[3]);

    int ret = chsched(algo, is_preemptive, a);
    if (ret == 0){
        printf("algorithm: %s\n", (algo==0?"SJF":"CFS"));
        if (algo == 0) {
            printf("is_preemptive: %d\n", is_preemptive);
            printf("a: %d\n", a);
        }
    }
    printf("return code: %d\n", ret);
    exit(0);
}

