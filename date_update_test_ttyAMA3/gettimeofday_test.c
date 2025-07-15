

#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

//  int gettimeofday(struct timeval *tv, struct timezone *tz);
int main()
{
    struct timeval mytime;

    // 현재 시간을 얻어온다.
    gettimeofday(&mytime, NULL);
    printf("%ld:%ld\n", mytime.tv_sec, mytime.tv_usec);

    printf("date:%ld\n", mytime.tv_sec);
	system("date");
//      // 시간을 1시간 뒤로 되돌려서 설정한다.
//      mytime.tv_sec -= 3600;
//  //  //      printf("%ld:%ld\n", mytime.tv_sec, mytime.tv_usec);
//  //      printf("date:%ld:%ld\n", mytime.tv_sec, mytime.tv_usec);
//      settimeofday(&mytime, NULL);
    return 0;
}


