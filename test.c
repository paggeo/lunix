#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <string.h>

int main() {
	char buf1[20];
	char buf2[20];
	ssize_t num1, num2;
	int fd;

	/**************************************************************/
	printf("\nRead 2 bytes and then the rest in one process\n");

	strcpy(buf1, "");
	strcpy(buf2, "");
	fd = open("/dev/lunix0-temp", O_RDONLY);
	num1 = read(fd, buf1, 2);
	num2 = read(fd, buf2, 10);

	printf("First: %s Second: %s", buf1, buf2);
	close(fd);
	/*************************************************************/
	printf("\nRead 3 bytes from child process and 3 from parent (enter would be the seventh)\n");

	fd = open("/dev/lunix0-temp", O_RDONLY);
	char buf3[20] = "";
	char buf4[20] = "";

	int pid = fork();
	if(pid == -1) printf("FAIL\n");

	if(!pid){
		num1 = read(fd, buf3, 3);
		printf("Child read: %s\n", buf3);
		exit(0);
	}
	else{
		num2 = read(fd, buf4, 3);
		wait(NULL);	// wait for child to finish
		printf("Parent read: %s\n", buf4);
	}
	close(fd);
	/*********************************************************/
	printf("\nShow all the measures\n");
	char temp0[20] = "", batt0[20] = "", light0[20] = "";
	char temp1[20] = "", batt1[20] = "", light1[20] = "";
	int size = 20;

	ssize_t fdnew[2][3];
	fdnew[0][0] = open("/dev/lunix0-temp", O_RDONLY);
	fdnew[0][1] = open("/dev/lunix0-batt", O_RDONLY);
	fdnew[0][2] = open("/dev/lunix0-light", O_RDONLY);
	fdnew[1][0] = open("/dev/lunix1-temp", O_RDONLY);
	fdnew[1][1] = open("/dev/lunix1-batt", O_RDONLY);
	fdnew[1][2] = open("/dev/lunix1-light", O_RDONLY);

	read(fdnew[0][0], temp0, size);
	read(fdnew[0][1], batt0, size);
	read(fdnew[0][2], light0, size);
	read(fdnew[1][0], temp1, size);
	read(fdnew[1][1], batt1, size);
	read(fdnew[1][2], light1, size);

	printf("Sensor 0 (temp, batt, light): %s %s %s\n", temp0, batt0, light0);
	printf("Sensor 1 (temp, batt, light): %s %s %s\n", temp1, batt1, light1);
	
	close(fdnew[0][0]);
	close(fdnew[0][1]);
	close(fdnew[0][2]);
	close(fdnew[1][0]);
	close(fdnew[1][1]);
	close(fdnew[1][2]);
}
