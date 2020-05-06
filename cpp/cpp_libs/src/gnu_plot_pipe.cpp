
// http://stackoverflow.com/questions/24121617/update-gnuplot-dataset-without-files-from-c

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>     // usleep
using namespace std;

int main(){
// Code for gnuplot pipe
FILE *pipe = popen("gnuplot -persist", "w");

// set axis ranges
fprintf(pipe,"set xrange [0:11]\n");
fprintf(pipe,"set yrange [0:11]\n");

int b;
for (int a=0;a<10;a++) // 10 plots
{
    fprintf(pipe,"plot '-' using 1:2 \n");  // so I want the first column to be x values, second column to be y
    for (b=0;b<10;b++)  // 10 datapoints per plot
    {
        fprintf(pipe, "%d %d \n",a,b);  // passing x,y data pairs one at a time to gnuplot
    }
    fprintf(pipe,"e \n");    // finally, e
    fflush(pipe);   // flush the pipe to update the plot
    usleep(1000000);// wait a second before updating again
}

// Don't forget to close the pipe
fclose(pipe);
return 0;
}


