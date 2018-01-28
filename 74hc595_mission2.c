#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "8x8font.h"
#include <pigpio.h>
#include "74hc595_functions.h"

#define MAX 1024
#define BOARD_ROW 8     // board의 Row
#define PADDING 1;     // 문자간의 기본간격



void dot(int row, int col);
void callback(void);

int cnt;
uint8_t *board[BOARD_ROW];      // board
int     board_size;
int count=0;
// board에 문자 writing, margin-이전 문자와의 간격
void boardWriter(int fontidx, int margin);

 
int main(int argc, char ** argv[])
{  

    int ret;
    ret = init();
    int fd;
    int readn;
    int writen;

    char buf[MAX];
    char buf2[256]="";
    int i,j;
    int total_col;
    int total_col2;


    int value=0;
    ///////////////////// file streaming/////////////// 
   
    fd = open("/home/pi/robotcode/ch03/data2.txt", O_RDWR);

    fscanf(stdin,"%[^\n]",buf2,sizeof(buf));

    //Open file and Get File Descripter//

    if(fd==-1){
        printf("File open failed.....\n");
        return 1;
    }

    //write buf2 int fd
    writen = write(fd, buf2, strlen(buf2));
    printf("fd is written with buf\n");

    //Set FD cursor to the starting point
    int current = lseek(fd, 0, SEEK_SET);

    memset(buf, 0x00, MAX);
    readn = read(fd, buf, sizeof(buf2));

    //////////////////////////////////////////////////////
    //여기는 Information으로 변환하는 과정
    while(count<20){
    if(buf[count]>=48 && buf[count]<=57){
        value=buf[count]+4;
        boardWriter(value,0);
    }
     else if(buf[count]>=97 && buf[count]<=122){
        value=buf[count]-71;
        boardWriter(value,0);
    
    }
     else if(buf[count]>=65 && buf[count]<=90){
        value=buf[count]-65;
        boardWriter(value,0);
    
    }
    else if(buf[count]==32){
        value=buf[count]+30;
        boardWriter(value,0);
    }
    else
        count++;
    }
    //여기는 실제로 화면에 글자를 띄우는 작업 + dot matrix scrolling
    printf("font data : %d\n", FONT88_LEN);
    printf("board info : %d X %d\n", board_size, BOARD_ROW);
    printf("< B O A R D >\n");
    for(int i = 0 ; i < BOARD_ROW ; i++){
        for(int j = 0 ; j < board_size ; j++)
            printf("%c ", board[i][j]); 
            printf("\n");
        
    }
    //여기서 부터가 실제 dot matrix 
    
    gpioSetTimerFunc(0, 100, callback);
//    total_col = sizeof(board) / 8;
//    printf("total_col= %d",total_col);

    while(1){
        for(i = 0 ; i < 8 ; i++)
            for(j = 0 ; j < 8 ; j++)
                if(board[i][(j+cnt)%board_size] == 'o')
                    dot(i+1,j+1);
        
             }

    close(fd);
    release();
    return 0;
}

void boardWriter(int fontidx, int margin)
{
    count++;
    int i, j;
    static int board_offset = 0; // 현재 문자의 offset
    uint8_t mask;
    uint8_t *new_board[BOARD_ROW];

    int new_board_size = board_offset + margin + FONT_COL + PADDING;

    for(i = 0 ; i < BOARD_ROW ; i++){
        new_board[i] = (uint8_t*)calloc(1, new_board_size);
        memset(new_board[i],0 , new_board_size);
        memcpy(new_board[i], board[i], board_size);
        free(board[i]);
        board[i] = new_board[i];
    }

    board_size = new_board_size;
    board_offset += margin;

    // writing on the board
    for(i = 0 ; i < FONT_ROW ; i++){      // row
        for(j = 0 ; j < FONT_COL ; j++){  // col
            mask = 0b1 << (7-j);
            if((mask & FONT88[fontidx][i]))
                board[i][j+board_offset] = 'o';
            else
                board[i][j+board_offset] = ' ';
        }
    }
    board_offset = board_size;
}
void dot(int row, int col)
{
    uint8_t row8, col8;
    uint16_t tmp;
    row8 = ~(1 << (8-row));
    col8 = 1 << (8-col);
    tmp = (row8<<8) | col8;
    set16(tmp);
}
void callback(void)
{
//	printf("500 milliseconds have elapsed\n");
	cnt++;	
}
