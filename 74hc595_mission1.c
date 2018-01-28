#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "8x8font.h"

#define BOARD_ROW 8     // board의 Row
#define PADDING 1;     // 문자간의 기본간격

uint8_t *board[BOARD_ROW];      // board
int     board_size;
int count=0;
// board에 문자 writing, margin-이전 문자와의 간격
void boardWriter(int fontidx, int margin);

int main(void)
{   
    char input[10];
    int write=0;
    printf("please input:");
    scanf("%[^\n]",input);
   while(count<10){
    if(input[count]>=48 && input[count]<=57){
        write=input[count]+4;
        boardWriter(write,0);
    }
     else if(input[count]>=97 && input[count]<=122){
        write=input[count]-71;
        boardWriter(write,0);
    
    }
     else if(input[count]>=65 && input[count]<=90){
        write=input[count]-65;
        boardWriter(write,0);
    
    }
    else if(input[count]==32){
        write=input[count]+30;
        boardWriter(write,0);
    }
    else
        count++;
}

    printf("font data : %d\n", FONT88_LEN);
    printf("board info : %d X %d\n", board_size, BOARD_ROW);
    printf("< B O A R D >\n");
    for(int i = 0 ; i < BOARD_ROW ; i++){
        for(int j = 0 ; j < board_size ; j++)
            printf("%c ", board[i][j]); 
            printf("\n");
        
    }

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

