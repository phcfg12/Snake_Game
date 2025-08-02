#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#define spi0 0
#define bt1 3
#define bt2 4
#define bt3 5
#define bt4 6

uint8_t dMAX7219_BUFFER[8];
uint8_t dMAX7219_BUFFER_COLUMN[8];

uint8_t doDaiToiDa = 20;            // ĐỘ DÀI TỐI ĐA CỦA RẮN.
int8_t ranX[20];                    // TOẠ ĐỘ X CỦA RẮN.                // (ranX[0],ranY[0]) là tọa độ đầu của con rắn
int8_t ranY[20];                    // TOẠ ĐỘ Y CUẢ RẮN.
int8_t ranX_prev[20];               // TOẠ ĐỘ X CŨ CỦA RẮN.
int8_t ranY_prev[20];               // TOẠ ĐỘ Y CŨ CỦA RẮN.
uint8_t doDaiRan;                   // ĐỘ DÀI HIỆN TẠI CỦA RẮN.
uint8_t moiX, moiY;                 // TỌA ĐỘ CỦA CON MỒI
uint8_t found;                      // XEM TỌA ĐỘ CON MỒI CÓ TRÙNG VỚI TỌA ĐỘ RẮN KO
uint8_t check=0;                      // KIỂM TRA RẮN CÓ ĐỤNG VÀO CHÍNH NÓ HAY KO

uint8_t btn=0;
uint8_t on=1;                       // on=1: ĐANG CHẠY     on=0: LOSS
uint32_t start_time=0;
uint32_t speed;                     // TỐC ĐỘ CỦA RẮN
uint8_t state1=0;                   
uint8_t state2=0;
uint8_t state3=0;
uint8_t state4=0;

void sendData(uint8_t address, uint16_t value){                 // GỬI DATA
    uint8_t data[2];
    data[0]=address;
    data[1]=value;
    wiringPiSPIDataRW(spi0, data, 2);
}
void dMAX7219_SEND_DISPLAY(){
    for(int i = 0; i<8 ; i++){
        sendData(i+1, dMAX7219_BUFFER[i]);
    }
}
void dMAX7219_SET_PIXEL(uint8_t toaDoX, uint8_t toaDoY){        // SET 1 TỌA ĐỘ LED
    dMAX7219_BUFFER[toaDoY] |= (1 << toaDoX);
    dMAX7219_SEND_DISPLAY();
}
void dMAX7219_CLEAR_PIXEL(uint8_t toaDoX, uint8_t toaDoY){      // XÓA 1 TỌA ĐỘ LED
    dMAX7219_BUFFER[toaDoY] &= ~(1 << toaDoX);
    dMAX7219_SEND_DISPLAY();
}
void dMAX7219_CLEAR_DISPLAY(){
    for(int i = 0; i < 8; i++){
        dMAX7219_BUFFER[i] = 0x00;
    }
    for(int i = 0; i<8 ; i++){
        sendData(i+1, dMAX7219_BUFFER[i]);
    }
}
void TAO_MOI(){                                                 // TẠO CON MỒI VÀ KIỂM TRA CÓ TRÙNG VỚI RẮN KO
    int8_t availablePositions[64]; // Mảng lưu các vị trí có thể chọn
    int8_t numAvailable = 0;
    // Tạo danh sách các vị trí có thể chọn
    for (int8_t i = 0; i < 64; i++) {
        int8_t x = i % 8; // Tọa độ X
        int8_t y = i / 8; // Tọa độ Y
        int8_t isSnake = 0;
        for (int j = 0; j < doDaiRan; j++) {
            if (x == ranX[j] && y == ranY[j]) {
                isSnake = 1;
                break;
            }
        }
        if (!isSnake) {
            availablePositions[numAvailable++] = i; // Lưu vị trí không trùng vào mảng
        }
    }
    // Chọn ngẫu nhiên một vị trí từ các vị trí có thể chọn
    int8_t randomIndex = rand() % numAvailable;
    int8_t randomPosition = availablePositions[randomIndex];
    moiX = randomPosition % 8; // Tọa độ X của mồi
    moiY = randomPosition / 8; // Tọa độ Y của mồi
}
void checkSnake(){                                       // KIỂM TRA RẮN CÓ ĐỤNG VÀO CHÍNH NÓ KO
    for(int i=1;i<doDaiRan;i++){
        if((ranX[0]==ranX[i])&&(ranY[0]==ranY[i])){
            check=1;
            break;
        }
    }
    if(check==1){
        dMAX7219_CLEAR_DISPLAY();
        for(int i=0;i<8;i++){
            dMAX7219_SET_PIXEL(i,i);
            dMAX7219_SET_PIXEL(7-i,i);
        }
        state1=0;                   
        state2=0;
        state3=0;
        state4=0;
        on=0;
        found=1;
    }
}
void display_MOI(){
    dMAX7219_SET_PIXEL(moiX,moiY);
    usleep(100000);
    dMAX7219_CLEAR_PIXEL(moiX,moiY);
    usleep(100000);
}
void delay_ms(uint16_t t){                                      // HÀM DELAY SỬ DỤNG MILLIS()
    start_time=millis();
    while(millis()-start_time<t){
        display_MOI();
    }
}
void m_snake(uint8_t doDaiRan){                                 // KHỞI TẠO CHIỀU DÀI RẮN
    for(int i=0;i<doDaiRan;i++){
        ranX[i]=7;
        ranY[i]=doDaiRan-(i+1);
    }
    for(int i=doDaiRan;i<doDaiToiDa;i++){
        ranX[i] = ranY[i] = -1;
    }
}
void dSNAKE_VE_RAN(){                                           // VẼ RẮN THEO CÁC TỌA ĐỘ ranX,ranY
    for(int i=0; i<doDaiRan; i++){                              
        dMAX7219_CLEAR_PIXEL(ranX_prev[i], ranY_prev[i]);       // XÓA RẮN Ở VỊ TRÍ CŨ
    }
    for(int i=0; i<doDaiRan; i++){
        dMAX7219_SET_PIXEL(ranX[i], ranY[i]);                   // CẬP NHẬT RẮN
    }
}
void restart(){
    while(on==0){                                                  // NHẤN NÚT BẤT KÌ ĐỂ CHƠI LẠI
        if(state1==1){
            dMAX7219_CLEAR_DISPLAY();
            doDaiRan=3;
            speed=speed;
            m_snake(doDaiRan);                                          // TẠO RẮN
            dSNAKE_VE_RAN();
            TAO_MOI();
            check=0;
            state1=0;                   
            state2=0;
            state3=0;
            state4=0;
            on=1;
            break;
        }
    }
}
void snakeMove(){
    ranX_prev[0]=ranX[0];
    ranY_prev[0]=ranY[0];
    if(state1!=0){
        ranY[0]=ranY[0]+1;                  
        if(ranY[0]==8)  ranY[0]=0;
    }
    if(state2!=0){
        ranX[0]=ranX[0]-1;
        if(ranX[0]==-1)  ranX[0]=7;
    }
    if(state3!=0){
        ranY[0]=ranY[0]-1;
        if(ranY[0]==-1)  ranY[0]=7;
    }
    if(state4!=0){
        ranX[0]=ranX[0]+1;
        if(ranX[0]==8)  ranX[0]=0;
    }
    for(int i = 1; i<doDaiRan; i++){
        ranX_prev[i]=ranX[i];
        ranY_prev[i]=ranY[i];
        ranX[i]=ranX_prev[i-1];                             // LẤY TỌA ĐỘ TRƯỚC ĐÓ CỦA RẮN
        ranY[i]=ranY_prev[i-1];
        if(state1!=0){
            if(ranY[i]==8)  ranY[i]=0;
        }
        if(state2!=0){
            if(ranX[0]==-1)  ranX[0]=7;
        }
        if(state3!=0){
            if(ranY[0]==-1)  ranY[0]=7;
        }
        if(state4!=0){
            if(ranX[0]==8)  ranX[0]=0;
        }
    }
    dSNAKE_VE_RAN();
}
void button1(void){                                              // Y+
    if(state3==0)   state1=1;                                   // ĐỂ KHI ĐI THEO Y+ THÌ Y- KO TÁC ĐỘNG VÀO ĐƯỢC
    state2=0;
    state4=0;
}
void button2(void){                                             // X-
    if(state4==0)   state2=1;
    state1=0;
    state3=0;
}
void button3(void){                                             // Y-
    if(state1==0)   state3=1;
    state2=0;
    state4=0;
}
void button4(void){                                             // X+
    if(state2==0)   state4=1;
    state1=0;
    state3=0;
}
void control(){                     // HÀM ĐIỀU KHIỂN RẮN                                       
    //restart();
    if((on==1)&&((state1==1)||(state2==1)||(state3==1)||(state4==1))){      // ĐANG CHẠY VÀ NHẤN BẤT KÌ NÚT NÀO
        snakeMove();
        if((check==0)&&(ranX[0]==moiX)&&(ranY[0]==moiY)){                   // NẾU RẮN ĂN ĐƯỢC MỒI THÌ doDaiRan TĂNG THÊM 1 VÀ TẠO TỌA ĐỘ MỚI CỦA MỒI
            TAO_MOI();
            doDaiRan+=1;
            for(int i = 1; i<doDaiRan; i++){
                ranX[i]=ranX_prev[i-1];                             // LẤY TỌA ĐỘ TRƯỚC ĐÓ CỦA RẮN
                ranY[i]=ranY_prev[i-1];
                if(state1!=0){
                    if(ranY[i]==8)  ranY[i]=0;
                }
                if(state2!=0){
                    if(ranX[0]==-1)  ranX[0]=7;
                }
                if(state3!=0){
                    if(ranY[0]==-1)  ranY[0]=7;
                }
                if(state4!=0){
                    if(ranX[0]==8)  ranX[0]=0;
                }
            }
            dSNAKE_VE_RAN();
            display_MOI(); 
        }
        checkSnake();
        restart();
        delay_ms(speed);
    }
    if((on==1)&&(state1!=1)&&(state2!=1)&&(state3!=1)&&(state4!=1)){        
        display_MOI();
    }
}

int main(void){
    wiringPiSetup();
    pinMode(bt1, INPUT);
    pinMode(bt2, INPUT);
    pinMode(bt3, INPUT);
    pinMode(bt4, INPUT);
    wiringPiSPISetup(spi0, 800000);
    //setup max7219
    sendData(0x09, 0x00);   //decode mode
    sendData(0x0A, 0x08);   //intensity
    sendData(0x0B, 7);  //scan limit
    sendData(0x0C, 1);  //normal mode
    sendData(0x0F, 0);  //turn off display test

    wiringPiISR(bt1, INT_EDGE_RISING, &button1);
    wiringPiISR(bt2, INT_EDGE_RISING, &button2);
    wiringPiISR(bt3, INT_EDGE_RISING, &button3);
    wiringPiISR(bt4, INT_EDGE_RISING, &button4);

    dMAX7219_CLEAR_DISPLAY();
    srand(time(NULL));
    doDaiRan=3;
    speed=250;
    m_snake(doDaiRan);                                          
    dSNAKE_VE_RAN();                        // TẠO RẮN
    TAO_MOI();                              // TẠO MỒI
    while(1){
        control();
    }
    return 0;
}
