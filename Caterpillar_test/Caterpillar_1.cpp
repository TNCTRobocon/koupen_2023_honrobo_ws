#include <iostream>

const int motor_amp = 100;

void recv_can(void)
void caterpillar_calc(uint8_t *axis_data[4],int *mtr_data[2]);
void motor_pwm_calc(int *mtr_data[2]);


void recv_can(void){
    uint8_t axis_data[4]; //1:左スティック上下 2:左スティック右左 3:右スティック上下 4:右スティック右左 0~256
    int mtr_data[2]; //1:右 2:左
    for(int i = 0;i < 4;i++){ //0初期化
        axis_data[i] -= 127;
    }

    caterpillar_calc(&axis_data,&mtr_data);
    motor_pwm_calc(&mtr_data);

    __HAL_TIM_SET_COMPARE(&htimX,TIM_CHANNELX,mtr_data[0] * motor_amp)
    __HAL_TIM_SET_COMPARE(&htimX,TIM_CHANNELX,mtr_data[0] * motor_amp)
    __HAL_TIM_SET_COMPARE(&htimX,TIM_CHANNELX,mtr_data[1] * motor_amp)
    __HAL_TIM_SET_COMPARE(&htimX,TIM_CHANNELX,mtr_data[1] * motor_amp)

}

void caterpillar_calc(uint8_t *axis_data[4],int *mtr_data[2]){
    int abs_value;

    mtr_data[0] = axis_data[0];
    mtr_data[1] = axis_data[0];

    abs_value = std::abs(axis_data[1]);

    if (data[1] > 0){ //右
        mtr_data[0] -= abs_value;
        mtr_data[1] += abs_value;
    }
    else if(data[1] < 0){ //左
        mtr_data[0] += abs_value;
        mtr_data[1] -= abs_value;
    }
}

void motor_pwm_calc(int *mtr_data[2]){
    if(mtr_data[0] < 0){
        mtr_data[0] *= -1;
        HAL_GPIO_WritePin(GPIOX,DIRECTIONX_Pin,GPIO_PIN_RESET);　//2つとも設定する
    }
    else{
        HAL_GPIO_WritePin(GPIOX,DIRECTIONX_Pin,GPIO_PIN_SET); //2つとも設定する
    }
    if(mtr_data[1] < 0){
        mtr_data[1] *= -1;
        HAL_GPIO_WritePin(GPIOX,DIRECTIONX_Pin,GPIO_PIN_RESET); //2つとも設定する
    }
    else{
        HAL_GPIO_WritePin(GPIOX,DIRECTIONX_Pin,GPIO_PIN_SET); //2つとも設定する
    }
    mtr_data[0] *= motor_amp;
    mtr_data[1] *= motor_amp;
}
