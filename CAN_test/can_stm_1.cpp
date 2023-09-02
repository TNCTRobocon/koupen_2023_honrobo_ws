/* USER CODE BEGIN 2 */
CAN_FilterTypeDef filter;
    filter.FilterIdHigh         = 0;                        // フィルターID(上位16ビット)
    filter.FilterIdLow          = 0;                        // フィルターID(下位16ビット)
    filter.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
    filter.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
    filter.FilterBank           = 14;                        // フィルターバンクNo
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
    filter.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
    filter.FilterActivation     = ENABLE;                   // フィルター無効／有効
CAN_ConfigFilter(&hcanx,&filter);
/* USER CODE END 2 */


/* USER CODE BEGIN WHILE */
while (1){
    uint8_t RxData[8];
    int rcv_data[8];
    CAN_HandleTypeDef RxHeader;

    /* USER CODE END WHILE */
    while(!can_rx_flag){
        ;;
    }
    if(HAL_CAN_GetMessage(hcan, CAN_RX_FIFO0,&RxHeader,RxData) == HAL_OK){
        id = (RxHeader.IDE == CAN_ID_FIFO0)? RxHeader.StdId : RxHeader.ExtId;
        dlc = RxHeader.DLC;
        for(i = 0;i < 8;i++){
            data[i] = RxData[0];
        }
}

/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

bool can_rx_flag = False;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    can_rx_flag = True;
}
