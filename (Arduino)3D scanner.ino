//====Include====
  // #include <MPU6050.h>
  #include <ADNS9800.h>
  #include <ButterworthFilter.h>
  #include <_Adafruit_MPU6050.h>

  // #include <KalmanFilter.h>
//=============== 

//====Define====

 //  需要校正時再開(有問題，先不使用)
  //#define CALIBRATE_MPU6050



 //  OneEquipment(MPU or ADNS)、TwoEquipment(MPU + ADNS)
  // #define OneEquipment    
  #define TwoEquipment
 //  選擇感測器
  #define MPU_ENABLE
  #define ADNS_ENABLE
 // 輸出顯示數據
  // #define LINEAR_ACCEL
  // #define LINEAR_VELOCITY
  // #define LINEAR_POSITION

  #ifdef MPU_ENABLE
    Adafruit_MPU6050 mpu;
  #endif
  #ifdef ADNS_ENABLE
    ADNS9800 adns;
  #endif
//==============  

//====mpu6050(變量、濾波類型)=======
  // 原始數據
    sensors_event_t a, g, temp;
  //  四元數
    static float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
  //  角度
    EulerAnglesStruct Angles;
    float AngleData[3];
  //  存放MPU6050校正偏差值
    int16_t offset[6];  
  //  濾波變量與類型
    volatile float Accel_X_Filted;
    volatile float Accel_Y_Filted;
    volatile float Accel_Z_Filted;
    volatile float Gyro_X_Filted;
    volatile float Gyro_Y_Filted;
    volatile float Gyro_Z_Filted;
    VectorFloat  Gra_RAW;
    VectorFloat  Gra_Filtered;
    float prelinear_Acc_X;
    float prelinear_Acc_Y;
    float prelinear_Acc_Z;
    float prelinear_VEL_X;
    float prelinear_VEL_Y;
    float prelinear_VEL_Z;
    float prelinear_Pos_X = 0;
    float prelinear_Pos_Y = 0;
    float prelinear_Pos_Z = 0;

    enum RawDataMode {
        RawData,
        Normalized,
        gravitational_accel,
        DegPerSec_gyro
    }; 
    enum FilterMode {
        MAF,
        TD_Lowpass,
        TD_Highpass,
        Butterworth_LowPass,
        Butterworth_HighPass,
        Butterworth_BandPass,
        No_Filted
    }; 
    // MPU6050單位、濾波類型、空間定位濾波選擇
      RawDataMode Accel_RawData         = Normalized;
      RawDataMode Gyro_RawData          = DegPerSec_gyro;
      FilterMode Accel_RawData_Filted   = Butterworth_LowPass;
      FilterMode Gyro_RawData_Filted    = Butterworth_LowPass;
      FilterMode Linear_Accel_Filted    = No_Filted; //  高通消除飄移(drift)
      FilterMode Linear_Velocity_Filted = No_Filted;
      FilterMode Linear_Position_Filted = No_Filted;



//====ADNS9800(變量、規格參數)====== 
  // 變量宣告
    #ifdef ADNS_ENABLE
    uint8_t pixel[900]; //影像像素
    static  int16_t dx = 0; // int16_t
    static  int16_t dy = 0; 
    #endif

  // 規格設定(晶片、濾波)
    uint16_t FrameRateMin = 5000; //  4000, frame rate = 762 時，可檢測低反射的待測面
    uint16_t Shutterrate = 100;   //  9000
    unsigned int FR;              //  幀率
    int16_t  threshold = 200;     //  位移閾值
    uint32_t Brightness = 0;      //  初始光亮度，分析後得出 56 為最佳
    uint32_t MaxBrightness = 100; //  自定義的光強度最大值
    uint8_t  PixelSUM;         //  亮度總合
    uint8_t  Squal;            //  表面特徵品質
    uint8_t  set_Lift_Detec = 27; //  高度檢測(LOD)，離地多高就不執行位移檢測
    uint8_t  get_Lift_Detec;   //  離地檢測高度
    uint8_t  set_CPI = 40;     //  解析度
    unsigned int get_CPI;      //  CPI
    uint8_t Y_CPI = 1;


  // 判斷式
    bool initialComp = 0;      //  初始化完成
    bool innitialADNS = 1;      //  點雲掃描模式初始化
    bool innitialFrameCap = 0;  //  影像擷取初始化
    bool MotionReady = 0;       //  ADNS位移數據已準備好
    bool ScanReady = 0;         //  開始掃描 / 規格輸出，按鈕中鍵觸發
    bool isFrameCapMode = 0;    //  是否為影像擷取模式
    bool isChecked_ADNS_specificatio = 0;   //  讀取ADNS規格
    bool ischangedFR = 0;             //  是否要開啟可更改快門或FR功能
    bool Shutterrate_beChanged = 1;   //  更改快門頻率
    bool FrameRateMin_beChanged = 1;  //  更改幀率下限頻率
    bool LEDstatus = 1;               //  LED狀態，預設開啟
    bool ResolutionMode = 0;        //  XY解析度狀態 (0 = 同步 / 1 = 非同步)
     
  // 上一刻狀態
    // Vector preAccel, preVelocity, prePos;   //  用於儲存上一刻狀態
    // Vector linear_VEL, linear_Pos, linear_ACCEL;  //  用於儲存上一刻狀態

//====輪詢(Polling)====
    long unsigned int previousTime = 0;      //  輪詢
    unsigned long previousOutputTime = 0;    //  輸出頻率
    long unsigned int preTimer3;             //  上一刻輸出時間
    long unsigned int preMicroTime;          //  計算線性位移
    float Accel_SamplePeriod;       
    int EachOperatingTime = 3448 ;       // 260Hz , 最小輪詢時間間格 3125 us, 3000 // 3846
//====輸出顯示字串===
  //--MPU6050--
    String Accel;    // 原始數據Accel
    String Gyro ;    // 原始數據Gyro
    String Accel_cali;   //  校正後Accel
    String Gyro_cali;    //  校正後Gyro
    String Accel_Filtered ;   //  Accel濾波
    String Gyro_Filtered ;     // Gyro濾波
    String Gravity_Nor ;     //  歸一化重力向量(四元數轉換)
    String Linear_Accel ;    //  線性加速度(未濾波)
    String Linear_Accel_filtered ;   //  線性加速度(濾波)
    String Linear_Acc_ ;             //  線性加速度 (vector數據類型)
    String Linear_Vel_filtered ;     //  線性速度(濾波)
    String Linear_Posi_filtered ;    //  線性位移(濾波)
    String Quar ;           //  四元數
    String Angle ;          //  角度(度)
    String currentTime ;    //  當下時間
  //--ADNS9800--
    String Dist;
    String Specification;

//====Butterworth濾波參數===
  //參考網址：https://github.com/curiores/ArduinoTutorials/tree/main/BasicFilters
  int SampleFreq = 290;
  //  加速度
    LowPass<2> lpax(15.0, SampleFreq, true);  //  截止頻率 = 15, 取樣頻率 = 290, true = 自動校正取樣頻率
    LowPass<2> lpay(15.0, SampleFreq, true);
    LowPass<2> lpaz(15.0, SampleFreq, true);
    HighPass<2> Hpax(30.0, SampleFreq, true);  
    HighPass<2> Hpay(30.0, SampleFreq, true);
    HighPass<2> Hpaz(30.0, SampleFreq, true);
  //  角速度  
    LowPass<2> lpgx(10, SampleFreq, true);  //  截止頻率 = 10, 取樣頻率 = SampleFreq, true = 自動校正取樣頻率
    LowPass<2> lpgy(10, SampleFreq, true);
    LowPass<2> lpgz(10, SampleFreq, true);
    HighPass<2> Hpgx(0.6, SampleFreq, true);
    HighPass<2> Hpgy(0.4, SampleFreq, true);
    HighPass<2> Hpgz(0.4, SampleFreq, true);  
    BandPass<2> bpgx(11.5, 10.5, SampleFreq, true); //  中心頻率 = 20、 頻寬 = 10
    BandPass<2> bpgy(11.5, 10.3, SampleFreq, true);
    BandPass<2> bpgz(11.5, 10.3, SampleFreq, true);
  //  線性加速度
    #ifdef LINEAR_ACCEL
    LowPass<2> lplax(20.0, SampleFreq, true);
    LowPass<2> lplay(20.0, SampleFreq, true);
    LowPass<2> lplaz(20.0, SampleFreq, true);
    HighPass<1> Hplax(0.5, SampleFreq, true);
    HighPass<1> Hplay(0.5, SampleFreq, true);
    HighPass<1> Hplaz(0.5, SampleFreq, true);
    BandPass<1> bplax(20, 10, SampleFreq, true); //  中心頻率 = 20、 頻寬 = 10
    BandPass<1> bplay(20, 10, SampleFreq, true);
    BandPass<1> bplaz(20, 10, SampleFreq, true);
    #endif
  //  線性速度
    #ifdef LINEAR_VELOCITY
    LowPass<2> lplvx(20.0, SampleFreq, true);
    LowPass<2> lplvy(20.0, SampleFreq, true);
    LowPass<2> lplvz(20.0, SampleFreq, true);
    HighPass<1> Hplvx(0.001, SampleFreq, true);  //  輸出震盪
    HighPass<1> Hplvy(0.001, SampleFreq, true);
    HighPass<1> Hplvz(0.1, SampleFreq, true);
    BandPass<1> bplvx(5, 4.9, SampleFreq, true);  //  輸出有問題
    BandPass<1> bplvy(5, 4.9, SampleFreq, true);
    BandPass<1> bplvz(5, 4.9, SampleFreq, true);
    #endif
  //  線性位移
    #ifdef LINEAR_POSITION
    LowPass<2> lplpx(1.2, SampleFreq, true);
    LowPass<2> lplpy(1.2, SampleFreq, true);
    LowPass<2> lplpz(1.2, SampleFreq, true);
    HighPass<1> Hplpx(1.5, SampleFreq, true);
    HighPass<1> Hplpy(1.5, SampleFreq, true);
    HighPass<1> Hplpz(1.5, SampleFreq, true);
    BandPass<2> bplpx(50, 20, SampleFreq, true);
    BandPass<2> bplpy(50, 20, SampleFreq, true);
    BandPass<2> bplpz(50, 20, SampleFreq, true);
    #endif
  //  靜態濾波
    #ifdef LINEAR_VELOCITY
    LowPass<2> lpss(15, SampleFreq, true);
    #endif
//====中值濾波=======
  //中值濾波大小
  #define WINDOW_SIZE 3
  static int16_t sensorValues[WINDOW_SIZE];  // 儲存感測器數據的緩衝區
  // static int16_t dx_MF = 0;   
  // static int16_t dy_MF = 0;
//====Kalman濾波=====

  // float KelmanDisX = 0;    
  // float KelmanDisY = 0;
  // KalmanFilter kalmanX(0.01, 0.003, 0.001);
  // KalmanFilter kalmanY(0.01, 0.003, 0.001);
//====滑動平均=======
  #define FILTER_N 15
  #define RawAccel_FILTER_N 15
  #define RawGyro_FILTER_N 15
  float filter_axbuf[FILTER_N + 1];
  float filter_aybuf[FILTER_N + 1];
  float filter_azbuf[FILTER_N + 1];
  float filter_gxbuf[FILTER_N + 1];
  float filter_gybuf[FILTER_N + 1];
  float filter_gzbuf[FILTER_N + 1];
  #ifdef LINEAR_ACCEL
    #define LinearAccel_FILTER_N 100
    float accel_sensorValues[LinearAccel_FILTER_N + 1];  // 儲存加速度數據變化的緩衝區
  #endif

//====遞增平均=======
  const int numReadings = 6;
  // 定義數據緩衝區
  float readings1[numReadings]; // 第一組數據緩衝區
  float readings2[numReadings]; // 第二組數據緩衝區
  //定義緩衝區索引 
  int index1 = 0; // 第一組緩衝區索引
  int index2 = 0; // 第二組緩衝區索引
  // 定義其他總和
  float total1 = 0; // 第一組總和
  float total2 = 0; // 第二組總和
//====ESP32按鈕======
  #define buttonUp 35  //靠近重置鍵
  #define buttonDown 0
  #define btn_Start 32

//====中斷程序=======
 #ifdef ADNS_ENABLE
 static void interruptCallBack(){
  MotionReady = 1;
  }
 #endif
//==================


void setup() {

   Serial.begin(2000000);

//==============ADNS9800================
    //腳位功能定義  
      pinMode(buttonUp, INPUT);
      pinMode(buttonDown, INPUT);
      pinMode(INTERRUPTER, INPUT_PULLUP); 
      pinMode(btn_Start, INPUT_PULLUP);

 #ifdef ADNS_ENABLE

    //  宣告中斷函數，(中斷腳位, 中斷後要執行的函數, 觸發中斷的電位變化)
      //attachInterrupt(digitalPinToInterrupt(INTERRUPTER), interruptCallBack, FALLING);

    //  初始化
      while (adns.initialization() != 1){
        delay(50);
        Serial.println("初始化失敗");    
      }

    //  設定光源
      adns.ledcAnalogWrite(LASER_PWM_CHANNEL, Brightness, MaxBrightness); //  雷射可調亮度，PWM
      // adns.ledcAnalogWrite(LED_PWM_CHANNEL, Brightness, MaxBrightness); //  雷射可調亮度，PWM

      // adns.LASER(ON); //  雷射常亮，3.3V
      adns.Led(ON);

    //  晶片功能設定
       adns.setF_Rest(Normal);  //  Defalut: Normal
       //adns.setFixed_FR(0);     //  Defalut: "非固定FrameRate"
       //adns.setNAGC(0);    //  Defalut: active
       adns.setSnapAngle(Enabled);  
      // adns.setLiftDetectionThr(set_Lift_Detec); //  Defalut: 16
       if(ischangedFR) adns.setMaxFrameRate(Shutterrate, Shutterrate_beChanged, FrameRateMin, FrameRateMin_beChanged);
       adns.setCPI(set_CPI);  //  Defalut: 1800
              //adns.setNAGC(0);    //  Defalut: active

      // adns.setRest_En(disabled);  //  Defalut: disabled

      // bool rpt = adns.getRpt_Mod();
      // if(rpt == 0) Serial.println("XY axes CPI setting in sync");  
      // else Serial.println("X軸與Y軸的CPI可獨立改變");
      // delay(50); 

    // }
    
 #endif
//=======================================

//==============MPU6050=================
  #if defined(MPU_ENABLE)
    // MPU初始化
      if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
          delay(10);
        }
      }
      Serial.println("===============");
      Serial.println("MPU6050 Found!"); 
      Serial.println("===============");

    // MPU6050 Gyroscope、Accelerometer 規格設定
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

    // 確認有無正確寫入
      // ## Gyro                                                ## Accel
      // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]      AFS_SEL=0, Full Scale Range = +/- 2 [g]
      // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]      AFS_SEL=1, Full Scale Range = +/- 4 [g]
      // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]     AFS_SEL=2, Full Scale Range = +/- 8 [g]
      // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]     AFS_SEL=3, Full Scale Range = +/- 10 [g]
      mpu6050_accel_range_t accel_range = mpu.getAccelerometerRange();
      mpu6050_gyro_range_t gyro_range = mpu.getGyroRange();
      Serial.print((mpu6050_accel_range_t)accel_range); Serial.print(" "); Serial.println((mpu6050_gyro_range_t)gyro_range);

    // 校正偏差 (取樣數量, 計算後的偏差值顯示) 
      while (!mpu.calculateOffset(1000, offset)) {}
      String Offset_Accel = String(offset[0]) + " , " +  String(offset[1]) + " , " +  String(offset[2]);     
      String Offset_Gyro = String(offset[3]) + " , " +  String(offset[4]) + " , " +  String(offset[5]);     
      Serial.println("Accel(x/y/z): " + Offset_Accel  + " || " + "Gyro(x/y/z): " + Offset_Gyro);



 #endif
//=======================================



  //  初始化完成
    Serial.println("initialization completed!");
    initialComp = 1;

}

void loop() {

//==========接受電腦端命令訊號============

  if (Serial.available()) {
    String message = Serial.readString(); // 讀取串口接收緩衝區中的訊息

    // if (message.startsWith("brightness")) {
    //   uint32_t value = message.substring(10).toInt();
    //   Brightness = value; // 將全域變數設定為收到的數值
    //   adns.ledcAnalogWrite(LASER_PWM_CHANNEL, Brightness, MaxBrightness);
    //   // adns.ledcAnalogWrite(LED_PWM_CHANNEL, Brightness, MaxBrightness);
    //   Serial.print(" # Brightness_updated:"); Serial.println(Brightness, DEC);
    // }
    if (message.startsWith("CPI")) {
      uint32_t value = message.substring(3).toInt();
      set_CPI = value;
      adns.setRpt_Mod(disabled);  //  同步解析度
      adns.setCPI(set_CPI);  //  Defalut: 1800
      get_CPI = adns.getCPI();   // 取得CPI 
      Serial.print(" | CPI: "); Serial.print(get_CPI); Serial.println(" cpi  "); 
    }
    else if (message == "SensorBtnON") { 
      ScanReady = 1;
    }
    else if (message == "SensorBtnOFF") { 
      ScanReady = 0;
    }
    else if (message == "ResetSensor") { 
      ESP.restart(); 
    }
    else if (message == "Lift_Detection_Thr_UP"){

        //====== 高度閾值檢測設定 ======          
         if(set_Lift_Detec >= 31) set_Lift_Detec = 0;
            set_Lift_Detec += 1 ; 
            adns.setLiftDetectionThr(set_Lift_Detec);
            uint8_t lift = adns.getLiftDetectionThr();    
            Serial.print("Lift Dectection -> ");     
            Serial.println(lift);       
    }
    else if (message == "Lift_Detection_Thr_Down"){
        //====== 高度閾值檢測設定 ======          
         if(set_Lift_Detec <= 0) set_Lift_Detec = 31;
            set_Lift_Detec -= 1 ; 
            adns.setLiftDetectionThr(set_Lift_Detec);
            uint8_t lift = adns.getLiftDetectionThr();      
            Serial.print("Lift Dectection -> ");     
            Serial.println(lift); 
    }
    else if (message == "MinBoundDown"){
          if(FrameRateMin <= 2500 ) FrameRateMin = 2500;
          else  FrameRateMin -= 500;
          ischangedFR = 1;
          FrameRateMin_beChanged = 1;
          Shutterrate_beChanged = 0;
          PrintSpecification();
    }
    else if (message == "MinBoundUP"){
          if(FrameRateMin >= 65536 ) FrameRateMin = 2500;
          else   FrameRateMin += 500;
          ischangedFR = 1;
          FrameRateMin_beChanged = 1;
          Shutterrate_beChanged = 0;
          PrintSpecification();
    } 
    else if (message == "ShutterDown"){
            if(Shutterrate <= 0 ) Shutterrate = 65000;
            else   Shutterrate -= 100;
            ischangedFR = 1;
            FrameRateMin_beChanged = 0;
            Shutterrate_beChanged = 1;
            PrintSpecification();
    }
    else if (message == "ShutterUP"){
            if(Shutterrate >= 65536 ) Shutterrate = 5000;
            else  Shutterrate += 100;
            ischangedFR = 1;
            FrameRateMin_beChanged = 0;
            Shutterrate_beChanged = 1;
            PrintSpecification();
    }
    if (message == "FrameCaptureModeChange") { 
      if(!isFrameCapMode)
      {
        isFrameCapMode = true;
        innitialFrameCap = false;
      }
      else 
      {
        isFrameCapMode = false;
        innitialADNS = false;
      }
    }

    // else if(message == "ResetSensor") { ESP.restart(); }
    // else if(message == "ResetSensor") { ESP.restart(); }
    // else if(message == "ResetSensor") { ESP.restart(); }
    // else if(message == "ResetSensor") { ESP.restart(); }
    // else if(message == "ResetSensor") { ESP.restart(); }
    // else if(message == "ResetSensor") { ESP.restart(); }
  
  }

//===================================

//==========手動 MPU6050校正=============
 #ifdef CALIBRATE_MPU6050
      //Serial.print("offset :"); Serial.print(*offset);
      // Serial.print("   AcX = "); Serial.print((int16_t)offset[0]);
      // Serial.print(" | AcY = "); Serial.print((int16_t)offset[1]);
      // Serial.print(" | AcZ = "); Serial.print((int16_t)offset[2]);
      // Serial.print(" | GyX = "); Serial.print((int16_t)offset[3]);
      // Serial.print(" | GyY = "); Serial.print((int16_t)offset[4]);
      // Serial.print(" | GyZ = "); Serial.println((int16_t)offset[5]);
      // delay(1000);
 #endif
//========================================

//==========按鈕中斷任務=================
  //左鍵
      if (digitalRead(buttonUp) == LOW){

        //===== 變量歸零 =======         
        // X_dis_buf = 0; Y_dis_buf = 0;
        // KelmanDisX = 0; KelmanDisY = 0;
        // Serial.println("規零");  

        //===== 高度閾值檢測設定 =====      
        //  if(set_Lift_Detec > 31) set_Lift_Detec =0;
        //     set_Lift_Detec += 1 ; 
        //     adns.setLiftDetectionThr(set_Lift_Detec);
        //     uint8_t lift = adns.getLiftDetectionThr();    
        //    // Serial.println('\n');        
        //     Serial.println(lift); 
        //     delay(300);

        //===== 雷射功率寄存器設定(不確定有沒有用) ======
        //  if(set_Lift_Detec > 7) set_Lift_Detec =0;
        // adns.EnableLaserCaliMode(set_Lift_Detec);
        // buff[0] = adns.ADNSRead(REG_LASER_CTRL0);
        // Serial.print(buff[0], BIN);
        // Serial.print(" | ");
        // Serial.println(set_Lift_Detec);
        // delay(500);

        //===== 亮度調節 =======
        // if(Brightness > 100 ) Brightness = 0;        
        // else Brightness += 10;
        // adns.ledcAnalogWrite(LASER_PWM_CHANNEL, Brightness, MaxBrightness);
        // Serial.print(" Brightness: "); Serial.println(Brightness);  //快門時間間格
        // delay(200);
      
        //===== LED 開關 =======
          if(LEDstatus)
          { 
            adns.Led(OFF); 
            LEDstatus = 0;
            delay(200);
          }
          else 
          { 
            adns.Led(ON);
            LEDstatus = 1; 
            delay(200);
          }     
       }
  //右鍵
   
      if (digitalRead(buttonDown) == LOW){

      //======= 幀數設定與顯示 =======
        // //  if(FrameRateMin >= 65536 ) FrameRateMin = 0;
        //     if(Shutterrate >= 65000 ) Shutterrate = 0;
        //      //FrameRateMin += 1000;
        //      // FrameRateMin_beChanged = 1;
        //        Shutterrate += 100;
        //        // adns.setMaxFrameRate(Shutterrate, Shutterrate_beChanged, FrameRateMin, FrameRateMin_beChanged);
        //        adns.get_Max_Bound();
        //        Serial.print(" Shutterrate: "); Serial.print(adns.Shutter);  //快門時間間格
        //        Serial.print(" | MinBound:  ");  Serial.print(adns.Min_Bound);   //幀數下限
        //        FR = adns.getFrameRate();   //取得FT   
        //        Serial.print(" | FrameRate: ");  Serial.print(FR);  Serial.print(" fps ");  
        //        get_CPI = adns.getCPI();                  //取得CPI
        //        Serial.print(" | CPI: "); Serial.print(get_CPI); Serial.println(" cpi  "); 
        // //     adns.get_Max_Bound();       //取得上下限
        //        delay(300);

      //======= 雷射連續模式 =======
        // adns.EnableLaserCW(1);
        // delay(200);     

      //======= 亮度調節 =======
        if(Brightness > 100 ) Brightness = 0;        
        else Brightness += 10;
        adns.ledcAnalogWrite(LASER_PWM_CHANNEL, Brightness, MaxBrightness);
        Serial.print(" Brightness: "); Serial.println(Brightness);  //快門時間間格
        delay(200);

      //======= XY解析度狀態轉換 =======
        // if(!ResolutionMode){
        //   adns.setRpt_Mod(Enabled); //  非同步解析度
        //   adns.setCPI_Y(Y_CPI);
        //   ResolutionMode = 1;

        //   unsigned int X_cpi = adns.getCPI();  
        //   Serial.print(" | X-Axis CPI: "); Serial.print(X_cpi); Serial.println(" cpi  "); 

        //   unsigned int Y_cpi = Y_CPI * 200;
        //   Serial.print(" | Y-Axis CPI: "); Serial.print(Y_cpi); Serial.println(" cpi  "); 
        //   delay(200);
        // }
        // else{
          
        //   adns.setRpt_Mod(disabled);  //  同步解析度
        //   adns.setCPI(set_CPI);  //  Defalut: 1800
        //   ResolutionMode = 0;
        //   get_CPI = adns.getCPI();   // 取得CPI 
        //   Serial.print(" | CPI: "); Serial.print(get_CPI); Serial.println(" cpi  "); 
        //   delay(200);
        // }
       
       }
    
  //中鍵
      if(digitalRead(btn_Start) == LOW) { 
          if(ScanReady != 1)  ScanReady = 1;
          else ScanReady = 0;
          delay(300);


      // if(Brightness > 8191 ) Brightness = 0;        
      // else Brightness += 1;
      // adns.ledcAnalogWrite(LASER_PWM_CHANNEL, Brightness, MaxBrightness);
      // Serial.print(" Brightness: "); Serial.println(Brightness);  //快門時間間格
      // delay(200);
      }

//==========================================

//==========ADNS即時畫面================

  if(isFrameCapMode){

  // 晶片初始化設定，只執行一次
    if(!innitialFrameCap){

      adns.ADNSWrite(REG_Power_Up_Reset, 0x5a);
      delay(50);
      if(ischangedFR){
        adns.setMaxFrameRate(Shutterrate, Shutterrate_beChanged , FrameRateMin, FrameRateMin_beChanged);
      }        
      adns.setCPI(set_CPI);
      adns.setLiftDetectionThr(set_Lift_Detec);
      innitialFrameCap = 1;
    }
  // 讀取像素
    else{
      
      String image = ""; 
      if( adns.frameCapture(pixel) == true) {
        for(int i=0; i < 900; i++){
         image += String(pixel[i]) + " ";
        }
        image += '\n'; // 添加換行符
      }
      Serial.print(image);
   
  // 取得規格
      Squal = adns.getSQUAL();
      PixelSUM = adns.getPixelSUM();
      FR = adns.getFrameRate();
      get_Lift_Detec = adns.getLiftDetectionThr();
      adns.get_Max_Bound();

  // 規格輸出
      Serial.print(" # Squal:"); Serial.print(Squal);
      Serial.print(" # PixelSUM:"); Serial.print(PixelSUM);
      Serial.print(" # FrameRate:"); Serial.print(FR);
      Serial.print(" # Lift_Detec:" ); Serial.print(get_Lift_Detec);
      String Bound =  String(adns.Max_Bound,DEC)+ "," + String(adns.Min_Bound,DEC)+ "," + String(adns.Shutter,DEC);
      Serial.print(" # Frame_period(Max/Min/Shutter):" ); Serial.println(Bound);
      
    }
  }

//=========================================

//==========轉換回 "點雲擷取模式" 後的初始化=======
  if(!isFrameCapMode){
    // 初始化
    if(!innitialADNS){
        adns.rebootAdns();  //  重至晶片
        ScanReady = 0;  //  停止顯示感測數據

      // 設定ADNS晶片規格
      if(ischangedFR){
        adns.setMaxFrameRate(Shutterrate, Shutterrate_beChanged , FrameRateMin, FrameRateMin_beChanged);
        ischangedFR = 0;
      }
      adns.setCPI(set_CPI);  
      adns.setLiftDetectionThr(set_Lift_Detec);
      // adns.setF_Rest(Normal);  //  Defalut: Normal
      // adns.setFixed_FR(0);     //  Defalut: "非固定FrameRate"
      adns.setSnapAngle(Enabled);  
      // adns.setRest_En(disabled);  //  Defalut: disabled
      PrintSpecification();   //  顯示感測時的ADNS規格

      resetDataSet(); //  重新感測數據
      innitialADNS = true;
      }
//=========================================

//==========ADNS狀態讀取 ==================  
  #ifdef ADNS_ENABLE  
      if(!isChecked_ADNS_specificatio){
        // uint8_t buff[2];
        // adns.getMaxMinPixel(buff[0],buff[1]);  //max,min
        // adns.getFixed_FR(); 
         Squal = adns.getSQUAL();
         PixelSUM = adns.getPixelSUM();
         get_Lift_Detec = adns.getLiftDetectionThr();
         FR = adns.getFrameRate();
         get_CPI = adns.getCPI();
         isChecked_ADNS_specificatio = 1;
      }

   #endif
//=========================================

//==========ADNS位移讀取==============
 #ifdef ADNS_ENABLE  
    
  if(initialComp == 1){
       adns.updateMotionBurstData();
       adns.getXY(&dx, &dy);
  }
 #endif
    
//=======================================

//==========MPU6050感測讀取(1)===========
 #ifdef MPU_ENABLE 
  //// 取得加速度 角速度//////
    mpu.getEvent(&a, &g, &temp);  //  呼叫事件方式取得
    float rax, ray, raz, rgx, rgy, rgz; 
    rax = a.acceleration.x; ray = a.acceleration.y; raz = a.acceleration.z;   //  (g)
    rgx = g.gyro.x; rgy = g.gyro.y; rgz = g.gyro.z;   //  (°/s)
  /////////////////////////////////    
  //// 取得加速度取樣頻率 //// 
    unsigned long Now = micros(); 
    float Accel_SamplePeriod = (float)((Now - preMicroTime) / 1e6); //  加速度取樣週期(s)，用於計算線性位移
    preMicroTime = Now; //  給予計算取樣頻率
  /////////////////////////////////
 #endif
//======================================

//==========MPU6050資料處理(1)==========
   #ifdef  MPU_ENABLE 

   ///// 1. 變量宣告 ////
          float linear_velocity_X_Filted, linear_velocity_Y_Filted, linear_velocity_Z_Filted; //  線性速度
          float linear_pos_X_Filted, linear_pos_Y_Filted, linear_pos_Z_Filted;  //  線性定位
   /////////////////////

   ///// 2. 原始數據 /////
              float ax, ay, az;
              float gx, gy, gz;
              switch (Accel_RawData)
              {
                  case RawData:
                      ax = rax * (float)mpu.rangePerDigit;   //  (LSBs)
                      ay = ray * (float)mpu.rangePerDigit;
                      az = raz * (float)mpu.rangePerDigit;
                      break;

                  case Normalized:
                      ax = (float)rax;  // (g)
                      ay = (float)ray;
                      az = (float)raz;
                      break;

                  case gravitational_accel:  
                      ax = (float)(rax * 9.8065f);   //  (m/s^2)
                      ay = (float)(ray * 9.8065f);
                      az = (float)(raz * 9.8065f);
                      break;
              }
              switch (Gyro_RawData)
              {
                  case RawData:
                      gx = rgx * mpu.dpsPerDigit;   //  (LSBs)
                      gy = rgy * mpu.dpsPerDigit;
                      gz = rgz * mpu.dpsPerDigit;
                      break;

                  case DegPerSec_gyro:
                      gx = (float)rgx;  //  (°/s)
                      gy = (float)rgy;
                      gz = (float)rgz;
                      break;
              }

   //////////////////////

   ///// 3. 【濾波】原始數據 ////
              switch (Accel_RawData_Filted)
              {
                  case MAF: // (a) 滑動濾波
                      Accel_X_Filted = MAFilter(ax, filter_axbuf, RawAccel_FILTER_N);
                      Accel_Y_Filted = MAFilter(ay, filter_aybuf, RawAccel_FILTER_N);
                      Accel_Z_Filted = MAFilter(az, filter_azbuf, RawAccel_FILTER_N);
                      break;

                  case TD_Lowpass: // (b) 時域低通濾波
                      Accel_X_Filted = lowPassFilter(Accel_X_Filted, ax, 0.65, 0.01);
                      Accel_Y_Filted = lowPassFilter(Accel_Y_Filted, ay, 0.65, 0.01);
                      Accel_Z_Filted = lowPassFilter(Accel_Z_Filted, az, 0.65, 0.01);
                      break;
                  case Butterworth_LowPass: // (c) Butterworth低通濾波
                      Accel_X_Filted = lpax.filt(ax);
                      Accel_Y_Filted = lpay.filt(ay);
                      Accel_Z_Filted = lpaz.filt(az);
                      break;
                  case No_Filted: // (d) 無濾波
                      Accel_X_Filted = ax;
                      Accel_Y_Filted = ay;
                      Accel_Z_Filted = az;
                      break;
              }

              switch (Gyro_RawData_Filted)
              {
                  case MAF: // (a) 滑動濾波
                      Gyro_X_Filted = MAFilter(gx, filter_gxbuf, RawGyro_FILTER_N);
                      Gyro_Y_Filted = MAFilter(gy, filter_gybuf, RawGyro_FILTER_N);
                      Gyro_Z_Filted = MAFilter(gz, filter_gzbuf, RawGyro_FILTER_N);
                      break;
                  case TD_Lowpass: // (b) 時域低通濾波
                      Gyro_X_Filted = lowPassFilter(Gyro_X_Filted, gx, 0.65, 0.01);
                      Gyro_Y_Filted = lowPassFilter(Gyro_Y_Filted, gy, 0.65, 0.01);
                      Gyro_Z_Filted = lowPassFilter(Gyro_Z_Filted, gz, 0.65, 0.01);
                      break;
                  case Butterworth_LowPass: // (c) Butterworth低通濾波
                      Gyro_X_Filted = lpgx.filt(gx);
                      Gyro_Y_Filted = lpgy.filt(gy);
                      Gyro_Z_Filted = lpgz.filt(gz);
                      break;
                  case Butterworth_HighPass: // (d) Butterworth高通濾波
                      Gyro_X_Filted = Hpgx.filt(gx);
                      Gyro_Y_Filted = Hpgy.filt(gy);
                      Gyro_Z_Filted = Hpgz.filt(gz);
                      break;
                  case Butterworth_BandPass: // (e) Butterworth 帶通濾波
                      Gyro_X_Filted = bpgx.filt(gx);
                      Gyro_Y_Filted = bpgy.filt(gy);
                      Gyro_Z_Filted = bpgz.filt(gz);
                      break;
                  case No_Filted: // (f)無濾波
                      Gyro_X_Filted = gx;
                      Gyro_Y_Filted = gy;
                      Gyro_Z_Filted = gz;
                      break;
              }
   ////////////////////////////

   ///// 4. 【閾值】濾波後數據 ////

        // Gyro_X_Filted = MPU_thresholdFileter(Gyro_X_Filted, 1.0f, "Min");
        // Gyro_Y_Filted = MPU_thresholdFileter(Gyro_Y_Filted, 0.1f, "Min");
        // Gyro_Z_Filted = MPU_thresholdFileter(Gyro_Z_Filted, 0.1f , "Min");    

   #endif
//=====================================

//==========MPU6050感測讀取(2)=======
 #ifdef MPU_ENABLE
 //// 取得四元數////// 
    float gx_filtered_rad = Gyro_X_Filted * 0.01745329f; // (rad/s)
    float gy_filtered_rad = Gyro_Y_Filted * 0.01745329f; 
    float gz_filtered_rad = Gyro_Z_Filted * 0.01745329f; 

    mpu.setMahonySampleFreq();  // 頻率
    mpu.updateMahony(gx_filtered_rad, gy_filtered_rad, gz_filtered_rad, Accel_X_Filted, Accel_Y_Filted, Accel_Z_Filted);  // gyro：rad/s || accel：g
    mpu.QuaternionMahony(&qw, &qx, &qy, &qz);
         
 //// 取得重力向量//////
    mpu.getGravity(&Gra_RAW.x, &Gra_RAW.y, &Gra_RAW.z);   //  旋轉重力向量至體座標系上

 //// 取得尤拉角//////
    Quaternion q;  q.w = qw; q.x = qx; q.y = qy; q.z = qz;
    float resforangles[3];
    // Angles = mpu.GetEuler(qw, qx, qy, qz);
     Angles = mpu.getEulerAngles();
    // mpu.GetYawPitchRoll(AngleData, &q, &Gra_RAW);
    mpu.quaternion2Euler(q, resforangles, zyx); //  rad

 #endif
//==================================

//==========MPU6050資料處理(2)========
  ///// 5. 【計算】線性加速度 /////////
    #ifdef LINEAR_ACCEL

              float linear_acc_X, linear_acc_Y, linear_acc_Z;
              switch (Accel_RawData)
              {
                  case Normalized:
                      linear_acc_X = (Accel_X_Filted - Gra_RAW.x) * 980.65f;  //  (cm/s)
                      linear_acc_Y = (Accel_Y_Filted - Gra_RAW.y) * 980.65f;
                      linear_acc_Z = (Accel_Z_Filted - Gra_RAW.z) * 980.65f;
                      break;
                  case gravitational_accel:  //  m/s^2
                      linear_acc_X = Accel_X_Filted - Gra_RAW.x * 980.65f;
                      linear_acc_Y = Accel_Y_Filted - Gra_RAW.y * 980.65f;
                      linear_acc_Z = Accel_Z_Filted - Gra_RAW.z * 980.65f;
                      break;
                  // default:
                  //     linear_acc_X = (mpu.na.XAxis - Gra_RAW.x) * 9.81f;
                  //     linear_acc_Y = (mpu.na.YAxis - Gra_RAW.y) * 9.81f;
                  //     linear_acc_Z = (mpu.na.ZAxis - Gra_RAW.z) * 9.81f;
                  //     break;
              }
  ///////////////////////////////////

  ///// 6. 【濾波】線性加速度 /////////

              float linear_acc_X_Filted, linear_acc_Y_Filted, linear_acc_Z_Filted;
              switch (Linear_Accel_Filted)
              {
                  case TD_Lowpass: // (b) 時域低通濾波
                      linear_acc_X_Filted = lowPassFilter(linear_acc_X_Filted, linear_acc_X, 0.25, 0.4);
                      linear_acc_Y_Filted = lowPassFilter(linear_acc_Y_Filted, linear_acc_Y, 0.25, 0.4);
                      linear_acc_Z_Filted = lowPassFilter(linear_acc_Z_Filted, linear_acc_Z, 0.25, 0.4);
                      break;
                  case TD_Highpass: // (b) 時域高通濾波
                      linear_acc_X_Filted = highPassFilter(linear_acc_X_Filted, linear_acc_X, 0.6);
                      linear_acc_Y_Filted = highPassFilter(linear_acc_Y_Filted, linear_acc_Y, 0.6);
                      linear_acc_Z_Filted = highPassFilter(linear_acc_Z_Filted, linear_acc_Z, 0.6);
                      break;
                  case Butterworth_LowPass: // (c) Butterworth 低通濾波
                      linear_acc_X_Filted = lplax.filt(linear_acc_X);
                      linear_acc_Y_Filted = lplay.filt(linear_acc_Y);
                      linear_acc_Z_Filted = lplaz.filt(linear_acc_Z);
                      break;
                  case Butterworth_HighPass: // (c) Butterworth 高通濾波
                      linear_acc_X_Filted = Hplax.filt(linear_acc_X);
                      linear_acc_Y_Filted = Hplay.filt(linear_acc_Y);
                      linear_acc_Z_Filted = Hplaz.filt(linear_acc_Z);
                      break;
                  case Butterworth_BandPass: // (c) Butterworth 帶通濾波
                      linear_acc_X_Filted = bplax.filt(linear_acc_X);
                      linear_acc_Y_Filted = bplay.filt(linear_acc_Y);
                      linear_acc_Z_Filted = bplaz.filt(linear_acc_Z);
                      break;
                  case No_Filted: // 無濾波
                      linear_acc_X_Filted = linear_acc_X;
                      linear_acc_Y_Filted = linear_acc_Y;
                      linear_acc_Z_Filted = linear_acc_Z;
                      break;
              }
    #endif//(LINEAR_ACCEL)
  ///////////////////////////////////

  ///// 7.【計算】靜態偵測 ////////  
    #ifdef LINEAR_VELOCITY
          //  以慣性感測器判斷狀態
          //float Accel_mag = sqrt(linear_acc_X_Filted * linear_acc_X_Filted + linear_acc_Y_Filted * linear_acc_Y_Filted + linear_acc_Z_Filted * linear_acc_Z_Filted);
          float Accel_mag = sqrt(linear_acc_Y_Filted * linear_acc_Y_Filted + linear_acc_Z_Filted * linear_acc_Z_Filted);
          float md = MAFilter(Accel_mag, accel_sensorValues, LinearAccel_FILTER_N);
          float _Accel_mag = Accel_mag - md;
          float Accel_mag_filtered = lpss.filt(_Accel_mag);

          bool stationary;
          if(Accel_mag_filtered < 0.5) stationary = true;
          else stationary = false;

          //  以位移感測器判斷狀態
          // bool stationary;
          // if(dx != 0 || dy != 0) stationary = false;
          // else stationary = true;
  ///////////////////////////////

  ///// 8.【計算】線性速度 ////////

          float linear_VEL_X, linear_VEL_Y, linear_VEL_Z;
          //  是否非靜止狀態
          if(!stationary){
            linear_VEL_X = prelinear_VEL_X + (linear_acc_X_Filted + prelinear_Acc_X) * Accel_SamplePeriod / 2.0;  //  依據梯形法則的積分
            linear_VEL_Y = prelinear_VEL_Y + (linear_acc_Y_Filted + prelinear_Acc_Y) * Accel_SamplePeriod / 2.0;
            linear_VEL_Z = prelinear_VEL_Z + (linear_acc_Z_Filted + prelinear_Acc_Z) * Accel_SamplePeriod / 2.0;

  ///////////////////////////////

   ///// 9.【濾波】線性速度 ////////
              
              // float _linear_velocity_X_Filted, _linear_velocity_Y_Filted, _linear_velocity_Z_Filted; //  線性速度
              switch (Linear_Velocity_Filted)
              {
                  case TD_Lowpass: // (b) 時域低通濾波
                      linear_velocity_X_Filted = lowPassFilter(linear_velocity_X_Filted, linear_VEL_X, 0.25, 0.4);
                      linear_velocity_Y_Filted = lowPassFilter(linear_velocity_Y_Filted, linear_VEL_Y, 0.25, 0.4);
                      linear_velocity_Z_Filted = lowPassFilter(linear_velocity_Z_Filted, linear_VEL_Z, 0.25, 0.4);
                      break;
                  case TD_Highpass: // (b) 時域高通濾波
                      linear_velocity_X_Filted = highPassFilter(linear_velocity_X_Filted, linear_VEL_X, 0.6);
                      linear_velocity_Y_Filted = highPassFilter(linear_velocity_Y_Filted, linear_VEL_Y, 0.6);
                      linear_velocity_Z_Filted = highPassFilter(linear_velocity_Z_Filted, linear_VEL_Z, 0.6);
                      break;
                  case Butterworth_LowPass: // (c) Butterworth 低通濾波
                      linear_velocity_X_Filted = lplvx.filt(linear_VEL_X);
                      linear_velocity_Y_Filted = lplvy.filt(linear_VEL_Y);
                      linear_velocity_Z_Filted = lplvz.filt(linear_VEL_Z);
                      break;
                  case Butterworth_HighPass: // (c) Butterworth 高通濾波
                      linear_velocity_X_Filted = Hplvx.filt(linear_VEL_X);
                      linear_velocity_Y_Filted = Hplvy.filt(linear_VEL_Y);
                      linear_velocity_Z_Filted = Hplvz.filt(linear_VEL_Z);
                      break;
                  case Butterworth_BandPass: // (c) Butterworth 帶通濾波
                      linear_velocity_X_Filted = bplvx.filt(linear_VEL_X);
                      linear_velocity_Y_Filted = bplvy.filt(linear_VEL_Y);
                      linear_velocity_Z_Filted = bplvz.filt(linear_VEL_Z);
                      break;
                  case No_Filted: // 無濾波
                      linear_velocity_X_Filted = linear_VEL_X;
                      linear_velocity_Y_Filted = linear_VEL_Y;
                      linear_velocity_Z_Filted = linear_VEL_Z;
                      break;
              }
                      // linear_velocity_X_Filted = Hplvx.filt(_linear_velocity_X_Filted);
                      // linear_velocity_Y_Filted = Hplvy.filt(_linear_velocity_Y_Filted);
                      // linear_velocity_Z_Filted = Hplvz.filt(_linear_velocity_Z_Filted);
   ////////////////////////////////

   ///// 10. 【計算】線性位移 ///////
    #ifdef LINEAR_POSITION
              float linear_Pos_X = prelinear_Pos_X + prelinear_VEL_X * Accel_SamplePeriod + 0.5f * prelinear_Acc_X * Accel_SamplePeriod * Accel_SamplePeriod;
              float linear_Pos_Y = prelinear_Pos_Y + prelinear_VEL_Y * Accel_SamplePeriod + 0.5f * prelinear_Acc_Y * Accel_SamplePeriod * Accel_SamplePeriod;
              float linear_Pos_Z = prelinear_Pos_Z + prelinear_VEL_Z * Accel_SamplePeriod + 0.5f * prelinear_Acc_Z * Accel_SamplePeriod * Accel_SamplePeriod;

              //float linear_Pos_Z = prelinear_Pos_Z + 0.5f * (prelinear_VEL_Z + linear_velocity_Z_Filted) * Accel_SamplePeriod;

   ////////////////////////////////

   ///// 11. 【濾波】線性位移 ///////

              float linear_pos_X_Filted, linear_pos_Y_Filted, linear_pos_Z_Filted;  //  線性定位
              switch (Linear_Position_Filted)
              {
                  case TD_Lowpass: // (a) 時域低通濾波
                      linear_pos_X_Filted = lowPassFilter(linear_pos_X_Filted, linear_Pos_X, 0.25, 0.4);
                      linear_pos_Y_Filted = lowPassFilter(linear_pos_Y_Filted, linear_Pos_Y, 0.25, 0.4);
                      linear_pos_Z_Filted = lowPassFilter(linear_pos_Z_Filted, linear_Pos_Z, 0.25, 0.4);
                      break;
                  case TD_Highpass: // (b) 時域高通濾波
                      linear_pos_X_Filted = highPassFilter(linear_pos_X_Filted, linear_Pos_X, 0.6);
                      linear_pos_Y_Filted = highPassFilter(linear_pos_Y_Filted, linear_Pos_Y, 0.6);
                      linear_pos_Z_Filted = highPassFilter(linear_pos_Z_Filted, linear_Pos_Z, 0.6);
                      break;
                  case Butterworth_LowPass: // (c) Butterworth 低通濾波
                      linear_pos_X_Filted = lplpx.filt(linear_Pos_X);
                      linear_pos_Y_Filted = lplpy.filt(linear_Pos_Y);
                      linear_pos_Z_Filted = lplpz.filt(linear_Pos_Z);
                      break;
                  case Butterworth_HighPass: // (d) Butterworth 高通濾波
                      linear_pos_X_Filted = Hplpx.filt(linear_Pos_X);
                      linear_pos_Y_Filted = Hplpy.filt(linear_Pos_Y);
                      linear_pos_Z_Filted = Hplpz.filt(linear_Pos_Z);
                      break;
                  case Butterworth_BandPass: // (e) Butterworth 帶通濾波
                      linear_pos_X_Filted = bplpx.filt(linear_Pos_X);
                      linear_pos_Y_Filted = bplpy.filt(linear_Pos_Y);
                      linear_pos_Z_Filted = bplpz.filt(linear_Pos_Z);
                      break;
                  case No_Filted: // (f)無濾波
                      linear_pos_X_Filted = linear_Pos_X;
                      linear_pos_Y_Filted = linear_Pos_Y;
                      linear_pos_Z_Filted = linear_Pos_Z;
                      break;
              }
    #endif//(LINEAR_POSITION)
   ///////////////////////////////

   ///// 12. 倘若為靜態 ///////
          } //  靜態檢測，不可注釋
          else{
            linear_velocity_X_Filted = 0;
            linear_velocity_Y_Filted = 0;
            linear_velocity_Z_Filted = 0;
          }
    #endif//(LINEAR_VELOCITY)
   ///////////////////////////

   ///// 13. 紀錄數據，供迭代運算使用 ///////
    #ifdef LINEAR_ACCEL   
              prelinear_Acc_X = linear_acc_X_Filted;
              prelinear_Acc_Y = linear_acc_Y_Filted;
              prelinear_Acc_Z = linear_acc_Z_Filted;
    #endif//(LINEAR_ACCEL)

    #ifdef LINEAR_VELOCITY
              prelinear_VEL_X = linear_velocity_X_Filted;
              prelinear_VEL_Y = linear_velocity_Y_Filted;
              prelinear_VEL_Z = linear_velocity_Z_Filted;
    #endif//(LINEAR_VELOCITY)

    #ifdef LINEAR_POSITION
              prelinear_Pos_X = linear_pos_X_Filted;
              prelinear_Pos_Y = linear_pos_Y_Filted;
              prelinear_Pos_Z = linear_pos_Z_Filted;
    #endif//(LINEAR_POSITION)

  //  //////////////////////////////////////
//===================================

//==========ADNS資料處理=============
  #ifdef ADNS_ENABLE
 //閾值濾波
    dx = ADNS_thresholdFileter(dx);
    dy = ADNS_thresholdFileter(dy);

 //數據後處理(總位移 + 瞬時速度) 
    // double  lenght = (double)(sqrt(dx * dx + dy * dy) * (2.54 / (get_CPI * 200.0)));  // 公分 cm
    // double  velocity = (double)(lenght / Accel_SamplePeriod);
    // double X_velocity = (double)(abs(dx) * (2.54 / (get_CPI * 200.0)) / Accel_SamplePeriod);
    // double Y_velocity = (double)(abs(dy) * (2.54 / (get_CPI * 200.0)) / Accel_SamplePeriod);
    // X_dis_buf += dx;
    // Y_dis_buf += dy;
    
 //卡爾曼濾波
    //  KelmanDisX = kalmanX.update(X_dis_buf, X_speed);
    //  KelmanDisY = kalmanY.update(Y_dis_buf, Y_speed);
    //  DeltaX = KelmanDisX * 0.000508;
    //  DeltaY = KelmanDisY * 0.000508;

 //中值濾波
    // int16_t dx_MF = median_filter(dx);    // 將X數據進行中值濾波
    // int16_t dy_MF = median_filter(dy);    // 將Y數據進行中值濾波
 #endif
//==================================

//==========資料轉換為字串============
    
 #ifdef MPU_ENABLE 
  //  Accel = String(mpu.rawAccX) + "," + String(mpu.rawAccY)+ "," + String(mpu.rawAccZ);    // 原始數據Accel，未校正偏差
  // Gyro = String(mpu.rawGyroX) + "," + String(mpu.rawGyroY)+ "," + String(mpu.rawGyroZ);     // 原始數據Gyro，未校正偏差
  // Accel_cali = String(mpu.rawAccX_cali) + "," + String(mpu.rawAccY_cali)+ "," + String(mpu.rawAccZ_cali);   //  校正後Accel
  //  Gyro_cali = String(mpu.rawGyroX_cali) + "," + String(mpu.rawGyroY_cali)+ "," + String(mpu.rawGyroZ_cali);    //  校正後Gyro
   Accel_Filtered = String(Accel_X_Filted, 3) + "," + String(Accel_Y_Filted, 3)+ "," + String(Accel_Z_Filted, 3);   //  Accel濾波
   Gyro_Filtered = String(Gyro_X_Filted, 2) + "," + String(Gyro_Y_Filted, 2)+ "," + String(Gyro_Z_Filted, 2);       // Gyro濾波
  //  Gravity_Nor = String(Gra_RAW.x, 4) + "," + String(Gra_RAW.y, 4)+ "," + String(Gra_RAW.z, 4);                               //  歸一化重力向量(四元數轉換)
  //  Linear_Accel_filtered = String(linear_acc_X_Filted, 3) + "," + String(linear_acc_Y_Filted, 3)+ "," + String(linear_acc_Z_Filted, 3);    //  線性加速度(自行選擇濾波)
  //  Linear_Acc_ = String(linear_ACCEL.XAxis, 4) + "," + String(linear_ACCEL.YAxis, 4)+ "," + String(linear_ACCEL.ZAxis, 4);               //  線性加速度 (vector數據類型)
  //  Linear_Vel_filtered = String(linear_velocity_X_Filted, 4) + "," + String(linear_velocity_Y_Filted, 4)+ "," + String(linear_velocity_Z_Filted, 4);       //  線性速度(濾波)
  //  Linear_Posi_filtered = String(linear_pos_X_Filted, 3) + "," + String(linear_pos_Y_Filted, 3)+ "," + String(linear_pos_Z_Filted, 3);                     //  線性位移(濾波)
    Quar = String(qw, 3) + "," + String(qx, 3) + "," + String(qy, 3) + "," + String(qz, 3);               //  四元數
    // Angle = String(Angles.roll * 100.0f, 2) + "," + String(Angles.pitch * 100.0f, 2) + "," + String(Angles.yaw * 100.0f, 2) ;        //  角度(度)
    // Angle = String(AngleData[0] * 100.0f, 2) + "," + String(AngleData[1] * 100.0f, 2) + "," + String(AngleData[2] * 100.0f, 2) ;        //  角度(度)
    Angle = String(mpu.rad2deg(resforangles[0]), 1) + "," + String(mpu.rad2deg(resforangles[1]), 1) + "," + String(mpu.rad2deg(resforangles[2]), 1) ;        //  角度(度)
    
    // String IMUSampleRate = String(Accel_SamplePeriod, 3); //  加速度取樣週期
    // String Staticstate = String(Accel_mag) + "," + String(_Accel_mag)+ "," + String(Accel_mag_filtered);       //  線性速度(濾波)
 #endif

 #ifdef ADNS_ENABLE   
    Dist = String(dx) + "," + String(dy);   //原始位移數據
    // Dist = String(X_dis_buf/649.4) + "," + String(Y_dis_buf/309.1);   //總位移數據換算實際距離
    // String KALDist = String(KelmanDisX) + "," + String(KelmanDisY);  //總位移數據經由濾波器
    // Dist = String(dx) + "  " + String(dy) + "  " +String(X_dis_buf) + "  " + String(Y_dis_buf) + "  " + String(X_velocity, 5) + "  " + String(Y_velocity, 5);   //原始位移 + 總位移 + 位移瞬時速度
    // Specification = String(Squal) + ","+ String(PixelSUM) + "," + String(get_Lift_Detec) + "," + String(FR) + "," + String(get_CPI) + "," + String(adns.Max_Bound,DEC)+ "," + String(adns.Min_Bound,DEC)+ "," + String(adns.Shutter,DEC);
    //  Specification = String(Squal) + ","+ String(PixelSUM) + "," + String(get_Lift_Detec) + "," + String(FR) + "," + String(get_CPI);

 #endif
//===================================

//==========感測器輸出數據============
 
  //  輪詢等待   
    while(micros() - previousTime < EachOperatingTime);
    previousTime = micros();  //  輪詢 
 
  
  //  掃描按鈕觸發    
    if (ScanReady == 1) {
        currentTime = micros();


            // 顯示輸出頻率(Execution_Frequenc)
            // float dt = (float)((micros() - preTimer3) / 1.0e6);   //  s
            // float Execution_Frequenc = 1 / dt;   //  Hz
            // printf("Execution_Frequenc : %.2f Hz\n", Execution_Frequenc); 


            #ifdef OneEquipment

              #ifdef MPU_ENABLE 
                // Serial.println(Accel + "," + Gyro + "," + Quar) ;   //加速度 + 角速度 + 四元數
                // Serial.println(Quar) ;   //四元數
              #endif  //  MPU_ENABLE

              #ifdef ADNS_ENABLE  
                // Serial.print(Dist + '\n');  //單獨位移數據
                // Serial.println(KALDist) ;   //卡爾曼濾波
                Serial.print(Dist + "," +  Specification + "," +  currentTime + '\n');
                // PrintSpecification();
              #endif  //  ADNS_ENABLE

            #endif  //  OneEquipment

            #ifdef TwoEquipment
              #if defined(ADNS_ENABLE) && defined(MPU_ENABLE)
                //  Serial.print(Accel + "," + Gyro + '\n') ;            //  【原始數據】 加速度 + 角速度  
                // Serial.print(Accel_cali + "," + Gyro_cali + '\n') ;  //  【校正原始數據】 加速度 + 角速度  
                // Serial.print(Accel_Nor + "," + Gyro_Nor + '\n') ;    //  【歸一化原始數據】 加速度 + 角速度  
                // Serial.print(Accel_Filtered + "," + Gyro_Filtered + '\n');                  //  【濾波數據】 加速度 + 角速度  
                // Serial.print(Accel_Filtered + "," + Gravity_Nor + "," + Quar + '\n');     //  【濾波數據】 加速度(濾波) + 重力向量 + 四元數

                // Serial.print(Linear_Accel_filtered + "," + Gyro_Filtered + "," + Quar +  "," + Dist + "," + currentTime  + '\n') ;   //  【濾波數據】 線性加速度 + 角速度 + 四元數 + 位移 + 時間
                Serial.print(Accel_Filtered + "," + Gyro_Filtered + "," + Quar +  "," + Dist + "," + currentTime  + '\n') ;   //  【濾波數據】 加速度 + 角速度 + 四元數 + 位移 + 時間
                // Serial.print(Dist + '\n');  //單獨輸出位移數據

                // Serial.print(Linear_Accel_filtered + "," + Gyro_Filtered + "," + Linear_Vel_filtered +  "," + Quar + '\n') ;   //  用於測試輸出，13筆數據
                // Serial.print(Angle + "," + Gyro_Filtered + "," + Quar +  "," + Dist + "," + currentTime  + '\n') ;   //  用於測試輸出，13筆數據


                // Serial.print(Staticstate + "," + Linear_Vel_filtered + "," + Quar +  ","  + Dist + "," + currentTime  + '\n') ;   // 【濾波數據】 線性速度 + 線性位移 + 四元數 + 位移 + 時間
                // Serial.print(Accel_Nor + "," + Gravity_Nor + "," + Quar + '\n');               //  加速度 + 重力向量 + 四元數 
                // Serial.print((String) "MPUdata"+ " " + Accel_X_Filted + " " + Accel_Y_Filted + " " + Accel_Z_Filted + " " + Gyro_X_Filted + " " + Gyro_Y_Filted + " " + Gyro_Z_Filted + "\n");  //  其他軟體需要

                  
              #endif  //  ADNS_ENABLE + MPU_ENABLE
            #endif  // TwoEquipment
            
            //Serial.flush();  //  等待所有數據都已傳出去並清除緩存
            //  preTimer3 =  micros(); //  輸出頻率
    } // ScanReady
      //previousTime = micros();  //  輪詢 
    
    } // isFrameCapMode = false
    resetDataSet();  //  重製變量與 MPU 讀取狀態
}

//=======暫放==============
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
  {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
  }
//=======變 量 歸 零=======

 void  resetDataSet(){
   #ifdef ADNS_ENABLE
   dx = 0;
   dy = 0; 
   //MotionReady = 0;
   adns._moved = 0; 
   #endif
   #ifdef MPU_ENABLE
  //  mpu.ng = {0,0,0};
  //  mpu.na = {0,0,0};
     mpu.RawaDataReady = false;
  //   mpu.AccelRawReady = false;
  //   mpu.AccelGraviReady = false;
  //   mpu.AccelNormReady = false;
  //   mpu.GyroRawReady = false;
  //   mpu.GyroNormReady = false;
    #endif
   
   
 }

 
//=======中 值 濾 波=======
 //========int16_t 數據類型=========
    int16_t median(int16_t* values, uint8_t size) {
      // 將緩衝區內的數據進行排序，由小到大排序
      for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = i + 1; j < size; j++) {
          if (values[j] < values[i]) {
            int16_t temp = values[i];
            values[i] = values[j];
            values[j] = temp;
          }
        }
      }

      // 返回中位數
      return values[size / 2];
    }

    int16_t median_filter(int16_t newValue) {
      // 將新的數據放入緩衝區
      for (uint8_t i = 0; i < WINDOW_SIZE - 1; i++) {
        sensorValues[i] = sensorValues[i + 1];
      }
      sensorValues[WINDOW_SIZE - 1] = newValue;

      // 計算中位數
      return median(sensorValues, WINDOW_SIZE);
    }
 //========float 數據類型===========
    float median(float* values, uint16_t size) {
      // 創建一個暫時的數組來保存排序後的數據
      float sortedValues[size];
      memcpy(sortedValues, values, size * sizeof(float)); // 複製數據到新的數組

      // 將緩衝區內的數據進行排序，由小到大排序
      for (uint16_t i = 0; i < size - 1; i++) {
          for (uint16_t j = i + 1; j < size; j++) {
              if (sortedValues[j] < sortedValues[i]) {
                  float temp = sortedValues[i];
                  sortedValues[i] = sortedValues[j];
                  sortedValues[j] = temp;
              }
          }
      }

      // 返回排序後的中間位置的值
      return sortedValues[size / 2];
    }

    float median_filter(float newValue, float sensorValuesArrays[], uint16_t window_size) {

      // 將新的數據放入緩衝區
      for (uint16_t i = 0; i < window_size - 1; i++) {
        sensorValuesArrays[i] = sensorValuesArrays[i + 1];
      }
      sensorValuesArrays[window_size - 1] = newValue;

      // 計算中位數
      return median(sensorValuesArrays, window_size);
    }
//=======閾 值 濾 波=======
  int16_t   ADNS_thresholdFileter(int16_t newValue){
   if (abs(newValue) <= 0) {
     return 0;
    }   
    else{
      if (abs(newValue) < threshold) {
        return newValue;
        }
      else {
        if(newValue < 0) { return (-threshold); }
        else if(newValue == 0) {return 0; } // 如果數據小於閾值，返回0
        else if(newValue > 0) {return threshold;}
      }
    } 
 }
  float MPU_thresholdFileter(float newValue, float thresholdvalue, String bound) {
    if (abs(newValue) != 0) {
        // 設定上限閾值
        if (bound == "Max") {
            if (abs(newValue) < thresholdvalue) {
                return newValue;
            } else {
                if (newValue < 0) {
                    return (-thresholdvalue);
                } else if (newValue == 0) {
                    return 0;
                } else if (newValue > 0) {
                    return thresholdvalue;
                }
            }
        }
        // 設定下限閾值
        else {
            if (abs(newValue) > thresholdvalue) {
                return newValue;
            } else {
                return 0;
            }
        }
    }
    // 沒有返回值，根據需求添加默認返回值
    return 0; // 這裡我假設若 abs(newValue) == 0，則返回 0
  }

//=======滑動平均濾波=======
  float MAFilter(float data, float buf[], uint16_t filter_num) {
    float filter_sum = 0;
    buf[filter_num - 1] = data; // 更新最後一個元素

    for (int8_t i = 0; i < filter_num - 1; i++) {
        buf[i] = buf[i + 1]; // 所有數據左移，低位仍掉
        filter_sum += buf[i]; // 計算總和
    }

    filter_sum += buf[filter_num - 1]; // 加上最後一個新加入的數據
    return filter_sum / filter_num; // 返回平均值
  }
    long int MAFilter(long int data, long int buf[]) 
  {
    int8_t i;
    long int filter_sum = 0;
    buf[FILTER_N] = data;
    for(i = 0; i < FILTER_N; i++) {
      buf[i] = buf[i + 1]; // 所有數據左移，低位仍掉
      filter_sum += buf[i];
    }
    return (long int)(filter_sum / FILTER_N);
  }
//=======遞增平均==========
  float incrementalAverage(float newValue, int bufferNum) {
    float* readings; // 指向數據緩衝區的指針
    int* index; // 指向索引的指針
    float* total; // 指向總和的指針

    // 根據緩衝區編號選擇對應的數據緩衝區、索引和總和
    if (bufferNum == 1) {
      readings = readings1;
      index = &index1;
      total = &total1;
    } else if (bufferNum == 2) {
      readings = readings2;
      index = &index2;
      total = &total2;
    }

    // 從總和中減去最早的數據，再加上新的數據
    *total = *total - readings[*index] + newValue;
    // 更新數據陣列
    readings[*index] = newValue;
    // 移動索引到下一個位置
    *index = (*index + 1) % numReadings;

    // 計算平均值並回傳
    return *total / numReadings;
  }
//=======設定ADNS規格========
  void PrintSpecification(){

    if(ischangedFR){
        adns.setMaxFrameRate(Shutterrate, Shutterrate_beChanged, FrameRateMin, FrameRateMin_beChanged);
    }
    adns.get_Max_Bound();
    FR = adns.getFrameRate();   // 取得FR  
    adns.setCPI(set_CPI);       // 設定CPI      //  Defalut: 1800
    get_CPI = adns.getCPI();    // 取得CPI
    adns.setLiftDetectionThr(set_Lift_Detec);  //  Defalut: 16
    get_Lift_Detec = adns.getLiftDetectionThr();
    printf(" Shutterrate  -> %6d ", adns.Shutter);    // 快門時間間格
    printf(" |  MinBound  -> %6d ", adns.Min_Bound);  // 幀數下限
    printf(" |  FrameRate -> %6d  fps", FR);
    printf(" |   CPI  -> %6d  cpi", get_CPI);
    printf(" |  Lift_Detection -> %3d \n", get_Lift_Detec);

  }
//======= <時域> 低通濾波=======
  float lowPassFilter(float data, float newData, float filter_koef, float filter_trigger)
  {
    if ((newData - data < filter_trigger) || (data - newData < filter_trigger)){
      data = data + filter_koef * (newData - data);
    }
    else{
      data = newData;
    }
    return data;
  }
//======= <時域> 高通濾波=======
 float highPassFilter(float data, float newData, float filter_koef)
 {
  data = newData - (data + filter_koef * (newData - data));
  return data;
 }

//=======變量初始化=========
  void initialLocalVarities(){

  }
//=======計算FPS(有問題)=========
 int calOperatingRate(unsigned long* preMicroTime) 
 {
    unsigned long curr = micros();
    double dt = (double)(curr - *preMicroTime) / 1000000.0;   //  s
    double Execution_Frequenc = 1 / dt;   //  Hz
    *preMicroTime = curr;
    return (int)Execution_Frequenc;
 }




  
