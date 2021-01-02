#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Arduino.h>

#include <SD.h>
#include <driver/i2s.h>

#include <WiFi.h>
#include <WebServer.h>

#define FFTN (256)
#define STP  (1)

volatile float realSpec[FFTN];
volatile float imagSpec[FFTN];
volatile bool fftReady = false;

// スマホ側リモコン画面コンテンツ
const char html[] = "<html><body><font size=15><a href='/p'><=</a> <a href='/n'>=></a></font></body></html>";

hw_timer_t *timer = NULL;   // 音声再生用
hw_timer_t *timer2 = NULL;  // パイロットランプ用
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long cnt = 0;

#define SD_BUFF_SIZE   (512)
#define SD_BUFF_NUM (8) 
unsigned char sdBuff[SD_BUFF_NUM][SD_BUFF_SIZE];
unsigned char dummyBuff[8];

int readBuffNum = 0;
int writeBuffNum = 0;
typedef enum {
  BUFF_EMPTY,
  BUFF_READING,
  BUFF_WRITING,
  BUFF_FULL,
} buff_status_t;
int buffStatus[SD_BUFF_NUM] = {BUFF_EMPTY};

// 音声データ出力(I2S)
void IRAM_ATTR sound_out() {
  portENTER_CRITICAL_ISR(&timerMux);
  size_t written = 0;
  size_t accumWritten = 0;

  if(buffStatus[readBuffNum] == BUFF_FULL && cnt == 0) {
    buffStatus[readBuffNum] = BUFF_READING;
  }
  if(buffStatus[readBuffNum] == BUFF_READING) {
    do {
      i2s_write(I2S_NUM_0, (char *)&sdBuff[readBuffNum][cnt], 256, &written, 0);
      cnt += written;
      accumWritten += written;
    } while(accumWritten == 256);

    if(cnt >= SD_BUFF_SIZE) {
      buffStatus[readBuffNum] = BUFF_EMPTY;
      int tmp_readBuffNum = readBuffNum;
      ++tmp_readBuffNum %= SD_BUFF_NUM;
      if(buffStatus[tmp_readBuffNum] == BUFF_FULL) {
        cnt = 0;
        ++readBuffNum %= SD_BUFF_NUM;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// SD
#define SD_SCK  (18)
#define SD_MISO (19)
#define SD_MOSI (23)
#define SD_SS   ( 5)
SPIClass hSpi(HSPI);

//////////// TFT ////////////
#define TFT_DC    (26)
#define TFT_RST   (25)
#define TFT_CS    ( 4)  // jtag tdo   (15)
#define TFT_MOSI  (32)  // jtag clock (13)
#define TFT_SCLK  (33)  // jtag mode  (14)
SPIClass vSpi(VSPI);
Adafruit_ILI9341 tft = Adafruit_ILI9341(&vSpi, TFT_DC, TFT_CS, TFT_RST);

volatile  float fReal[FFTN], fImag[FFTN];
volatile  float fftBuff[FFTN];
float oldFftBuff[FFTN];

// FFT計算本体(特別感謝：FPGAマガジン)
inline void calcFFT(volatile float *iW, volatile float *oR, volatile float *oI) {
    short N = FFTN;
    short stg;
    short halfN = N / 2;
    short i, j, k, kp, m, h;
    float w1, w2, s1, s2, t1, t2;

    // ステージ数を求める
    i = N;
    stg = 0;
    while (i != 1) {
        i = i / 2;
        stg++;
    }

    // データ入力
    for (i = 0; i < N; i += STP) {
      float x = (float)i / N;
        // ブラックマン窓を適用
        // fReal[i] = iW[i] * (0.42 - 0.5 * cos(2 * PI * x) + 0.08 * cos(4 * PI * x));
        // ナットール窓を適用
        fReal[i] = iW[i] * (0.355768 - 0.487396 * cos(2 * PI * x) + 0.144232 * cos(4 * PI * x) - 0.012604 * cos(6 * PI * x));
        fImag[i] = 0;
    }

    // ビット逆順ソート
    j = 0;
    for (i = 0; i <= N - 2; i += STP) {
        if (i < j) {
            t1 = fReal[j];
            fReal[j] = fReal[i];
            fReal[i] = t1;
        }
        k = halfN;
        while (k <= j) {
            j = j - k;
            k = k / 2;
        }
        j += k;
    }

    // バタフライ演算
    for (i = 1; i <= stg; i++) {
        m = pow(2, i);
        h = m / 2;
        for (j = 0; j < h; j++) {
            float w1in = j * (N / m);
            float w2in = w1in + halfN;
                w1 = cos(w1in / (FFTN / (2 * PI)));
                w2 = sin(w2in / (FFTN / (2 * PI)));
            for (k = j; k < N; k += m) {
                kp = k + h;
                s1 = fReal[kp] * w1 - fImag[kp] * w2;
                s2 = fReal[kp] * w2 + fImag[kp] * w1;
                t1 = fReal[k] + s1;
                fReal[kp] = fReal[k] - s1;
                fReal[k] = t1;
                t2 = fImag[k] + s2;
                fImag[kp] = fImag[k] - s2;
                fImag[k] = t2;
            }
        }
    }

    // 結果を出力用配列に格納
    for (i = 0; i < N; i += STP) {
        oR[i] = fReal[i];
    }
}

// FFT計算は別コアで
void fft(void *param) {
  for(;;) {
    fftReady = false;
    // short -> float 変換
    for(int i = 0; i < FFTN; i++) {
      fftBuff[i] = (fftBuff[i] + 0.5) / 32767.5;
    }
    calcFFT(fftBuff , realSpec, imagSpec);
    fftReady = true;
    delay(25);
  }
}

WebServer server(80);
volatile bool prev = false;
volatile bool next = false;

void handleRoot() {
  char buf[1024];
  sprintf(buf, "%s", html);
  server.send(200, "text/html", buf);
}

void handlePrev() {
  char buf[1024];
  prev = true;
  next = false;
  sprintf(buf, html);
  server.send(200, "text/html", buf);
}

void handleNext() {
  char buf[1024];
  prev = false;
  next = true;
  sprintf(buf, html);
  server.send(200, "text/html", buf);
}

File gRoot;
int gFileCnt = 0;
#define PLAY_LIST_MAX (256)
char *playList[PLAY_LIST_MAX] = {0};

// プレイリストにファイルパスを格納
void getFileNames(char **playList, File dir) {
  for(;;) {
    File item = dir.openNextFile();
    if(!item || gFileCnt >= PLAY_LIST_MAX) {
      break;
    } 

    if(item.isDirectory()) {
      // 特定ディレクトリは中に入らないよう除外
      if(strcmp(item.name(), "/System Volume Information") != 0 &&
         strcmp(item.name(), "/hidden") != 0) {
        getFileNames(playList, item);
      }
    } else {
      playList[gFileCnt] = (char *)malloc(256);
      strcpy(playList[gFileCnt], item.name()); 
      gFileCnt++;
    }
  }
}

// プレイリスト作成
void makePlayList(char **playList) {
  File root = SD.open("/");
  getFileNames(playList, root);
  root.close();
}

// NTPテスト
void ntpTest(const char *srv) {
  configTime(9 * 3600, 0, srv);
  struct tm timeInfo;
  getLocalTime(&timeInfo);
  Serial.printf(" %04d/%02d/%02d %02d:%02d:%02d\n",
          timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
          timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
}

void setup(void) {
  // FFTを別コアで実行するよう指定
  xTaskCreatePinnedToCore(fft, "fft", 4096 * 4, NULL, 1, NULL, 0);

  Serial.begin(115200);

  WiFi.begin("rs500m-14aaf8-1", "6a6a227fb3f17");
  while(WiFi.status() != WL_CONNECTED) {
    delay(250);
  }
  Serial.print("IP Addr. : ");
  Serial.println(WiFi.localIP());

  // リモコンのハンドラ登録
  server.on("/", handleRoot);
  server.on("/n", handleNext);
  server.on("/p", handlePrev);
  server.begin();

  // SDカード
  vSpi.end();
  vSpi.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  vSpi.setFrequency(80000000);
  vSpi.setDataMode(SPI_MODE1);

  // LCD
  tft.begin(80000000);
  tft.setRotation(3);
  tft.fillRect(0, 0, 320, 240, ILI9341_BLACK);

  // NTPテスト
  ntpTest("time.google.com");
  
  // (撮影用)ウェイト
  delay(5000);

  tft.setCursor(70, 30);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_RED);
  tft.println("Powered by");  
  tft.setCursor(100, 40);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.println("PCM5102A");  

  hSpi.end();
  hSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_SS);
  hSpi.setFrequency(15800000);
  hSpi.setDataMode(SPI_MODE1);

  if(!SD.begin(SD_SS, hSpi, 15800000)) {
    Serial.println("SD card mount failed!!");
  }

  makePlayList(playList);

  // I2S設定
  int i2sRet;
  const i2s_config_t config = {
    (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    44100,
    I2S_BITS_PER_SAMPLE_16BIT,
    I2S_CHANNEL_FMT_RIGHT_LEFT,
    (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    ESP_INTR_FLAG_LEVEL1,
    64,
    256,
    true,
    false,
    12000000  // 12.0MHz外部オシレータ使用時の設定値    
  };

  i2sRet = i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_pin_config_t pinConfig = {
    16, // BCK
     2, // WS
    21, // DAT
    I2S_PIN_NO_CHANGE};
  i2sRet = i2s_set_pin(I2S_NUM_0, &pinConfig);

  // タイマー割り込み設定 
  timer = timerBegin(1, 4, true);
  timerAttachInterrupt(timer, sound_out, true);
  timerAlarmWrite(timer, 882 << 5, true);
  timerAlarmDisable(timer);

  Serial.println("Initialized");
}

float oldSpec[60] = {0};
float peaks[60] = {0};
unsigned char oldPeaks[60] = {0};

void loop() {
  // プレイリストのインデックス
  static int wavNum = 0;

  bool flg_skipHeader = true;
  bool flg_waitForBufferFilled = true;

  //LCD
  static int x = 0;
  static int y = 0;
  static bool first = true;
  static int segment = 0;

  // 描画範囲指定
  typedef struct {
    int from;
    int to;
  } T_RANGE;
  static T_RANGE ranges[4] = {{ 0,  8},
                              { 8, 16},
                              {16, 24},
                              {24, 32},
  };

  File file = SD.open(playList[wavNum], FILE_READ);

  if(!file) {
    Serial.println("FILE_READ error...");
  } else {
    // 再生データ長を取得
    int rest = file.size();
    cnt = 0;
    readBuffNum = 0;
    writeBuffNum = 0;
    memset(buffStatus, BUFF_EMPTY, sizeof(buffStatus));

    while(rest > 0) {
      if(flg_skipHeader) {
        // wavヘッダ部簡易読み込み
        unsigned char wavHeader[44];
        file.read(wavHeader, 44);
        rest -= 44;
        flg_skipHeader = false;

        // バッファ全面にデータを読み込んでおく
        do {
          file.read(sdBuff[writeBuffNum], rest < SD_BUFF_SIZE ? rest : SD_BUFF_SIZE);
          rest -= SD_BUFF_SIZE;
          ++writeBuffNum;
        } while(writeBuffNum < SD_BUFF_NUM);

        for(int i = 0; i < SD_BUFF_NUM; i++) {
          buffStatus[i] = BUFF_FULL;
        }
        readBuffNum = 0;
        writeBuffNum = 0;

        if(flg_waitForBufferFilled) {
          // 再生開始
          timerAlarmEnable(timer);
          flg_waitForBufferFilled = false;
        }
      }
      // 空のバッファにデータ読み込み
      if(buffStatus[writeBuffNum] == BUFF_EMPTY) {
        buffStatus[writeBuffNum] = BUFF_WRITING;
        file.read(sdBuff[writeBuffNum], rest < SD_BUFF_SIZE ? rest : SD_BUFF_SIZE);
        rest -= SD_BUFF_SIZE;
        buffStatus[writeBuffNum] = BUFF_FULL;

        // 描画
        if(first) {
          for(int i = 0; i < 60; i++) {
            peaks[i] = 240;
          }
          memset(oldFftBuff, 120, 60);
        }         
        // loop() 1サイクルで全部描画せず区画を分けて描画(速度問題対策)
        for(x = ranges[segment].from; x < ranges[segment].to; x++) {
          if(fftReady) {
            for(int i = 0; i < FFTN; i += 4) {
              fftBuff[i] = sdBuff[readBuffNum][cnt + i + 0] + sdBuff[readBuffNum][cnt + i + 1] * 256;
            }
          }

          int x8 = (x << 3) + 32;
          //tft.drawFastVLine(x8,  oldSpec[x],  -80, ILI9341_BLACK);
          tft.drawFastHLine(x8 - 1, oldPeaks[x], 2, ILI9341_BLACK);

          int spec;    
          // バーの長さ補正
          if(x < 2) {
            spec = realSpec[x + 1] /  1;
          } else if(x < 8) {
            spec = realSpec[x + 1] *  9;
          } else {
            spec = realSpec[x + 1] * 17;
          }
          // バーの長さをクリップ
          if(abs(spec) > 160) {
            spec = 0;
          }

          y = 240 - (spec * 1);

          // 頂点の押し出し
          if(peaks[x] > y) {
            peaks[x] = y;
          }

          // 痕跡消去
          tft.drawFastVLine(x8, y, 80 - y, ILI9341_BLACK);
          // 書き込み
          tft.drawFastVLine(x8, 240, 240 - y, ILI9341_BLUE);
          tft.drawFastHLine(x8 - 1, peaks[x],  2, ILI9341_WHITE);

          oldSpec[x] = y;
          oldPeaks[x] = peaks[x];
          if(peaks[x] < 240) {
            peaks[x] += 0.5;
          } else {
            peaks[x] = 240;
          }
          first = false;
        }
        ++segment %= 4;

        // デバッグ用表示
        if(writeBuffNum == 0) {
          tft.fillRect(0, 0, 60, 10, ILI9341_BLACK); 
          tft.setCursor(0,0);
          tft.setTextSize(0);
          tft.setTextColor(ILI9341_WHITE);
          tft.printf("%d", rest);
        } 
        tft.fillRect(0, 10, 30, 10, ILI9341_BLACK); 
        tft.setCursor(0,10);
        tft.setTextSize(0);
        tft.setTextColor(ILI9341_GREEN);
        tft.printf("%d:%d", readBuffNum, writeBuffNum);

        ++writeBuffNum %= SD_BUFF_NUM;
      } else {
        ;
      }
      // 選曲制御
      if(prev == true) {
        prev = false;
        --wavNum;
        if(wavNum == -1) {
          wavNum = gFileCnt - 1;
        }
        goto SKIP_FILE;
      }
      if(next == true) {
        next = false;
        ++wavNum %= gFileCnt;
        goto SKIP_FILE;
      }
      server.handleClient();
    }
    ++wavNum %= gFileCnt;
SKIP_FILE:
    // ダミー
    ;
  }
}
