#include "LPS.h"
#include "BLE.h"

LPS lps;
BLE ble;
uint8_t device_address = 0x05;
uint64_t timeA1;
uint64_t timeA2;
uint64_t timeA3;
uint64_t receivedTime;
uint8_t anchor_id;

bool A1Status = false;
bool A2Status = false;
bool A3Status = false;

double skew_rate_A2 = 1.0;
double skew_rate_A3 = 1.0;
double skewRateTag = 1.0;

uint8_t expectedAnchor = 0x01;

// --- Median Filter ---
const int FILTER_SIZE = 5;
int64_t deltaBuffer12[FILTER_SIZE] = {0, 0, 0, 0, 0};
int bufferIndex12 = 0;

// Fungsi untuk memfilter delta tick menggunakan metode Moving Median
int64_t movingMedianFilter12(int64_t rawDelta) {
  // 1. Masukkan data baru ke dalam buffer berjalan
  deltaBuffer12[bufferIndex12] = rawDelta;
  bufferIndex12 = (bufferIndex12 + 1) % FILTER_SIZE;

  // 2. Salin isi buffer ke array sementara untuk diurutkan
  int64_t sortedBuffer[FILTER_SIZE];
  for (int i = 0; i < FILTER_SIZE; i++) {
    sortedBuffer[i] = deltaBuffer12[i];
  }

  // 3. Algoritma Bubble Sort Sederhana
  for (int i = 0; i < FILTER_SIZE - 1; i++) {
    for (int j = 0; j < FILTER_SIZE - i - 1; j++) {
      if (sortedBuffer[j] > sortedBuffer[j + 1]) {
        int64_t temp = sortedBuffer[j];
        sortedBuffer[j] = sortedBuffer[j + 1];
        sortedBuffer[j + 1] = temp;
      }
    }
  }

  // 4. Kembalikan nilai tengah (Index ke-2)
  return sortedBuffer[2];
}

// --- Median Filter ---
// const int FILTER_SIZE = 5;
int64_t deltaBuffer13[FILTER_SIZE] = {0, 0, 0, 0, 0};
int bufferIndex13 = 0;

// Fungsi untuk memfilter delta tick menggunakan metode Moving Median
int64_t movingMedianFilter13(int64_t rawDelta) {
  // 1. Masukkan data baru ke dalam buffer berjalan
  deltaBuffer13[bufferIndex13] = rawDelta;
  bufferIndex13 = (bufferIndex13 + 1) % FILTER_SIZE;

  // 2. Salin isi buffer ke array sementara untuk diurutkan
  int64_t sortedBuffer[FILTER_SIZE];
  for (int i = 0; i < FILTER_SIZE; i++) {
    sortedBuffer[i] = deltaBuffer13[i];
  }

  // 3. Algoritma Bubble Sort Sederhana
  for (int i = 0; i < FILTER_SIZE - 1; i++) {
    for (int j = 0; j < FILTER_SIZE - i - 1; j++) {
      if (sortedBuffer[j] > sortedBuffer[j + 1]) {
        int64_t temp = sortedBuffer[j];
        sortedBuffer[j] = sortedBuffer[j + 1];
        sortedBuffer[j + 1] = temp;
      }
    }
  }

  // 4. Kembalikan nilai tengah (Index ke-2)
  return sortedBuffer[2];
}

// --- VARIABEL GLOBAL UNTUK MOVING AVERAGE FILTER ---
const int AVG_FILTER_SIZE = 50; // Menggunakan 10 sampel agar lebih halus
double avgBuffer12[AVG_FILTER_SIZE] = {0};
int avgIndex12 = 0;
double avgSum12 = 0;
bool bufferFull12 = false; // Memastikan pembagian akurat di awal start alat

double movingAverageFilter12(double rawDelta) {
  // 1. Kurangi total sum dengan data tertua yang akan dibuang
  avgSum12 -= avgBuffer12[avgIndex12];

  // 2. Masukkan data baru ke dalam buffer
  avgBuffer12[avgIndex12] = rawDelta;

  // 3. Tambahkan data baru tersebut ke total sum
  avgSum12 += rawDelta;

  // 4. Geser indeks buffer berjalan
  avgIndex12 = (avgIndex12 + 1) % AVG_FILTER_SIZE;

  // 5. Cek apakah buffer sudah terisi penuh sejak pertama kali alat menyala
  if (avgIndex12 == 0) bufferFull12 = true;

  // 6. Hitung rata-rata pembagian
  if (bufferFull12) {
    return avgSum12 / AVG_FILTER_SIZE;
  } else {
    // Jika baru menyala dan buffer belum penuh, bagi dengan jumlah data yang ada saat ini
    return avgSum12 / avgIndex12;
  }
}

int cnt = 0;
int64_t sum = 0;
int64_t offset12 = 16170;
int64_t offset13 = 16002;
const double TICK_TO_METER = 0.004691751;
int64_t delta12Filtered;
int64_t delta13Filtered;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  // ble.begin();
  // while(!ble.connectionStatus()){
  //   delay(1);
  // }
  // digitalWrite(led, HIGH);
  // delay(200);
  // digitalWrite(led, LOW);
  // delay(200);
  // digitalWrite(led, HIGH);

  lps.begin(device_address);
  lps.printModulInfo();
  lps.restartTRX();
  lps.startReceive();
}

void loop() {
  if(receiveAck){
    receiveAck = false;
    receivedTime = DW1000Ng::getReceiveTimestamp();
    lps.commitData();
    if(lps.getLastSender() == expectedAnchor) {
      if(expectedAnchor == ANCHOR_MAIN){
        timeA1 = receivedTime;

        skewRateTag = lps.getSkewRateTag(receivedTime);

        expectedAnchor = ANCHOR_B;
      }
      else if(expectedAnchor == ANCHOR_B){
        timeA2 = receivedTime;
        skew_rate_A2 = lps.getSkewRate();
        expectedAnchor = ANCHOR_C; //debug, ganti ke anchor c kalau sudah ready

        ///////////DEBUG/////////////
        // uint64_t calibratedA2 = (uint64_t)((double)timeA2 * skew_rate_A2);
        // uint64_t calibrateDelayUs = (uint64_t)((double)lps.uS2UWB(10000) * skew_rate_A2);
        // uint64_t deltaTick12 = timeA2 - calibrateDelayUs - timeA1;
        //uint64_t deltaTick12 = calibratedA2 - timeA1;

        // skewRateTag = movingAverageFilter12(skewRateTag);
        // skew_rate_A2 = movingAverageFilter12(skew_rate_A2);
        //Working code
        double deltaRx_Tag = (double)(timeA2 - timeA1) * skewRateTag; //* skewRateTag
        double deltaDelay_B = (double)lps.uS2UWB(10000) * skew_rate_A2; //* skew_rate_A2
        int64_t deltaTick12_raw = (int64_t)(deltaRx_Tag - deltaDelay_B);
        delta12Filtered = movingMedianFilter12(deltaTick12_raw);

        delta12Filtered = delta12Filtered-offset12;
        //double deltaD12 = (double)delta12Filtered * TICK_TO_METER;

        // // Serial.print("DeltaD12:"); Serial.println(delta12Filtered);
        // Serial.println(delta12Filtered);
        //End of Working code
        // sum += delta12Filtered;
        // cnt++;

        // if(cnt>100){
        //   int64_t avg = (int64_t)((double)sum/(double)cnt);
        //   // Serial.print("Delta13 : "); Serial.println((long)delta12Filtered);
        //   Serial.print("Avg_Delta12Filtered : "); Serial.println((long)avg);
        //   sum = 0;
        //   cnt = 0;
        // }

        //Serial.print("Delta12 : "); Serial.println((long)delta12Filtered);
      
        
        //digitalWrite(led, digitalRead(led) ^ 1);
        /////////////////////////////

      }
      else if(expectedAnchor == ANCHOR_C){
        timeA3 = receivedTime;
        skew_rate_A3 = lps.getSkewRate();
        expectedAnchor = ANCHOR_MAIN;
        double deltaRx_Tag = (double)(timeA3 - timeA1) * skewRateTag; //* skewRateTag
        double deltaDelay_C = (double)lps.uS2UWB(20000) * skew_rate_A3; //* skew_rate_A2
        int64_t deltaTick13_raw = (int64_t)(deltaRx_Tag - deltaDelay_C);
        delta13Filtered = movingMedianFilter13(deltaTick13_raw);
        delta13Filtered = delta13Filtered - offset13;
        
        digitalWrite(led, digitalRead(led) ^ 1);
        Serial.print(delta12Filtered); Serial.print(","); Serial.println(delta13Filtered);
      }
    }
    else{
      //wrong anchor
      //reset all status
      expectedAnchor = ANCHOR_MAIN;
    }


    lps.startReceive();
  }
}
