const int sensorPin = A0;
const int sampleInterval = 100; // ms
const int sampleLength = 100;   // 10초 (100개 샘플)
const float R1 = 10000.0;      // 기준 저항 10kΩ

int samples[sampleLength];

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
}

void loop() {
  // 데이터 수집
  for(int i=0; i<sampleLength; i++) {
    int rawValue = analogRead(sensorPin);
    // 저항 값 계산 (옴 단위)
    float resistance = R1 * (1023.0/rawValue - 1.0);
    
    Serial.print("Raw Value: ");
    Serial.print(rawValue);
    Serial.print(", Resistance: ");
    Serial.print(resistance/1000.0); // kΩ 단위로 변환
    Serial.println(" kΩ");
    
    samples[i] = rawValue;
    delay(sampleInterval);
  }
  
  // 호흡수 계산
  int breathRate = countBreaths(samples, sampleLength);
  Serial.print("호흡수(분): ");
  Serial.println(breathRate);
  Serial.println("-------------------");
}

int countBreaths(int* vals, int len) {
  // 간단 이동평균으로 신호 평활화
  int maWin = 5;
  float filtered[len];
  float lastPeakPos = 0;  // 마지막 피크의 위치
  
  // 이동평균 필터링
  for(int i=0; i<len; i++) {
    float sum = 0;
    int count = 0;
    for(int k=0; k<maWin; k++) {
      if(i-k >= 0) {
        sum += vals[i-k];
        count++;
      }
    }
    filtered[i] = sum / count;
    
    // 저항 값으로 변환하여 출력
    float resistance = R1 * (1023.0/filtered[i] - 1.0);
    Serial.print("필터링된 저항 값 [");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(resistance/1000.0);
    Serial.println(" kΩ");
  }
  
  // 봉우리(peak) 검출
  int peaks = 0;
  float minPeakHeight = 0.5;  // 최소 피크 높이
  float minPeakDistance = 10; // 최소 피크 간격 (샘플 수)
  
  for(int i=2; i<len-2; i++) {
    // 5포인트 비교로 피크 검출
    if(filtered[i] > filtered[i-2] && 
       filtered[i] > filtered[i-1] && 
       filtered[i] >= filtered[i+1] && 
       filtered[i] > filtered[i+2]) {
      
      // 이전 피크와의 거리 확인
      if(i - lastPeakPos >= minPeakDistance) {
        // 이전 값과의 차이가 최소 높이보다 큰지 확인
        float peakHeight = filtered[i] - filtered[i-2];
        if(peakHeight >= minPeakHeight) {
          peaks++;
          lastPeakPos = i;
          // 피크에서의 저항 값 출력
          float peakResistance = R1 * (1023.0/filtered[i] - 1.0);
          Serial.print("피크 감지! 위치: ");
          Serial.print(i);
          Serial.print(", 저항: ");
          Serial.print(peakResistance/1000.0);
          Serial.println(" kΩ");
        }
      }
    }
  }
  
  Serial.print("검출된 총 피크 수: ");
  Serial.println(peaks);
  
  // 10초 동안 측정된 호흡수를 1분 기준으로 환산 (x6)
  return peaks * 6;
}